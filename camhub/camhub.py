"""camhub: discovers camnode ESP32-CAMs on the LAN and shows them on one page.

Cameras are found automatically via mDNS (_espcam._tcp). Cameras can also be added
by IP from the page (useful when mDNS is blocked). Each camera's MJPEG stream is
pulled once and relayed to any number of browser viewers.

Also: motion events (with snapshots and optional desktop notifications), SD-card
recordings and time-lapse browsing, time-lapse -> MP4, and OTA firmware updates.

Run:  uv run camhub.py [--port 8765]
"""

import argparse
import asyncio
import json
import logging
import os
import re
import socket
import time
import uuid
from pathlib import Path

import aiohttp
from aiohttp import web
from zeroconf import ServiceStateChange
from zeroconf.asyncio import AsyncServiceBrowser, AsyncServiceInfo, AsyncZeroconf

SERVICE = "_espcam._tcp.local."
HERE = Path(__file__).parent
CONFIG = HERE / "cameras.json"
EVENTS_FILE = HERE / "events.json"
EVENTS_DIR = HERE / "events"
VIDEOS_DIR = HERE / "videos"
# Prefer the ESP-IDF firmware; fall back to the legacy Arduino sketch if it is the
# only tree present (an older checkout, or cameras not yet migrated).
IDF_DIR = HERE.parent / "camnode-idf"
ARDUINO_DIR = HERE.parent / "camnode"
FIRMWARE_DIR = IDF_DIR if (IDF_DIR / "update-all.sh").exists() else ARDUINO_DIR
EVENT_KEEP_DAYS = 14
EVENT_KEEP_MAX = 2000
BOUNDARY = "camhubframe"

log = logging.getLogger("camhub")


class Camera:
    def __init__(self, cam_id: str, host: str, port: int = 80, source: str = "mdns"):
        self.id = cam_id
        self.host = host
        self.port = port
        self.stream_port = 81
        self.files_port = 82
        self.source = source
        self.status: dict = {}
        self.motion: dict = {}
        self.event: dict | None = None  # open motion event
        self.last_notify = 0.0
        self.frame: bytes | None = None
        self.frame_time = 0.0
        self.fps = 0.0
        self.online = False
        self.viewers = 0
        self.new_frame = asyncio.Condition()
        self.task: asyncio.Task | None = None
        self.status_task: asyncio.Task | None = None
        self.motion_task: asyncio.Task | None = None
        self.last_seen = time.time()

    @property
    def base(self) -> str:
        return f"http://{self.host}:{self.port}"

    @property
    def stream_url(self) -> str:
        return f"http://{self.host}:{self.stream_port}/stream"

    @property
    def files_base(self) -> str:
        return f"http://{self.host}:{self.files_port}"

    def info(self, prefs: dict) -> dict:
        p = prefs.get(self.id, {})
        age = time.time() - self.frame_time if self.frame_time else None
        return {
            "id": self.id,
            "name": self.status.get("label") or self.id,
            "host": self.host,
            "online": self.online,
            "stale": self.online and age is not None and age > 5,
            "fps": round(self.fps, 1),
            "viewers": self.viewers,
            "source": self.source,
            "rotation": p.get("rotation", 0),
            "hidden": p.get("hidden", False),
            "status": self.status,
            "motion": {k: self.motion.get(k) for k in ("active", "level", "events", "enabled")},
        }


class Hub:
    def __init__(self):
        self.cameras: dict[str, Camera] = {}
        self.session: aiohttp.ClientSession | None = None
        cfg = self.load_config()
        self.prefs: dict = cfg.get("prefs", {})   # per-camera hub-side view settings
        self.manual: list = cfg.get("manual", [])
        self.settings: dict = {"notify": False, **cfg.get("settings", {})}
        self.events: list = self.load_events()

    def load_config(self) -> dict:
        try:
            return json.loads(CONFIG.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def save_config(self):
        CONFIG.write_text(json.dumps({"prefs": self.prefs, "manual": self.manual,
                                      "settings": self.settings}, indent=2))

    def load_events(self) -> list:
        try:
            return json.loads(EVENTS_FILE.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def save_events(self):
        cutoff = time.time() - EVENT_KEEP_DAYS * 86400
        keep = [e for e in self.events if e["start"] >= cutoff][-EVENT_KEEP_MAX:]
        for e in self.events:
            if e not in keep and e.get("snap"):
                (EVENTS_DIR / e["snap"]).unlink(missing_ok=True)
        self.events = keep
        tmp = EVENTS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.events))
        tmp.replace(EVENTS_FILE)

    def add(self, cam_id: str, host: str, port: int = 80, source: str = "mdns",
            stream_port: int = 81) -> Camera:
        cam = self.cameras.get(cam_id)
        if cam:
            cam.host, cam.port, cam.stream_port, cam.last_seen = host, port, stream_port, time.time()
            return cam
        cam = Camera(cam_id, host, port, source)
        cam.stream_port = stream_port
        self.cameras[cam_id] = cam
        cam.task = asyncio.create_task(self.pull(cam))
        cam.status_task = asyncio.create_task(self.poll_status(cam))
        cam.motion_task = asyncio.create_task(self.poll_motion(cam))
        log.info("camera added: %s at %s:%s (%s)", cam_id, host, port, source)
        return cam

    async def refresh_status(self, cam: Camera):
        async with self.session.get(f"{cam.base}/status", timeout=aiohttp.ClientTimeout(total=4)) as r:
            cam.status = await r.json(content_type=None)
            cam.stream_port = cam.status.get("stream_port", cam.port)
            cam.files_port = cam.status.get("files_port", 0)

    async def poll_status(self, cam: Camera):
        while cam.id in self.cameras:
            try:
                await self.refresh_status(cam)
            except asyncio.CancelledError:
                raise
            except Exception:
                pass
            await asyncio.sleep(10)

    async def poll_motion(self, cam: Camera):
        """Watch the camera's motion detector once a second and log events."""
        while cam.id in self.cameras:
            await asyncio.sleep(1)
            if "md" not in cam.status:  # firmware without motion detection
                continue
            try:
                async with self.session.get(f"{cam.base}/motion", timeout=aiohttp.ClientTimeout(total=3)) as r:
                    m = await r.json(content_type=None)
            except asyncio.CancelledError:
                raise
            except Exception:
                continue
            was = bool(cam.motion.get("active"))
            cam.motion = m
            now = time.time()
            if m.get("active") and not was:
                self.motion_started(cam, now)
            elif m.get("active") and cam.event:
                cam.event["end"] = now
                cam.event["peak"] = max(cam.event["peak"], m.get("level", 0))
            elif not m.get("active") and was and cam.event:
                cam.event["end"] = now
                cam.event = None
                self.save_events()

    def motion_started(self, cam: Camera, now: float):
        EVENTS_DIR.mkdir(exist_ok=True)
        snap = None
        if cam.frame:
            snap = f"{cam.id}_{time.strftime('%Y%m%d-%H%M%S', time.localtime(now))}_{uuid.uuid4().hex[:6]}.jpg"
            (EVENTS_DIR / snap).write_bytes(cam.frame)
        cam.event = {"id": uuid.uuid4().hex[:12], "cam": cam.id, "name": cam.status.get("label") or cam.id,
                     "start": now, "end": now, "peak": cam.motion.get("level", 0), "snap": snap}
        self.events.append(cam.event)
        self.save_events()
        log.info("motion on %s", cam.id)
        if self.settings.get("notify") and now - cam.last_notify > 60:
            cam.last_notify = now
            asyncio.create_task(notify(f"Motion: {cam.event['name']}",
                                       time.strftime("%H:%M:%S", time.localtime(now)),
                                       EVENTS_DIR / snap if snap else None))

    async def pull(self, cam: Camera):
        """Keep one upstream MJPEG connection per camera, parse frames, notify viewers."""
        backoff = 1
        while cam.id in self.cameras:
            try:
                timeout = aiohttp.ClientTimeout(total=None, connect=5, sock_read=10)
                async with self.session.get(cam.stream_url, timeout=timeout) as resp:
                    resp.raise_for_status()
                    cam.online = True
                    backoff = 1
                    buf = b""
                    count, t0 = 0, time.time()
                    async for chunk in resp.content.iter_any():
                        buf += chunk
                        while True:
                            start = buf.find(b"\xff\xd8")
                            end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                            if start < 0 or end < 0:
                                if start > 0:
                                    buf = buf[start:]
                                break
                            frame = buf[start : end + 2]
                            buf = buf[end + 2 :]
                            async with cam.new_frame:
                                cam.frame, cam.frame_time = frame, time.time()
                                cam.new_frame.notify_all()
                            count += 1
                            if time.time() - t0 >= 2:
                                cam.fps = count / (time.time() - t0)
                                count, t0 = 0, time.time()
                        if len(buf) > 2_000_000:
                            buf = b""
            except asyncio.CancelledError:
                raise
            except Exception as e:
                if cam.online:
                    log.warning("camera %s offline: %s", cam.id, e)
                cam.online, cam.fps = False, 0.0
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)


hub = Hub()


async def on_service(zc, service_type, name, state_change):
    if state_change not in (ServiceStateChange.Added, ServiceStateChange.Updated):
        return
    info = AsyncServiceInfo(service_type, name)
    if not await info.async_request(zc, 3000):
        return
    addrs = [socket.inet_ntoa(a) for a in info.addresses if len(a) == 4]
    props = {k.decode(): (v or b"").decode() for k, v in info.properties.items()}
    sport = int(props.get("stream_port") or info.port or 80)
    if addrs:
        hub.add(name.split(".")[0], addrs[0], info.port or 80, "mdns", sport)


async def discover(app):
    azc = AsyncZeroconf()

    def handler(zeroconf, service_type, name, state_change):
        asyncio.ensure_future(on_service(zeroconf, service_type, name, state_change))

    browser = AsyncServiceBrowser(azc.zeroconf, [SERVICE], handlers=[handler])
    app["zc"] = (azc, browser)
    yield
    await browser.async_cancel()
    await azc.async_close()


async def client_session(app):
    hub.session = aiohttp.ClientSession()
    for host in hub.manual:
        await add_manual(host)
    yield
    for cam in list(hub.cameras.values()):
        stop(cam)
    await hub.session.close()


def stop(cam: Camera):
    for t in (cam.task, cam.status_task, cam.motion_task):
        if t:
            t.cancel()


async def add_manual(host: str) -> Camera | None:
    """Ask the camera for its name so manual and mDNS entries share one id."""
    try:
        async with hub.session.get(f"http://{host}/status", timeout=aiohttp.ClientTimeout(total=4)) as r:
            st = await r.json(content_type=None)
        name, sport = st.get("name") or host, st.get("stream_port", 80)
    except Exception:
        name, sport = host, 81
    return hub.add(name, host, 80, "manual", sport)


# ---------- HTTP handlers ----------

async def index(request):
    return web.FileResponse(HERE / "index.html")


async def api_cameras(request):
    cams = [c.info(hub.prefs) for c in hub.cameras.values()]
    cams.sort(key=lambda c: c["name"].lower())
    return web.json_response(cams)


async def api_add(request):
    data = await request.json()
    host = str(data.get("host", "")).strip()
    if not host:
        raise web.HTTPBadRequest(text="host required")
    cam = await add_manual(host)
    if host not in hub.manual:
        hub.manual.append(host)
        hub.save_config()
    return web.json_response(cam.info(hub.prefs))


async def api_view(request):
    """Hub-side view settings: rotation, hidden."""
    cam_id = request.match_info["id"]
    data = await request.json()
    p = hub.prefs.setdefault(cam_id, {})
    if "rotation" in data and int(data["rotation"]) in (0, 90, 180, 270):
        p["rotation"] = int(data["rotation"])
    if "hidden" in data:
        p["hidden"] = bool(data["hidden"])
    hub.save_config()
    return web.json_response(p)


async def api_remove(request):
    cam = hub.cameras.pop(request.match_info["id"], None)
    if cam:
        stop(cam)
        if cam.host in hub.manual:
            hub.manual.remove(cam.host)
            hub.save_config()
    return web.json_response({"ok": True})


async def api_control(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam:
        raise web.HTTPNotFound()
    try:
        async with hub.session.get(f"{cam.base}/control", params=request.query,
                                   timeout=aiohttp.ClientTimeout(total=5)) as r:
            cam.status = await r.json(content_type=None)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)
    return web.json_response(cam.status)


async def api_reboot(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam:
        raise web.HTTPNotFound()
    try:
        async with hub.session.get(f"{cam.base}/reboot", timeout=aiohttp.ClientTimeout(total=5)) as r:
            await r.read()
    except Exception:
        pass  # camera drops the connection as it restarts
    return web.json_response({"ok": True})


async def api_status(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam:
        raise web.HTTPNotFound()
    try:
        await hub.refresh_status(cam)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)
    return web.json_response(cam.status)


async def snapshot(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam or not cam.frame:
        raise web.HTTPNotFound()
    headers = {"Cache-Control": "no-store"}
    if "download" in request.query:
        name = (cam.status.get("label") or cam.id).replace(" ", "_")
        headers["Content-Disposition"] = f'attachment; filename="{name}_{time.strftime("%Y%m%d-%H%M%S")}.jpg"'
    return web.Response(body=cam.frame, content_type="image/jpeg", headers=headers)


async def stream(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam:
        raise web.HTTPNotFound()
    resp = web.StreamResponse(headers={
        "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        "Cache-Control": "no-store",
    })
    await resp.prepare(request)
    cam.viewers += 1
    try:
        last = 0.0
        while True:
            async with cam.new_frame:
                await asyncio.wait_for(cam.new_frame.wait_for(lambda: cam.frame_time > last), 30)
            last, frame = cam.frame_time, cam.frame
            await resp.write(
                f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode()
                + frame + b"\r\n"
            )
    except (asyncio.TimeoutError, ConnectionResetError, asyncio.CancelledError):
        pass
    finally:
        cam.viewers -= 1
    return resp


# ---------- recordings (clips on the camera's SD card) ----------

def rec_camera(request) -> Camera:
    cam = hub.cameras.get(request.match_info["id"])
    if not cam or not cam.files_port:
        raise web.HTTPNotFound(text="camera has no recordings support")
    return cam


def clip_path(request) -> str:
    path = request.query.get("path", "")
    if not path.startswith("/rec/") or ".." in path:
        raise web.HTTPBadRequest(text="bad path")
    return path


async def api_recordings(request):
    cam = rec_camera(request)
    try:
        async with hub.session.get(f"{cam.files_base}/recordings",
                                   timeout=aiohttp.ClientTimeout(total=20)) as r:
            return web.json_response(await r.json(content_type=None))
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)


async def api_delete_recording(request):
    cam = rec_camera(request)
    async with hub.session.get(f"{cam.files_base}/delete", params={"path": clip_path(request)},
                               timeout=aiohttp.ClientTimeout(total=10)) as r:
        return web.json_response({"ok": r.status == 200}, status=200 if r.status == 200 else 502)


def clip_filename(cam: Camera, path: str) -> str:
    # /rec/20260919/154500.avi -> Front_door_2026-09-19_15-45-00.avi
    parts = path.strip("/").split("/")
    name = (cam.status.get("label") or cam.id).replace(" ", "_")
    day, stem = parts[1], parts[2].rsplit(".", 1)[0]
    tag = "_motion" if stem.endswith("_M") else ""
    stem = stem.removesuffix("_M")
    if len(day) == 8 and len(stem) == 6 and day != "00000000":
        return f"{name}_{day[:4]}-{day[4:6]}-{day[6:]}_{stem[:2]}-{stem[2:4]}-{stem[4:]}{tag}.avi"
    return f"{name}_{stem}{tag}.avi"


async def download_recording(request):
    cam = rec_camera(request)
    path = clip_path(request)
    timeout = aiohttp.ClientTimeout(total=None, connect=5, sock_read=30)
    async with hub.session.get(f"{cam.files_base}/file", params={"path": path}, timeout=timeout) as r:
        if r.status != 200:
            raise web.HTTPBadGateway(text=await r.text())
        resp = web.StreamResponse(headers={
            "Content-Type": "video/x-msvideo",
            "Content-Disposition": f'attachment; filename="{clip_filename(cam, path)}"',
        })
        await resp.prepare(request)
        async for chunk in r.content.iter_chunked(65536):
            await resp.write(chunk)
        return resp


async def play_recording(request):
    """Replay a clip as MJPEG at its recorded speed, so browsers can show AVI files."""
    cam = rec_camera(request)
    path = clip_path(request)
    timeout = aiohttp.ClientTimeout(total=None, connect=5, sock_read=30)
    speed = max(0.25, min(8.0, float(request.query.get("speed", "1"))))
    resp = web.StreamResponse(headers={
        "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        "Cache-Control": "no-store",
    })
    async with hub.session.get(f"{cam.files_base}/file", params={"path": path}, timeout=timeout) as r:
        if r.status != 200:
            raise web.HTTPBadGateway(text=await r.text())
        await resp.prepare(request)
        buf = b""
        interval = 0.1
        header_done = False
        next_t = time.monotonic()
        try:
            async for chunk in r.content.iter_chunked(65536):
                buf += chunk
                if not header_done:
                    if len(buf) < 224:
                        continue
                    us = int.from_bytes(buf[32:36], "little")  # avih dwMicroSecPerFrame
                    interval = (us / 1e6 if us else 0.1) / speed
                    buf = buf[224:]
                    header_done = True
                while len(buf) >= 8:
                    fourcc, size = buf[:4], int.from_bytes(buf[4:8], "little")
                    if fourcc != b"00dc":
                        return resp  # reached idx1 (or an incomplete clip): done
                    if len(buf) < 8 + size:
                        break
                    frame, buf = buf[8:8 + size], buf[8 + size:]
                    next_t += interval
                    delay = next_t - time.monotonic()
                    if delay > 0:
                        await asyncio.sleep(delay)
                    else:
                        next_t = time.monotonic()  # download fell behind; don't try to catch up
                    await resp.write(
                        f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode()
                        + frame + b"\r\n")
        except (ConnectionResetError, asyncio.CancelledError):
            pass
    return resp


# ---------- notifications ----------

async def notify(title: str, body: str, image: Path | None):
    """Desktop notification on this PC (KDE/GNOME via notify-send)."""
    args = ["notify-send", "-a", "Camera Hub", "-u", "normal"]
    if image:
        args += ["-i", str(image)]
    try:
        p = await asyncio.create_subprocess_exec(*args, title, body)
        await asyncio.wait_for(p.wait(), 10)
    except Exception as e:
        log.warning("notification failed: %s", e)


async def api_settings(request):
    if request.method == "POST":
        data = await request.json()
        if "notify" in data:
            hub.settings["notify"] = bool(data["notify"])
        hub.save_config()
        if data.get("test"):
            await notify("Camera Hub", "Notifications are working", None)
    return web.json_response(hub.settings)


# ---------- motion events ----------

async def api_events(request):
    cam_id = request.query.get("cam")
    limit = int(request.query.get("limit", 200))
    ev = [e for e in hub.events if not cam_id or e["cam"] == cam_id]
    out = []
    for e in reversed(ev[-limit:]):
        c = hub.cameras.get(e["cam"])
        out.append({**e, "name": (c.status.get("label") if c else None) or e.get("name") or e["cam"],
                    "open": bool(c and c.event is e)})
    return web.json_response(out)


async def api_clear_events(request):
    for e in hub.events:
        if e.get("snap"):
            (EVENTS_DIR / e["snap"]).unlink(missing_ok=True)
    hub.events = []
    for c in hub.cameras.values():
        c.event = None
    hub.save_events()
    return web.json_response({"ok": True})


async def event_snapshot(request):
    name = request.match_info["name"]
    path = EVENTS_DIR / name
    if "/" in name or not path.is_file():
        raise web.HTTPNotFound()
    return web.FileResponse(path)


async def api_motion(request):
    cam = hub.cameras.get(request.match_info["id"])
    if not cam:
        raise web.HTTPNotFound()
    try:
        async with hub.session.get(f"{cam.base}/motion", timeout=aiohttp.ClientTimeout(total=3)) as r:
            return web.json_response(await r.json(content_type=None))
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)


# ---------- time-lapse ----------

def tl_path(request, data: dict | None = None) -> str:
    path = (data or request.query).get("path", "")
    if not re.fullmatch(r"/tl/\d{8}(/\d{2}(/[\w.]+\.jpg)?)?", path):
        raise web.HTTPBadRequest(text="bad path")
    return path


async def cam_json(cam: Camera, path: str, params=None, timeout=30):
    async with hub.session.get(f"{cam.files_base}{path}", params=params,
                               timeout=aiohttp.ClientTimeout(total=timeout)) as r:
        return await r.json(content_type=None)


async def api_timelapse(request):
    cam = rec_camera(request)
    try:
        return web.json_response(await cam_json(cam, "/timelapse", timeout=60))
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)


async def api_tlframes(request):
    cam = rec_camera(request)
    try:
        return web.json_response(await cam_json(cam, "/tlframes", {"path": tl_path(request)}))
    except web.HTTPException:
        raise
    except Exception as e:
        return web.json_response({"error": str(e)}, status=502)


async def tl_frame(request):
    cam = rec_camera(request)
    async with hub.session.get(f"{cam.files_base}/tlframe", params={"path": tl_path(request)},
                               timeout=aiohttp.ClientTimeout(total=15)) as r:
        if r.status != 200:
            raise web.HTTPNotFound()
        return web.Response(body=await r.read(), content_type="image/jpeg",
                            headers={"Cache-Control": "max-age=86400"})


async def api_delete_timelapse(request):
    cam = rec_camera(request)
    async with hub.session.get(f"{cam.files_base}/delete", params={"path": tl_path(request)},
                               timeout=aiohttp.ClientTimeout(total=120)) as r:
        return web.json_response({"ok": r.status == 200}, status=200 if r.status == 200 else 502)


def jpeg_size(data: bytes) -> tuple[int, int] | None:
    """Width/height from a JPEG's SOF marker."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            return None
        marker, seg = data[i + 1], int.from_bytes(data[i + 2:i + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2):
            return int.from_bytes(data[i + 7:i + 9], "big"), int.from_bytes(data[i + 5:i + 7], "big")
        i += 2 + seg
    return None


jobs: dict[str, dict] = {}


async def make_video(job: dict, cam: Camera, path: str, fps: int):
    """Download a day or hour of time-lapse frames and encode them to MP4 with ffmpeg."""
    proc = None
    try:
        hours = [path]
        if path.count("/") == 2:  # a whole day
            days = await cam_json(cam, "/timelapse", timeout=60)
            day = next((d for d in days if d["day"] == path.split("/")[2]), None)
            hours = [f"{path}/{h['hour']}" for h in (day or {}).get("hours", [])]
        frames = []
        for h in hours:
            frames += [f"{h}/{f}" for f in await cam_json(cam, "/tlframes", {"path": h}, timeout=60)]
        job["total"] = len(frames)
        if not frames:
            raise RuntimeError("no frames")
        VIDEOS_DIR.mkdir(exist_ok=True)
        name = (cam.status.get("label") or cam.id).replace(" ", "_")
        out = VIDEOS_DIR / f"{name}_{path.strip('/').replace('/', '-')[3:]}_{fps}fps.mp4"
        tmp = out.with_suffix(".part.mp4")
        size = None
        for f in frames:
            async with hub.session.get(f"{cam.files_base}/tlframe", params={"path": f},
                                       timeout=aiohttp.ClientTimeout(total=20)) as r:
                data = await r.read() if r.status == 200 else None
            if data and proc is None:
                size = jpeg_size(data) or (640, 480)
                w, h = size[0] // 2 * 2, size[1] // 2 * 2
                proc = await asyncio.create_subprocess_exec(
                    "ffmpeg", "-y", "-loglevel", "error", "-f", "image2pipe", "-c:v", "mjpeg",
                    "-framerate", str(fps), "-i", "-",
                    # frames after a resolution change are fitted to the first frame's size
                    "-vf", f"scale={w}:{h}:force_original_aspect_ratio=decrease,"
                           f"pad={w}:{h}:(ow-iw)/2:(oh-ih)/2,setsar=1",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(tmp),
                    stdin=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            if data:
                proc.stdin.write(data)
                await proc.stdin.drain()
            job["done"] += 1
        proc.stdin.close()
        err = (await proc.stderr.read()).decode()[-500:]
        if await proc.wait() != 0:
            raise RuntimeError(f"ffmpeg failed: {err}")
        tmp.replace(out)
        job.update(state="done", video=out.name)
    except Exception as e:
        log.warning("time-lapse video failed: %s", e)
        job.update(state="error", error=str(e))
        if proc and proc.returncode is None:
            proc.kill()


async def api_make_video(request):
    cam = rec_camera(request)
    data = await request.json()
    path = tl_path(request, data)
    fps = max(1, min(60, int(data.get("fps", 30))))
    job = {"id": uuid.uuid4().hex[:8], "cam": cam.id, "path": path, "fps": fps,
           "state": "running", "done": 0, "total": 0, "started": time.time()}
    jobs[job["id"]] = job
    asyncio.create_task(make_video(job, cam, path, fps))
    return web.json_response(job)


async def api_jobs(request):
    return web.json_response(sorted(jobs.values(), key=lambda j: -j["started"]))


async def api_videos(request):
    VIDEOS_DIR.mkdir(exist_ok=True)
    vids = [{"name": p.name, "size": p.stat().st_size, "mtime": p.stat().st_mtime}
            for p in VIDEOS_DIR.glob("*.mp4") if not p.name.endswith(".part.mp4")]
    return web.json_response(sorted(vids, key=lambda v: -v["mtime"]))


def video_file(request) -> Path:
    name = request.match_info["name"]
    path = VIDEOS_DIR / name
    if "/" in name or not name.endswith(".mp4") or not path.is_file():
        raise web.HTTPNotFound()
    return path


async def video(request):
    path = video_file(request)
    headers = {"Content-Disposition": f'attachment; filename="{path.name}"'} if "download" in request.query else {}
    return web.FileResponse(path, headers=headers)


async def api_delete_video(request):
    video_file(request).unlink()
    return web.json_response({"ok": True})


# ---------- firmware updates ----------

fw_state = {"running": False, "log": [], "started": None, "exit": None}


def source_version() -> str | None:
    """Version in the firmware source, to flag cameras running something older."""
    for src in (IDF_DIR / "main" / "camnode.h", ARDUINO_DIR / "camnode.ino"):
        try:
            m = re.search(r'#define FW_VERSION "([^"]+)"', src.read_text())
        except FileNotFoundError:
            continue
        if m:
            return m.group(1)
    return None


async def api_firmware(request):
    src = source_version()
    cams = [{"id": c.id, "name": c.status.get("label") or c.id, "host": c.host,
             "version": c.status.get("version"), "online": c.online,
             "outdated": bool(src and c.status.get("version") and c.status.get("version") != src)}
            for c in hub.cameras.values()]
    return web.json_response({"source": src, "cameras": cams, **fw_state, "log": fw_state["log"][-60:]})


async def run_update(hosts: list[str]):
    fw_state.update(running=True, log=[], started=time.time(), exit=None)
    env = {**os.environ, "PATH": f"{Path.home() / '.local/bin'}:{os.environ.get('PATH', '')}"}
    try:
        proc = await asyncio.create_subprocess_exec(
            str(FIRMWARE_DIR / "update-all.sh"), *hosts, env=env,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        async for line in proc.stdout:
            fw_state["log"].append(line.decode(errors="replace").rstrip())
        fw_state["exit"] = await proc.wait()
    except Exception as e:
        fw_state["log"].append(f"error: {e}")
        fw_state["exit"] = -1
    fw_state["running"] = False
    for c in hub.cameras.values():  # pick up new versions promptly
        asyncio.create_task(hub.refresh_status(c))


async def api_firmware_update(request):
    if fw_state["running"]:
        raise web.HTTPConflict(text="update already running")
    data = await request.json() if request.can_read_body else {}
    src = source_version()
    cams = [c for c in hub.cameras.values() if c.online]
    if not data.get("all"):
        cams = [c for c in cams if c.status.get("version") != src]
    if not cams:
        return web.json_response({"started": False, "reason": "all cameras up to date"})
    asyncio.create_task(run_update([c.host for c in cams]))
    return web.json_response({"started": True, "hosts": [c.host for c in cams]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    app = web.Application()
    app.cleanup_ctx.append(client_session)
    app.cleanup_ctx.append(discover)
    app.router.add_get("/", index)
    app.router.add_get("/api/cameras", api_cameras)
    app.router.add_post("/api/cameras", api_add)
    app.router.add_post("/api/cameras/{id}/view", api_view)
    app.router.add_post("/api/cameras/{id}/reboot", api_reboot)
    app.router.add_delete("/api/cameras/{id}", api_remove)
    app.router.add_get("/api/cameras/{id}/control", api_control)
    app.router.add_get("/api/cameras/{id}/status", api_status)
    app.router.add_get("/cam/{id}/stream", stream)
    app.router.add_get("/cam/{id}/snapshot.jpg", snapshot)
    app.router.add_get("/api/cameras/{id}/recordings", api_recordings)
    app.router.add_delete("/api/cameras/{id}/recordings", api_delete_recording)
    app.router.add_get("/cam/{id}/recording.avi", download_recording)
    app.router.add_get("/cam/{id}/play", play_recording)
    app.router.add_get("/api/cameras/{id}/motion", api_motion)
    app.router.add_get("/api/events", api_events)
    app.router.add_delete("/api/events", api_clear_events)
    app.router.add_get("/events/{name}", event_snapshot)
    app.router.add_get("/api/cameras/{id}/timelapse", api_timelapse)
    app.router.add_delete("/api/cameras/{id}/timelapse", api_delete_timelapse)
    app.router.add_get("/api/cameras/{id}/tlframes", api_tlframes)
    app.router.add_get("/cam/{id}/tlframe.jpg", tl_frame)
    app.router.add_post("/api/cameras/{id}/timelapse/video", api_make_video)
    app.router.add_get("/api/jobs", api_jobs)
    app.router.add_get("/api/videos", api_videos)
    app.router.add_get("/videos/{name}", video)
    app.router.add_delete("/api/videos/{name}", api_delete_video)
    app.router.add_get("/api/firmware", api_firmware)
    app.router.add_post("/api/firmware/update", api_firmware_update)
    app.router.add_get("/api/settings", api_settings)
    app.router.add_post("/api/settings", api_settings)
    web.run_app(app, host=args.host, port=args.port, print=lambda *_: None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
