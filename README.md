# esp32cam-hub

Turn a pile of ESP32-CAM boards into a set of WiFi cameras with one web page showing
all of them. No cloud, no app, no account. An SD card is optional — it only adds
recording and time-lapse.

Two pieces:

- **`camnode-idf/`** — ESP-IDF firmware for the camera boards. Each one joins your WiFi,
  announces itself over mDNS, and serves a live MJPEG stream plus a small REST API.
- **`camhub/`** — a Python server for your PC. Finds every camera automatically and
  serves one page with all the feeds, settings, recordings and motion events.

Point a browser at `http://localhost:8765` and everything is there. Adding a camera
means flashing it and powering it on; nothing to configure on the hub.

## What it does

- **Live view** of every camera on one page, plus a full-screen wall view.
- **Auto-discovery** via mDNS (`_espcam._tcp`). Cameras are never configured by IP.
- **Per-camera settings** from the browser: name, resolution, quality, brightness,
  contrast, saturation, white balance, exposure, gain, flip/mirror, flash LED, and a
  mains-flicker filter for rooms lit by cheap LED bulbs.
- **Motion detection on the camera itself** — a 32x24 grid compared against a slowly
  adapting background, with a paintable zone mask so you can ignore a road or a TV.
  Snapshots and an event log land on the hub; optional desktop notifications.
- **Loop recording to SD** as MJPEG AVI clips, continuously or only while there is
  motion. Motion clips include a few seconds of pre-roll from before the trigger.
  Oldest footage is deleted automatically as the card fills.
- **Time-lapse** at any interval, browsable and exportable to MP4 via ffmpeg.
- **Over-the-air updates** to every camera from a button in the UI.

The camera does the motion detection, so the hub is not decoding video and your CPU
stays idle. A Raspberry Pi is plenty.

## Hardware

- One or more **ESP32-CAM boards with PSRAM** (AI-Thinker and the usual clones).
  PSRAM is required — the frame buffers, pre-roll ring and AVI index all live there.
  Ten board pin-outs are built in and auto-detected, so ESP-EYE, M5Stack, TTGO and
  WROVER-KIT boards work from the same binary.
- A **USB-serial adapter** for the first flash. The ESP32-CAM-MB works; so does any
  3.3V FTDI/CP2102/CH340 wired to U0R/U0T.
- Optional **microSD card**, FAT32 or exFAT, for recording and time-lapse.
- A **decent 5V supply**. Camera plus WiFi transmit draws real current, and a weak
  USB hub port causes brownouts that look like random reboots.

## Quick start

### 1. Flash a camera

```sh
cd camnode-idf
cp main/secrets.h.example main/secrets.h     # put your 2.4GHz SSID and password in
. ~/esp-idf/export.sh                        # ESP-IDF v5.3 or newer
./flash-usb.sh
```

`flash-usb.sh` builds, then waits for a board in download mode and flashes it.

To get a board into download mode: **take the SD card out**, unplug it, hold the IO0
button, plug it back in, release after a couple of seconds. Some adapters do this on
their own and need no button at all.

> The SD card matters. GPIO2 is also SD DATA0, and the ESP32 only enters download mode
> when GPIO0 *and* GPIO2 are low. With a card in the slot the ROM prints `boot:0x0b`
> and refuses. This costs people hours.

Unplug and replug the board to run it. It will print the address it landed on:

```
camnode 2.0.0
  AI_THINKER         yes, sensor 0x0026
Connecting to your-ssid....
Connected, IP 192.168.1.42
Ready: http://espcam-4979f0.local/  http://192.168.1.42/
```

Each camera names itself `espcam-<last 3 bytes of MAC>`, so no two clash. Repeat for
every board.

### 2. Run the hub

```sh
cd camhub
uv run camhub.py --port 8765
```

Open **http://localhost:8765**. Cameras appear on their own within a few seconds.

To keep it running in the background:

```sh
systemctl --user enable --now camhub
loginctl enable-linger $USER      # so it survives logout
```

### 3. Update cameras later

```sh
cd camnode-idf
./update-all.sh                   # every camera found on the network
./update-all.sh 192.168.1.42      # or just one
```

Also available as a button in the hub's Firmware panel. Updates go into the spare OTA
slot and the camera only marks the new image valid after WiFi is up and its servers are
listening — otherwise the bootloader rolls back to the version that worked. A bad update
cannot leave a camera unreachable.

## The camera's HTTP API

Each camera runs three servers on separate ports so a long stream or a multi-megabyte
download never blocks the others.

| Port | Endpoint | Purpose |
|---|---|---|
| 80 | `/` | tiny built-in viewer page |
| 80 | `/capture` | single JPEG |
| 80 | `/status` | JSON: identity, RSSI, uptime, heap, SD usage, every setting |
| 80 | `/control?<key>=<n>` | change a setting; `reset=1` restores defaults |
| 80 | `/motion` | live motion grid and zone mask |
| 80 | `/reboot`, `/reg?addr=` | reboot; read a sensor register |
| 80 | `POST /update` | firmware upload |
| 81 | `/stream` | MJPEG stream |
| 82 | `/recordings`, `/file?path=`, `/delete?path=` | clips |
| 82 | `/timelapse`, `/tlframes?path=`, `/tlframe?path=` | time-lapse |

Settings are plain query parameters, so anything can drive a camera directly:

```sh
curl "http://espcam-4979f0.local/control?framesize=12&quality=10&vflip=1"
curl "http://espcam-4979f0.local/control?rec=2&md_sens=7&pre_sec=3"   # motion-only recording
curl  http://espcam-4979f0.local/status | jq
```

Full list of keys is in `camnode-idf/main/settings.cpp`.

## Recording

`rec=0` off, `rec=1` continuous, `rec=2` only while motion is active.

Clips are MJPEG AVI under `/rec/YYYYMMDD/HHMMSS.avi`, cut every `rec_min` minutes, at
`rec_fps` frames per second (lower than the live stream, to save card space). Clips that
contain motion get an `_M` suffix so the hub can filter them. Time-lapse frames are JPEGs
under `/tl/YYYYMMDD/HH/MMSS.jpg`.

When the card gets within 5% of full, the oldest clip or time-lapse hour is deleted to
make room, so it runs indefinitely without attention.

Picture settings apply to recordings as well as the live view — the recorder saves the
same frames the stream sends.

## Repository layout

    camnode-idf/     ESP-IDF firmware (current)
    camhub/          Python hub: aiohttp server, mDNS discovery, web UI
    camnode/         the original Arduino sketch (legacy, see below)

### About `camnode/`

The firmware started as an Arduino sketch and was ported to ESP-IDF. The sketch is kept
because it still runs on cameras that have not been reflashed yet, and `camnode/update-all.sh`
still pushes to those over ArduinoOTA. New installs should use `camnode-idf/`.

The port was not cosmetic. The Arduino ESP32 core ships prebuilt with a frozen
`sdkconfig`, and three things that cost us came directly from that:

- **exFAT was compiled out**, capping SD cards at 32GB FAT32.
- **No OTA rollback**, so a bad update or a bad setting could strand a camera until
  someone physically retrieved it and reflashed over USB.
- No control over the partition table, flash timing or lwIP buffers.

All three are fixed in `camnode-idf/`, and the binary is about 94KB smaller. Details in
`camnode-idf/README.md`.

## Troubleshooting

**Board will not enter download mode, ROM prints `boot:0x0b`.** GPIO2 is high. Take the
SD card out. `boot:0x03` is what you want.

**Camera joins WiFi but is slow or drops out**, while `/status` shows a strong RSSI.
RSSI only measures what the camera *hears*. Check the 5V supply first: transmit is where
the current goes, and an underpowered board receives fine and transmits badly. A rear
motherboard USB port or a proper charger, not a keyboard hub.

**2.4GHz only.** The ESP32 has no 5GHz radio. If your SSID is shared across both bands
this usually still works, but a 5GHz-only SSID will never connect.

**Cameras do not appear in the hub.** They are found by mDNS, so the PC and the cameras
must be on the same network segment. If your PC is on ethernet and the cameras are on
WiFi behind a router that does not forward multicast, they will not be discovered — put
the PC on the same WiFi, or add the camera by IP in the hub.

**Bands or flicker in the picture** under LED or fluorescent light is the mains flicker,
not a camera fault. Set `flicker=1` for 50Hz mains or `flicker=2` for 60Hz.

**Recording stops, or clips will not play.** Check `sd` in `/status`. Cheap or worn cards
fail writes; the firmware unmounts and retries every 30 seconds. FAT32 and exFAT are
both supported.

## License

MIT
