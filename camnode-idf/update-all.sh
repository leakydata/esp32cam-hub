#!/bin/bash
# Build and push this firmware to every camera on the network, over the air.
# Usage: ./update-all.sh [camera-ip ...]   (no args = all cameras found via mDNS)
#
# Upload is a plain HTTP POST of the app image to /update. The camera writes it to its
# spare OTA slot and reboots into it; if it fails to come up the bootloader rolls back,
# so a bad push cannot strand a camera. Boards get their first install over USB with
# ./flash-usb.sh -- there is no cross-flashing from the old Arduino firmware.
set -o pipefail
cd "$(dirname "$0")"
BIN=build/camnode.bin

. ~/esp-idf/export.sh >/dev/null 2>&1
idf.py build | tail -1 || exit 1
[ -f "$BIN" ] || { echo "no firmware at $BIN"; exit 1; }

# camhub holds a live MJPEG stream to every camera, and four of those saturate a
# 2.4GHz channel: measured 75% packet loss during an upload with it running, 0% with it
# stopped. Pause it for the duration. (The hub's own Firmware button pauses its streams
# by itself; this is for running the script by hand.)
HUB_STOPPED=0
if systemctl --user is-active --quiet camhub 2>/dev/null; then
  echo "pausing camhub so the upload has the air"
  systemctl --user stop camhub && HUB_STOPPED=1
  sleep 3
fi
restore_hub() { [ "$HUB_STOPPED" = 1 ] && systemctl --user start camhub && echo "camhub restarted"; }
trap restore_hub EXIT

ips=("$@")
if [ ${#ips[@]} -eq 0 ]; then
  mapfile -t ips < <(avahi-browse -rtp _espcam._tcp 2>/dev/null |
                     awk -F';' '$1=="=" && $3=="IPv4" {print $8}' | sort -u)
fi
[ ${#ips[@]} -eq 0 ] && { echo "No cameras found"; exit 1; }

fail=0
for ip in "${ips[@]}"; do
  echo "== $ip"
  code=$(curl -s -m 300 -o /dev/null -w '%{http_code}' -H "Expect:" \
           -X POST --data-binary "@$BIN" "http://$ip/update" 2>/dev/null)
  if [ "$code" != "200" ]; then
    # 404 means it is still on the Arduino firmware and has no /update endpoint
    [ "$code" = "404" ] && echo "   no /update - flash this one over USB first (./flash-usb.sh)" \
                        || echo "   upload FAILED (http ${code:-none})"
    fail=1; continue
  fi
  echo "   uploaded, waiting for reboot..."
  ok=0
  for _ in $(seq 1 20); do
    sleep 3
    st=$(curl -s -m 3 "http://$ip/status") && [ -n "$st" ] && { ok=1; break; }
  done
  if [ $ok = 1 ]; then
    echo "$st" | python3 -c "import json,sys; d=json.load(sys.stdin); print('   now running', d['name'], 'v'+d['version'])"
  else
    echo "   not responding after update (bootloader should roll back)"; fail=1
  fi
done
exit $fail
