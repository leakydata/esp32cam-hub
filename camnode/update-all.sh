#!/bin/bash
# Build camnode and push it over WiFi to every camera found on the network.
# Usage: ./update-all.sh [camera-ip ...]   (no ips = all cameras found via mDNS)
#
# This is the legacy Arduino firmware. Cameras migrated to the ESP-IDF build in
# ../camnode-idf are updated by that tree's own update-all.sh instead; they are flashed
# once over USB (camnode-idf/flash-usb.sh) rather than cross-flashed from here.
set -o pipefail
cd "$(dirname "$0")"
FQBN="esp32:esp32:esp32cam:PartitionScheme=min_spiffs"
ESPOTA=~/.arduino15/packages/esp32/hardware/esp32/3.3.11/tools/espota.py
OTA_PASS=$(sed -nE 's/.*OTA_PASS *"(.*)".*/\1/p' secrets.h)

BIN=../build-camnode/camnode.ino.bin
arduino-cli compile --fqbn "$FQBN" --output-dir ../build-camnode . | tail -1 || exit 1
[ -f "$BIN" ] || { echo "no firmware at $BIN"; exit 1; }

ips=("$@")
if [ ${#ips[@]} -eq 0 ]; then
  mapfile -t ips < <(avahi-browse -rtp _espcam._tcp 2>/dev/null | awk -F';' '$1=="=" && $3=="IPv4" {print $8}' | sort -u)
fi
[ ${#ips[@]} -eq 0 ] && { echo "No cameras found"; exit 1; }

fail=0
for ip in "${ips[@]}"; do
  echo "== $ip"
  if ! python3 "$ESPOTA" -i "$ip" -f "$BIN" ${OTA_PASS:+-a "$OTA_PASS"} >/dev/null 2>&1; then
    echo "   upload FAILED"; fail=1; continue
  fi
  echo "   uploaded, waiting for reboot..."
  ok=0
  for _ in $(seq 1 20); do
    sleep 3
    st=$(curl -s -m 3 "http://$ip/status") && [ -n "$st" ] && { ok=1; break; }
  done
  if [ $ok = 1 ]; then
    echo "$st" | python3 -c "import json,sys; d=json.load(sys.stdin); print(\"   now running\", d[\"name\"], \"v\"+d[\"version\"])"
  else
    echo "   not responding after update"; fail=1
  fi
done
exit $fail
