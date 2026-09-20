#!/bin/bash
# First install of the ESP-IDF firmware on one board, over USB.
# Usage: ./flash-usb.sh [/dev/ttyUSB0]
#
# Writes the bootloader, partition table, OTA data and app -- a complete image, not an
# OTA into someone else's layout. After this the board takes updates over the air with
# ./update-all.sh. Put the board in download mode first: SD card OUT (GPIO2 is SD DATA0
# and must be low), then unplug, hold IO0, replug, release. Some adapters manage it by
# themselves; this script waits either way.
set -euo pipefail
cd "$(dirname "$0")"
PORT="${1:-}"

. ~/esp-idf/export.sh >/dev/null 2>&1
idf.py build | tail -1

# wait for a board in download mode, on the given port or any of them
echo "Waiting for a board in download mode (Ctrl-C to stop)..."
for _ in $(seq 1 600); do
  for p in ${PORT:-/dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2}; do
    [ -e "$p" ] || continue
    if esptool.py --chip esp32 --port "$p" --before no_reset --connect-attempts 1 \
         chip_id >/dev/null 2>&1; then
      echo "Found bootloader on $p"
      idf.py -p "$p" -b 460800 flash
      echo "Flashed. Unplug and replug the board to boot it."
      exit 0
    fi
  done
  sleep 0.5
done
echo "No board entered download mode." >&2
exit 1
