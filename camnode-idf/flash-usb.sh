#!/bin/bash
# First install of the ESP-IDF firmware on a board, over USB.
#
#   ./flash-usb.sh              flash one board, then exit
#   ./flash-usb.sh --all        keep going: flash a board, then wait for the next one
#   ./flash-usb.sh /dev/ttyUSB0 name the port instead of scanning
#
# Writes the bootloader, partition table, OTA data and app -- a complete image, not an
# OTA into someone else's layout. After this a board takes updates over the air with
# ./update-all.sh and never needs the cable again.
#
# Getting a board into download mode: take the SD card OUT (GPIO2 is SD DATA0 and has
# to be low, or the ROM reports boot:0x0b and refuses), seat it firmly in the adapter,
# then unplug, hold IO0, replug, release. Many adapters manage it on their own and need
# no button at all -- this script tries that first.
set -uo pipefail
cd "$(dirname "$0")"

LOOP=0
PORT=""
for a in "$@"; do
  case "$a" in
    --all) LOOP=1 ;;
    /dev/*) PORT="$a" ;;
    *) echo "unknown argument: $a" >&2; exit 2 ;;
  esac
done

. ~/esp-idf/export.sh >/dev/null 2>&1 || { echo "could not load ESP-IDF" >&2; exit 1; }
ET="$(dirname "$(command -v idf.py)")/esptool.py"
[ -x "$ET" ] || ET="esptool.py"
idf.py build | tail -1 || exit 1

# 115200, not 460800: some boards fail the stub upload at the higher rate
# ("Only got 1 byte status response"). 70s for the app is a fair trade for it working.
flash_one() {
  local p="$1"
  "$ET" --chip esp32 --port "$p" --baud 115200 --connect-attempts 3 \
    write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m \
    0x1000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    0xe000 build/ota_data_initial.bin \
    0x20000 build/camnode.bin 2>&1 | tr '\r' '\n' | grep -Ei 'MAC:|Wrote|verified|fatal|error'
  return "${PIPESTATUS[0]}"
}

ports() { [ -n "$PORT" ] && echo "$PORT" || ls /dev/ttyUSB* 2>/dev/null; }

done_any=0
while :; do
  echo "Waiting for a board (Ctrl-C to stop)..."
  target=""
  while [ -z "$target" ]; do
    for p in $(ports); do
      # in download mode already?
      if "$ET" --chip esp32 --port "$p" --before no_reset --connect-attempts 1 \
           chip_id >/dev/null 2>&1; then target="$p"; break; fi
      # or the adapter can reset it into download mode by itself
      if "$ET" --chip esp32 --port "$p" --connect-attempts 1 chip_id >/dev/null 2>&1; then
        target="$p"; break
      fi
    done
    [ -z "$target" ] && sleep 1
  done

  echo "=== flashing on $target ==="
  if flash_one "$target"; then
    done_any=$((done_any+1))
    echo "=== flashed OK. Unplug and replug to boot it. ==="
  else
    echo "=== FAILED on $target. Reseat the board firmly and try again --" >&2
    echo "    a loose contact drops the adapter off the USB bus mid-write. ===" >&2
  fi

  [ "$LOOP" = 1 ] || break
  echo
  echo "Swap in the next board (waiting for this one to be unplugged)..."
  while [ -n "$(ports)" ]; do sleep 1; done
done

echo "$done_any board(s) flashed."
