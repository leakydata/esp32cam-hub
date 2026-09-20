# camnode (ESP-IDF)

Port of `../camnode/camnode.ino` to a plain ESP-IDF project. The HTTP API is unchanged,
so `camhub` and its web UI work against either firmware and the fleet can migrate one
camera at a time.

## Why the port

The Arduino ESP32 core ships prebuilt, so its `sdkconfig` is fixed. Three things that
cost us came from that:

- **exFAT is compiled out**, capping SD cards at 32GB FAT32. Now enabled.
- No control over the partition table, and **no OTA rollback**. Both are set here:
  two equal OTA slots, no SPIFFS, and `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`.
- ~94KB of extra flash (1,196,798 -> 1,103,488 bytes).

## Build and flash

Each board gets one clean USB install: bootloader, partition table, OTA data and app.
No cross-flashing an image into the Arduino firmware's layout.

```sh
./flash-usb.sh              # waits for any board in download mode, then flashes it
./flash-usb.sh /dev/ttyUSB0 # or name the port
```

Download mode: **take the SD card out first** (GPIO2 is SD DATA0 and has to be low, or
the ROM reports `boot:0x0b` and never enters download mode), then unplug, hold IO0,
replug, release. Some adapters do it on their own.

After that first install, updates go over the air:

```sh
./update-all.sh                  # every camera found via mDNS
./update-all.sh 192.168.0.224    # just one
```

That posts the app image to `/update`. The camera writes it to its spare OTA slot and
reboots into it, and only marks it valid once WiFi is up and the servers are listening --
otherwise the bootloader rolls back to the slot that was working. A bad push cannot
strand a camera.

Cameras still on the Arduino firmware answer `404` to `/update`; flash those over USB.
`../camnode/update-all.sh` still drives the old espota path for whatever is left on it.

## What changed from the Arduino build

| Arduino | ESP-IDF |
|---|---|
| `WiFi` | `esp_wifi` + `esp_netif` + `esp_event` |
| `Preferences` | `nvs` (helpers in `settings.cpp`) |
| `SD_MMC` | `esp_vfs_fat_sdmmc_mount`, exFAT on |
| `ArduinoOTA` / `espota.py` | `esp_ota_ops` behind `POST /update` |
| `MDNS` | `espressif/mdns` component |
| `analogWrite` | LEDC channel for the flash LED |
| `String` | `std::string` |

Unchanged: the motion-detection algorithm, the AVI container layout (including both
corruption fixes -- the pad byte excluded from the chunk size, and the recursive SD lock
that keeps a FatFs listing from interleaving with the writer), the pre-roll ring, the
board auto-detect table and every HTTP response.

## Layout

    main/main.cpp         app_main: NVS, camera, WiFi, SNTP, servers, mDNS, tasks
    main/settings.cpp     NVS-backed settings table and sensor setters
    main/camera.cpp       board auto-detect, capture task, shared frame
    main/sdcard.cpp       mount, listings, "delete oldest until there is room"
    main/avi.cpp          MJPEG AVI writer
    main/recorder.cpp     loop/motion recording with PSRAM pre-roll ring
    main/motion.cpp       1/8-scale decode, 32x24 grid, zone mask
    main/timelapse.cpp    periodic JPEGs
    main/http_api.cpp     ports 80 and 81, plus OTA
    main/http_files.cpp   port 82
