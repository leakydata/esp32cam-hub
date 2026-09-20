// SD card mount, directory listings and the "delete oldest until there is room" sweep.
//
// Every SD access goes through sdLock. listFiles() talks to FatFs directly, below the
// VFS layer that serializes fopen/fwrite, so a /recordings listing could interleave with
// the recorder mid-write and misplace an 8KB block inside the clip. Recursive: openSegment
// calls makeRoom, which locks too.
#include "camnode.h"

#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "sdmmc_cmd.h"

volatile bool sdMounted = false;
volatile uint32_t sdTotalMB = 0, sdUsedMB = 0;
SemaphoreHandle_t sdLock = NULL;
static sdmmc_card_t *card = NULL;

void refreshSdUsage() {
  if (!sdMounted) return;
  FATFS *fs;
  DWORD freeClusters;
  if (f_getfree("0:", &freeClusters, &fs) != FR_OK) return;
  uint64_t sector = CONFIG_WL_SECTOR_SIZE;
  uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * sector;
  uint64_t freeB = (uint64_t)freeClusters * fs->csize * sector;
  sdTotalMB = total / 1048576;
  sdUsedMB = (total - freeB) / 1048576;
}

bool mountSd() {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  // 20MHz. The frame corruption once blamed on 40MHz was really two software bugs (the AVI
  // pad byte counted in the chunk size, and unlocked FatFs listings racing the writer);
  // SDMMC_FREQ_HIGHSPEED is worth retrying now that both are fixed.
  host.max_freq_khz = SDMMC_FREQ_DEFAULT;
  host.flags = SDMMC_HOST_FLAG_1BIT;  // 1-bit leaves GPIO4 (flash LED) and 12/13 free
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  esp_vfs_fat_sdmmc_mount_config_t cfg = {};
  cfg.format_if_mount_failed = false;
  cfg.max_files = 5;
  cfg.allocation_unit_size = 16 * 1024;
  // exFAT is compiled in here (CONFIG_FATFS_FS_EXFAT), so cards over 32GB work
  // without reformatting -- the main reason this firmware left the Arduino core.
  if (esp_vfs_fat_sdmmc_mount(SD_MOUNT, &host, &slot, &cfg, &card) != ESP_OK) return false;
  sdMounted = true;
  refreshSdUsage();
  printf("SD card mounted: %lu MB, %lu MB used\n", (unsigned long)sdTotalMB, (unsigned long)sdUsedMB);
  return true;
}

void unmountSd() {
  if (card) esp_vfs_fat_sdcard_unmount(SD_MOUNT, card);
  card = NULL;
  sdMounted = false;
}

// sorted entries of a directory (names only)
std::vector<std::string> listDir(const char *path, bool dirs) {
  std::vector<std::string> out;
  DIR *d = opendir(path);
  if (!d) return out;
  struct dirent *e;
  while ((e = readdir(d)) != NULL) {
    if (e->d_name[0] == '.') continue;
    bool isDir = e->d_type == DT_DIR;
    if (isDir == dirs) out.push_back(std::string(e->d_name));
  }
  closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

// Files in a folder with their sizes. Goes through FatFs directly: stat() on each file
// rescans the whole directory, which took ~1 min for 500 clips.
std::vector<FileEnt> listFiles(const char *path) {
  std::vector<FileEnt> out;
  FF_DIR d;
  FILINFO fi;
  std::string ffPath = std::string("0:") + (path + strlen(SD_MOUNT));  // FatFs drive 0
  sdTake();
  if (f_opendir(&d, ffPath.c_str()) == FR_OK) {
    while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
      if (!(fi.fattrib & AM_DIR) && fi.fname[0] != '.')
        out.push_back({std::string(fi.fname), (uint32_t)fi.fsize});
    }
    f_closedir(&d);
    sdGive();
  } else {  // fallback: slow but always works
    sdGive();
    for (const std::string &f : listDir(path, false)) {
      struct stat st;
      std::string full = std::string(path) + "/" + f;
      out.push_back({f, stat(full.c_str(), &st) == 0 ? (uint32_t)st.st_size : 0});
    }
  }
  std::sort(out.begin(), out.end(),
            [](const FileEnt &a, const FileEnt &b) { return a.name < b.name; });
  return out;
}

void removeTree(const std::string &path) {  // time-lapse hour: files, then the folder
  for (const std::string &f : listDir(path.c_str(), false)) unlink((path + "/" + f).c_str());
  rmdir(path.c_str());
}

// delete the oldest footage (a clip, or a whole time-lapse hour) until there is room
void makeRoom() {
  sdTake();
  refreshSdUsage();
  uint32_t keepFreeMB = std::max<uint32_t>(sdTotalMB / 20, 256);  // 5%, at least 256MB
  int guard = 200;
  while (sdTotalMB && sdTotalMB - sdUsedMB < keepFreeMB && guard-- > 0) {
    // oldest clip, keyed "YYYYMMDDHH"
    std::string recKey, recDay, recFile;
    std::vector<std::string> rdays = listDir(REC_DIR, true);
    if (!rdays.empty()) {
      recDay = std::string(REC_DIR) + "/" + rdays[0];
      std::vector<std::string> files = listDir(recDay.c_str(), false);
      if (files.empty()) { rmdir(recDay.c_str()); continue; }
      recFile = recDay + "/" + files[0];
      recKey = rdays[0] + files[0].substr(0, 2);
    }
    // oldest time-lapse hour
    std::string tlKey, tlDay, tlHour;
    std::vector<std::string> tdays = listDir(TL_DIR, true);
    if (!tdays.empty()) {
      tlDay = std::string(TL_DIR) + "/" + tdays[0];
      std::vector<std::string> hours = listDir(tlDay.c_str(), true);
      if (hours.empty()) { rmdir(tlDay.c_str()); continue; }
      tlHour = tlDay + "/" + hours[0];
      tlKey = tdays[0] + hours[0];
    }
    if (recKey.empty() && tlKey.empty()) break;
    if (!tlKey.empty() && (recKey.empty() || tlKey <= recKey)) {
      printf("Card full, deleting time-lapse %s\n", tlHour.c_str());
      removeTree(tlHour);
      rmdir(tlDay.c_str());  // only succeeds once empty
    } else {
      printf("Card full, deleting %s\n", recFile.c_str());
      unlink(recFile.c_str());
      rmdir(recDay.c_str());
    }
    refreshSdUsage();
  }
  sdGive();
}
