// Time-lapse: one JPEG every tl_sec seconds into /tl/YYYYMMDD/HH/MMSS.jpg.
#include "camnode.h"

#include <sys/stat.h>

volatile uint32_t tlFrames = 0;
volatile time_t tlLast = 0;

void timelapseTask(void *) {
  uint32_t lastShot = 0, sinceRoomCheck = 0;
  bool first = true;
  for (;;) {
    delayMs(100);
    if (!values[S_TL] || !sdMounted || pauseRec || !cameraOk) { first = true; continue; }
    if (!first && millis() - lastShot < (uint32_t)values[S_TL_SEC] * 1000) continue;
    size_t len = 0;
    uint8_t *buf = dupFrame(&len);
    if (!buf) continue;
    lastShot = millis();
    first = false;
    if (++sinceRoomCheck >= 50) { sinceRoomCheck = 0; makeRoom(); }

    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    char path[64];
    if (t.tm_year + 1900 >= 2024) {
      snprintf(path, sizeof(path), TL_DIR "/%04d%02d%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
      mkdir(TL_DIR, 0777);
      mkdir(path, 0777);
      snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%02d", t.tm_hour);
      mkdir(path, 0777);
      snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%02d%02d.jpg", t.tm_min, t.tm_sec);
    } else {  // clock not set yet
      mkdir(TL_DIR, 0777);
      mkdir(TL_DIR "/00000000", 0777);
      mkdir(TL_DIR "/00000000/00", 0777);
      snprintf(path, sizeof(path), TL_DIR "/00000000/00/%09lu.jpg", (unsigned long)millis());
    }
    sdTake();
    FILE *f = fopen(path, "wb");
    bool ok = f && fwrite(buf, 1, len, f) == len;
    if (f) ok = fclose(f) == 0 && ok;
    sdGive();
    free(buf);
    if (ok) {
      tlFrames++;
      tlLast = now;
    } else {
      printf("Time-lapse write failed: %s\n", path);
    }
  }
}
