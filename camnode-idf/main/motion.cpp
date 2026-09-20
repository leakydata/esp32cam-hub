// Motion detection. Frames are decoded at 1/8 scale (80x60 for VGA), averaged into a
// fixed 32x24 grid and compared with a slowly-updated background. Only cells enabled in
// the zone mask count. Row-median compensation keeps PWM lamp flicker from registering.
#include "camnode.h"

#include <algorithm>
#include <math.h>

#include "jpeg_decoder.h"

#define MD_INTERVAL_MS 200

uint8_t mdChanged[MD_CELLS / 8];   // cells changed in the latest check
volatile bool motionActive = false;
volatile uint8_t motionLevel = 0;  // % of watched cells changed
volatile uint32_t motionEvents = 0, motionSeq = 0;
volatile time_t motionLastStart = 0;

static inline bool cellGet(const uint8_t *b, int i) { return b[i >> 3] & (1 << (i & 7)); }
static inline void cellSet(uint8_t *b, int i, bool v) {
  if (v) b[i >> 3] |= 1 << (i & 7); else b[i >> 3] &= ~(1 << (i & 7));
}

void maskToHex(const uint8_t *b, char *out) {
  for (int i = 0; i < MD_CELLS / 8; i++) sprintf(out + i * 2, "%02x", b[i]);
}

bool hexToMask(const char *hex, uint8_t *b) {
  if (strlen(hex) != MD_CELLS / 4) return false;
  for (int i = 0; i < MD_CELLS / 8; i++) {
    char h[3] = {hex[i * 2], hex[i * 2 + 1], 0};
    if (!isxdigit((uint8_t)h[0]) || !isxdigit((uint8_t)h[1])) return false;
    b[i] = strtol(h, NULL, 16);
  }
  return true;
}

void motionTask(void *) {
  uint8_t *jpg = (uint8_t *)psAlloc(FRAME_BUF_SIZE);
  uint8_t *rgb = (uint8_t *)psAlloc(200 * 150 * 3);  // 1/8 of UXGA
  // static: ~8KB of working arrays would overflow the task stack
  static float bg[MD_CELLS];
  static uint8_t cur[MD_CELLS];
  static uint32_t sum[MD_CELLS];
  static uint16_t cnt[MD_CELLS];
  int warmup = 10, hits = 0;
  uint32_t lastSeq = 0, lastHitMs = 0;
  uint16_t lastW = 0, lastH = 0;
  if (!jpg || !rgb) vTaskDelete(NULL);
  for (;;) {
    delayMs(MD_INTERVAL_MS);
    if (!values[S_MD] || !cameraOk) {
      if (motionActive) motionActive = false;
      motionLevel = 0;
      warmup = 10;
      continue;
    }
    uint16_t w, h;
    size_t len = copyFrame(jpg, FRAME_BUF_SIZE, &lastSeq, &w, &h);
    if (!len) continue;

    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata = jpg;
    cfg.indata_size = len;
    cfg.outbuf = rgb;
    cfg.outbuf_size = 200 * 150 * 3;
    cfg.out_format = JPEG_IMAGE_FORMAT_RGB888;
    cfg.out_scale = JPEG_IMAGE_SCALE_1_8;
    esp_jpeg_image_output_t img;
    if (esp_jpeg_decode(&cfg, &img) != ESP_OK || img.width < MD_W || img.height < MD_H) continue;
    if (w != lastW || h != lastH) { warmup = 10; lastW = w; lastH = h; }

    // average decoded pixels into the grid (luma ~ (R + 2G + B) / 4)
    memset(sum, 0, sizeof(sum));
    memset(cnt, 0, sizeof(cnt));
    for (int y = 0; y < img.height; y++) {
      int gy = y * MD_H / img.height;
      const uint8_t *px = rgb + y * img.width * 3;
      for (int x = 0; x < img.width; x++, px += 3) {
        int c = gy * MD_W + x * MD_W / img.width;
        sum[c] += (px[0] + 2 * px[1] + px[2]) >> 2;
        cnt[c]++;
      }
    }
    for (int c = 0; c < MD_CELLS; c++) cur[c] = cnt[c] ? sum[c] / cnt[c] : 0;

    if (warmup > 0) {
      for (int c = 0; c < MD_CELLS; c++) bg[c] = cur[c];
      warmup--;
      continue;
    }

    int sens = values[S_MD_SENS];
    // Change needed for a cell to count, relative to its brightness: flickering lights and
    // exposure changes scale brightness, so bright areas swing more than dark ones.
    // sens 5 -> 12 + 30% of luma; sens 10 -> 6 + 15%; sens 1 -> 17 + 42%.
    float tBase = 18 - 1.2f * sens, tRel = 0.45f - 0.03f * sens;
    // Flickering lights brighten/darken whole rows (the sensor reads out line by line):
    // divide out each row's median brightness ratio before comparing.
    float rowRatio[MD_H];
    for (int y = 0; y < MD_H; y++) {
      float r[MD_W];
      for (int x = 0; x < MD_W; x++) r[x] = (cur[y * MD_W + x] + 4.0f) / (bg[y * MD_W + x] + 4.0f);
      std::sort(r, r + MD_W);
      rowRatio[y] = (r[MD_W / 2 - 1] + r[MD_W / 2]) / 2;
    }
    int watched = 0, changed = 0;
    for (int y = 0; y < MD_H; y++) {
      int rowWatched = 0, rowChanged = 0;
      for (int x = 0; x < MD_W; x++) {
        int c = y * MD_W + x;
        bool on = cellGet(mdMask, c);
        float expect = (bg[c] + 4) * rowRatio[y] - 4;
        bool blown = cur[c] > 235 && bg[c] > 235;  // lamps, windows: saturated, no detail
        bool diff = on && !blown &&
                    fabsf(cur[c] - expect) > tBase + tRel * std::max((float)cur[c], bg[c]);
        cellSet(mdChanged, c, diff);
        rowWatched += on;
        rowChanged += diff;
      }
      // a band across most of the row is flicker, not something moving
      if (rowWatched && rowChanged * 100 / rowWatched > 35) {
        for (int x = 0; x < MD_W; x++) cellSet(mdChanged, y * MD_W + x, false);
        rowChanged = 0;
      }
      watched += rowWatched;
      changed += rowChanged;
    }
    motionLevel = watched ? changed * 100 / watched : 0;

    bool lightingChange = watched && changed * 100 / watched > 60;  // lights on/off
    int minCells = std::max(1, watched * (11 - sens) / 200);
    bool hit = !lightingChange && changed >= minCells;
    hits = hit ? hits + 1 : 0;

    // background follows slowly, and snaps to the new scene after a lighting change
    float alpha = lightingChange ? 1.0f : (hit ? 0.02f : 0.1f);
    for (int c = 0; c < MD_CELLS; c++) bg[c] += alpha * (cur[c] - bg[c]);

    uint32_t now = millis();
    if (hits >= 2) {  // two consecutive checks to ignore single-frame noise
      lastHitMs = now;
      if (!motionActive) {
        motionActive = true;
        motionEvents++;
        motionLastStart = time(NULL);
        printf("Motion start (%d%%)\n", motionLevel);
      }
    } else if (motionActive && now - lastHitMs > (uint32_t)values[S_MD_HOLD] * 1000) {
      motionActive = false;
      printf("Motion end\n");
    }
    motionSeq++;
  }
}
