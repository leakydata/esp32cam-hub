// camnode: ESP32-CAM (AI-Thinker) network camera.
// Joins WiFi from secrets.h, advertises itself via mDNS (_espcam._tcp) so the
// hub on the PC finds it automatically, serves MJPEG + JPEG, accepts OTA updates,
// detects motion on the camera, and, when a microSD card is present, loop-records AVI
// clips continuously or only on motion, and/or saves time-lapse JPEGs (oldest deleted first).
//
//   port 80  /            small info page
//            /capture     single JPEG
//            /status      JSON status
//            /control?<setting>=<n>&label=<text>   see SETTINGS[]; reset=1 restores defaults
//            /reboot
//            /motion      live motion grid (for the hub's zone editor and events)
//   port 81  /stream      MJPEG live stream
//   port 82  /recordings  JSON list of clips, newest first
//            /file?path=  download a clip
//            /delete?path=  (a clip, or a time-lapse day/hour folder)
//            /timelapse   JSON list of time-lapse days and hours
//            /tlframes?path=/tl/<day>/<hour>   JSON list of frame names
//            /tlframe?path=  one time-lapse JPEG
// Each port is its own server task, so a long stream or download never blocks the others.

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <SD_MMC.h>
#include <dirent.h>
#include <sys/stat.h>
#include <vector>
#include <algorithm>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "jpeg_decoder.h"
#include "ff.h"
#include "esp_wifi.h"
#include "secrets.h"

#define FW_VERSION "1.7.5"

#ifndef TZ_INFO
#define TZ_INFO "EST5EDT,M3.2.0,M11.1.0"  // POSIX TZ used for clip names
#endif

// Known ESP32 camera boards. On first boot each is tried until the camera answers,
// and the working one is remembered, so the same firmware fits any of these boards.
struct PinMap {
  const char *name;
  // pwdn reset xclk sda scl d7 d6 d5 d4 d3 d2 d1 d0 vsync href pclk
  int8_t pin[16];
  int8_t led;  // flash LED, -1 if none
};
static const PinMap BOARDS[] = {
  {"AI_THINKER", {32, -1, 0, 26, 27, 35, 34, 39, 36, 21, 19, 18, 5, 25, 23, 22}, 4},
  {"WROVER_KIT", {-1, -1, 21, 26, 27, 35, 34, 39, 36, 19, 18, 5, 4, 25, 23, 22}, -1},
  {"ESP_EYE", {-1, -1, 4, 18, 23, 36, 37, 38, 39, 35, 14, 13, 34, 5, 27, 25}, 22},
  {"M5STACK_PSRAM", {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 32, 22, 26, 21}, -1},
  {"M5STACK_V2_PSRAM", {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}, -1},
  {"M5STACK_WIDE", {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}, 2},
  {"M5STACK_ESP32CAM", {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}, -1},
  {"M5STACK_UNITCAM", {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 32, 22, 26, 21}, -1},
  {"TTGO_T_JOURNAL", {0, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}, -1},
  {"ESP32_CAM_BOARD", {32, 33, 4, 18, 23, 36, 19, 21, 39, 13, 14, 35, 34, 5, 27, 25}, -1},
};
static const int NUM_BOARDS = sizeof(BOARDS) / sizeof(BOARDS[0]);
static int boardIdx = -1;
static int ledPin = -1;
static bool cameraOk = false;
static uint16_t sensorPid = 0;

#define SD_MOUNT "/sdcard"
#define REC_DIR SD_MOUNT "/rec"
#define TL_DIR SD_MOUNT "/tl"
#define FRAME_BUF_SIZE (256 * 1024)  // largest JPEG we pass around (UXGA at quality 10 is ~150KB)

static char hostName[24];
static httpd_handle_t httpd = NULL, streamd = NULL, filed = NULL;
static Preferences prefs;
#define PREFS_NS "camnode2"  // v1.0 used "camnode" with different value types

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---- persisted camera settings ----
// Each entry maps a /control key to its setter and allowed range.
struct Setting {
  const char *key;
  int (*apply)(sensor_t *, int);
  int lo, hi, def;
};
static int setFramesize(sensor_t *s, int v) { return s->set_framesize(s, (framesize_t)v); }
static int setQuality(sensor_t *s, int v) { return s->set_quality(s, v); }
static int setBrightness(sensor_t *s, int v) { return s->set_brightness(s, v); }
static int setContrast(sensor_t *s, int v) { return s->set_contrast(s, v); }
static int setSaturation(sensor_t *s, int v) { return s->set_saturation(s, v); }
static int setEffect(sensor_t *s, int v) { return s->set_special_effect(s, v); }
static int setWbMode(sensor_t *s, int v) { s->set_awb_gain(s, 1); return s->set_wb_mode(s, v); }
static int setAeLevel(sensor_t *s, int v) { return s->set_ae_level(s, v); }
static int setGainCeiling(sensor_t *s, int v) { return s->set_gainceiling(s, (gainceiling_t)v); }
static int setVflip(sensor_t *s, int v) { return s->set_vflip(s, v); }
static int setHmirror(sensor_t *s, int v) { return s->set_hmirror(s, v); }
static int setLed(sensor_t *, int v) { if (ledPin >= 0) analogWrite(ledPin, v); return 0; }
static int noop(sensor_t *, int) { return 0; }  // read directly by the recorder
// Radio mode. "Robust" drops 802.11n, leaving b/g: lower rates with sturdier modulation
// and no HT aggregation, which helps a board with a poor antenna. It deliberately keeps
// 11g -- an 802.11b-only client is refused outright ("Refused basic rates mismatch") by
// any AP with legacy rates disabled, and this setting persists in NVS.
static int setRadio(sensor_t *, int v) {
  esp_wifi_set_protocol(WIFI_IF_STA, v ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G)
                                       : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // maximum, in both modes
  return 0;
}
// Mains-light banding filter: exposure is kept to multiples of the flicker period so
// lamps don't paint bright/dark stripes. OV2640 sensor bank (0x100 | reg):
// COM8 0x13 bit5 = banding filter on, COM3 0x0C bit2 = 50Hz (else 60Hz).
static int setFlicker(sensor_t *s, int v) {
  if (s->id.PID != OV2640_PID) return 0;
  s->set_reg(s, 0x10C, 0x04, v == 1 ? 0x04 : 0x00);
  return s->set_reg(s, 0x113, 0x20, v ? 0x20 : 0x00);
}

enum { S_FRAMESIZE, S_QUALITY, S_BRIGHTNESS, S_CONTRAST, S_SATURATION, S_EFFECT, S_WB, S_AE,
       S_GAIN, S_VFLIP, S_HMIRROR, S_LED, S_REC, S_REC_FPS, S_REC_MIN,
       S_MD, S_MD_SENS, S_MD_HOLD, S_PRE_SEC, S_TL, S_TL_SEC, S_FLICKER, S_RADIO };
enum { REC_OFF, REC_CONTINUOUS, REC_MOTION };
static const Setting SETTINGS[] = {
  {"framesize", setFramesize, 0, FRAMESIZE_UXGA, FRAMESIZE_VGA},
  {"quality", setQuality, 4, 63, 12},
  {"brightness", setBrightness, -2, 2, 0},
  {"contrast", setContrast, -2, 2, 0},
  {"saturation", setSaturation, -2, 2, 0},
  {"effect", setEffect, 0, 6, 0},
  {"wb_mode", setWbMode, 0, 4, 0},
  {"ae_level", setAeLevel, -2, 2, 0},
  {"gainceiling", setGainCeiling, 0, 6, 2},
  {"vflip", setVflip, 0, 1, 0},
  {"hmirror", setHmirror, 0, 1, 0},
  {"led", setLed, 0, 255, 0},
  {"rec", noop, 0, 2, REC_CONTINUOUS},  // REC_OFF / REC_CONTINUOUS / REC_MOTION
  {"rec_fps", noop, 1, 25, 10},    // recorded frame rate (saves card space)
  {"rec_min", noop, 1, 30, 5},     // clip length in minutes
  {"md", noop, 0, 1, 1},           // motion detection on/off
  {"md_sens", noop, 1, 10, 5},     // sensitivity, 10 = most sensitive
  {"md_hold", noop, 2, 120, 10},   // seconds of stillness before motion ends
  {"pre_sec", noop, 0, 5, 3},      // seconds kept before motion starts (motion recording)
  {"tl", noop, 0, 1, 0},           // time-lapse on/off
  {"tl_sec", noop, 1, 3600, 10},   // seconds between time-lapse frames
  {"flicker", setFlicker, 0, 2, 2},  // 0 off, 1 = 50Hz mains, 2 = 60Hz mains
  {"radio", setRadio, 0, 1, 0},      // 0 normal (b/g/n), 1 = robust (b/g, no 11n)
};
static const int NUM_SETTINGS = sizeof(SETTINGS) / sizeof(SETTINGS[0]);
static int values[NUM_SETTINGS];
static char label[33] = "";
#define MD_W 32  // motion grid, see motionTask
#define MD_H 24
#define MD_CELLS (MD_W * MD_H)
static uint8_t mdMask[MD_CELLS / 8];  // 1 bit per cell, 1 = watched

// ---- shared latest frame ----
// One capture task owns the camera; stream, snapshot and recorder copy from here,
// so recording doesn't halve the live frame rate.
static uint8_t *sharedBuf;
static size_t sharedLen = 0;
static uint16_t sharedW = 0, sharedH = 0;
static volatile uint32_t frameSeq = 0;
static SemaphoreHandle_t frameLock;

// copies the latest frame into dst; returns its length (0 if none/too big) and seq
static size_t copyFrame(uint8_t *dst, size_t cap, uint32_t *seq, uint16_t *w = NULL, uint16_t *h = NULL) {
  size_t len = 0;
  xSemaphoreTake(frameLock, portMAX_DELAY);
  if (sharedLen && sharedLen <= cap) {
    memcpy(dst, sharedBuf, sharedLen);
    len = sharedLen;
    if (w) *w = sharedW;
    if (h) *h = sharedH;
  }
  *seq = frameSeq;
  xSemaphoreGive(frameLock);
  return len;
}

// copy of the latest frame in a buffer of exactly its size (caller frees); NULL if none
static uint8_t *dupFrame(size_t *len) {
  uint8_t *out = NULL;
  xSemaphoreTake(frameLock, portMAX_DELAY);
  if (sharedLen && (out = (uint8_t *)ps_malloc(sharedLen)) != NULL) {
    memcpy(out, sharedBuf, sharedLen);
    *len = sharedLen;
  }
  xSemaphoreGive(frameLock);
  return out;
}

static void captureTask(void *) {
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(50); continue; }
    if (fb->len <= FRAME_BUF_SIZE) {
      xSemaphoreTake(frameLock, portMAX_DELAY);
      memcpy(sharedBuf, fb->buf, fb->len);
      sharedLen = fb->len;
      sharedW = fb->width;
      sharedH = fb->height;
      frameSeq++;
      xSemaphoreGive(frameLock);
    }
    esp_camera_fb_return(fb);
  }
}

// ---- SD card + recorder state ----
static volatile bool sdMounted = false;
static volatile uint32_t sdTotalMB = 0, sdUsedMB = 0;
static char activeFile[48] = "";  // path relative to mount, e.g. /rec/20260919/154500.avi
static volatile bool recording = false;
static volatile bool pauseRec = false;  // set before reboot/OTA so the clip is finalised

// ask the recorder to finish the current clip and wait for it (max 2 s)
static void stopRecording() {
  pauseRec = true;
  for (int i = 0; i < 40 && recording; i++) delay(50);
}

static void urlDecode(const char *in, char *out, size_t outLen) {
  size_t j = 0;
  for (size_t i = 0; in[i] && j < outLen - 1; i++) {
    if (in[i] == '%' && isxdigit((uint8_t)in[i + 1]) && isxdigit((uint8_t)in[i + 2])) {
      char hex[3] = {in[i + 1], in[i + 2], 0};
      out[j++] = (char)strtol(hex, NULL, 16);
      i += 2;
    } else out[j++] = in[i] == '+' ? ' ' : in[i];
  }
  out[j] = 0;
}

static bool tryBoard(const PinMap &b) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_pwdn = b.pin[0]; c.pin_reset = b.pin[1]; c.pin_xclk = b.pin[2];
  c.pin_sccb_sda = b.pin[3]; c.pin_sccb_scl = b.pin[4];
  c.pin_d7 = b.pin[5]; c.pin_d6 = b.pin[6]; c.pin_d5 = b.pin[7]; c.pin_d4 = b.pin[8];
  c.pin_d3 = b.pin[9]; c.pin_d2 = b.pin[10]; c.pin_d1 = b.pin[11]; c.pin_d0 = b.pin[12];
  c.pin_vsync = b.pin[13]; c.pin_href = b.pin[14]; c.pin_pclk = b.pin[15];
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode = CAMERA_GRAB_LATEST;
  // allocate for the largest frame so resolution can be raised at runtime
  c.frame_size = psramFound() ? FRAMESIZE_UXGA : FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    esp_camera_deinit();
    Serial.printf("  %-18s no (0x%x)\n", b.name, err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  sensorPid = s ? s->id.PID : 0;
  Serial.printf("  %-18s yes, sensor 0x%04x\n", b.name, sensorPid);
  return true;
}

// Try the remembered board first, then every other known layout.
static bool initCamera() {
  prefs.begin(PREFS_NS, true);
  int saved = prefs.getInt("board", -1);
  prefs.end();
  if (saved < 0 || saved >= NUM_BOARDS) saved = 0;
  Serial.println("Looking for the camera:");
  for (int attempt = 0; attempt < NUM_BOARDS && !cameraOk; attempt++) {
    int i = attempt == 0 ? saved : attempt - 1;
    if (attempt && i >= saved) i = attempt;          // skip the one already tried
    if (i >= NUM_BOARDS) break;
    if (tryBoard(BOARDS[i])) {
      boardIdx = i;
      ledPin = BOARDS[i].led;
      cameraOk = true;
      prefs.begin(PREFS_NS, false);
      prefs.putInt("board", i);
      prefs.end();
    }
    delay(50);
  }
  if (!cameraOk) {
    Serial.println("No supported camera found - check the ribbon cable is seated.");
    return false;
  }
  Serial.printf("Camera: %s board, sensor 0x%04x\n", BOARDS[boardIdx].name, sensorPid);
  if (ledPin >= 0) { pinMode(ledPin, OUTPUT); analogWrite(ledPin, 0); }

  sensor_t *s = esp_camera_sensor_get();
  prefs.begin(PREFS_NS, false);
  for (int i = 0; i < NUM_SETTINGS; i++) {
    values[i] = prefs.getInt(SETTINGS[i].key, SETTINGS[i].def);
    if (!prefs.isKey(SETTINGS[i].key)) prefs.putInt(SETTINGS[i].key, values[i]);
    if (i == S_LED) values[i] = 0;  // light always starts off
    SETTINGS[i].apply(s, values[i]);
  }
  prefs.getString("label", label, sizeof(label));
  memset(mdMask, 0xff, sizeof(mdMask));  // default: watch the whole picture
  if (prefs.getBytesLength("mask") == sizeof(mdMask)) prefs.getBytes("mask", mdMask, sizeof(mdMask));
  prefs.end();
  return true;
}

// ================= motion detection =================
// Frames are decoded at 1/8 scale (80x60 for VGA), averaged into a fixed 32x24 grid and
// compared with a slowly-updated background. Only cells enabled in the zone mask count.
#define MD_INTERVAL_MS 200

static uint8_t mdChanged[MD_CELLS / 8];   // cells changed in the latest check
static volatile bool motionActive = false;
static volatile uint8_t motionLevel = 0;  // % of watched cells changed
static volatile uint32_t motionEvents = 0, motionSeq = 0;
static volatile time_t motionLastStart = 0;

static inline bool cellGet(const uint8_t *b, int i) { return b[i >> 3] & (1 << (i & 7)); }
static inline void cellSet(uint8_t *b, int i, bool v) {
  if (v) b[i >> 3] |= 1 << (i & 7); else b[i >> 3] &= ~(1 << (i & 7));
}

static void maskToHex(const uint8_t *b, char *out) {
  for (int i = 0; i < MD_CELLS / 8; i++) sprintf(out + i * 2, "%02x", b[i]);
}
static bool hexToMask(const char *hex, uint8_t *b) {
  if (strlen(hex) != MD_CELLS / 4) return false;
  for (int i = 0; i < MD_CELLS / 8; i++) {
    char h[3] = {hex[i * 2], hex[i * 2 + 1], 0};
    if (!isxdigit((uint8_t)h[0]) || !isxdigit((uint8_t)h[1])) return false;
    b[i] = strtol(h, NULL, 16);
  }
  return true;
}

static void motionTask(void *) {
  uint8_t *jpg = (uint8_t *)ps_malloc(FRAME_BUF_SIZE);
  uint8_t *rgb = (uint8_t *)ps_malloc(200 * 150 * 3);  // 1/8 of UXGA
  // static: ~8KB of working arrays would overflow the task stack
  static float bg[MD_CELLS];
  static uint8_t cur[MD_CELLS];
  static uint32_t sum[MD_CELLS];
  static uint16_t cnt[MD_CELLS];
  int warmup = 10, hits = 0;
  uint32_t lastSeq = 0, lastHitMs = 0;
  uint16_t lastW = 0, lastH = 0;
  for (;;) {
    delay(MD_INTERVAL_MS);
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
        bool diff = on && !blown && fabsf(cur[c] - expect) > tBase + tRel * max((float)cur[c], bg[c]);
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

    bool lightingChange = watched && changed * 100 / watched > 60;  // lights switched on/off
    int minCells = max(1, watched * (11 - sens) / 200);
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
        Serial.printf("Motion start (%d%%)\n", motionLevel);
      }
    } else if (motionActive && now - lastHitMs > (uint32_t)values[S_MD_HOLD] * 1000) {
      motionActive = false;
      Serial.println("Motion end");
    }
    motionSeq++;
  }
}

// ================= AVI writer =================
// Minimal MJPEG AVI: fixed 224-byte header rewritten on close, '00dc' chunks, idx1 index.
#define AVI_HDR_LEN 224
#define AVI_MOVI_POS 220  // offset of the 'movi' fourcc; idx1 offsets are relative to it

static FILE *aviFile = NULL;
static uint8_t *aviIdx = NULL;   // 16 bytes per frame, in PSRAM
static uint32_t aviIdxCap = 0, aviFrames = 0, aviMoviBytes = 0, aviMaxFrame = 0;
static uint32_t aviStartMs = 0;
static uint16_t aviW = 0, aviH = 0;
static int aviFramesize = -1;
static bool aviHadMotion = false;

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void putCC(uint8_t *p, const char *cc) { memcpy(p, cc, 4); }

static void buildAviHeader(uint8_t *h, uint32_t frames, uint32_t durationMs, uint32_t idxBytes) {
  memset(h, 0, AVI_HDR_LEN);
  uint32_t usPerFrame = frames > 1 ? (uint32_t)((uint64_t)durationMs * 1000 / frames) : 100000;
  uint32_t rate = usPerFrame ? (uint32_t)(1000000000ULL / usPerFrame) : 10000;  // fps * 1000
  uint32_t moviSize = 4 + aviMoviBytes;
  uint32_t riffSize = AVI_HDR_LEN - 8 + aviMoviBytes + (idxBytes ? 8 + idxBytes : 0);
  putCC(h + 0, "RIFF"); put32(h + 4, riffSize); putCC(h + 8, "AVI ");
  putCC(h + 12, "LIST"); put32(h + 16, 192); putCC(h + 20, "hdrl");
  putCC(h + 24, "avih"); put32(h + 28, 56);
  put32(h + 32, usPerFrame);
  put32(h + 36, durationMs ? (uint32_t)((uint64_t)aviMoviBytes * 1000 / durationMs) : 0);
  put32(h + 44, 0x10);  // AVIF_HASINDEX
  put32(h + 48, frames);
  put32(h + 56, 1);  // streams
  put32(h + 60, aviMaxFrame + 8);
  put32(h + 64, aviW); put32(h + 68, aviH);
  putCC(h + 88, "LIST"); put32(h + 92, 116); putCC(h + 96, "strl");
  putCC(h + 100, "strh"); put32(h + 104, 56);
  putCC(h + 108, "vids"); putCC(h + 112, "MJPG");
  put32(h + 128, 1000);  // scale
  put32(h + 132, rate);  // rate / scale = fps
  put32(h + 140, frames);
  put32(h + 144, aviMaxFrame + 8);
  put32(h + 148, 0xFFFFFFFF);  // quality: default
  put16(h + 160, aviW); put16(h + 162, aviH);  // rcFrame right/bottom
  putCC(h + 164, "strf"); put32(h + 168, 40);
  put32(h + 172, 40); put32(h + 176, aviW); put32(h + 180, aviH);
  put16(h + 184, 1); put16(h + 186, 24);
  putCC(h + 188, "MJPG"); put32(h + 192, (uint32_t)aviW * aviH * 3);
  putCC(h + 212, "LIST"); put32(h + 216, moviSize); putCC(h + 220, "movi");
}

static void refreshSdUsage() {
  if (!sdMounted) return;
  sdTotalMB = SD_MMC.totalBytes() / 1048576;
  sdUsedMB = SD_MMC.usedBytes() / 1048576;
}

// sorted entries of a directory (names only)
static std::vector<String> listDir(const char *path, bool dirs) {
  std::vector<String> out;
  DIR *d = opendir(path);
  if (!d) return out;
  struct dirent *e;
  while ((e = readdir(d)) != NULL) {
    if (e->d_name[0] == '.') continue;
    bool isDir = e->d_type == DT_DIR;
    if (isDir == dirs) out.push_back(String(e->d_name));
  }
  closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

// Every SD access goes through sdLock. listFiles() talks to FatFs directly, below the
// VFS layer that serializes fopen/fwrite, so a /recordings listing could interleave with
// the recorder mid-write and misplace an 8KB block inside the clip. Recursive: openSegment
// calls makeRoom, which locks too.
static SemaphoreHandle_t sdLock;
static inline void sdTake() { if (sdLock) xSemaphoreTakeRecursive(sdLock, portMAX_DELAY); }
static inline void sdGive() { if (sdLock) xSemaphoreGiveRecursive(sdLock); }

// Files in a folder with their sizes. Goes through FatFs directly: stat() on each file
// rescans the whole directory, which took ~1 min for 500 clips.
struct FileEnt { String name; uint32_t size; };
static std::vector<FileEnt> listFiles(const char *path) {  // path under SD_MOUNT
  std::vector<FileEnt> out;
  FF_DIR d;
  FILINFO fi;
  String ffPath = String("0:") + (path + strlen(SD_MOUNT));  // SD card is FatFs drive 0
  sdTake();
  if (f_opendir(&d, ffPath.c_str()) == FR_OK) {
    while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
      if (!(fi.fattrib & AM_DIR) && fi.fname[0] != '.') out.push_back({String(fi.fname), (uint32_t)fi.fsize});
    }
    f_closedir(&d);
    sdGive();
  } else {  // fallback: slow but always works
    sdGive();
    for (const String &f : listDir(path, false)) {
      struct stat st;
      out.push_back({f, stat((String(path) + "/" + f).c_str(), &st) == 0 ? (uint32_t)st.st_size : 0});
    }
  }
  std::sort(out.begin(), out.end(), [](const FileEnt &a, const FileEnt &b) { return a.name < b.name; });
  return out;
}

static void removeTree(const String &path) {  // time-lapse hour folder: files, then the folder
  for (const String &f : listDir(path.c_str(), false)) unlink((path + "/" + f).c_str());
  rmdir(path.c_str());
}

// delete the oldest footage (a clip, or a whole time-lapse hour) until there is room
static void makeRoom() {
  sdTake();
  refreshSdUsage();
  uint32_t keepFreeMB = max<uint32_t>(sdTotalMB / 20, 256);  // 5%, at least 256MB
  int guard = 200;
  while (sdTotalMB && sdTotalMB - sdUsedMB < keepFreeMB && guard-- > 0) {
    // oldest clip, keyed "YYYYMMDDHH"
    String recKey, recDay, recFile;
    std::vector<String> rdays = listDir(REC_DIR, true);
    if (!rdays.empty()) {
      recDay = String(REC_DIR) + "/" + rdays[0];
      std::vector<String> files = listDir(recDay.c_str(), false);
      if (files.empty()) { rmdir(recDay.c_str()); continue; }
      recFile = recDay + "/" + files[0];
      recKey = rdays[0] + files[0].substring(0, 2);
    }
    // oldest time-lapse hour
    String tlKey, tlDay, tlHour;
    std::vector<String> tdays = listDir(TL_DIR, true);
    if (!tdays.empty()) {
      tlDay = String(TL_DIR) + "/" + tdays[0];
      std::vector<String> hours = listDir(tlDay.c_str(), true);
      if (hours.empty()) { rmdir(tlDay.c_str()); continue; }
      tlHour = tlDay + "/" + hours[0];
      tlKey = tdays[0] + hours[0];
    }
    if (recKey.isEmpty() && tlKey.isEmpty()) break;
    if (!tlKey.isEmpty() && (recKey.isEmpty() || tlKey <= recKey)) {
      Serial.printf("Card full, deleting time-lapse %s\n", tlHour.c_str());
      removeTree(tlHour);
      rmdir(tlDay.c_str());  // only succeeds once empty
    } else {
      Serial.printf("Card full, deleting %s\n", recFile.c_str());
      unlink(recFile.c_str());
      rmdir(recDay.c_str());
    }
    refreshSdUsage();
  }
  sdGive();
}

// ---- time-lapse: one JPEG every tl_sec seconds into /tl/YYYYMMDD/HH/MMSS.jpg ----
static volatile uint32_t tlFrames = 0;
static volatile time_t tlLast = 0;

static void timelapseTask(void *) {
  uint32_t lastShot = 0, sinceRoomCheck = 0;
  bool first = true;
  for (;;) {
    delay(100);
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
      Serial.printf("Time-lapse write failed: %s\n", path);
    }
  }
}

static bool openSegment(uint16_t w, uint16_t h) {
  makeRoom();
  mkdir(REC_DIR, 0777);
  time_t now = time(NULL);
  struct tm t;
  localtime_r(&now, &t);
  char dir[40], path[64];
  if (t.tm_year + 1900 >= 2024) {
    snprintf(dir, sizeof(dir), REC_DIR "/%04d%02d%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    snprintf(path, sizeof(path), "%s/%02d%02d%02d.avi", dir, t.tm_hour, t.tm_min, t.tm_sec);
  } else {  // clock not set yet (no internet): still record, named by uptime
    snprintf(dir, sizeof(dir), REC_DIR "/00000000");
    snprintf(path, sizeof(path), "%s/up%09lu.avi", dir, (unsigned long)millis());
  }
  mkdir(dir, 0777);

  sdTake();
  aviIdxCap = (values[S_REC_MIN] * 60 + 5) * values[S_REC_FPS] + 64;  // + pre-roll
  aviIdx = (uint8_t *)ps_malloc(aviIdxCap * 16);
  if (!aviIdx) { sdGive(); return false; }
  aviFile = fopen(path, "wb");
  if (!aviFile) {
    free(aviIdx);
    aviIdx = NULL;
    sdGive();
    return false;
  }
  setvbuf(aviFile, NULL, _IOFBF, 32 * 1024);
  aviFrames = aviMoviBytes = aviMaxFrame = 0;
  aviW = w;
  aviH = h;
  aviFramesize = values[S_FRAMESIZE];
  aviStartMs = millis();
  aviHadMotion = false;
  uint8_t hdr[AVI_HDR_LEN];
  buildAviHeader(hdr, 0, 0, 0);
  fwrite(hdr, 1, AVI_HDR_LEN, aviFile);
  sdGive();
  strlcpy(activeFile, path + strlen(SD_MOUNT), sizeof(activeFile));
  recording = true;
  Serial.printf("Recording %s\n", path);
  return true;
}

static bool writeFrame(const uint8_t *jpg, size_t len) {
  uint8_t ch[8];
  sdTake();
  uint32_t padded = (len + 1) & ~1u;  // chunks start on even offsets; the pad byte
  putCC(ch, "00dc");                  // is NOT counted in the chunk/index size
  put32(ch + 4, len);
  uint32_t offset = 4 + aviMoviBytes;  // relative to 'movi'
  bool wrote = fwrite(ch, 1, 8, aviFile) == 8 && fwrite(jpg, 1, len, aviFile) == len
               && (padded == len || fputc(0, aviFile) != EOF);
  sdGive();
  if (!wrote) return false;
  uint8_t *e = aviIdx + aviFrames * 16;
  putCC(e, "00dc");
  put32(e + 4, 0x10);  // AVIIF_KEYFRAME
  put32(e + 8, offset);
  put32(e + 12, len);
  aviFrames++;
  aviMoviBytes += 8 + padded;
  if (padded > aviMaxFrame) aviMaxFrame = padded;
  return true;
}

static void closeSegment() {
  if (!aviFile) return;
  sdTake();
  uint32_t duration = millis() - aviStartMs;
  uint32_t idxBytes = aviFrames * 16;
  uint8_t ch[8];
  putCC(ch, "idx1");
  put32(ch + 4, idxBytes);
  bool ok = fwrite(ch, 1, 8, aviFile) == 8 && fwrite(aviIdx, 1, idxBytes, aviFile) == idxBytes;
  uint8_t hdr[AVI_HDR_LEN];
  buildAviHeader(hdr, aviFrames, duration, idxBytes);
  ok = fseek(aviFile, 0, SEEK_SET) == 0 && fwrite(hdr, 1, AVI_HDR_LEN, aviFile) == AVI_HDR_LEN && ok;
  ok = fclose(aviFile) == 0 && ok;  // fclose flushes: a failure here truncates the clip
  aviFile = NULL;
  if (!ok) Serial.printf("Clip close failed: %s\n", activeFile);
  free(aviIdx);
  aviIdx = NULL;
  sdGive();
  if (aviHadMotion) {  // tag clips containing motion: 154500.avi -> 154500_M.avi
    char from[80], to[80];
    snprintf(from, sizeof(from), SD_MOUNT "%s", activeFile);
    strlcpy(to, from, sizeof(to));
    char *dot = strrchr(to, '.');
    if (dot) { strcpy(dot, "_M.avi"); rename(from, to); }
  }
  Serial.printf("Closed %s: %lu frames, %lu s%s\n", activeFile, (unsigned long)aviFrames,
                (unsigned long)(duration / 1000), aviHadMotion ? ", motion" : "");
  activeFile[0] = 0;
  recording = false;
}

static void abortSegment() {  // card pulled or write error
  if (aviFile) fclose(aviFile);
  aviFile = NULL;
  free(aviIdx);
  aviIdx = NULL;
  activeFile[0] = 0;
  recording = false;
}

static bool mountSd() {
  // 1-bit mode leaves GPIO4 (flash LED) and GPIO12/13 free
  // 20MHz. The frame corruption once blamed on 40MHz was really two software bugs (the AVI
  // pad byte counted in the chunk size, and unlocked FatFs listings racing the writer);
  // SDMMC_FREQ_HIGHSPEED is worth retrying now that both are fixed.
  if (!SD_MMC.begin(SD_MOUNT, true, false, SDMMC_FREQ_DEFAULT, 5)) return false;
  if (SD_MMC.cardType() == CARD_NONE) {
    SD_MMC.end();
    return false;
  }
  sdMounted = true;
  refreshSdUsage();
  Serial.printf("SD card mounted: %lu MB, %lu MB used\n", (unsigned long)sdTotalMB, (unsigned long)sdUsedMB);
  return true;
}

// ---- pre-roll: the last few seconds of frames, kept in PSRAM for motion recording ----
#define RING_BYTES (768 * 1024)
#define RING_MAX 128
struct RingEnt { uint32_t off, len, ms; };
static uint8_t *ring;
static RingEnt ringEnts[RING_MAX];
static int ringHead = 0, ringCount = 0;
static uint32_t ringWrite = 0;

static void ringPop() { ringHead = (ringHead + 1) % RING_MAX; ringCount--; }
static void ringClear() { ringHead = ringCount = 0; ringWrite = 0; }

static void ringPush(const uint8_t *jpg, uint32_t len, uint32_t ms, uint32_t keepMs) {
  if (!ring || len > RING_BYTES / 2) return;
  while (ringCount && ms - ringEnts[ringHead].ms > keepMs) ringPop();
  if (ringWrite + len > RING_BYTES) ringWrite = 0;
  // drop oldest frames until the new one doesn't overwrite anything still queued
  for (;;) {
    bool overlap = false;
    for (int i = 0; i < ringCount && !overlap; i++) {
      const RingEnt &e = ringEnts[(ringHead + i) % RING_MAX];
      overlap = e.off < ringWrite + len && ringWrite < e.off + e.len;
    }
    if (!overlap && ringCount < RING_MAX) break;
    ringPop();
  }
  memcpy(ring + ringWrite, jpg, len);
  ringEnts[(ringHead + ringCount) % RING_MAX] = {ringWrite, len, ms};
  ringCount++;
  ringWrite += len;
}

static void dropCard(const char *why) {
  Serial.println(why);
  abortSegment();
  SD_MMC.end();
  sdMounted = false;
}

static void recorderTask(void *) {
  uint8_t *buf = (uint8_t *)ps_malloc(FRAME_BUF_SIZE);
  ring = (uint8_t *)ps_malloc(RING_BYTES);
  uint32_t lastSeq = 0, lastFrameMs = 0, lastMountTry = 0;
  for (;;) {
    if (!sdMounted) {
      if (millis() - lastMountTry > 30000 || lastMountTry == 0) {
        lastMountTry = millis();
        mountSd();
      }
      delay(500);
      continue;
    }
    int mode = cameraOk ? values[S_REC] : REC_OFF;
    if (mode == REC_OFF || pauseRec) {
      closeSegment();
      ringClear();
      delay(300);
      continue;
    }
    uint32_t interval = 1000 / values[S_REC_FPS];
    if (millis() - lastFrameMs < interval || frameSeq == lastSeq) {
      delay(5);
      continue;
    }
    uint16_t w = 0, h = 0;
    size_t len = copyFrame(buf, FRAME_BUF_SIZE, &lastSeq, &w, &h);
    if (!len) { delay(5); continue; }
    uint32_t now = lastFrameMs = millis();

    // resolution change: start a fresh clip (AVI frames must share one size)
    if (aviFile && (aviFramesize != values[S_FRAMESIZE] || w != aviW || h != aviH)) closeSegment();
    if (ringCount && (w != aviW || h != aviH) && !aviFile) ringClear();

    bool want = mode == REC_CONTINUOUS || motionActive;
    if (!want) {
      closeSegment();  // motion ended
      if (values[S_PRE_SEC]) { ringPush(buf, len, now, values[S_PRE_SEC] * 1000); aviW = w; aviH = h; }
      continue;
    }
    if (!aviFile) {
      if (!openSegment(w, h)) { dropCard("Could not open clip, remounting card"); continue; }
      if (mode == REC_MOTION && ringCount) {  // prepend the pre-roll
        aviStartMs = ringEnts[ringHead].ms;
        bool ok = true;
        while (ringCount && ok) {
          const RingEnt &e = ringEnts[ringHead];
          ok = writeFrame(ring + e.off, e.len);
          ringPop();
        }
        ringClear();
        if (!ok) { dropCard("SD write failed, card removed?"); continue; }
      }
    }
    if (motionActive) aviHadMotion = true;
    if (!writeFrame(buf, len)) { dropCard("SD write failed, card removed?"); continue; }
    // millis() here, not `now`: openSegment() may have set aviStartMs after `now` was taken
    if (millis() - aviStartMs >= (uint32_t)values[S_REC_MIN] * 60000 || aviFrames >= aviIdxCap) {
      closeSegment();
      refreshSdUsage();
    }
  }
}

// ================= HTTP =================
static esp_err_t addCors(httpd_req_t *req) {
  return httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

static esp_err_t streamHandler(httpd_req_t *req) {
  char part[64];
  uint8_t *buf = (uint8_t *)ps_malloc(FRAME_BUF_SIZE);
  if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
  addCors(req);
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  uint32_t lastSeq = 0;
  while (res == ESP_OK) {
    if (frameSeq == lastSeq) { delay(5); continue; }
    size_t len = copyFrame(buf, FRAME_BUF_SIZE, &lastSeq);
    if (!len) { delay(5); continue; }
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)buf, len);
  }
  free(buf);
  return res;
}

static esp_err_t captureHandler(httpd_req_t *req) {
  size_t len = 0;
  uint8_t *buf = dupFrame(&len);
  if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
  addCors(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t res = httpd_resp_send(req, (const char *)buf, len);
  free(buf);
  return res;
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char json[1024];
  int n = snprintf(json, sizeof(json),
           "{\"name\":\"%s\",\"label\":\"%s\",\"version\":\"%s\",\"ip\":\"%s\",\"mac\":\"%s\","
           "\"ssid\":\"%s\",\"rssi\":%d,\"uptime\":%lu,\"heap\":%u,\"psram\":%u,\"psram_block\":%u,"
           "\"stream_port\":81,\"files_port\":82,\"time\":%lu,"
           "\"sd\":%d,\"sd_total_mb\":%lu,\"sd_used_mb\":%lu,\"recording\":%d,\"rec_file\":\"%s\","
           "\"motion\":%d,\"motion_level\":%d,\"motion_events\":%lu,\"motion_last\":%lu,"
           "\"tl_frames\":%lu,\"tl_last\":%lu,\"camera\":%d,\"board\":\"%s\",\"sensor\":%u",
           hostName, label, FW_VERSION, WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str(),
           WiFi.SSID().c_str(), WiFi.RSSI(), millis() / 1000, ESP.getFreeHeap(), ESP.getFreePsram(),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
           (unsigned long)time(NULL), sdMounted ? 1 : 0, (unsigned long)sdTotalMB,
           (unsigned long)sdUsedMB, recording ? 1 : 0, activeFile, motionActive ? 1 : 0, motionLevel,
           (unsigned long)motionEvents, (unsigned long)motionLastStart,
           (unsigned long)tlFrames, (unsigned long)tlLast, cameraOk ? 1 : 0,
           boardIdx >= 0 ? BOARDS[boardIdx].name : "unknown", sensorPid);
  for (int i = 0; i < NUM_SETTINGS; i++)
    n += snprintf(json + n, sizeof(json) - n, ",\"%s\":%d", SETTINGS[i].key, values[i]);
  snprintf(json + n, sizeof(json) - n, "}");
  addCors(req);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, json);
}

static esp_err_t controlHandler(httpd_req_t *req) {
  char query[512], val[200];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    return ESP_FAIL;
  }
  sensor_t *s = esp_camera_sensor_get();
  prefs.begin(PREFS_NS, false);
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (httpd_query_key_value(query, SETTINGS[i].key, val, sizeof(val)) != ESP_OK) continue;
    int v = constrain(atoi(val), SETTINGS[i].lo, SETTINGS[i].hi);
    SETTINGS[i].apply(s, v);
    values[i] = v;
    prefs.putInt(SETTINGS[i].key, v);
  }
  if (httpd_query_key_value(query, "mask", val, sizeof(val)) == ESP_OK && hexToMask(val, mdMask)) {
    prefs.putBytes("mask", mdMask, sizeof(mdMask));
  }
  if (httpd_query_key_value(query, "label", val, sizeof(val)) == ESP_OK) {
    char decoded[100];
    urlDecode(val, decoded, sizeof(decoded));
    for (char *c = decoded; *c; c++) if (*c == '"' || *c == '\\' || (uint8_t)*c < 0x20) *c = ' ';
    strlcpy(label, decoded, sizeof(label));
    prefs.putString("label", label);
  }
  if (httpd_query_key_value(query, "reset", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
    prefs.clear();
    label[0] = 0;
    memset(mdMask, 0xff, sizeof(mdMask));
    for (int i = 0; i < NUM_SETTINGS; i++) { values[i] = SETTINGS[i].def; SETTINGS[i].apply(s, values[i]); }
  }
  prefs.end();
  return statusHandler(req);
}

static esp_err_t motionHandler(httpd_req_t *req) {
  char json[512], changed[MD_CELLS / 4 + 1], mask[MD_CELLS / 4 + 1];
  maskToHex(mdChanged, changed);
  maskToHex(mdMask, mask);
  snprintf(json, sizeof(json),
           "{\"active\":%d,\"level\":%d,\"events\":%lu,\"last\":%lu,\"seq\":%lu,\"enabled\":%d,"
           "\"w\":%d,\"h\":%d,\"changed\":\"%s\",\"mask\":\"%s\"}",
           motionActive ? 1 : 0, motionLevel, (unsigned long)motionEvents, (unsigned long)motionLastStart,
           (unsigned long)motionSeq, values[S_MD], MD_W, MD_H, changed, mask);
  addCors(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, json);
}

// /reg?addr=0x113 reads a sensor register (0x100 | reg = sensor bank) for troubleshooting
static esp_err_t regHandler(httpd_req_t *req) {
  char query[48], val[16], out[48];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "addr", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "addr required");
    return ESP_FAIL;
  }
  int addr = strtol(val, NULL, 0);
  sensor_t *s = esp_camera_sensor_get();
  snprintf(out, sizeof(out), "{\"addr\":%d,\"value\":%d}", addr, s->get_reg(s, addr, 0xFF));
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, out);
}

static esp_err_t rebootHandler(httpd_req_t *req) {
  addCors(req);
  httpd_resp_sendstr(req, "{\"rebooting\":true}");
  stopRecording();
  ESP.restart();
  return ESP_OK;
}

static esp_err_t indexHandler(httpd_req_t *req) {
  char html[640];
  snprintf(html, sizeof(html),
           "<!doctype html><title>%s</title><body style='margin:0;background:#111;color:#ddd;font-family:sans-serif'>"
           "<p style='padding:8px;margin:0'>%s &middot; %s &middot; v%s</p>"
           "<img id=v style='width:100%%;max-width:1024px;display:block'>"
           "<script>v.src=location.protocol+'//'+location.hostname+':81/stream'</script></body>",
           label[0] ? label : hostName, label[0] ? label : hostName, hostName, FW_VERSION);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_sendstr(req, html);
}

// ---- recordings (port 82) ----
// path param must look like /rec/<day>/<file>.avi
static bool getRecPath(httpd_req_t *req, char *full, size_t fullLen, char *rel, size_t relLen) {
  char query[160], val[120];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  if (httpd_query_key_value(query, "path", val, sizeof(val)) != ESP_OK) return false;
  urlDecode(val, rel, relLen);
  if ((strncmp(rel, "/rec/", 5) != 0 && strncmp(rel, "/tl/", 4) != 0) || strstr(rel, "..")) return false;
  snprintf(full, fullLen, SD_MOUNT "%s", rel);
  return true;
}

static esp_err_t recordingsHandler(httpd_req_t *req) {
  addCors(req);
  httpd_resp_set_type(req, "application/json");
  if (!sdMounted) return httpd_resp_sendstr(req, "[]");
  httpd_resp_send_chunk(req, "[", 1);
  std::vector<String> days = listDir(REC_DIR, true);
  int count = 0;
  bool first = true;
  for (int d = days.size() - 1; d >= 0 && count < 2000; d--) {
    String dayPath = String(REC_DIR) + "/" + days[d];
    std::vector<FileEnt> files = listFiles(dayPath.c_str());
    for (int f = files.size() - 1; f >= 0 && count < 2000; f--, count++) {
      String rel = "/rec/" + days[d] + "/" + files[f].name;
      size_t size = files[f].size;
      char item[160];
      int n = snprintf(item, sizeof(item), "%s{\"path\":\"%s\",\"size\":%u,\"active\":%s}",
                       first ? "" : ",", rel.c_str(), (unsigned)size,
                       strcmp(rel.c_str(), activeFile) == 0 ? "true" : "false");
      httpd_resp_send_chunk(req, item, n);
      first = false;
    }
  }
  httpd_resp_send_chunk(req, "]", 1);
  return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t fileHandler(httpd_req_t *req) {
  char full[160], rel[120];
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
    return ESP_FAIL;
  }
  if (strcmp(rel, activeFile) == 0) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "clip still recording");
  }
  FILE *f = fopen(full, "rb");
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  addCors(req);
  httpd_resp_set_type(req, "video/x-msvideo");
  const size_t CHUNK = 64 * 1024;
  char *buf = (char *)ps_malloc(CHUNK);
  size_t n;
  esp_err_t res = ESP_OK;
  while (res == ESP_OK && (n = fread(buf, 1, CHUNK, f)) > 0) res = httpd_resp_send_chunk(req, buf, n);
  fclose(f);
  free(buf);
  if (res == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

static esp_err_t timelapseHandler(httpd_req_t *req) {
  addCors(req);
  httpd_resp_set_type(req, "application/json");
  if (!sdMounted) return httpd_resp_sendstr(req, "[]");
  httpd_resp_send_chunk(req, "[", 1);
  std::vector<String> days = listDir(TL_DIR, true);
  for (int d = days.size() - 1; d >= 0; d--) {
    String dayPath = String(TL_DIR) + "/" + days[d];
    String item = String(d == (int)days.size() - 1 ? "" : ",") + "{\"day\":\"" + days[d] + "\",\"hours\":[";
    std::vector<String> hours = listDir(dayPath.c_str(), true);
    for (size_t h = 0; h < hours.size(); h++) {
      std::vector<String> frames = listDir((dayPath + "/" + hours[h]).c_str(), false);
      item += String(h ? "," : "") + "{\"hour\":\"" + hours[h] + "\",\"count\":" + frames.size() + "}";
    }
    item += "]}";
    httpd_resp_send_chunk(req, item.c_str(), item.length());
  }
  httpd_resp_send_chunk(req, "]", 1);
  return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t tlFramesHandler(httpd_req_t *req) {
  char full[160], rel[120];
  addCors(req);
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) || strncmp(rel, "/tl/", 4) != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send_chunk(req, "[", 1);
  std::vector<String> frames = listDir(full, false);
  String chunk;
  for (size_t i = 0; i < frames.size(); i++) {
    chunk += String(i ? ",\"" : "\"") + frames[i] + "\"";
    if (chunk.length() > 1500) { httpd_resp_send_chunk(req, chunk.c_str(), chunk.length()); chunk = ""; }
  }
  chunk += "]";
  httpd_resp_send_chunk(req, chunk.c_str(), chunk.length());
  return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t tlFrameHandler(httpd_req_t *req) {
  char full[160], rel[120];
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) || strncmp(rel, "/tl/", 4) != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
    return ESP_FAIL;
  }
  FILE *f = fopen(full, "rb");
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = (char *)ps_malloc(size > 0 ? size : 1);
  size_t n = buf ? fread(buf, 1, size, f) : 0;
  fclose(f);
  addCors(req);
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = n ? httpd_resp_send(req, buf, n) : httpd_resp_send_500(req);
  free(buf);
  return res;
}

static esp_err_t deleteHandler(httpd_req_t *req) {
  char full[160], rel[120];
  addCors(req);
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) || strcmp(rel, activeFile) == 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "cannot delete");
    return ESP_FAIL;
  }
  bool ok;
  if (strncmp(rel, "/tl/", 4) == 0) {
    // time-lapse: a day (/tl/YYYYMMDD) or an hour (/tl/YYYYMMDD/HH)
    struct stat st;
    ok = stat(full, &st) == 0 && S_ISDIR(st.st_mode);
    if (ok) {
      std::vector<String> hours = listDir(full, true);
      if (hours.empty()) removeTree(full);
      else for (const String &h : hours) removeTree(String(full) + "/" + h);
      rmdir(full);
      char *slash = strrchr(full, '/');
      if (slash) { *slash = 0; rmdir(full); }
    }
  } else {
    ok = unlink(full) == 0;
    // drop the day folder if that was its last clip
    char *slash = strrchr(full, '/');
    if (slash) { *slash = 0; rmdir(full); }
  }
  refreshSdUsage();
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static httpd_handle_t startHttpd(uint16_t port, uint16_t ctrlPort, int sockets, const httpd_uri_t *uris, int n) {
  httpd_handle_t h = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port;
  config.ctrl_port = ctrlPort;
  config.max_open_sockets = sockets;
  config.lru_purge_enable = true;
  config.stack_size = 6144;
  if (httpd_start(&h, &config) != ESP_OK) {
    Serial.printf("HTTP server on port %u failed to start\n", port);
    return NULL;
  }
  for (int i = 0; i < n; i++) httpd_register_uri_handler(h, &uris[i]);
  return h;
}

static void startServers() {
  static const httpd_uri_t api[] = {
    {"/", HTTP_GET, indexHandler, NULL},
    {"/capture", HTTP_GET, captureHandler, NULL},
    {"/status", HTTP_GET, statusHandler, NULL},
    {"/control", HTTP_GET, controlHandler, NULL},
    {"/reboot", HTTP_GET, rebootHandler, NULL},
    {"/motion", HTTP_GET, motionHandler, NULL},
    {"/reg", HTTP_GET, regHandler, NULL},
  };
  static const httpd_uri_t stream[] = {{"/stream", HTTP_GET, streamHandler, NULL}};
  static const httpd_uri_t files[] = {
    {"/recordings", HTTP_GET, recordingsHandler, NULL},
    {"/file", HTTP_GET, fileHandler, NULL},
    {"/delete", HTTP_GET, deleteHandler, NULL},
    {"/timelapse", HTTP_GET, timelapseHandler, NULL},
    {"/tlframes", HTTP_GET, tlFramesHandler, NULL},
    {"/tlframe", HTTP_GET, tlFrameHandler, NULL},
  };
  // lwIP has 16 sockets total; each server also uses a listen + control socket
  httpd = startHttpd(80, 32768, 4, api, 7);
  streamd = startHttpd(81, 32769, 2, stream, 1);
  filed = startHttpd(82, 32770, 2, files, 6);
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostName);
  WiFi.setSleep(false);
  setRadio(NULL, values[S_RADIO]);
  if (values[S_RADIO]) Serial.println("Radio: robust mode (802.11b/g, no 11n)");
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    // An AP with legacy rates disabled refuses an 802.11b-only client outright
    // ("Refused basic rates mismatch"), and S_RADIO is persistent -- without this
    // fallback one toggle would strand the camera until someone reflashed it by USB.
    if (values[S_RADIO] && millis() - start > 15000) {
      Serial.println("\nNo link in robust mode, falling back to b/g/n");
      values[S_RADIO] = 0;
      prefs.begin(PREFS_NS, false);
      prefs.putInt("radio", 0);
      prefs.end();
      setRadio(NULL, 0);
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
      continue;
    }
    if (millis() - start > 30000) {
      Serial.println("\nWiFi connect timeout, restarting");
      ESP.restart();
    }
  }
  Serial.printf("\nConnected, IP %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  Serial.println("\ncamnode " FW_VERSION);
  // factory MAC from efuse; WiFi.macAddress() is a placeholder until WiFi starts
  uint64_t mac = ESP.getEfuseMac();
  snprintf(hostName, sizeof(hostName), "espcam-%02x%02x%02x",
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));

  frameLock = xSemaphoreCreateMutex();
  sdLock = xSemaphoreCreateRecursiveMutex();
  sharedBuf = (uint8_t *)ps_malloc(FRAME_BUF_SIZE);
  if (!sharedBuf) {
    Serial.println("No PSRAM - this firmware needs a camera board with PSRAM");
    delay(5000);
    ESP.restart();
  }
  // Without a working camera the rest still starts, so the board stays reachable
  // over WiFi (status, OTA) instead of rebooting in a loop.
  if (initCamera()) xTaskCreatePinnedToCore(captureTask, "capture", 4096, NULL, 5, NULL, 1);

  connectWiFi();
  configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
  startServers();
  xTaskCreatePinnedToCore(recorderTask, "recorder", 8192, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(motionTask, "motion", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(timelapseTask, "timelapse", 6144, NULL, 2, NULL, 0);

  if (MDNS.begin(hostName)) {
    MDNS.addService("espcam", "tcp", 80);
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("espcam", "tcp", "version", FW_VERSION);
    MDNS.addServiceTxt("espcam", "tcp", "stream_port", "81");
    MDNS.addServiceTxt("espcam", "tcp", "files_port", "82");
  }

  ArduinoOTA.setHostname(hostName);
  if (strlen(OTA_PASS)) ArduinoOTA.setPassword(OTA_PASS);
  // close the current clip so it stays playable across the update
  ArduinoOTA.onStart([]() { stopRecording(); });
  ArduinoOTA.onError([](ota_error_t) { pauseRec = false; });
  ArduinoOTA.begin();
  Serial.printf("Ready: http://%s.local/  http://%s/\n", hostName, WiFi.localIP().toString().c_str());
}

void loop() {
  ArduinoOTA.handle();
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting");
      WiFi.reconnect();
    }
  }
  delay(10);
}
