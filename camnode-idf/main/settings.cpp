// Persisted settings, backed by NVS. Each entry maps a /control key to its setter
// and allowed range, exactly as in the Arduino build, so the hub's UI is unchanged.
#include "camnode.h"

#include "driver/ledc.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "settings";

// ---- NVS helpers (these replace Arduino's Preferences) ----
static nvs_handle_t openNvs(bool readOnly) {
  nvs_handle_t h = 0;
  if (nvs_open(PREFS_NS, readOnly ? NVS_READONLY : NVS_READWRITE, &h) != ESP_OK) return 0;
  return h;
}

bool prefsGetInt(const char *key, int *out) {
  nvs_handle_t h = openNvs(true);
  if (!h) return false;
  int32_t v = 0;
  esp_err_t err = nvs_get_i32(h, key, &v);
  nvs_close(h);
  if (err != ESP_OK) return false;
  *out = v;
  return true;
}

void prefsSetInt(const char *key, int v) {
  nvs_handle_t h = openNvs(false);
  if (!h) return;
  nvs_set_i32(h, key, v);
  nvs_commit(h);
  nvs_close(h);
}

void prefsGetStr(const char *key, char *out, size_t len) {
  out[0] = 0;
  nvs_handle_t h = openNvs(true);
  if (!h) return;
  size_t n = len;
  if (nvs_get_str(h, key, out, &n) != ESP_OK) out[0] = 0;
  nvs_close(h);
}

void prefsSetStr(const char *key, const char *v) {
  nvs_handle_t h = openNvs(false);
  if (!h) return;
  nvs_set_str(h, key, v);
  nvs_commit(h);
  nvs_close(h);
}

bool prefsGetBlob(const char *key, void *out, size_t len) {
  nvs_handle_t h = openNvs(true);
  if (!h) return false;
  size_t n = len;
  esp_err_t err = nvs_get_blob(h, key, out, &n);
  nvs_close(h);
  return err == ESP_OK && n == len;
}

void prefsSetBlob(const char *key, const void *v, size_t len) {
  nvs_handle_t h = openNvs(false);
  if (!h) return;
  nvs_set_blob(h, key, v, len);
  nvs_commit(h);
  nvs_close(h);
}

void prefsClear() {
  nvs_handle_t h = openNvs(false);
  if (!h) return;
  nvs_erase_all(h);
  nvs_commit(h);
  nvs_close(h);
}

// ---- setters ----
static int setFramesize(sensor_t *s, int v) { return s ? s->set_framesize(s, (framesize_t)v) : 0; }
static int setQuality(sensor_t *s, int v) { return s ? s->set_quality(s, v) : 0; }
static int setBrightness(sensor_t *s, int v) { return s ? s->set_brightness(s, v) : 0; }
static int setContrast(sensor_t *s, int v) { return s ? s->set_contrast(s, v) : 0; }
static int setSaturation(sensor_t *s, int v) { return s ? s->set_saturation(s, v) : 0; }
static int setEffect(sensor_t *s, int v) { return s ? s->set_special_effect(s, v) : 0; }
static int setWbMode(sensor_t *s, int v) { if (!s) return 0; s->set_awb_gain(s, 1); return s->set_wb_mode(s, v); }
static int setAeLevel(sensor_t *s, int v) { return s ? s->set_ae_level(s, v) : 0; }
static int setGainCeiling(sensor_t *s, int v) { return s ? s->set_gainceiling(s, (gainceiling_t)v) : 0; }
static int setVflip(sensor_t *s, int v) { return s ? s->set_vflip(s, v) : 0; }
static int setHmirror(sensor_t *s, int v) { return s ? s->set_hmirror(s, v) : 0; }
static int setLed(sensor_t *, int v) { setLedDuty(v); return 0; }
static int noop(sensor_t *, int) { return 0; }  // read directly by the recorder

// Radio mode. "Robust" drops 802.11n, leaving b/g: lower rates with sturdier modulation
// and no HT aggregation, which helps a board with a poor antenna. It deliberately keeps
// 11g -- an 802.11b-only client is refused outright ("Refused basic rates mismatch") by
// any AP with legacy rates disabled, and this setting persists in NVS.
int setRadio(sensor_t *, int v) {
  esp_wifi_set_protocol(WIFI_IF_STA, v ? (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G)
                                       : (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
  esp_wifi_set_max_tx_power(78);  // 19.5 dBm, the maximum, in both modes
  return 0;
}

// Mains-light banding filter: exposure is kept to multiples of the flicker period so
// lamps don't paint bright/dark stripes. OV2640 sensor bank (0x100 | reg):
// COM8 0x13 bit5 = banding filter on, COM3 0x0C bit2 = 50Hz (else 60Hz).
static int setFlicker(sensor_t *s, int v) {
  if (!s || s->id.PID != OV2640_PID) return 0;
  s->set_reg(s, 0x10C, 0x04, v == 1 ? 0x04 : 0x00);
  return s->set_reg(s, 0x113, 0x20, v ? 0x20 : 0x00);
}

const Setting SETTINGS[] = {
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
  {"pre_sec", noop, 0, 5, 3},      // seconds kept before motion starts
  {"tl", noop, 0, 1, 0},           // time-lapse on/off
  {"tl_sec", noop, 1, 3600, 10},   // seconds between time-lapse frames
  {"flicker", setFlicker, 0, 2, 2},  // 0 off, 1 = 50Hz mains, 2 = 60Hz mains
  {"radio", setRadio, 0, 1, 0},      // 0 normal (b/g/n), 1 = robust (b/g, no 11n)
};
const int NUM_SETTINGS = sizeof(SETTINGS) / sizeof(SETTINGS[0]);
int values[sizeof(SETTINGS) / sizeof(SETTINGS[0])];
char label[33] = "";
uint8_t mdMask[MD_CELLS / 8];

void loadSettings(sensor_t *s) {
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (!prefsGetInt(SETTINGS[i].key, &values[i])) {
      values[i] = SETTINGS[i].def;
      prefsSetInt(SETTINGS[i].key, values[i]);  // persist defaults on first boot
    }
    if (i == S_LED) values[i] = 0;  // light always starts off
    SETTINGS[i].apply(s, values[i]);
  }
  prefsGetStr("label", label, sizeof(label));
  memset(mdMask, 0xff, sizeof(mdMask));  // default: watch the whole picture
  prefsGetBlob("mask", mdMask, sizeof(mdMask));
}
