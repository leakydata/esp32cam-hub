// Camera bring-up and the single capture task that owns the sensor.
// One task grabs frames into a shared buffer; stream, snapshot, recorder, motion and
// time-lapse all copy from there, so recording doesn't halve the live frame rate.
#include "camnode.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "camera";

// Known ESP32 camera boards. On first boot each is tried until the camera answers,
// and the working one is remembered, so the same firmware fits any of these boards.
const PinMap BOARDS[] = {
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
const int NUM_BOARDS = sizeof(BOARDS) / sizeof(BOARDS[0]);
int boardIdx = -1;
int ledPin = -1;
bool cameraOk = false;
uint16_t sensorPid = 0;

// ---- shared latest frame ----
static uint8_t *sharedBuf;
static size_t sharedLen = 0;
static uint16_t sharedW = 0, sharedH = 0;
volatile uint32_t frameSeq = 0;
static SemaphoreHandle_t frameLock;

// The flash LED is a plain LEDC channel here; the Arduino build used analogWrite().
#define LED_CHANNEL LEDC_CHANNEL_2
#define LED_TIMER LEDC_TIMER_2
static bool ledReady = false;

void setLedDuty(int duty) {
  if (!ledReady || ledPin < 0) return;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL);
}

static void initLed() {
  if (ledPin < 0) return;
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_8_BIT;
  t.timer_num = LED_TIMER;
  t.freq_hz = 5000;
  t.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&t) != ESP_OK) return;
  ledc_channel_config_t c = {};
  c.gpio_num = ledPin;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.channel = LED_CHANNEL;
  c.timer_sel = LED_TIMER;
  c.duty = 0;
  if (ledc_channel_config(&c) == ESP_OK) ledReady = true;
}

// copies the latest frame into dst; returns its length (0 if none/too big) and seq
size_t copyFrame(uint8_t *dst, size_t cap, uint32_t *seq, uint16_t *w, uint16_t *h) {
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
uint8_t *dupFrame(size_t *len) {
  uint8_t *out = NULL;
  xSemaphoreTake(frameLock, portMAX_DELAY);
  if (sharedLen && (out = (uint8_t *)psAlloc(sharedLen)) != NULL) {
    memcpy(out, sharedBuf, sharedLen);
    *len = sharedLen;
  }
  xSemaphoreGive(frameLock);
  return out;
}

void captureTask(void *) {
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delayMs(50); continue; }
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
  c.frame_size = FRAMESIZE_UXGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    esp_camera_deinit();
    printf("  %-18s no (0x%x)\n", b.name, err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  sensorPid = s ? s->id.PID : 0;
  printf("  %-18s yes, sensor 0x%04x\n", b.name, sensorPid);
  return true;
}

// Try the remembered board first, then every other known layout.
bool initCamera() {
  frameLock = xSemaphoreCreateMutex();
  sharedBuf = (uint8_t *)psAlloc(FRAME_BUF_SIZE);
  if (!sharedBuf) {
    ESP_LOGE(TAG, "No PSRAM - this firmware needs a camera board with PSRAM");
    return false;
  }
  int saved = 0;
  if (!prefsGetInt("board", &saved) || saved < 0 || saved >= NUM_BOARDS) saved = 0;
  printf("Looking for the camera:\n");
  for (int attempt = 0; attempt < NUM_BOARDS && !cameraOk; attempt++) {
    int i = attempt == 0 ? saved : attempt - 1;
    if (attempt && i >= saved) i = attempt;  // skip the one already tried
    if (i >= NUM_BOARDS) break;
    if (tryBoard(BOARDS[i])) {
      boardIdx = i;
      ledPin = BOARDS[i].led;
      cameraOk = true;
      prefsSetInt("board", i);
    }
    delayMs(50);
  }
  if (!cameraOk) {
    printf("No supported camera found - check the ribbon cable is seated.\n");
    return false;
  }
  printf("Camera: %s board, sensor 0x%04x\n", BOARDS[boardIdx].name, sensorPid);
  initLed();
  loadSettings(esp_camera_sensor_get());
  return true;
}
