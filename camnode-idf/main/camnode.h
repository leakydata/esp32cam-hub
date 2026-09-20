// camnode (ESP-IDF): ESP32-CAM network camera.
//
// Ported from the Arduino sketch in ../camnode. The HTTP API is byte-for-byte the
// same so camhub and its web UI need no changes:
//   port 80  /  /capture  /status  /control  /reboot  /motion  /reg  /update
//   port 81  /stream
//   port 82  /recordings  /file  /delete  /timelapse  /tlframes  /tlframe
// Each port is its own server task, so a long stream or download never blocks the others.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define FW_VERSION "2.0.0"

#ifndef TZ_INFO
#define TZ_INFO "EST5EDT,M3.2.0,M11.1.0"  // POSIX TZ used for clip names
#endif

#define SD_MOUNT "/sdcard"
#define REC_DIR SD_MOUNT "/rec"
#define TL_DIR SD_MOUNT "/tl"
#define FRAME_BUF_SIZE (256 * 1024)  // largest JPEG we pass around (UXGA q10 is ~150KB)

#define MD_W 32  // motion grid
#define MD_H 24
#define MD_CELLS (MD_W * MD_H)

// ---- small helpers that stand in for the Arduino ones ----
static inline uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline void delayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }
static inline void *psAlloc(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  return p ? p : heap_caps_malloc(n, MALLOC_CAP_8BIT);
}

// ---- NVS-backed settings store (was Arduino Preferences) ----
#define PREFS_NS "camnode2"
bool prefsGetInt(const char *key, int *out);
void prefsSetInt(const char *key, int v);
void prefsGetStr(const char *key, char *out, size_t len);
void prefsSetStr(const char *key, const char *v);
bool prefsGetBlob(const char *key, void *out, size_t len);
void prefsSetBlob(const char *key, const void *v, size_t len);
void prefsClear();

// ---- settings table ----
struct Setting {
  const char *key;
  int (*apply)(sensor_t *, int);
  int lo, hi, def;
};
enum { S_FRAMESIZE, S_QUALITY, S_BRIGHTNESS, S_CONTRAST, S_SATURATION, S_EFFECT, S_WB, S_AE,
       S_GAIN, S_VFLIP, S_HMIRROR, S_LED, S_REC, S_REC_FPS, S_REC_MIN,
       S_MD, S_MD_SENS, S_MD_HOLD, S_PRE_SEC, S_TL, S_TL_SEC, S_FLICKER, S_RADIO };
enum { REC_OFF, REC_CONTINUOUS, REC_MOTION };
extern const Setting SETTINGS[];
extern const int NUM_SETTINGS;
extern int values[];
extern char label[33];
void loadSettings(sensor_t *s);
int setRadio(sensor_t *, int v);

// ---- camera ----
struct PinMap {
  const char *name;
  int8_t pin[16];  // pwdn reset xclk sda scl d7 d6 d5 d4 d3 d2 d1 d0 vsync href pclk
  int8_t led;      // flash LED, -1 if none
};
extern const PinMap BOARDS[];
extern const int NUM_BOARDS;
extern int boardIdx, ledPin;
extern bool cameraOk;
extern uint16_t sensorPid;
extern volatile uint32_t frameSeq;
bool initCamera();
void captureTask(void *);
size_t copyFrame(uint8_t *dst, size_t cap, uint32_t *seq, uint16_t *w = NULL, uint16_t *h = NULL);
uint8_t *dupFrame(size_t *len);
void setLedDuty(int duty);

// ---- SD card ----
extern volatile bool sdMounted;
extern volatile uint32_t sdTotalMB, sdUsedMB;
extern SemaphoreHandle_t sdLock;
static inline void sdTake() { if (sdLock) xSemaphoreTakeRecursive(sdLock, portMAX_DELAY); }
static inline void sdGive() { if (sdLock) xSemaphoreGiveRecursive(sdLock); }
struct FileEnt { std::string name; uint32_t size; };
bool mountSd();
void unmountSd();
void refreshSdUsage();
std::vector<std::string> listDir(const char *path, bool dirs);
std::vector<FileEnt> listFiles(const char *path);
void removeTree(const std::string &path);
void makeRoom();

// ---- AVI writer ----
extern FILE *aviFile;
extern uint32_t aviIdxCap, aviFrames, aviStartMs;
extern uint16_t aviW, aviH;
extern int aviFramesize;
extern bool aviHadMotion;
extern char activeFile[48];
extern volatile bool recording, pauseRec;
bool openSegment(uint16_t w, uint16_t h);
bool writeFrame(const uint8_t *jpg, size_t len);
void closeSegment();
void abortSegment();
void stopRecording();

// ---- recorder / motion / time-lapse ----
void recorderTask(void *);
void motionTask(void *);
void timelapseTask(void *);
extern uint8_t mdMask[MD_CELLS / 8], mdChanged[MD_CELLS / 8];
extern volatile bool motionActive;
extern volatile uint8_t motionLevel;
extern volatile uint32_t motionEvents, motionSeq;
extern volatile time_t motionLastStart;
extern volatile uint32_t tlFrames;
extern volatile time_t tlLast;
void maskToHex(const uint8_t *b, char *out);
bool hexToMask(const char *hex, uint8_t *b);

// ---- HTTP ----
extern char hostName[24];
void startServers();
esp_err_t addCors(httpd_req_t *req);
void urlDecode(const char *in, char *out, size_t outLen);
extern const httpd_uri_t *apiUris(int *n);
extern const httpd_uri_t *streamUris(int *n);
extern const httpd_uri_t *fileUris(int *n);
