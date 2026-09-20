// Ports 80 (control API) and 81 (MJPEG stream).
//
// One difference from the Arduino build: OTA is a plain HTTP upload to POST /update
// instead of ArduinoOTA's UDP-invite protocol, so updates are a single curl and need
// no espota.py. update-all.sh tries this first and falls back to espota for cameras
// still running the old firmware.
#include "camnode.h"

#include "esp_app_desc.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

char hostName[24];

esp_err_t addCors(httpd_req_t *req) {
  return httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

void urlDecode(const char *in, char *out, size_t outLen) {
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

// ---- current network identity, for /status ----
static void localIp(char *out, size_t len) {
  esp_netif_ip_info_t ip = {};
  esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (nif) esp_netif_get_ip_info(nif, &ip);
  snprintf(out, len, IPSTR, IP2STR(&ip.ip));
}

static void macStr(char *out, size_t len) {
  uint8_t m[6] = {};
  esp_wifi_get_mac(WIFI_IF_STA, m);
  snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  char part[64];
  uint8_t *buf = (uint8_t *)psAlloc(FRAME_BUF_SIZE);
  if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
  addCors(req);
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  uint32_t lastSeq = 0;
  while (res == ESP_OK) {
    if (frameSeq == lastSeq) { delayMs(5); continue; }
    size_t len = copyFrame(buf, FRAME_BUF_SIZE, &lastSeq);
    if (!len) { delayMs(5); continue; }
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
  char json[1024], ip[20], mac[20];
  localIp(ip, sizeof(ip));
  macStr(mac, sizeof(mac));
  wifi_ap_record_t ap = {};
  esp_wifi_sta_get_ap_info(&ap);
  int n = snprintf(json, sizeof(json),
           "{\"name\":\"%s\",\"label\":\"%s\",\"version\":\"%s\",\"ip\":\"%s\",\"mac\":\"%s\","
           "\"ssid\":\"%s\",\"rssi\":%d,\"uptime\":%lu,\"heap\":%u,\"psram\":%u,\"psram_block\":%u,"
           "\"stream_port\":81,\"files_port\":82,\"time\":%lu,"
           "\"sd\":%d,\"sd_total_mb\":%lu,\"sd_used_mb\":%lu,\"recording\":%d,\"rec_file\":\"%s\","
           "\"motion\":%d,\"motion_level\":%d,\"motion_events\":%lu,\"motion_last\":%lu,"
           "\"tl_frames\":%lu,\"tl_last\":%lu,\"camera\":%d,\"board\":\"%s\",\"sensor\":%u",
           hostName, label, FW_VERSION, ip, mac, (const char *)ap.ssid, ap.rssi,
           (unsigned long)(millis() / 1000),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
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

static int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static esp_err_t controlHandler(httpd_req_t *req) {
  char query[512], val[200];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    return ESP_FAIL;
  }
  sensor_t *s = esp_camera_sensor_get();
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (httpd_query_key_value(query, SETTINGS[i].key, val, sizeof(val)) != ESP_OK) continue;
    int v = clampInt(atoi(val), SETTINGS[i].lo, SETTINGS[i].hi);
    SETTINGS[i].apply(s, v);
    values[i] = v;
    prefsSetInt(SETTINGS[i].key, v);
  }
  if (httpd_query_key_value(query, "mask", val, sizeof(val)) == ESP_OK && hexToMask(val, mdMask)) {
    prefsSetBlob("mask", mdMask, sizeof(mdMask));
  }
  if (httpd_query_key_value(query, "label", val, sizeof(val)) == ESP_OK) {
    char decoded[100];
    urlDecode(val, decoded, sizeof(decoded));
    for (char *c = decoded; *c; c++) if (*c == '"' || *c == '\\' || (uint8_t)*c < 0x20) *c = ' ';
    size_t ln = strnlen(decoded, sizeof(label) - 1);  // deliberate truncation
    memcpy(label, decoded, ln);
    label[ln] = 0;
    prefsSetStr("label", label);
  }
  if (httpd_query_key_value(query, "reset", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
    prefsClear();
    label[0] = 0;
    memset(mdMask, 0xff, sizeof(mdMask));
    for (int i = 0; i < NUM_SETTINGS; i++) {
      values[i] = SETTINGS[i].def;
      SETTINGS[i].apply(s, values[i]);
    }
  }
  return statusHandler(req);
}

static esp_err_t motionHandler(httpd_req_t *req) {
  char json[512], changed[MD_CELLS / 4 + 1], mask[MD_CELLS / 4 + 1];
  maskToHex(mdChanged, changed);
  maskToHex(mdMask, mask);
  snprintf(json, sizeof(json),
           "{\"active\":%d,\"level\":%d,\"events\":%lu,\"last\":%lu,\"seq\":%lu,\"enabled\":%d,"
           "\"w\":%d,\"h\":%d,\"changed\":\"%s\",\"mask\":\"%s\"}",
           motionActive ? 1 : 0, motionLevel, (unsigned long)motionEvents,
           (unsigned long)motionLastStart, (unsigned long)motionSeq, values[S_MD],
           MD_W, MD_H, changed, mask);
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
  snprintf(out, sizeof(out), "{\"addr\":%d,\"value\":%d}", addr, s ? s->get_reg(s, addr, 0xFF) : -1);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, out);
}

static esp_err_t rebootHandler(httpd_req_t *req) {
  addCors(req);
  httpd_resp_sendstr(req, "{\"rebooting\":true}");
  stopRecording();
  esp_restart();
  return ESP_OK;
}

// POST /update: raw firmware image in the body. Finish the current clip first so it
// stays playable, then write to the inactive OTA slot and reboot into it.
static esp_err_t updateHandler(httpd_req_t *req) {
  addCors(req);
  const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
  if (!target || req->content_len <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no image");
    return ESP_FAIL;
  }
  quiesceForOta();
  esp_ota_handle_t ota = 0;
  // OTA_WITH_SEQUENTIAL_WRITES erases the partition a block at a time as data arrives.
  // Passing the image size instead makes esp_ota_begin() erase all 1.875MB in one
  // blocking call, which starves the task watchdog and resets the chip mid-upload
  // ("rst:0x8 (TG1WDT_SYS_RESET)").
  if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
    pauseRec = false;
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed");
    return ESP_FAIL;
  }
  // Must be internal RAM. On the ESP32 PSRAM is reached through the same cache as
  // flash, so while esp_ota_write() has the cache off, touching a PSRAM buffer stalls
  // the CPU -- and SPIRAM_MALLOC_ALWAYSINTERNAL=4096 would put a plain malloc(4096)
  // exactly there.
  char *buf = (char *)heap_caps_malloc(4096, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) {
    esp_ota_abort(ota);
    pauseRec = false;
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    return ESP_FAIL;
  }
  int remaining = req->content_len;
  esp_err_t err = ESP_OK;
  while (remaining > 0 && err == ESP_OK) {
    int got = httpd_req_recv(req, buf, remaining < 4096 ? remaining : 4096);
    if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;  // slow link, not a failure
    if (got <= 0) { err = ESP_FAIL; break; }
    err = esp_ota_write(ota, buf, got);
    remaining -= got;
  }
  free(buf);
  if (err == ESP_OK) err = esp_ota_end(ota);
  else esp_ota_abort(ota);
  if (err == ESP_OK) err = esp_ota_set_boot_partition(target);
  if (err != ESP_OK) {
    pauseRec = false;
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota write failed");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}");
  delayMs(500);
  esp_restart();
  return ESP_OK;
}

static esp_err_t indexHandler(httpd_req_t *req) {
  char html[640];
  snprintf(html, sizeof(html),
           "<!doctype html><title>%s</title><body style='margin:0;background:#111;color:#ddd;"
           "font-family:sans-serif'><p style='padding:8px;margin:0'>%s &middot; %s &middot; v%s</p>"
           "<img id=v style='width:100%%;max-width:1024px;display:block'>"
           "<script>v.src=location.protocol+'//'+location.hostname+':81/stream'</script></body>",
           label[0] ? label : hostName, label[0] ? label : hostName, hostName, FW_VERSION);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_sendstr(req, html);
}

static const httpd_uri_t API[] = {
  {"/", HTTP_GET, indexHandler, NULL},
  {"/capture", HTTP_GET, captureHandler, NULL},
  {"/status", HTTP_GET, statusHandler, NULL},
  {"/control", HTTP_GET, controlHandler, NULL},
  {"/reboot", HTTP_GET, rebootHandler, NULL},
  {"/motion", HTTP_GET, motionHandler, NULL},
  {"/reg", HTTP_GET, regHandler, NULL},
  {"/update", HTTP_POST, updateHandler, NULL},
};
static const httpd_uri_t STREAM[] = {{"/stream", HTTP_GET, streamHandler, NULL}};

const httpd_uri_t *apiUris(int *n) { *n = sizeof(API) / sizeof(API[0]); return API; }
const httpd_uri_t *streamUris(int *n) { *n = sizeof(STREAM) / sizeof(STREAM[0]); return STREAM; }
