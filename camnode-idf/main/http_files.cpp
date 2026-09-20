// Port 82: browsing and downloading what is on the card. Its own server task so a
// multi-megabyte clip download never blocks the control API or the live stream.
#include "camnode.h"

#include <sys/stat.h>
#include <sys/unistd.h>

// path param must look like /rec/<day>/<file>.avi or /tl/...
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
  std::vector<std::string> days = listDir(REC_DIR, true);
  int count = 0;
  bool first = true;
  for (int d = days.size() - 1; d >= 0 && count < 2000; d--) {
    std::string dayPath = std::string(REC_DIR) + "/" + days[d];
    std::vector<FileEnt> files = listFiles(dayPath.c_str());
    for (int f = files.size() - 1; f >= 0 && count < 2000; f--, count++) {
      std::string rel = "/rec/" + days[d] + "/" + files[f].name;
      char item[160];
      int n = snprintf(item, sizeof(item), "%s{\"path\":\"%s\",\"size\":%u,\"active\":%s}",
                       first ? "" : ",", rel.c_str(), (unsigned)files[f].size,
                       rel == activeFile ? "true" : "false");
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
  char *buf = (char *)psAlloc(CHUNK);
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
  std::vector<std::string> days = listDir(TL_DIR, true);
  for (int d = days.size() - 1; d >= 0; d--) {
    std::string dayPath = std::string(TL_DIR) + "/" + days[d];
    std::string item = std::string(d == (int)days.size() - 1 ? "" : ",") +
                       "{\"day\":\"" + days[d] + "\",\"hours\":[";
    std::vector<std::string> hours = listDir(dayPath.c_str(), true);
    for (size_t h = 0; h < hours.size(); h++) {
      std::vector<std::string> frames = listDir((dayPath + "/" + hours[h]).c_str(), false);
      item += std::string(h ? "," : "") + "{\"hour\":\"" + hours[h] +
              "\",\"count\":" + std::to_string(frames.size()) + "}";
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
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) ||
      strncmp(rel, "/tl/", 4) != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send_chunk(req, "[", 1);
  std::vector<std::string> frames = listDir(full, false);
  std::string chunk;
  for (size_t i = 0; i < frames.size(); i++) {
    chunk += std::string(i ? ",\"" : "\"") + frames[i] + "\"";
    if (chunk.length() > 1500) {
      httpd_resp_send_chunk(req, chunk.c_str(), chunk.length());
      chunk.clear();
    }
  }
  chunk += "]";
  httpd_resp_send_chunk(req, chunk.c_str(), chunk.length());
  return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t tlFrameHandler(httpd_req_t *req) {
  char full[160], rel[120];
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) ||
      strncmp(rel, "/tl/", 4) != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad path");
    return ESP_FAIL;
  }
  FILE *f = fopen(full, "rb");
  if (!f) { httpd_resp_send_404(req); return ESP_FAIL; }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = (char *)psAlloc(size > 0 ? size : 1);
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
  if (!sdMounted || !getRecPath(req, full, sizeof(full), rel, sizeof(rel)) ||
      strcmp(rel, activeFile) == 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "cannot delete");
    return ESP_FAIL;
  }
  bool ok;
  if (strncmp(rel, "/tl/", 4) == 0) {
    // time-lapse: a day (/tl/YYYYMMDD) or an hour (/tl/YYYYMMDD/HH)
    struct stat st;
    ok = stat(full, &st) == 0 && S_ISDIR(st.st_mode);
    if (ok) {
      std::vector<std::string> hours = listDir(full, true);
      if (hours.empty()) removeTree(full);
      else for (const std::string &h : hours) removeTree(std::string(full) + "/" + h);
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

static const httpd_uri_t FILES[] = {
  {"/recordings", HTTP_GET, recordingsHandler, NULL},
  {"/file", HTTP_GET, fileHandler, NULL},
  {"/delete", HTTP_GET, deleteHandler, NULL},
  {"/timelapse", HTTP_GET, timelapseHandler, NULL},
  {"/tlframes", HTTP_GET, tlFramesHandler, NULL},
  {"/tlframe", HTTP_GET, tlFrameHandler, NULL},
};

const httpd_uri_t *fileUris(int *n) { *n = sizeof(FILES) / sizeof(FILES[0]); return FILES; }
