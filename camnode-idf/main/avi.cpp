// Minimal MJPEG AVI writer: fixed 224-byte header rewritten on close, '00dc' chunks,
// idx1 index. Byte layout is unchanged from the Arduino build, so ffmpeg/VLC and the
// hub's re-streaming player read these clips the same way.
#include "camnode.h"

#include <sys/stat.h>
#include <sys/unistd.h>

#define AVI_HDR_LEN 224
#define AVI_MOVI_POS 220  // offset of the 'movi' fourcc; idx1 offsets are relative to it

FILE *aviFile = NULL;
static uint8_t *aviIdx = NULL;  // 16 bytes per frame, in PSRAM
uint32_t aviIdxCap = 0, aviFrames = 0, aviStartMs = 0;
static uint32_t aviMoviBytes = 0, aviMaxFrame = 0;
uint16_t aviW = 0, aviH = 0;
int aviFramesize = -1;
bool aviHadMotion = false;
char activeFile[48] = "";  // path relative to mount, e.g. /rec/20260919/154500.avi
volatile bool recording = false;
volatile bool pauseRec = false;  // set before reboot/OTA so the clip is finalised

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void putCC(uint8_t *p, const char *cc) { memcpy(p, cc, 4); }

// ask the recorder to finish the current clip and wait for it (max 2 s)
void stopRecording() {
  pauseRec = true;
  for (int i = 0; i < 40 && recording; i++) delayMs(50);
}

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

bool openSegment(uint16_t w, uint16_t h) {
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
  aviIdx = (uint8_t *)psAlloc(aviIdxCap * 16);
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
  const char *rel = path + strlen(SD_MOUNT);
  size_t rl = strnlen(rel, sizeof(activeFile) - 1);
  memcpy(activeFile, rel, rl);
  activeFile[rl] = 0;
  recording = true;
  printf("Recording %s\n", path);
  return true;
}

bool writeFrame(const uint8_t *jpg, size_t len) {
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

void closeSegment() {
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
  if (!ok) printf("Clip close failed: %s\n", activeFile);
  free(aviIdx);
  aviIdx = NULL;
  sdGive();
  if (aviHadMotion) {  // tag clips containing motion: 154500.avi -> 154500_M.avi
    char from[80], to[80];
    snprintf(from, sizeof(from), SD_MOUNT "%s", activeFile);
    snprintf(to, sizeof(to), "%s", from);
    char *dot = strrchr(to, '.');
    if (dot) { strcpy(dot, "_M.avi"); rename(from, to); }
  }
  printf("Closed %s: %lu frames, %lu s%s\n", activeFile, (unsigned long)aviFrames,
         (unsigned long)(duration / 1000), aviHadMotion ? ", motion" : "");
  activeFile[0] = 0;
  recording = false;
}

void abortSegment() {  // card pulled or write error
  if (aviFile) fclose(aviFile);
  aviFile = NULL;
  free(aviIdx);
  aviIdx = NULL;
  activeFile[0] = 0;
  recording = false;
}
