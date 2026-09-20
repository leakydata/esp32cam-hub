// Loop recorder: writes MJPEG AVI clips to the card, continuously or only while motion
// is active. In motion mode the last few seconds of frames are kept in a PSRAM ring so
// the clip starts before whatever triggered it.
#include "camnode.h"

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
  printf("%s\n", why);
  abortSegment();
  unmountSd();
}

void recorderTask(void *) {
  uint8_t *buf = (uint8_t *)psAlloc(FRAME_BUF_SIZE);
  ring = (uint8_t *)psAlloc(RING_BYTES);
  uint32_t lastSeq = 0, lastFrameMs = 0, lastMountTry = 0;
  for (;;) {
    if (!sdMounted) {
      if (millis() - lastMountTry > 30000 || lastMountTry == 0) {
        lastMountTry = millis();
        mountSd();
      }
      delayMs(500);
      continue;
    }
    int mode = cameraOk ? values[S_REC] : REC_OFF;
    if (mode == REC_OFF || pauseRec) {
      closeSegment();
      ringClear();
      delayMs(300);
      continue;
    }
    uint32_t interval = 1000 / values[S_REC_FPS];
    if (millis() - lastFrameMs < interval || frameSeq == lastSeq) {
      delayMs(5);
      continue;
    }
    uint16_t w = 0, h = 0;
    size_t len = copyFrame(buf, FRAME_BUF_SIZE, &lastSeq, &w, &h);
    if (!len) { delayMs(5); continue; }
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
