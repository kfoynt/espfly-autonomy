/*
 * Host harness for components/core/crazyflie/modules/src/auto_link.c: stubs the
 * IDF / drone APIs (stub/), captures sendto, recvfrom and setsockopt, and plays
 * the stabilizer through a fake autonomy.h built on the step-0 mailbox
 * (components/core/crazyflie/utils/src/auto_mailbox.c, compiled in: the same
 * post / take / publish / cancel code as the drone).
 *
 * Stage A1: the HELLO flow, sessions and duplicates, the rate limit, a 2-frame
 * photo with ENOMEM pacing, CRC against zlib, CHUNKS re-sends, free and
 * auto-free, and a missing frame. Colour photos (2026-10-15): the 44-byte
 * META with the pixel format, 38 chunks of the YUYV frame, CHUNKS limits per
 * format, the grey META unchanged.
 * Always-on colour (2026-10-16, docs/autonomy-a2b.md 28): the camera stub runs the split's snapshot core
 * (cam_split_core.c, compiled in) with the driver's chunk copies: a 2-frame colour set, and the 1000 ms frame timeout
 * cancelling a capture stalled mid-frame leaves the slot untouched by the frame's late chunks.
 * Stage A2b (docs/autonomy-a2b.md section 3): KILL (any session and seq, its
 * own budget, answered at the sender, never a peer change), asynchronous
 * commands and the three BUSY kinds, ACK byte 6 and STATUS byte 7,
 * AUTO_STATUS replies and pushes, PHOTO routing and the held-set BUSY, the
 * state machine's photo sets (inside the take-off window), SO_RCVTIMEO, push
 * pacing while a command is pending, and the PSRAM task stack.
 * Review fixes: the KILL payload limit (32 bytes), AUTO_STATUS to a non-peer
 * never uses up the peer's push, the replayed-HELLO peer move, the forged
 * KILL flood (accepted risk), PHOTO OK detail 0 when no set starts (camera
 * down, no PSRAM slots), the BUSY detail kinds (integrator decision D1), and
 * KILL latency while a photo set is pushed.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=gnu99 -w -Itools/tests/auto_link_host/stub -Icomponents/core/crazyflie/utils/interface \
 *      -Icomponents/core/crazyflie/modules/interface -Icomponents/core/crazyflie/modules/src \
 *      -Icomponents/core/crazyflie/hal/interface \
 *      tools/tests/auto_link_host/test_auto_link_host.c components/core/crazyflie/utils/src/auto_proto.c \
 *      components/core/crazyflie/utils/src/auto_mailbox.c components/core/crazyflie/utils/src/cam_split_core.c \
 *      -lz -o /tmp/test_auto_link_host && /tmp/test_auto_link_host
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <zlib.h>

#include "cam_split_core.h"

static ssize_t stubSendto(int s, const void *buf, size_t len, int flags, const struct sockaddr *to, socklen_t tolen);
static ssize_t stubRecvfrom(int s, void *buf, size_t len, int flags, struct sockaddr *from, socklen_t *fromLen);
static int stubSetsockopt(int s, int level, int name, const void *v, socklen_t len);
#define sendto stubSendto
#define recvfrom stubRecvfrom
#define setsockopt stubSetsockopt
#include "auto_link.c"
#undef sendto
#undef recvfrom
#undef setsockopt

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- stubs ---- */
static int64_t fakeUs = 5000000;
void vTaskDelay(TickType_t t) { fakeUs += (int64_t)t * 1000; }
static int dynCalls, staticCalls;
static uint32_t lastDynDepth, lastStaticDepth;
static StackType_t *lastStaticStack;
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int pri,
                                   TaskHandle_t *h, int core)
{
  dynCalls++;
  lastDynDepth = stack;
  *h = (TaskHandle_t)&dynCalls;
  return pdPASS;
}
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int pri,
                                           StackType_t *stackBuf, StaticTask_t *tcb, int core)
{
  staticCalls++;
  lastStaticDepth = stack;
  lastStaticStack = stackBuf;
  return stackBuf && tcb ? (TaskHandle_t)tcb : NULL;
}
static bool psramFail;
typedef struct { size_t n; int caps; void *p; } allocRec_t;
static allocRec_t allocs[64];
static int nAllocs;
void *heap_caps_malloc(size_t n, int caps)
{
  void *p = ((caps & MALLOC_CAP_SPIRAM) && psramFail) ? NULL : malloc(n);
  if (nAllocs < 64) allocs[nAllocs++] = (allocRec_t){n, caps, p};
  return p;
}
static int capsOf(const void *p) { for (int i = nAllocs - 1; i >= 0; i--) if (allocs[i].p == p) return allocs[i].caps; return -1; }
static esp_app_desc_t app = {.version = "f7e67fb-dirty"};
const esp_app_desc_t *esp_app_get_description(void) { return &app; }
static uint32_t rnd = 12345;
uint32_t esp_random(void) { rnd = rnd * 1103515245u + 12345u; return rnd; }
void esp_fill_random(void *b, size_t n) { for (size_t i = 0; i < n; i++) ((uint8_t *)b)[i] = (uint8_t)esp_random(); }
int64_t esp_timer_get_time(void) { return fakeUs; }
static flightRecord_t rec = {.thrust = 0, .roll = -150, .pitch = 220, .yaw = 9000, .vbatMv = 3870};
static bool haveRec = true;
bool flightRecorderGetLatest(flightRecord_t *o) { *o = rec; return haveRec; }
static uint8_t testKey[16];
esp_err_t autoKeyStoreLoad(uint8_t k[16]) { memcpy(k, testKey, 16); return ESP_OK; }
int motorsGetRatio(uint32_t id) { return 0; }
static bool takeoffWin;
bool stabilizerTakeoffWindow(void) { return takeoffWin; }
bool stabilizerOnGround(void) { return true; }
/* camera */
static cameraFlowState_t camState = CAMERA_FLOW_RUNNING;
static int snap;   /* 0 idle 1 requested 2 filled */
static uint8_t *snapB;
static cameraFlowSnapshot_t *snapM;
static uint32_t camSeq = 100;
static bool camColour;   /* the camera of a photocol 1 boot: YUYV snapshots (camera_flow.c snapshotFill) */
bool cameraFlowGetLatest(cameraFlowSample_t *o) { memset(o, 0, sizeof(*o)); o->lum = 97; o->texture = 31.4f; return true; }
cameraFlowState_t cameraFlowState(void) { return camState; }
float cameraFlowFps(void) { return 15.96f; }
void cameraFlowStats(uint32_t *f, uint32_t *t, uint32_t *r, uint32_t *a) { *f = camSeq; *t = 0; *r = 0; *a = 10; }
/* the colour split (camera_flow.c with photocol 1): the snapshot core, the driver's chunks played by splitFrame() */
static bool camSplit;
static camSplit_t split;
bool cameraFlowSnapshotRequest(uint8_t *b, cameraFlowSnapshot_t *m)
{
  if (camSplit) {
    if (camState != CAMERA_FLOW_RUNNING || camSplitPending(&split)) return false;
    snapM = m;
    return camSplitRequest(&split, b);
  }
  if (camState != CAMERA_FLOW_RUNNING || snap != 0) return false;
  snapB = b; snapM = m; snap = 1; return true;
}
bool cameraFlowSnapshotTake(void)
{
  if (camSplit) return camSplitTake(&split);
  if (snap != 2) return false;
  snap = 0; return true;
}
bool cameraFlowSnapshotCancel(void)
{
  if (camSplit) return camSplitCancel(&split);
  snap = 0; return true;
}
/* ground standby (docs/autonomy-a2b.md 17.1): asleep until a wake request, awake ~20 ms later */
static bool gndAsleep;          /* the stub's ground standby */
static bool gndNeverWakes;      /* the wake write never lands */
static int gndWakes;            /* cameraFlowGroundWake() calls */
static uint32_t gndWakeHold;
static int64_t gndWakeAtUs = -1;
static uint32_t camFresh = 1000;   /* frames since the latest wake */
static uint32_t camSeqAtWake;
bool cameraFlowGroundStandby(void) { return gndAsleep; }
void cameraFlowGroundWake(uint32_t holdMs)
{
  gndWakes++;
  gndWakeHold = holdMs;
  if (gndAsleep && gndWakeAtUs < 0 && !gndNeverWakes) gndWakeAtUs = fakeUs + 20000;
}
uint32_t cameraFlowGroundHoldMs(void) { return 45000; }
bool cameraFlowAwakeFresh(uint32_t n) { return camState == CAMERA_FLOW_RUNNING && camFresh >= n; }
/* The split's driver side: frame f's YUYV (Y a pattern, the chroma bytes per frame), chunks [from, to) of it into
 * frame buffer idx through the ring's half buffers, as cam_hal.c copies them. */
static uint8_t splitFb[2][19200];
static uint64_t splitTs[2];
static uint8_t splitRing[4 * 3840];
static uint32_t splitRingPos;
static uint8_t splitYuyv(uint32_t f, uint32_t p, bool chroma)
{
  const uint32_t x = (p / 2) % 160, y = (p / 2) / 160;
  return chroma ? (uint8_t)(((p / 2) & 1 ? 200 - y : 40 + y) + f) : (uint8_t)(x * 7 + y * 3);
}
static void splitChunks(uint32_t f, int idx, int from, int to)
{
  if (from == 0) splitTs[idx] = (uint64_t)fakeUs;
  for (int k = from; k < to; k++) {
    uint8_t *half = splitRing + (splitRingPos++ % 4) * 3840;
    for (uint32_t i = 0; i < 3840; i++) half[i] = splitYuyv(f, k * 3840 + i, i & 1);
    uint8_t *out = splitFb[idx] + k * 1920;
    for (uint32_t i = 0; i < 1920; i++) out[i] = half[2 * i];   /* the driver's own Y copy */
    const uint8_t *bufs[2] = {splitFb[0], splitFb[1]};
    camSplitStep(&split, bufs, splitTs, 2, 19200, out, half, 3840);
  }
}
static int splitStallAt = -1;   /* the next frame stops after this many chunks (the camera stalls) */
static uint32_t splitFrameNo, splitStalledFrame;
static int splitStalledIdx;
/* one frame of the split camera: the driver's chunks, then the flow (camera_flow.c snapshotFillSplit) */
static void splitFrame(void)
{
  const int idx = (int)(splitFrameNo & 1);
  const uint32_t f = ++splitFrameNo;
  if (splitStallAt >= 0) {
    splitChunks(f, idx, 0, splitStallAt);
    splitStalledFrame = f;
    splitStalledIdx = idx;
    camState = CAMERA_FLOW_RESTARTING;   /* no more frames until the test resumes it */
    return;
  }
  splitChunks(f, idx, 0, 10);
  if (camSplitFlowFrame(&split, splitFb[idx], splitTs[idx]) == CAM_SPLIT_FLOW_FILL) {
    snapM->seq = camSeq; snapM->frameTimeUs = splitTs[idx]; snapM->lum = 97; snapM->texture = 31.4f;
    snapM->pixFmt = CAMERA_FLOW_PIXFMT_YUYV;
    snapM->bytes = CAMERA_FLOW_SNAPSHOT_MAX_BYTES;
    camSplitFilled(&split);
  }
}
static void camTick(void)
{
  camSeq++;
  if (camSplit) {
    if (camState == CAMERA_FLOW_RUNNING) splitFrame();
    return;
  }
  if (gndAsleep && gndWakeAtUs >= 0 && fakeUs >= gndWakeAtUs) {
    gndAsleep = false;
    gndWakeAtUs = -1;
    camState = CAMERA_FLOW_RUNNING;
    camFresh = 0;
    camSeqAtWake = camSeq;
  }
  if (camState == CAMERA_FLOW_RUNNING && camFresh < 1000) camFresh++;
  if (snap == 1 && camState == CAMERA_FLOW_RUNNING) {
    for (int y = 0; y < 120; y++)
      for (int x = 0; x < 160; x++) {
        const uint8_t g = (uint8_t)((x * 7 + y * 3 + (camSeq - 101) * 0) & 0xFF);
        if (camColour) {   /* Y the grey pattern, U on the even bytes after it, V on the odd */
          snapB[(y * 160 + x) * 2] = g;
          snapB[(y * 160 + x) * 2 + 1] = (uint8_t)(x & 1 ? 200 - y : 40 + y);
        } else {
          snapB[y * 160 + x] = g;
        }
      }
    snapM->seq = camSeq; snapM->frameTimeUs = (uint64_t)fakeUs - 20000; snapM->lum = 97; snapM->texture = 31.4f;
    snapM->pixFmt = camColour ? CAMERA_FLOW_PIXFMT_YUYV : CAMERA_FLOW_PIXFMT_GRAY8;
    snapM->bytes = camColour ? CAMERA_FLOW_SNAPSHOT_MAX_BYTES : CAMERA_FLOW_SNAPSHOT_BYTES;
    snap = 2;
  }
}

/* ---- fake autonomy.h: the step-0 mailbox, the test plays the stabilizer ---- */
static autoMailbox_t mb;
static autoState_t fakeState = AUTO_IDLE;
static uint32_t fakeStatusSeq;
static autoStatusMsg_t fakeStatus;
static bool bumpOnGet;          /* a transition between the link's seq check and its read */
static bool stubLike;           /* as the step-0 stub: every post answered BAD_STATE at once */
static int killCalls, sessionNotes, hbNotes, sendCalls, sendsAtKill;
static int64_t killUs;          /* fake time of the last autonomyKillFromLink() */
static bool pushingAtKill;      /* a photo set was being captured or pushed then */
static bool camAwake;
static bool capPending;
static uint32_t capReqId;
static uint8_t capN;
static uint16_t capGap;
typedef struct { uint32_t reqId; autoCapture_t progress; uint16_t photoId; } capNote_t;
static capNote_t capNotes[64];
static int nCapNotes;

bool autonomyPostCommand(const autoCmd_t *cmd, uint32_t *ticket)
{
  const bool ok = autoMbPost(&mb, cmd, ticket) == AUTO_MB_POSTED;
  if (ok && stubLike) {
    autoCmd_t c; uint32_t t; bool fb;
    if (autoMbTake(&mb, &c, &t, &fb)) autoMbPublish(&mb, t, AUTO_R_BAD_STATE, 0u, AUTO_IDLE);
  }
  return ok;
}
bool autonomyTakeResult(uint32_t ticket, autoCmdResult_t *out) { return autoMbTakeResult(&mb, ticket, out); }
autoMbCancel_t autonomyCancelCommand(uint32_t ticket) { return autoMbCancel(&mb, ticket); }
void autonomyKillFromLink(void) { killCalls++; sendsAtKill = sendCalls; killUs = fakeUs; pushingAtKill = photoActive(); }
void autonomyNoteSession(void) { sessionNotes++; }
void autonomyNoteHeartbeat(void) { hbNotes++; }
bool autonomyTakeCaptureRequest(uint32_t *reqId, uint8_t *n, uint16_t *gapMs)
{
  if (!capPending) return false;
  capPending = false;
  *reqId = capReqId; *n = capN; *gapMs = capGap;
  return true;
}
void autonomyNoteCapture(uint32_t reqId, autoCapture_t progress, uint16_t photoId)
{
  if (nCapNotes < 64) capNotes[nCapNotes++] = (capNote_t){reqId, progress, photoId};
}
bool autonomyCameraAwake(void) { return camAwake; }
bool autonomyActive(void) { return fakeState != AUTO_IDLE; }
autoState_t autonomyState(void) { return fakeState; }
uint32_t autonomyStatusSeq(void) { return fakeStatusSeq; }
uint32_t autonomyGetStatus(autoStatusMsg_t *out)
{
  if (bumpOnGet) {
    bumpOnGet = false;
    fakeStatusSeq++;
    fakeStatus.transitions = fakeStatusSeq;
  }
  *out = fakeStatus;
  out->bootNonce = 0;
  return fakeStatusSeq;
}

static autoCmd_t lastTaken;
static uint32_t takenTicket;
/* One stabilizer step: takes the command and publishes this result. */
static bool stabStep(autoResult_t res, uint32_t detail, autoState_t state)
{
  autoCmd_t c; bool fb;
  if (!autoMbTake(&mb, &c, &takenTicket, &fb)) return false;
  lastTaken = c;
  return autoMbPublish(&mb, takenTicket, res, detail, state);
}
/* A stalled stabilizer that took the command and never answers. */
static bool stabTakeOnly(void) { autoCmd_t c; bool fb; return autoMbTake(&mb, &c, &takenTicket, &fb); }
/* PHOTO accepted: the set is requested in the step that answers. */
static bool stabPhotoOk(uint32_t reqId, uint8_t n, uint16_t gap)
{
  autoCmd_t c; bool fb;
  if (!autoMbTake(&mb, &c, &takenTicket, &fb)) return false;
  lastTaken = c;
  capPending = true; capReqId = reqId; capN = n; capGap = gap;
  return autoMbPublish(&mb, takenTicket, AUTO_R_OK, (uint32_t)fakeState, fakeState);
}
static int notesFor(uint32_t reqId, autoCapture_t p)
{
  int c = 0;
  for (int i = 0; i < nCapNotes; i++) c += capNotes[i].reqId == reqId && capNotes[i].progress == p;
  return c;
}
static const capNote_t *lastNote(void) { return nCapNotes ? &capNotes[nCapNotes - 1] : NULL; }

/* ---- sockets: capture sendto, queue datagrams for recvfrom ---- */
typedef struct { autoProtoHdr_t h; uint8_t p[AUTO_PROTO_MAX_PAYLOAD]; struct sockaddr_in to; int64_t us; } pkt_t;
static pkt_t cap[4096];
static int ncap;
static int nomemEvery;   /* every Nth send fails with ENOMEM */

static void hm(const uint8_t *k, size_t kl, const uint8_t *d, size_t n, uint8_t o[32]) { CCHmac(kCCHmacAlgSHA256, k, kl, d, n, o); }

static ssize_t stubSendto(int s, const void *buf, size_t len, int flags, const struct sockaddr *to, socklen_t tolen)
{
  sendCalls++;
  if (nomemEvery && sendCalls % nomemEvery == 0) { errno = ENOMEM; return -1; }
  autoProtoHdr_t h; const uint8_t *p;
  if (autoProtoDecode(buf, len, testKey, hm, &h, &p) != AUTO_DEC_OK) { printf("drone sent undecodable packet\n"); failures++; return (ssize_t)len; }
  if (ncap < 4096) {
    cap[ncap].h = h; memcpy(cap[ncap].p, p, h.len);
    memcpy(&cap[ncap].to, to, sizeof(struct sockaddr_in));
    cap[ncap].us = fakeUs;
    ncap++;
  }
  return (ssize_t)len;
}

/* atUs: the datagram arrives at that fake time (0: already there). */
typedef struct { uint8_t b[128]; size_t n; struct sockaddr_in from; int64_t atUs; } dgram_t;
#define RXQ 512
static dgram_t rxq[RXQ];
static int rxHead, rxTail, recvCalls, recvBlocking;
static ssize_t stubRecvfrom(int s, void *buf, size_t len, int flags, struct sockaddr *from, socklen_t *fromLen)
{
  recvCalls++;
  if (!(flags & MSG_DONTWAIT)) recvBlocking++;
  if (rxHead == rxTail || rxq[rxHead % RXQ].atUs > fakeUs) {
    bool arrived = false;
    if (!(flags & MSG_DONTWAIT)) {
      /* Blocks until the next datagram arrives, or to the timeout. */
      const int64_t until = fakeUs + (int64_t)rxTimeoutMs * 1000;
      arrived = rxHead != rxTail && rxq[rxHead % RXQ].atUs <= until;
      fakeUs = arrived ? rxq[rxHead % RXQ].atUs : until;
    }
    if (!arrived) {
      errno = EAGAIN;
      return -1;
    }
  }
  const dgram_t *d = &rxq[rxHead++ % RXQ];
  const size_t n = d->n < len ? d->n : len;
  memcpy(buf, d->b, n);
  memcpy(from, &d->from, sizeof(d->from));
  *fromLen = sizeof(d->from);
  return (ssize_t)n;
}
static int rcvTimeoutSets, lastRcvTimeoutMs = -1;
static int stubSetsockopt(int s, int level, int name, const void *v, socklen_t len)
{
  if (level == SOL_SOCKET && name == SO_RCVTIMEO) {
    const struct timeval *tv = v;
    lastRcvTimeoutMs = (int)(tv->tv_sec * 1000 + tv->tv_usec / 1000);
    rcvTimeoutSets++;
  }
  return 0;
}

/* Mac A (the daemon), the kill client B, another station C. */
static struct sockaddr_in addrOf(uint32_t ip, uint16_t port)
{
  struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(port)};
  a.sin_addr.s_addr = htonl(ip);
  return a;
}
#define ADDR_A addrOf(0xC0A82B2C, 50123)
#define ADDR_B addrOf(0xC0A82B03, 60000)
#define ADDR_C addrOf(0xC0A82B63, 40000)
static bool sameAddr(const struct sockaddr_in *x, struct sockaddr_in y)
{
  return x->sin_addr.s_addr == y.sin_addr.s_addr && x->sin_port == y.sin_port;
}

static int encodeMac(uint8_t *b, uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq, const uint8_t *pl, uint16_t len)
{
  autoProtoHdr_t h = {.type = type, .flags = flags, .session = sess, .seq = seq, .len = len};
  return autoProtoEncode(&h, pl, testKey, hm, b, AUTO_PROTO_MAX_PACKET);
}
static void macSendFrom(struct sockaddr_in from, uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq,
                        const uint8_t *pl, uint16_t len)
{
  uint8_t b[AUTO_PROTO_MAX_PACKET];
  const int n = encodeMac(b, type, flags, sess, seq, pl, len);
  handleDatagram(b, (size_t)n, &from);
}
static void macSend(uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq, const uint8_t *pl, uint16_t len)
{
  macSendFrom(ADDR_A, type, flags, sess, seq, pl, len);
}
/* Into the receive queue: the link task reads it in linkPass() (and
 * truncates it to AUTO_RX_BUF_LEN, as lwIP does), from atUs on. */
static void macQueueAt(struct sockaddr_in from, int64_t atUs, uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq,
                       const uint8_t *pl, uint16_t len)
{
  uint8_t b[AUTO_PROTO_MAX_PACKET];
  const int n = encodeMac(b, type, flags, sess, seq, pl, len);
  dgram_t *d = &rxq[rxTail++ % RXQ];
  if (n < 0 || (size_t)n > sizeof(d->b)) { printf("test datagram too long\n"); failures++; d->n = 0; return; }
  memcpy(d->b, b, (size_t)n);
  d->n = (size_t)n;
  d->from = from;
  d->atUs = atUs;
}
static void macQueueFrom(struct sockaddr_in from, uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq,
                         const uint8_t *pl, uint16_t len)
{
  macQueueAt(from, 0, type, flags, sess, seq, pl, len);
}
static void macQueue(uint8_t type, uint8_t flags, uint32_t sess, uint32_t seq, const uint8_t *pl, uint16_t len)
{
  macQueueFrom(ADDR_A, type, flags, sess, seq, pl, len);
}

static pkt_t *lastOf(uint8_t type) { for (int i = ncap - 1; i >= 0; i--) if (cap[i].h.type == type) return &cap[i]; return NULL; }
static int countOf(uint8_t type) { int c = 0; for (int i = 0; i < ncap; i++) c += cap[i].h.type == type; return c; }
static int countSince(int from, uint8_t type) { int c = 0; for (int i = from; i < ncap; i++) c += cap[i].h.type == type; return c; }
/* The ACK of seq sent since capture index from, or NULL. */
static pkt_t *ackOf(uint32_t seq, int from)
{
  for (int i = ncap - 1; i >= from; i--) if (cap[i].h.type == AUTO_T_ACK && autoGetU32(cap[i].p) == seq) return &cap[i];
  return NULL;
}
static bool ackCached(uint32_t seq)
{
  for (int i = 0; i < AUTO_PROTO_SEQ_RECENT; i++) if (ackCache[i].valid && ackCache[i].seq == seq) return true;
  return false;
}

static void runLoop(int ms)
{
  for (int t = 0; t < ms; t += AUTO_PUSH_GAP_MS) {
    if (t % 60 == 0) camTick();
    photoStep(nowMs());
    vTaskDelay(AUTO_PUSH_GAP_MS);
  }
}

/* Task loop passes for at least ms of fake time; the camera delivers a frame
 * every 60 ms. */
static uint8_t rxBuf[AUTO_RX_BUF_LEN];
static int64_t camNextUs;
static void pass(void)
{
  if (fakeUs >= camNextUs) { camTick(); camNextUs = fakeUs + 60000; }
  linkPass(rxBuf);
}
static void runPasses(int ms)
{
  const int64_t end = fakeUs + (int64_t)ms * 1000;
  while (fakeUs < end) pass();
}

static uint32_t macSeq;
static uint32_t helloNonceNext = 0x70000000;
static uint32_t newSession(void)
{
  uint8_t pl[4];
  autoPutU32(pl, helloNonceNext++);
  macSeq = 1;
  macSend(AUTO_T_HELLO, 0, 0, macSeq, pl, 4);
  return session;
}

static void freeHeld(uint32_t sess)
{
  if (!photo.held) return;
  uint8_t pl[2];
  autoPutU16(pl, photo.id);
  macSend(AUTO_T_PHOTO_FREE, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 2);
}

/* ---- stage A1 (unchanged behaviour) ---- */
static void testA1(void)
{
  /* Before HELLO: BAD_SESSION */
  uint8_t pl[16] = {0};
  int n0 = ncap;
  macSend(AUTO_T_STATUS_REQ, 0, 0x1234, 1, NULL, 0);
  CHECK(ncap == n0 + 1 && cap[n0].h.type == AUTO_T_ACK && cap[n0].p[5] == AUTO_R_BAD_SESSION &&
        cap[n0].h.session == 0x1234 && sameAddr(&cap[n0].to, ADDR_A), "no session -> BAD_SESSION in sender's session");

  /* HELLO */
  autoPutU32(pl, 0x11223344);
  macSend(AUTO_T_HELLO, 0, 0, 1, pl, 4);
  pkt_t *ha = lastOf(AUTO_T_HELLO_ACK);
  CHECK(ha && ha->h.len == 40 && ha->h.seq == 1, "hello_ack");
  const uint32_t sess = ha ? autoGetU32(ha->p) : 0;
  CHECK(sess != 0 && ha->h.session == sess && autoGetU32(ha->p + 4) == bootNonce && autoGetU32(ha->p + 8) == 0x11223344 &&
        memcmp(ha->p + 12, keyFp, 4) == 0 && strcmp((char *)ha->p + 16, "f7e67fb-dirty") == 0, "hello_ack fields");
  CHECK(sessionNotes == 1, "a new session is noted for the state machine");
  /* HELLO repeat: same session */
  macSend(AUTO_T_HELLO, AUTO_FLAG_RETX, 0, 1, pl, 4);
  CHECK(autoGetU32(lastOf(AUTO_T_HELLO_ACK)->p) == sess && session == sess, "hello repeat keeps the session");
  CHECK(sessionNotes == 1, "a replayed HELLO is not a new session");

  /* STATUS (then AUTO_STATUS) */
  n0 = ncap;
  macSend(AUTO_T_STATUS_REQ, 0, sess, 2, NULL, 0);
  pkt_t *st = lastOf(AUTO_T_STATUS);
  CHECK(st && st->h.len == 48 && autoGetU16(st->p + 4) == 3870 && st->p[6] == CAMERA_FLOW_RUNNING &&
        st->p[7] == AUTO_IDLE && autoGetU16(st->p + 8) == 160 && (int16_t)autoGetU16(st->p + 16) == -150 &&
        st->p[22] == 97 && st->p[23] == 31 && st->p[24] == 0x05 && st->p[25] == 3 && autoGetU32(st->p + 32) == 27000 &&
        autoGetU32(st->p + 44) == bootNonce, "status fields (flags %02x)", st ? st->p[24] : 0);
  CHECK(ncap == n0 + 2 && cap[n0].h.type == AUTO_T_STATUS && cap[n0 + 1].h.type == AUTO_T_AUTO_STATUS,
        "STATUS_REQ: STATUS, then AUTO_STATUS");

  /* HB, no ack */
  int before = ncap;
  autoPutU32(pl, 123456);
  macSend(AUTO_T_HB, 0, sess, 3, pl, 4);
  CHECK(ncap == before && haveHb && hbNotes == 1, "hb recorded and noted, no reply");

  /* Unknown types, a malformed YAW (the interactive hover's: 4 bytes) and a malformed ARM */
  /* (0x33 was the unknown type here until SET_HEIGHT took it on 2026-10-15: an older drone answers it this way) */
  macSend(0x34, AUTO_FLAG_ACKREQ, sess, 4, NULL, 0);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_BAD_STATE && lastOf(AUTO_T_ACK)->p[4] == 0x34 &&
        autoGetU32(lastOf(AUTO_T_ACK)->p + 8) == 0u, "unknown 0x34 -> BAD_STATE detail 0");
  before = ncap;
  macSend(0x34, 0, sess, 5, NULL, 0);
  CHECK(ncap == before, "unknown without ACKREQ: silent");

  /* PHOTO range */
  pl[0] = 4; pl[1] = 0; autoPutU16(pl + 2, 300);
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 6, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "n=4 RANGE");
  /* PHOTO in take-off window */
  takeoffWin = true;
  pl[0] = 2;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 7, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_TAKEOFF_WINDOW, "TAKEOFF_WINDOW");
  takeoffWin = false;
  /* the take-off standby (not the ground standby): CAM_UNAVAIL as before */
  camState = CAMERA_FLOW_STANDBY;
  const int wakes0 = gndWakes;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 8, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_CAM_UNAVAIL && gndWakes == wakes0, "take-off standby: CAM_UNAVAIL");
  camState = CAMERA_FLOW_RUNNING;

  /* PHOTO ok, n=2 gap 300 */
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 9, pl, 4);
  pkt_t *ack = lastOf(AUTO_T_ACK);
  CHECK(ack->p[5] == AUTO_R_OK && autoGetU32(ack->p + 8) == 1 && autoGetU32(ack->p) == 9, "PHOTO OK id 1");
  CHECK(gndWakes == wakes0 + 1 && gndWakeHold == 45000, "an awake camera's hold extended by the PHOTO");
  /* duplicate: cached ack, no new photo */
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, sess, 9, pl, 4);
  ack = lastOf(AUTO_T_ACK);
  CHECK(ack->p[5] == AUTO_R_OK && autoGetU32(ack->p + 8) == 1 && photosAccepted == 1, "dup -> cached ack");
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 10, pl, 4);
  ack = lastOf(AUTO_T_ACK);
  CHECK(ack->p[5] == AUTO_R_BUSY && autoGetU32(ack->p + 8) == (AUTO_DETAIL_BUSY_HELD_SET | 1) &&
        (autoGetU32(ack->p + 8) & 0xFFFF) == 1, "BUSY, detail HELD_SET | held id (low 16 bits the id, as A1)");
  /* old seq dropped */
  before = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 2, pl, 4);
  CHECK(ncap == before && oldSeqs == 1, "old seq dropped");

  nomemEvery = 7;
  int t0 = ncap;
  runLoop(3000);
  nomemEvery = 0;
  int metas = 0, chunks = 0;
  static uint8_t img[2][19200];
  uint32_t crcs[2] = {0}, seqs[2] = {0}, capMs[2] = {0};
  for (int i = t0; i < ncap; i++) {
    if (cap[i].h.type == AUTO_T_META) {
      metas++;
      const uint8_t *p = cap[i].p;
      uint8_t idx = p[2];
      crcs[idx] = autoGetU32(p + 12); seqs[idx] = autoGetU32(p + 16); capMs[idx] = autoGetU32(p + 20);
      CHECK(autoGetU16(p) == 1 && p[3] == 2 && autoGetU16(p + 4) == 160 && autoGetU16(p + 6) == 120 &&
            autoGetU32(p + 8) == 19200 && (int16_t)autoGetU16(p + 24) == -150 && autoGetU16(p + 34) == 1024 &&
            autoGetU16(p + 36) == 19 && autoGetU16(p + 38) == 3870 && p[33] == 31, "meta fields");
    } else if (cap[i].h.type == AUTO_T_CHUNK) {
      chunks++;
      const uint8_t *p = cap[i].p;
      uint16_t ci = autoGetU16(p + 4), dl = autoGetU16(p + 6);
      CHECK(dl == (ci == 18 ? 768 : 1024) && cap[i].h.len == 8 + dl, "chunk len");
      memcpy(img[p[2]] + ci * 1024, p + 8, dl);
    }
  }
  CHECK(metas == 2 && chunks == 38, "pushed 2 META + 38 CHUNK (got %d, %d), nomem=%u", metas, chunks, (unsigned)txNoMem);
  CHECK(crcs[0] == 1038996360u && crc32(0, img[0], 19200) == crcs[0] && crc32(0, img[1], 19200) == crcs[1],
        "CRC matches zlib golden");
  CHECK(seqs[1] > seqs[0] && capMs[1] - capMs[0] >= 300, "gap respected (%u ms)", capMs[1] - capMs[0]);
  CHECK(photo.pushDone && photo.held, "held after push");
  CHECK(nCapNotes == 0, "an A1 set reports nothing to the state machine");

  /* CHUNKS re-request */
  autoPutU16(pl, 1); pl[2] = 1; pl[3] = 4; autoPutU16(pl + 4, 12);
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, 11, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK, "CHUNKS OK");
  t0 = ncap;
  runLoop(200);
  CHECK(ncap - t0 == 4 && cap[t0].h.type == AUTO_T_CHUNK && autoGetU16(cap[t0].p + 4) == 12 &&
        autoGetU16(cap[ncap - 1].p + 4) == 15 && cap[t0].p[2] == 1, "4 chunks resent");
  pl[3] = 8; autoPutU16(pl + 4, 12);
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, 12, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "CHUNKS past the end RANGE");
  autoPutU16(pl, 2); pl[3] = 1;
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, 13, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_NO_PHOTO, "CHUNKS wrong id NO_PHOTO");

  /* PHOTO_FREE */
  autoPutU16(pl, 1);
  macSend(AUTO_T_PHOTO_FREE, AUTO_FLAG_ACKREQ, sess, 14, pl, 2);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK && !photo.held, "PHOTO_FREE");
  macSend(AUTO_T_PHOTO_FREE, AUTO_FLAG_ACKREQ, sess, 15, pl, 2);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_NO_PHOTO, "second free NO_PHOTO");

  /* auto-free after 30 s; camera stopping mid-set -> missing frame META size 0 */
  pl[0] = 2; autoPutU16(pl + 2, 0);
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, 16, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK && autoGetU32(lastOf(AUTO_T_ACK)->p + 8) == 2, "photo 2");
  runLoop(60);   /* first frame */
  camState = CAMERA_FLOW_FAILED;
  t0 = ncap;
  runLoop(3000);
  pkt_t *m = lastOf(AUTO_T_META);
  CHECK(m && m->p[2] == 1 && autoGetU32(m->p + 8) == 0 && autoGetU16(m->p + 36) == 0, "missing frame META size 0");
  camState = CAMERA_FLOW_RUNNING;
  CHECK(photo.held, "still held");
  runLoop(31000);
  CHECK(!photo.held, "auto-freed after 30 s");

  /* wrong session, bad tag, rate limit */
  macSend(AUTO_T_STATUS_REQ, 0, sess + 1, 30, NULL, 0);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_BAD_SESSION, "wrong session");
  uint32_t af = authFails;
  uint8_t b[64];
  autoProtoHdr_t h = {.type = AUTO_T_STATUS_REQ, .session = sess, .seq = 31, .len = 0};
  int n = autoProtoEncode(&h, NULL, testKey, hm, b, sizeof(b));
  b[n - 1] ^= 1;
  struct sockaddr_in from = {.sin_family = AF_INET};
  before = ncap;
  handleDatagram(b, (size_t)n, &from);
  CHECK(authFails == af + 1 && ncap == before, "bad tag dropped");
  vTaskDelay(1000);
  uint32_t rd = rateDrops;
  for (int i = 0; i < 70; i++) macSend(AUTO_T_HB, 0, sess, 40 + i, (uint8_t[]){1, 2, 3, 4}, 4);
  CHECK(rateDrops == rd + 10, "rate limit 60/s (drops %u)", (unsigned)(rateDrops - rd));

  /* STATUS_REQ duplicate answered again */
  vTaskDelay(1000);
  int sc = countOf(AUTO_T_STATUS);
  macSend(AUTO_T_STATUS_REQ, 0, sess, 200, NULL, 0);
  macSend(AUTO_T_STATUS_REQ, AUTO_FLAG_RETX, sess, 200, NULL, 0);
  CHECK(countOf(AUTO_T_STATUS) == sc + 2, "dup STATUS_REQ answered");

  /* new HELLO -> new session, seq restarts */
  autoPutU32(pl, 0x55667788);
  macSend(AUTO_T_HELLO, 0, 0, 201, pl, 4);
  CHECK(session != sess && lastOf(AUTO_T_HELLO_ACK)->h.seq == 1 && sessionNotes == 2, "new session");
  macSend(AUTO_T_STATUS_REQ, 0, session, 202, NULL, 0);
  CHECK(lastOf(AUTO_T_STATUS)->h.session == session, "status in new session");

  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(s.peerIp[0] == 192 && s.peerIp[3] == 44 && s.peerPort == 50123, "peer %u.%u.%u.%u:%u", s.peerIp[0],
        s.peerIp[1], s.peerIp[2], s.peerIp[3], s.peerPort);
}

/* ---- stage A2b ---- */

/* At boot: the stack and the TX buffer in PSRAM. */
static void testTaskAtBoot(void)
{
  CHECK(staticCalls == 1 && dynCalls == 0 && lastStaticDepth == AUTO_LINK_STACK_PSRAM && lastStaticStack != NULL &&
        capsOf(lastStaticStack) == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) && stackInPsram,
        "task stack: %u bytes from the SPIRAM allocation, xTaskCreateStaticPinnedToCore", (unsigned)lastStaticDepth);
  CHECK(txBuf != NULL && capsOf(txBuf) == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT), "txBuf in PSRAM");
  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(s.stackInPsram && s.running, "stats: stack in PSRAM");
}

/* Without PSRAM: the A1 internal task and an internal TX buffer. */
static void testTaskFallback(void)
{
  psramFail = true;
  taskHandle = NULL;
  const int st0 = staticCalls;
  CHECK(linkTaskStart() && dynCalls == 1 && lastDynDepth == AUTO_LINK_STACK_INTERNAL && staticCalls == st0 &&
        !stackInPsram && taskHandle != NULL, "PSRAM malloc NULL: internal task, %u bytes", (unsigned)lastDynDepth);
  uint8_t *b = txBufAlloc();
  CHECK(b != NULL && capsOf(b) == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), "txBuf falls back to internal RAM");
  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(!s.stackInPsram, "stats: stack internal");
  free(b);
  psramFail = false;
}

/* Before any session: a KILL still acts; nothing is pushed. */
static void testNoSession(void)
{
  int n0 = ncap;
  fakeState = AUTO_HOVER;
  fakeStatusSeq++;
  autoStatusStep(nowMs());
  CHECK(ncap == n0, "no AUTO_STATUS without a session");
  fakeState = AUTO_IDLE;
  autoStatusStep(nowMs());
  macSendFrom(ADDR_B, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 77, NULL, 0);
  CHECK(killCalls == 1 && ncap == n0 + 1 && cap[n0].h.type == AUTO_T_ACK && cap[n0].p[4] == AUTO_T_KILL &&
        cap[n0].p[5] == AUTO_R_OK && cap[n0].h.session == 0 && sameAddr(&cap[n0].to, ADDR_B) && !havePeer,
        "KILL before any HELLO: acted on, answered at the sender, no peer set");
  fakeStatusSeq = 0;
  statusSeqSent = 0;
}

static void testKill(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  /* A seq far ahead in our session: acted on, the filter untouched. */
  const uint32_t lastSeq = seqFilter.lastSeq;
  int k0 = killCalls, n0 = ncap;
  macSend(AUTO_T_KILL, AUTO_FLAG_ACKREQ, sess, 1000000, NULL, 0);
  pkt_t *a = ackOf(1000000, n0);
  CHECK(killCalls == k0 + 1 && ncap == n0 + 1 && a && a->p[4] == AUTO_T_KILL && a->p[5] == AUTO_R_OK &&
        a->h.session == sess && sameAddr(&a->to, ADDR_A), "KILL in our session: OK");
  CHECK(sendsAtKill == sendCalls - 1, "autonomyKillFromLink before the ACK");
  CHECK(seqFilter.lastSeq == lastSeq && !ackCached(1000000), "seq filter untouched, ACK not cached");
  /* The same seq again: acted on again (a replayed KILL is fail-safe). */
  macSend(AUTO_T_KILL, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, sess, 1000000, NULL, 0);
  CHECK(killCalls == k0 + 2, "repeat acted on");
  /* The kill client: session 0 from its own address. */
  n0 = ncap;
  macSendFrom(ADDR_B, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 5, NULL, 0);
  a = ackOf(5, n0);
  CHECK(killCalls == k0 + 3 && a && a->h.session == 0 && sameAddr(&a->to, ADDR_B), "session 0: ACK to the sender");
  CHECK(session == sess && sameAddr(&peer, ADDR_A), "session and peer unchanged");
  /* Another session from another station. */
  n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0xDEADBEEF, 9, NULL, 0);
  a = ackOf(9, n0);
  CHECK(killCalls == k0 + 4 && a && a->h.session == 0xDEADBEEF && sameAddr(&a->to, ADDR_C) && session == sess &&
        sameAddr(&peer, ADDR_A), "other session: acted on, ours and the peer unchanged");
  /* No ACKREQ, a payload: acted on, silent. */
  n0 = ncap;
  macSendFrom(ADDR_B, AUTO_T_KILL, 0, 0, 6, (uint8_t[]){1, 2, 3, 4}, 4);
  CHECK(killCalls == k0 + 5 && ncap == n0, "no ACKREQ, any payload: acted on, no reply");
  /* ACK detail and byte 6 carry the state. */
  fakeState = AUTO_HOVER;
  n0 = ncap;
  macSendFrom(ADDR_B, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 7, NULL, 0);
  a = ackOf(7, n0);
  CHECK(a && a->p[6] == AUTO_HOVER && autoGetU32(a->p + 8) == AUTO_HOVER, "KILL ACK: state byte 6 and detail");
  /* The next AUTO_STATUS still goes to the daemon. */
  fakeStatusSeq++;
  n0 = ncap;
  autoStatusStep(nowMs());
  a = lastOf(AUTO_T_AUTO_STATUS);
  CHECK(ncap == n0 + 1 && a && sameAddr(&a->to, ADDR_A) && a->h.session == sess, "AUTO_STATUS to the daemon's address");
  fakeState = AUTO_IDLE;
  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(s.kills == (uint32_t)killCalls, "stats: kills %u", (unsigned)s.kills);
}

static void testPeerRule(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  /* A BAD_SESSION reply goes to the sender; the peer stays. */
  int n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_STATUS_REQ, 0, sess ^ 1, 3, NULL, 0);
  CHECK(ncap == n0 + 1 && sameAddr(&cap[n0].to, ADDR_C) && sameAddr(&peer, ADDR_A), "BAD_SESSION to the sender only");
  /* A replayed packet of our session (a duplicate) from elsewhere: answered there, no peer move. */
  macSend(AUTO_T_STATUS_REQ, AUTO_FLAG_ACKREQ, sess, ++macSeq, NULL, 0);
  const uint32_t dupSeq = macSeq;
  n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_STATUS_REQ, AUTO_FLAG_ACKREQ, sess, dupSeq, NULL, 0);
  CHECK(ncap > n0 && sameAddr(&cap[ncap - 1].to, ADDR_C) && sameAddr(&peer, ADDR_A), "duplicate: no peer move");
  /* A new packet of our session moves it. */
  macSendFrom(ADDR_C, AUTO_T_HB, 0, sess, ++macSeq, (uint8_t[]){1, 2, 3, 4}, 4);
  CHECK(sameAddr(&peer, ADDR_C), "a new packet of the current session sets the peer");
  macSend(AUTO_T_HB, 0, sess, ++macSeq, (uint8_t[]){1, 2, 3, 4}, 4);
  CHECK(sameAddr(&peer, ADDR_A), "and back");
  /* A HELLO with a session: refused at the sender, no peer move. */
  n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_HELLO, 0, 0x4242, 1, (uint8_t[]){1, 2, 3, 4}, 4);
  CHECK(ncap == n0 + 1 && cap[n0].p[5] == AUTO_R_BAD_SESSION && sameAddr(&cap[n0].to, ADDR_C) && sameAddr(&peer, ADDR_A),
        "HELLO with a session: BAD_SESSION at the sender");
}

static void testKillBudget(void)
{
  const uint32_t sess = newSession();
  vTaskDelay(1100);   /* fresh windows */
  const uint32_t rd = rateDrops, krd = killRateDrops;
  const int k0 = killCalls;
  for (int i = 0; i < 70; i++) macSend(AUTO_T_HB, 0, sess, ++macSeq, (uint8_t[]){1, 2, 3, 4}, 4);
  macSendFrom(ADDR_B, AUTO_T_KILL, 0, 0, 1, NULL, 0);
  CHECK(rateDrops == rd + 10 && killCalls == k0 + 1 && killRateDrops == krd,
        "70 HB in a second (10 dropped) do not drop a KILL after them");
  for (int i = 0; i < 105; i++) macSendFrom(ADDR_B, AUTO_T_KILL, 0, 0, 2 + i, NULL, 0);
  CHECK(killCalls == k0 + 100 && killRateDrops == krd + 6 && rateDrops == rd + 10,
        "KILL budget 100/s (acted %d, dropped %u)", killCalls - k0, (unsigned)(killRateDrops - krd));
  vTaskDelay(1000);
  const uint32_t af = authFails, rc = rateCount;
  const int k1 = killCalls;
  uint8_t b[64];
  const int n = encodeMac(b, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 1, NULL, 0);
  b[n - 1] ^= 1;
  struct sockaddr_in from = ADDR_B;
  const int n0 = ncap;
  handleDatagram(b, (size_t)n, &from);
  CHECK(authFails == af + 1 && killCalls == k1 && ncap == n0 && killRateCount == 1 && rateCount == rc,
        "0x13 with a bad tag: counted in the KILL budget, not acted on");
}

static void testCommands(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_IDLE;
  /* ARM: posted once, answered with the FSM's result; byte 6 = r.state */
  uint8_t pl[16];
  const autoProtoArm_t arm = {.mode = AUTO_MODE_DRY, .airMaxMs = 3000};
  autoProtoPackArm(&arm, pl);
  const uint32_t posts0 = mb.posts, cmds0 = autoCmds;
  int n0 = ncap;
  const uint32_t sArm = ++macSeq;
  int wk = gndWakes;
  macSend(AUTO_T_ARM, AUTO_FLAG_ACKREQ, sess, sArm, pl, AUTO_ARM_LEN);
  CHECK(mb.posts == posts0 + 1 && pend.active && ncap == n0 && autoCmds == cmds0 + 1, "ARM posted, no ACK yet");
  CHECK(gndWakes == wk + 1 && gndWakeHold == 45000, "ARM: the camera's ground standby woken (held camgndms)");
  CHECK(stabStep(AUTO_R_OK, 0x5EED0001u, AUTO_ARMED) && lastTaken.type == AUTO_CMD_ARM &&
        lastTaken.mode == AUTO_MODE_DRY && lastTaken.airMaxMs == 3000, "the stabilizer took ARM with its fields");
  autoCmdStep(nowMs());
  pkt_t *a = ackOf(sArm, n0);
  CHECK(a && a->p[4] == AUTO_T_ARM && a->p[5] == AUTO_R_OK && a->p[6] == AUTO_ARMED && autoGetU32(a->p + 8) == 0x5EED0001u &&
        a->h.session == sess && sameAddr(&a->to, ADDR_A) && !pend.active, "ACK: FSM result, byte 6 = r.state (ARMED, "
        "not autonomyState() IDLE)");
  CHECK(ackCached(sArm), "the result is cached");
  n0 = ncap;
  macSend(AUTO_T_ARM, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, sess, sArm, pl, AUTO_ARM_LEN);
  a = ackOf(sArm, n0);
  CHECK(a && ncap == n0 + 1 && a->p[5] == AUTO_R_OK && a->p[6] == AUTO_ARMED && autoGetU32(a->p + 8) == 0x5EED0001u &&
        mb.posts == posts0 + 1, "duplicate seq: the cached ACK, no re-post");

  /* The interactive hover: MOVE, YAW and STOP go to the state machine with their fields */
  {
    const autoProtoMove_t mv = {.rightCm = -75, .fwdCm = 120, .upCm = -20};
    uint8_t mp[AUTO_MOVE_LEN];
    autoProtoPackMove(&mv, mp);
    const uint32_t sMv = ++macSeq;
    int m0 = ncap;
    wk = gndWakes;
    macSend(AUTO_T_MOVE, AUTO_FLAG_ACKREQ, sess, sMv, mp, AUTO_MOVE_LEN);
    CHECK(gndWakes == wk, "MOVE wakes nothing (autonomy is active anyway)");
    CHECK(pend.active && stabStep(AUTO_R_CAM_UNAVAIL, 3u | 5u << 8 | 1u << 16, AUTO_ARMED) &&
          lastTaken.type == AUTO_CMD_MOVE && lastTaken.moveRightCm == -75 && lastTaken.moveFwdCm == 120 &&
          lastTaken.moveUpCm == -20, "MOVE posted with its fields (signed)");
    autoCmdStep(nowMs());
    pkt_t *am = ackOf(sMv, m0);
    CHECK(am && am->p[4] == AUTO_T_MOVE && am->p[5] == AUTO_R_CAM_UNAVAIL && autoGetU32(am->p + 8) == (3u | 5u << 8 | 1u << 16),
          "MOVE: the state machine's refusal and its reason detail");
    const autoProtoYaw_t yw = {.deg = -135};
    uint8_t yp[AUTO_YAW_LEN];
    autoProtoPackYaw(&yw, yp);
    const uint32_t sYw = ++macSeq;
    macSend(AUTO_T_YAW, AUTO_FLAG_ACKREQ, sess, sYw, yp, AUTO_YAW_LEN);
    CHECK(pend.active && stabStep(AUTO_R_OK, 3u, AUTO_ARMED) && lastTaken.type == AUTO_CMD_YAW && lastTaken.yawDeg == -135,
          "YAW posted with its angle");
    autoCmdStep(nowMs());
    const uint32_t sSt = ++macSeq;
    macSend(AUTO_T_STOP, AUTO_FLAG_ACKREQ, sess, sSt, NULL, 0);
    CHECK(pend.active && stabStep(AUTO_R_OK, 3u, AUTO_ARMED) && lastTaken.type == AUTO_CMD_STOP, "STOP posted");
    autoCmdStep(nowMs());
    /* SET_HEIGHT (0x33, 2026-10-15): to the state machine with X in heightCm and h_at in moveUpCm; a bad length or
     * X < 0: RANGE detail 0 at the link, not posted */
    {
      const autoProtoSetHeight_t shp = {.xCm = 112, .atCm = -7};
      uint8_t shb[AUTO_SET_HEIGHT_LEN];
      autoProtoPackSetHeight(&shp, shb);
      const uint32_t sSh = ++macSeq;
      int s0 = ncap;
      macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sSh, shb, AUTO_SET_HEIGHT_LEN);
      CHECK(pend.active && stabStep(AUTO_R_OK, 3u | 100u << 16, AUTO_ARMED) && lastTaken.type == AUTO_CMD_SET_HEIGHT &&
            lastTaken.heightCm == 112 && lastTaken.moveUpCm == -7, "SET_HEIGHT posted with X 112 and h_at -7 (signed)");
      autoCmdStep(nowMs());
      pkt_t *sa = ackOf(sSh, s0);
      CHECK(sa && sa->p[4] == AUTO_T_SET_HEIGHT && sa->p[5] == AUTO_R_OK && autoGetU32(sa->p + 8) == (3u | 100u << 16),
            "SET_HEIGHT: the state machine's result and detail (the old estimate in bits 16-31)");
      const autoProtoSetHeight_t shNow = {.xCm = 80, .atCm = INT16_MIN};
      autoProtoPackSetHeight(&shNow, shb);
      const uint32_t sSh2 = ++macSeq;
      macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sSh2, shb, AUTO_SET_HEIGHT_LEN);
      CHECK(pend.active && stabStep(AUTO_R_OK, 3u, AUTO_ARMED) && lastTaken.moveUpCm == INT16_MIN &&
            lastTaken.heightCm == 80 && lastTaken.mode == 0 && lastTaken.moveRightCm == 0,
            "SET_HEIGHT now: h_at INT16_MIN passed through; the 4-byte form (mode 0, no target)");
      autoCmdStep(nowMs());
      /* the "go to the target" form (2026-10-16): 6 bytes, mode AUTO_SH_FORM_TO, the target in moveRightCm */
      {
        const autoProtoSetHeight_t shTo = {.xCm = 105, .atCm = 91, .toCm = INT16_MIN, .toForm = true},
                                   shTo2 = {.xCm = 90, .atCm = INT16_MIN, .toCm = 120, .toForm = true};
        uint8_t tob[AUTO_SET_HEIGHT_TO_LEN + 1];
        autoProtoPackSetHeightTo(&shTo, tob);
        const uint32_t sTo = ++macSeq;
        macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sTo, tob, AUTO_SET_HEIGHT_TO_LEN);
        CHECK(pend.active && stabStep(AUTO_R_OK, 3u | 90u << 16, AUTO_ARMED) && lastTaken.type == AUTO_CMD_SET_HEIGHT &&
              lastTaken.heightCm == 105 && lastTaken.moveUpCm == 91 && lastTaken.mode == AUTO_SH_FORM_TO &&
              lastTaken.moveRightCm == AUTO_SH_TO_REF, "SET_HEIGHT to the commanded height: mode 1, target INT16_MIN");
        autoCmdStep(nowMs());
        autoProtoPackSetHeightTo(&shTo2, tob);
        const uint32_t sTo2 = ++macSeq;
        macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sTo2, tob, AUTO_SET_HEIGHT_TO_LEN);
        CHECK(pend.active && stabStep(AUTO_R_OK, 3u, AUTO_ARMED) && lastTaken.mode == AUTO_SH_FORM_TO &&
              lastTaken.moveRightCm == 120 && lastTaken.moveUpCm == INT16_MIN, "SET_HEIGHT --to 1.20: target 120");
        autoCmdStep(nowMs());
        int t0 = ncap;
        const uint32_t sTo7 = ++macSeq;
        macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sTo7, tob, AUTO_SET_HEIGHT_TO_LEN + 1);
        pkt_t *ta = ackOf(sTo7, t0);
        CHECK(ta && ta->p[5] == AUTO_R_RANGE && autoGetU32(ta->p + 8) == 0u && !pend.active,
              "SET_HEIGHT of 7 bytes: RANGE detail 0, not posted (the reply an older drone gives the 6-byte form)");
      }
      s0 = ncap;
      const uint32_t sShBad = ++macSeq;
      macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sShBad, shb, AUTO_SET_HEIGHT_LEN - 1);
      sa = ackOf(sShBad, s0);
      CHECK(sa && sa->p[4] == AUTO_T_SET_HEIGHT && sa->p[5] == AUTO_R_RANGE && autoGetU32(sa->p + 8) == 0u &&
            !pend.active, "SET_HEIGHT of 3 bytes: RANGE detail 0, not posted");
      const autoProtoSetHeight_t shNeg = {.xCm = -1, .atCm = 50};
      autoProtoPackSetHeight(&shNeg, shb);
      s0 = ncap;
      const uint32_t sShNeg = ++macSeq;
      macSend(AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, sess, sShNeg, shb, AUTO_SET_HEIGHT_LEN);
      sa = ackOf(sShNeg, s0);
      CHECK(sa && sa->p[5] == AUTO_R_RANGE && autoGetU32(sa->p + 8) == 0u && !pend.active,
            "SET_HEIGHT with X < 0: RANGE detail 0, not posted");
    }
    uint8_t bad[AUTO_MOVE_LEN + 1] = {0};
    m0 = ncap;
    const uint32_t sBad = ++macSeq;
    macSend(AUTO_T_MOVE, AUTO_FLAG_ACKREQ, sess, sBad, bad, AUTO_MOVE_LEN + 1);
    am = ackOf(sBad, m0);
    CHECK(am && am->p[5] == AUTO_R_RANGE && !pend.active, "MOVE of 9 bytes: RANGE, not posted");
    m0 = ncap;
    const uint32_t sBadY = ++macSeq;
    macSend(AUTO_T_YAW, AUTO_FLAG_ACKREQ, sess, sBadY, NULL, 0);
    am = ackOf(sBadY, m0);
    CHECK(am && am->p[4] == AUTO_T_YAW && am->p[5] == AUTO_R_RANGE && !pend.active,
          "YAW without its 4 bytes: RANGE, not posted");
  }

  /* TAKEOFF with its fields */
  const autoProtoTakeoff_t to = {.token = 0x5EED0001u, .heightCm = 35, .photoN = 2, .photoGapMs = 300, .actWaitMs = 0};
  autoProtoPackTakeoff(&to, pl);
  const uint32_t sTo = ++macSeq;
  n0 = ncap;
  wk = gndWakes;
  macSend(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, sTo, pl, AUTO_TAKEOFF_LEN);
  CHECK(gndWakes == wk + 1, "TAKEOFF: the ground standby woken");
  CHECK(stabStep(AUTO_R_OK, AUTO_ARMED, AUTO_TAKEOFF) && lastTaken.type == AUTO_CMD_TAKEOFF &&
        lastTaken.token == 0x5EED0001u && lastTaken.heightCm == 35 && lastTaken.photoN == 2 && lastTaken.photoGapMs == 300 &&
        lastTaken.actWaitMs == 0, "TAKEOFF fields");
  autoCmdStep(nowMs());
  a = ackOf(sTo, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && a->p[6] == AUTO_TAKEOFF && autoGetU32(a->p + 8) == AUTO_ARMED, "TAKEOFF ACK");

  /* TAKEOFF with a planned move (2026-10-14, docs/autonomy-a2b.md 21.4): 18 bytes, the plan in the MOVE fields;
   * 17 or 19 bytes, or a nonzero reserved field: RANGE detail 0 at the link (what an older drone answers the 18), not
   * posted */
  {
    const autoProtoTakeoff_t tp = {.token = 0x5EED0001u, .heightCm = 100, .photoN = 1, .photoGapMs = 300,
                                   .actWaitMs = 60000, .planRightCm = -80, .planFwdCm = 250};
    uint8_t pp[AUTO_TAKEOFF_PLAN_LEN + 2] = {0};
    CHECK(autoProtoPackTakeoff(&tp, pp) == AUTO_TAKEOFF_PLAN_LEN, "the plan's 18 bytes");
    const uint32_t sP = ++macSeq;
    n0 = ncap;
    macSend(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, sP, pp, AUTO_TAKEOFF_PLAN_LEN);
    CHECK(stabStep(AUTO_R_RANGE, AUTO_TAKEOFF, AUTO_TAKEOFF) && lastTaken.type == AUTO_CMD_TAKEOFF &&
          lastTaken.moveRightCm == -80 && lastTaken.moveFwdCm == 250 && lastTaken.moveUpCm == 0 &&
          lastTaken.actWaitMs == 60000 && lastTaken.heightCm == 100, "TAKEOFF 18 bytes: posted, the plan in "
          "moveRightCm / moveFwdCm (%d, %d)", lastTaken.moveRightCm, lastTaken.moveFwdCm);
    autoCmdStep(nowMs());
    a = ackOf(sP, n0);
    CHECK(a && a->p[4] == AUTO_T_TAKEOFF && a->p[5] == AUTO_R_RANGE && autoGetU32(a->p + 8) == AUTO_TAKEOFF,
          "its ACK: the state machine's answer (here RANGE, detail the state)");
    const uint32_t posts0 = mb.posts;
    static const uint16_t badLen[3] = {AUTO_TAKEOFF_PLAN_LEN - 1, AUTO_TAKEOFF_PLAN_LEN + 1, AUTO_TAKEOFF_PLAN_LEN};
    for (int i = 0; i < 3; i++) {
      autoProtoPackTakeoff(&tp, pp);
      if (i == 2) {
        pp[16] = 1;   /* the reserved field */
      }
      const uint32_t sb = ++macSeq;
      n0 = ncap;
      macSend(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, sb, pp, badLen[i]);
      a = ackOf(sb, n0);
      CHECK(a && a->p[5] == AUTO_R_RANGE && autoGetU32(a->p + 8) == 0 && !pend.active,
            "TAKEOFF of %u bytes%s: RANGE detail 0 at the link, not posted", badLen[i],
            i == 2 ? " with reserved 1" : "");
    }
    CHECK(mb.posts == posts0, "none of them posted");
  }

  /* LAND and DISARM */
  const uint8_t types[2] = {AUTO_T_LAND, AUTO_T_DISARM};
  const autoCmdType_t cmds[2] = {AUTO_CMD_LAND, AUTO_CMD_DISARM};
  for (int i = 0; i < 2; i++) {
    const uint32_t s = ++macSeq;
    n0 = ncap;
    macSend(types[i], AUTO_FLAG_ACKREQ, sess, s, NULL, 0);
    CHECK(stabStep(AUTO_R_OK, AUTO_HOVER, AUTO_LAND) && lastTaken.type == cmds[i], "%s posted once",
          i ? "DISARM" : "LAND");
    autoCmdStep(nowMs());
    a = ackOf(s, n0);
    CHECK(a && ncap == n0 + 1 && a->p[4] == types[i] && a->p[5] == AUTO_R_OK && a->p[6] == AUTO_LAND &&
          autoGetU32(a->p + 8) == AUTO_HOVER, "%s ACK", i ? "DISARM" : "LAND");
  }

  /* TAKEOFF byte 7 (a pad until 2026-10-14) is actWaitMs bits 16-23: 0x02 posts 131072 + the low 16 bits; the
   * link does not range-check it (the state machine answers RANGE above AUTO_HOVER_WAIT_MAX_MS) */
  const autoProtoTakeoff_t toBig = {.token = 0x5EED0001u, .heightCm = 105, .photoN = 1, .photoGapMs = 300,
                                    .actWaitMs = 0x1234u};
  autoProtoPackTakeoff(&toBig, pl);
  pl[7] = 0x02;
  const uint32_t sBig = ++macSeq;
  n0 = ncap;
  macSend(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, sBig, pl, AUTO_TAKEOFF_LEN);
  CHECK(stabStep(AUTO_R_RANGE, AUTO_LAND, AUTO_LAND) && lastTaken.type == AUTO_CMD_TAKEOFF &&
        lastTaken.actWaitMs == 131072u + 0x1234u && lastTaken.heightCm == 105,
        "TAKEOFF byte 7 0x02: posted with actWaitMs %u (131072 + 0x1234), the FSM answers", (unsigned)lastTaken.actWaitMs);
  autoCmdStep(nowMs());
  a = ackOf(sBig, n0);
  CHECK(a && a->p[4] == AUTO_T_TAKEOFF && a->p[5] == AUTO_R_RANGE, "its ACK: the FSM's RANGE");

  /* RANGE at the link: bad length or pad; nothing posted; cached (a definitive answer) */
  const uint32_t posts1 = mb.posts;
  uint32_t s = ++macSeq;
  macSend(AUTO_T_ARM, AUTO_FLAG_ACKREQ, sess, s, pl, 3);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE && autoGetU32(lastOf(AUTO_T_ACK)->p) == s, "ARM length 3: RANGE");
  autoProtoPackTakeoff(&to, pl);
  macSend(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, AUTO_TAKEOFF_LEN - 1);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "TAKEOFF of 11 bytes: RANGE");
  macSend(AUTO_T_LAND, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 1);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "LAND with a payload: RANGE");
  CHECK(mb.posts == posts1 && !pend.active && ackCached(s), "nothing posted; RANGE cached");

  /* Without ACKREQ: processed, no ACK */
  n0 = ncap;
  macSend(AUTO_T_LAND, 0, sess, ++macSeq, NULL, 0);
  CHECK(stabStep(AUTO_R_OK, AUTO_IDLE, AUTO_IDLE) && lastTaken.type == AUTO_CMD_LAND, "LAND without ACKREQ posted");
  autoCmdStep(nowMs());
  CHECK(ncap == n0 && !pend.active, "and not answered");
}

/* Ground standby (docs/autonomy-a2b.md 17.1): a PHOTO in IDLE while the
 * camera sleeps is accepted and wakes it; its frame is captured only after
 * AUTO_CAM_WAKE_FRAMES fresh frames; a camera that never wakes reports the
 * frame missing after AUTO_CAM_WAKE_MAX_MS + AUTO_FRAME_TIMEOUT_MS. */
static void testGroundStandbyPhoto(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_IDLE;
  freeHeld(sess);
  runLoop(100);
  uint8_t pl[4] = {1, 0, 0, 0};
  /* asleep: OK at once, wake requested */
  gndAsleep = true;
  gndNeverWakes = false;
  camState = CAMERA_FLOW_STANDBY;
  int wk = gndWakes;
  const uint32_t seqPhoto = ++macSeq;
  int n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, seqPhoto, pl, 4);
  pkt_t *a = ackOf(seqPhoto, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && gndWakes == wk + 1 && photo.held, "asleep: PHOTO OK, the camera woken");
  const uint16_t id = photo.id;
  const int64_t t0 = fakeUs;
  n0 = ncap;
  int64_t metaUs = -1;
  for (int t = 0; t < 3000 && metaUs < 0; t += AUTO_PUSH_GAP_MS) {
    if (t % 60 == 0) camTick();
    photoStep(nowMs());
    vTaskDelay(AUTO_PUSH_GAP_MS);
    if (lastOf(AUTO_T_META) && ncap > n0 && countSince(n0, AUTO_T_META) > 0) metaUs = fakeUs;
  }
  pkt_t *m = lastOf(AUTO_T_META);
  CHECK(!gndAsleep && metaUs > 0 && m && autoGetU16(m->p) == id && autoGetU32(m->p + 8) == 19200,
        "the frame comes after the wake (%.2f s)", (metaUs - t0) / 1e6);
  CHECK(m && autoGetU32(m->p + 16) >= camSeqAtWake + AUTO_CAM_WAKE_FRAMES,
        "captured only after %d fresh frames (frame %u, wake at %u)", AUTO_CAM_WAKE_FRAMES,
        m ? (unsigned)autoGetU32(m->p + 16) : 0u, (unsigned)camSeqAtWake);
  CHECK(metaUs - t0 < 1500000, "and within 1.5 s of the PHOTO (%.2f s)", (metaUs - t0) / 1e6);
  freeHeld(sess);
  runLoop(100);

  /* asleep and the wake never lands: OK, then the frame reported missing after 2.5 s + 1 s */
  gndAsleep = true;
  gndNeverWakes = true;
  camState = CAMERA_FLOW_STANDBY;
  const uint32_t seq2 = ++macSeq;
  n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, seq2, pl, 4);
  CHECK(ackOf(seq2, n0) && ackOf(seq2, n0)->p[5] == AUTO_R_OK, "asleep, wake fails: still OK");
  n0 = ncap;
  runLoop(AUTO_CAM_WAKE_MAX_MS + AUTO_FRAME_TIMEOUT_MS - 200);
  CHECK(countSince(n0, AUTO_T_META) == 0, "no frame before 2.5 s + 1 s");
  runLoop(400);
  m = lastOf(AUTO_T_META);
  CHECK(countSince(n0, AUTO_T_META) == 1 && m && autoGetU32(m->p + 8) == 0, "then reported missing (META size 0)");
  freeHeld(sess);
  runLoop(100);
  gndAsleep = false;
  gndNeverWakes = false;
  camState = CAMERA_FLOW_RUNNING;
  camFresh = 1000;
}

static void testStalled(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_ARMED;
  uint8_t pl[16];
  const autoProtoTakeoff_t to = {.token = 1, .heightCm = 40, .photoN = 0, .photoGapMs = 300};
  autoProtoPackTakeoff(&to, pl);

  /* The stabilizer stalls: a posted TAKEOFF does not delay a KILL or an HB. */
  const uint32_t sTo = ++macSeq;
  macQueue(AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, sess, sTo, pl, AUTO_TAKEOFF_LEN);
  pass();
  CHECK(pend.active && pend.seq == sTo, "TAKEOFF posted");
  const int k0 = killCalls, hb0 = hbNotes, blk0 = recvBlocking;
  const int64_t t0 = fakeUs;
  int n0 = ncap;
  macQueue(AUTO_T_KILL, AUTO_FLAG_ACKREQ, sess, 999999, NULL, 0);
  macQueue(AUTO_T_HB, 0, sess, ++macSeq, (uint8_t[]){1, 2, 3, 4}, 4);
  pass();
  CHECK(killCalls == k0 + 1 && hbNotes == hb0 + 1 && ackOf(999999, n0) != NULL, "KILL and HB served in one pass");
  CHECK(recvBlocking == blk0 && fakeUs - t0 == 1000, "non-blocking receives, then one tick (%lld us)",
        (long long)(fakeUs - t0));

  /* A second command while one is pending: BUSY (detail = the pending seq), not posted, not cached. */
  const uint32_t posts0 = mb.posts, busy0 = cmdBusy;
  const uint32_t sLand = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ, sess, sLand, NULL, 0);
  pass();
  pkt_t *a = ackOf(sLand, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == (AUTO_DETAIL_BUSY_PENDING | sTo) &&
        a->p[6] == AUTO_ARMED && mb.posts == posts0 && cmdBusy == busy0 + 1 && !ackCached(sLand),
        "second command: BUSY, detail = PENDING | pending seq, not posted or cached");
  n0 = ncap;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, sess, sLand, NULL, 0);
  pass();
  CHECK(ackOf(sLand, n0) == NULL, "its RETX gets nothing");

  /* Not taken by 100 ms: withdrawn with BUSY (state); never processed. */
  const uint32_t to0 = cmdTimeouts;
  n0 = ncap;
  while (!ackOf(sTo, n0) && fakeUs - t0 < 500000) pass();
  a = ackOf(sTo, n0);
  const int64_t waited = a ? a->us - (int64_t)pend.t0 * 1000 : -1;
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == (AUTO_DETAIL_BUSY_UNTAKEN | AUTO_ARMED) &&
        a->p[6] == AUTO_ARMED && cmdTimeouts == to0 + 1 && !ackCached(sTo) && waited >= 100000 && waited <= 102000,
        "not taken by 100 ms: BUSY (UNTAKEN | state), uncached (%lld us)", (long long)waited);
  CHECK(!stabStep(AUTO_R_OK, 0, AUTO_TAKEOFF) && mb.cmdTicket == 0, "the stabilizer never sees it afterwards");
  CHECK(!pend.active, "slot free");
  fakeState = AUTO_IDLE;
}

static void testTakenUnknown(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_HOVER;
  /* Taken, never answered: BUSY with AUTO_DETAIL_TAKEN_UNKNOWN at 200 ms, uncached. */
  const uint32_t s1 = ++macSeq;
  int n0 = ncap;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ, sess, s1, NULL, 0);
  pass();
  CHECK(stabTakeOnly(), "stabilizer took LAND and stalls");
  const uint32_t stale = takenTicket;
  const uint32_t tu0 = cmdTakenUnknown, to0 = cmdTimeouts;
  const int64_t t0 = (int64_t)pend.t0 * 1000;
  while (!ackOf(s1, n0) && fakeUs < t0 + 500000) pass();
  pkt_t *a = ackOf(s1, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == AUTO_DETAIL_TAKEN_UNKNOWN && a->p[6] == AUTO_HOVER &&
        cmdTakenUnknown == tu0 + 1 && cmdTimeouts == to0 && !ackCached(s1),
        "taken, no result by 200 ms: BUSY TAKEN_UNKNOWN, uncached");
  CHECK(a && a->us - t0 >= 200000 && a->us - t0 <= 202000 && !pend.active, "at 200 ms (%lld us); link slot free",
        a ? (long long)(a->us - t0) : -1LL);

  /* The mailbox slot is still held: a new post is refused at once (BUSY, state). */
  const uint32_t busy0 = cmdBusy;
  const uint32_t s2 = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_DISARM, AUTO_FLAG_ACKREQ, sess, s2, NULL, 0);
  pass();
  a = ackOf(s2, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == (AUTO_DETAIL_BUSY_UNTAKEN | AUTO_HOVER) &&
        cmdBusy == busy0 + 1 && !ackCached(s2) && !pend.active, "slot held by a taken command: BUSY (UNTAKEN | state), uncached");

  /* The late result never reaches a later command. */
  CHECK(autoMbPublish(&mb, stale, AUTO_R_OK, 0xBADBADu, AUTO_LAND), "the stalled step publishes late");
  const uint32_t stale0 = mb.staleResults;
  const uint32_t s3 = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_DISARM, AUTO_FLAG_ACKREQ, sess, s3, NULL, 0);
  pass();
  CHECK(pend.active && ackOf(s3, n0) == NULL && mb.staleResults == stale0 + 1, "posted; the stale result dropped");
  stabStep(AUTO_R_OK, 0x1234u, AUTO_LAND);
  pass();
  a = ackOf(s3, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && autoGetU32(a->p + 8) == 0x1234u && a->p[6] == AUTO_LAND,
        "the new command gets its own result");
  fakeState = AUTO_IDLE;
}

/* A new HELLO while a command is pending: its ACK still goes to the session
 * and address it came from, and is not cached in the new session (whose
 * seq numbers restart). */
static void testSessionChangePending(void)
{
  vTaskDelay(1100);
  const uint32_t s1 = newSession();
  fakeState = AUTO_HOVER;
  const uint32_t seq = ++macSeq;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ, s1, seq, NULL, 0);
  pass();
  CHECK(pend.active && pend.session == s1, "LAND pending in session 1");
  const uint32_t s2 = newSession();
  CHECK(s2 != s1 && seq > macSeq, "session 2 (seq restarts below %u)", (unsigned)seq);
  stabStep(AUTO_R_OK, AUTO_HOVER, AUTO_LAND);
  int n0 = ncap;
  pass();
  pkt_t *a = ackOf(seq, n0);
  CHECK(a && a->p[4] == AUTO_T_LAND && a->h.session == s1 && sameAddr(&a->to, ADDR_A) && !ackCached(seq),
        "ACK in session 1, not cached in session 2");
  /* The same seq number in session 2 is its own command; its RETX replays its own answer. */
  macSeq = seq;
  macSend(0x34, AUTO_FLAG_ACKREQ, s2, seq, NULL, 0);
  n0 = ncap;
  macSend(0x34, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, s2, seq, NULL, 0);
  a = ackOf(seq, n0);
  CHECK(a && a->p[4] == 0x34 && a->p[5] == AUTO_R_BAD_STATE && a->h.session == s2, "replay is session 2's answer");
  fakeState = AUTO_IDLE;
}

/* With a fake built like the step-0 stub: every command BAD_STATE detail 0. */
static void testStubLike(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  stubLike = true;
  uint8_t pl[16];
  const autoProtoArm_t arm = {.mode = AUTO_MODE_BENCH};
  const autoProtoTakeoff_t to = {.token = 7, .heightCm = 30, .photoN = 1, .photoGapMs = 200};
  const uint8_t types[4] = {AUTO_T_ARM, AUTO_T_TAKEOFF, AUTO_T_LAND, AUTO_T_DISARM};
  for (int i = 0; i < 4; i++) {
    uint16_t len = 0;
    if (types[i] == AUTO_T_ARM) { autoProtoPackArm(&arm, pl); len = AUTO_ARM_LEN; }
    if (types[i] == AUTO_T_TAKEOFF) { autoProtoPackTakeoff(&to, pl); len = AUTO_TAKEOFF_LEN; }
    const uint32_t s = ++macSeq;
    const int n0 = ncap;
    macQueue(types[i], AUTO_FLAG_ACKREQ, sess, s, pl, len);
    pass();
    pkt_t *a = ackOf(s, n0);
    CHECK(a && a->p[4] == types[i] && a->p[5] == AUTO_R_BAD_STATE && autoGetU32(a->p + 8) == 0 && a->p[6] == AUTO_IDLE &&
          !pend.active, "stub: type 0x%02x -> BAD_STATE detail 0 in the same pass", types[i]);
  }
  stubLike = false;
}

static void testStatusPush(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  fakeState = AUTO_HOVER;
  memset(&fakeStatus, 0, sizeof(fakeStatus));
  fakeStatus.upMs = 4242;
  fakeStatus.state = AUTO_HOVER;
  fakeStatus.flags = AUTO_STF_WAITING;
  fakeStatus.waitLeftMs = 4100;
  fakeStatus.hMm = -5;
  fakeStatus.killCount = 3;
  fakeStatusSeq = 10;
  fakeStatus.transitions = 10;

  /* STATUS byte 7 = state; STATUS then AUTO_STATUS */
  int n0 = ncap;
  macSend(AUTO_T_STATUS_REQ, 0, sess, ++macSeq, NULL, 0);
  CHECK(ncap == n0 + 2 && cap[n0].h.type == AUTO_T_STATUS && cap[n0].p[7] == AUTO_HOVER &&
        cap[n0 + 1].h.type == AUTO_T_AUTO_STATUS && cap[n0 + 1].h.len == AUTO_AUTO_STATUS_LEN,
        "STATUS byte 7 = state, then AUTO_STATUS");
  autoStatusMsg_t m;
  CHECK(autoProtoParseAutoStatus(cap[n0 + 1].p, cap[n0 + 1].h.len, &m) && m.bootNonce == bootNonce && m.upMs == 4242 &&
        m.state == AUTO_HOVER && m.waitLeftMs == 4100 && m.hMm == -5 && m.killCount == 3 && m.transitions == 10,
        "AUTO_STATUS: the snapshot, bootNonce from the link");

  /* Active: RCVTIMEO 20 ms, a push every 100 ms. */
  pass();
  CHECK(lastRcvTimeoutMs == 20 && rxTimeoutMs == 20, "SO_RCVTIMEO 20 ms while active");
  n0 = ncap;
  runPasses(1000);
  int pushes = 0;
  int64_t prev = 0, minGap = 1 << 30, maxGap = 0;
  for (int i = n0; i < ncap; i++) {
    if (cap[i].h.type != AUTO_T_AUTO_STATUS) continue;
    if (pushes++ > 0) {
      const int64_t g = cap[i].us - prev;
      minGap = g < minGap ? g : minGap;
      maxGap = g > maxGap ? g : maxGap;
    }
    prev = cap[i].us;
  }
  CHECK(pushes >= 8 && pushes <= 10 && minGap >= 100000 && maxGap <= 120000,
        "active: %d pushes in 1 s, gaps %lld..%lld us", pushes, (long long)minGap, (long long)maxGap);

  /* A transition: pushed at once, with the new snapshot. */
  pass();
  n0 = ncap;
  fakeStatusSeq++;
  fakeStatus.transitions = fakeStatusSeq;
  fakeStatus.state = AUTO_LAND;
  const int64_t t0 = fakeUs;
  pass();
  pkt_t *a = lastOf(AUTO_T_AUTO_STATUS);
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && a->us - t0 <= 20000 && a->p[16] == AUTO_LAND &&
        autoGetU32(a->p + 8) == fakeStatusSeq, "transition pushed within one pass");

  /* The pushed snapshot is the one whose seq autonomyGetStatus returned. */
  pass();
  fakeStatusSeq++;
  bumpOnGet = true;   /* another transition between the check and the read */
  n0 = ncap;
  pass();
  a = lastOf(AUTO_T_AUTO_STATUS);
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && autoGetU32(a->p + 8) == fakeStatusSeq && statusSeqSent == fakeStatusSeq,
        "snapshot and seq from one read (seq %u)", (unsigned)fakeStatusSeq);
  n0 = ncap;
  pass();
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 0, "no second push for the same seq");

  /* Back to IDLE: at once, then every 500 ms for 2 s, then quiet. */
  fakeState = AUTO_IDLE;
  fakeStatusSeq++;
  n0 = ncap;
  pass();
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && lastRcvTimeoutMs == 100, "IDLE: pushed at once, RCVTIMEO 100 ms");
  n0 = ncap;
  runPasses(3000);
  const int tail = countSince(n0, AUTO_T_AUTO_STATUS);
  CHECK(tail >= 3 && tail <= 4, "tail: %d pushes at 500 ms in the 2 s after", tail);
  n0 = ncap;
  runPasses(3000);
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 0, "then quiet in IDLE");
  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(s.autoStatusTx == (uint32_t)countOf(AUTO_T_AUTO_STATUS), "stats: autoStatusTx %u", (unsigned)s.autoStatusTx);
}

/* Runs passes until the current set is captured and pushed. */
static void captureAndPush(void)
{
  for (int i = 0; i < 400 && photo.held && (photo.captured < photo.n || !photo.pushDone); i++) pass();
}

static void armSortie(uint32_t sess)
{
  fakeState = AUTO_IDLE;
  uint8_t pl[4];
  const autoProtoArm_t arm = {.mode = AUTO_MODE_DRY};
  autoProtoPackArm(&arm, pl);
  macQueue(AUTO_T_ARM, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, AUTO_ARM_LEN);
  pass();
  stabStep(AUTO_R_OK, 0x5EED0002u, AUTO_ARMED);
  pass();
  fakeState = AUTO_HOVER;
}

static void testPhotoRouting(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
  /* IDLE: an A1 set, still held when the sortie starts */
  uint8_t pl[4] = {1, 0, 0, 0};
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK && photo.held && photo.autoReq == 0, "IDLE: the A1 way");
  const uint16_t a1Id = photo.id;
  const uint32_t arms0 = armsAccepted;
  armSortie(sess);
  CHECK(armsAccepted == arms0 + 1, "ARM OK: a new sortie");

  /* The state machine's own set (STABLE + 1 s) releases the A1 set. */
  const uint32_t pre0 = photosPreempted, caps0 = autoCaptures;
  capPending = true; capReqId = 7; capN = 2; capGap = 150;
  int n0 = ncap;
  pass();
  CHECK(photosPreempted == pre0 + 1 && autoCaptures == caps0 + 1 && photo.held && photo.autoReq == 7 &&
        photo.id != a1Id && lastNote()->reqId == 7 && lastNote()->progress == AUTO_CAP_BUSY &&
        lastNote()->photoId == photo.id, "capture request: A1 set released, set started, BUSY reported");
  const uint16_t id7 = photo.id;
  captureAndPush();
  CHECK(notesFor(7, AUTO_CAP_DONE) == 1 && notesFor(7, AUTO_CAP_FAILED) == 0 && countSince(n0, AUTO_T_META) == 2 &&
        countSince(n0, AUTO_T_CHUNK) == 38, "DONE once; pushed in the hover");

  /* PHOTO while that set is held: BUSY, detail = held id, not posted, not cached. */
  const uint32_t posts0 = mb.posts, hb0 = photoHeldBusy;
  pl[0] = 1; autoPutU16(pl + 2, 150);
  uint32_t s = ++macSeq;
  n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pkt_t *a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == (AUTO_DETAIL_BUSY_HELD_SET | id7) && mb.posts == posts0 &&
        photoHeldBusy == hb0 + 1 && !ackCached(s), "held set: BUSY (HELD_SET | held id), not posted, uncached");

  /* PHOTO_FREE, then PHOTO while waiting: to the FSM; OK carries the photo id. */
  freeHeld(sess);
  CHECK(!photo.held && notesFor(7, AUTO_CAP_FAILED) == 0, "freed after its last frame: nothing more reported");
  s = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  CHECK(pend.active && pend.type == AUTO_T_PHOTO, "PHOTO in a sortie: posted");
  CHECK(stabPhotoOk(8, 1, 150) && lastTaken.type == AUTO_CMD_PHOTO && lastTaken.photoN == 1 && lastTaken.photoGapMs == 150,
        "the FSM got AUTO_CMD_PHOTO");
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && autoGetU32(a->p + 8) == photo.id && photo.id != AUTO_HOVER && a->p[6] == AUTO_HOVER &&
        photo.autoReq == 8 && notesFor(8, AUTO_CAP_BUSY) == 1 && ackCached(s), "OK: detail = photo id %u",
        a ? (unsigned)autoGetU32(a->p + 8) : 0);
  captureAndPush();
  CHECK(notesFor(8, AUTO_CAP_DONE) == 1, "DONE");
  freeHeld(sess);

  /* Active, not waiting: the FSM's BAD_STATE. */
  s = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  stabStep(AUTO_R_BAD_STATE, AUTO_HOVER, AUTO_HOVER);
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_BAD_STATE && autoGetU32(a->p + 8) == AUTO_HOVER && !photo.held, "not waiting: BAD_STATE");
  /* Range at the link (pad) */
  pl[1] = 1;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "PHOTO pad: RANGE");
  pl[1] = 0;

  /* A set left over from an earlier sortie is not held against a PHOTO. */
  capPending = true; capReqId = 9; capN = 1; capGap = 150;
  pass();
  captureAndPush();
  const uint16_t id9 = photo.id;
  CHECK(photo.held && photo.autoReq == 9 && notesFor(9, AUTO_CAP_DONE) == 1, "set 9 captured and held");
  armSortie(sess);
  s = ++macSeq;
  n0 = ncap;
  const uint32_t pre1 = photosPreempted;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  CHECK(pend.active, "earlier sortie's set held: PHOTO posted");
  stabPhotoOk(10, 1, 150);
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && photosPreempted == pre1 + 1 && photo.autoReq == 10 && photo.id != id9 &&
        autoGetU32(a->p + 8) == photo.id, "and its capture releases the old set");
  captureAndPush();
  freeHeld(sess);

  /* Camera not running at the request: FAILED at once, photo id 0. */
  camState = CAMERA_FLOW_FAILED;
  capPending = true; capReqId = 11; capN = 1; capGap = 150;
  pass();
  CHECK(notesFor(11, AUTO_CAP_FAILED) == 1 && lastNote()->photoId == 0 && !photo.held, "camera down: FAILED at once");
  camState = CAMERA_FLOW_RUNNING;
  /* A frame missing: FAILED once. */
  capPending = true; capReqId = 12; capN = 2; capGap = 150;
  pass();
  for (int i = 0; i < 100 && photo.captured < 1; i++) pass();
  camState = CAMERA_FLOW_FAILED;
  for (int i = 0; i < 400 && photo.captured < 2; i++) pass();
  camState = CAMERA_FLOW_RUNNING;
  CHECK(notesFor(12, AUTO_CAP_FAILED) == 1 && notesFor(12, AUTO_CAP_DONE) == 0 && photo.frame[1].missing,
        "missing frame: FAILED once");
  captureAndPush();
  freeHeld(sess);
  /* Released before its last frame: FAILED once. */
  capPending = true; capReqId = 13; capN = 3; capGap = 1000;
  pass();
  freeHeld(sess);
  CHECK(notesFor(13, AUTO_CAP_FAILED) == 1 && !photo.held, "freed before its last frame: FAILED once");
  fakeState = AUTO_IDLE;
}

static void testWindow(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_TAKEOFF;
  /* Camera awake: the state machine's set is captured inside the window; the push waits. */
  takeoffWin = true;
  camAwake = true;
  capPending = true; capReqId = 20; capN = 1; capGap = 150;
  int n0 = ncap;
  runPasses(300);
  CHECK(photo.captured == 1 && notesFor(20, AUTO_CAP_DONE) == 1 && countSince(n0, AUTO_T_META) == 0 &&
        countSince(n0, AUTO_T_CHUNK) == 0, "captured in the window, not pushed");
  takeoffWin = false;
  runPasses(300);
  CHECK(countSince(n0, AUTO_T_META) == 1 && countSince(n0, AUTO_T_CHUNK) == 19, "pushed after the window");
  freeHeld(sess);
  /* Camera not held awake: the capture waits for the window. */
  takeoffWin = true;
  camAwake = false;
  capPending = true; capReqId = 21; capN = 1; capGap = 150;
  runPasses(400);
  CHECK(photo.held && photo.captured == 0 && snap == 0, "camera not awake: no snapshot during the window");
  takeoffWin = false;
  runPasses(300);
  CHECK(photo.captured == 1 && notesFor(21, AUTO_CAP_DONE) == 1, "captured after the window");
  freeHeld(sess);
  fakeState = AUTO_IDLE;
}

/* A pending command polls every tick; the photo push keeps its pace. */
static void testPacing(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_HOVER;
  takeoffWin = true;
  camAwake = true;
  capPending = true; capReqId = 30; capN = 3; capGap = 150;
  runPasses(800);
  CHECK(photo.captured == 3 && photo.pushFrame == 0, "3 frames captured, push held");
  takeoffWin = false;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ, sess, ++macSeq, NULL, 0);
  pass();
  CHECK(pend.active, "LAND pending (stabilizer stalled)");
  const int n0 = ncap;
  const int64_t t0 = fakeUs;
  while (fakeUs - t0 < 90000) pass();
  const int sent = countSince(n0, AUTO_T_META) + countSince(n0, AUTO_T_CHUNK);
  CHECK(sent >= 16 && sent <= 24, "90 ms of 1 ms passes: %d photo packets (<= 4 per 18 ms)", sent);
  /* Let the command time out, then the push goes on at its normal pace. */
  runPasses(300);
  captureAndPush();
  CHECK(photo.pushDone && !pend.active, "push complete");
  freeHeld(sess);
  fakeState = AUTO_IDLE;
}

/* ---- review fixes ---- */

/* WP2-1: a KILL carries 0..32 payload bytes; a longer one is truncated by the
 * 64-byte receive buffer, fails the length check and is not acted on. */
static void testKillPayloadLimit(void)
{
  vTaskDelay(1100);   /* fresh budgets */
  const uint32_t sess = newSession();
  uint8_t pl[AUTO_KILL_MAX_PAYLOAD + 1];
  memset(pl, 0xA5, sizeof(pl));
  CHECK(AUTO_KILL_MAX_PAYLOAD == 32 && AUTO_RX_BUF_LEN == AUTO_PROTO_HDR_LEN + 32 + AUTO_PROTO_TAG_LEN,
        "KILL payload limit 32 bytes (64-byte receive buffer)");
  int k0 = killCalls, n0 = ncap;
  macQueueFrom(ADDR_B, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 501, pl, AUTO_KILL_MAX_PAYLOAD);
  pass();
  pkt_t *a = ackOf(501, n0);
  CHECK(killCalls == k0 + 1 && a && a->p[5] == AUTO_R_OK && sameAddr(&a->to, ADDR_B), "KILL with 32 bytes: acted on");
  const uint32_t af = authFails, kc = killRateCount;
  n0 = ncap;
  macQueueFrom(ADDR_B, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 502, pl, AUTO_KILL_MAX_PAYLOAD + 1);
  pass();
  CHECK(killCalls == k0 + 1 && authFails == af + 1 && ncap == n0 && killRateCount == kc + 1,
        "KILL with 33 bytes: truncated, authFails, not acted on, no reply");
  CHECK(session == sess && sameAddr(&peer, ADDR_A), "session and peer unchanged");
}

/* WP2-2: the AUTO_STATUS answering a duplicate STATUS_REQ from another
 * address does not use up the peer's push. */
static void testStatusDupNotPeer(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_HOVER;
  memset(&fakeStatus, 0, sizeof(fakeStatus));
  macSend(AUTO_T_STATUS_REQ, 0, sess, ++macSeq, NULL, 0);
  const uint32_t dupSeq = macSeq;
  CHECK(statusSeqSent == fakeStatusSeq && sameAddr(&lastOf(AUTO_T_AUTO_STATUS)->to, ADDR_A), "STATUS_REQ from the peer");
  /* A transition, then the captured STATUS_REQ replayed from C. */
  fakeStatusSeq++;
  fakeStatus.state = AUTO_LAND;
  const uint32_t sent0 = statusSeqSent, last0 = statusLastMs;
  int n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_STATUS_REQ, 0, sess, dupSeq, NULL, 0);
  CHECK(ncap == n0 + 2 && cap[n0].h.type == AUTO_T_STATUS && cap[n0 + 1].h.type == AUTO_T_AUTO_STATUS &&
        sameAddr(&cap[n0].to, ADDR_C) && sameAddr(&cap[n0 + 1].to, ADDR_C) && sameAddr(&peer, ADDR_A),
        "duplicate from C: answered at C, the peer stays");
  CHECK(statusSeqSent == sent0 && statusLastMs == last0, "the answer at C is not counted as the push");
  n0 = ncap;
  autoStatusStep(nowMs());
  pkt_t *a = lastOf(AUTO_T_AUTO_STATUS);
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && sameAddr(&a->to, ADDR_A) && a->p[16] == AUTO_LAND &&
        statusSeqSent == fakeStatusSeq, "the transition still goes to the peer at once");
  /* A duplicate from the peer itself is the push. */
  fakeStatusSeq++;
  n0 = ncap;
  macSend(AUTO_T_STATUS_REQ, 0, sess, dupSeq, NULL, 0);
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && statusSeqSent == fakeStatusSeq, "duplicate from the peer counts");
  n0 = ncap;
  autoStatusStep(nowMs());
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 0, "no second push for that seq");
  fakeState = AUTO_IDLE;
}

/* WP2-3: a replay of the session's own HELLO moves the peer (by design, plan
 * 3.2): the pushes go to the replayer until the daemon's next new packet. */
static void testHelloReplayMovesPeer(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  const uint32_t nonce = helloNonceNext - 1;
  const int notes0 = sessionNotes;
  fakeState = AUTO_HOVER;
  uint8_t pl[4];
  autoPutU32(pl, nonce);
  int n0 = ncap;
  macSendFrom(ADDR_C, AUTO_T_HELLO, 0, 0, 1, pl, 4);
  pkt_t *ha = lastOf(AUTO_T_HELLO_ACK);
  CHECK(session == sess && sessionNotes == notes0 && countSince(n0, AUTO_T_HELLO_ACK) == 1 && sameAddr(&ha->to, ADDR_C) &&
        autoGetU32(ha->p) == sess, "replayed HELLO from C: same session, not a new one, HELLO_ACK at C");
  CHECK(sameAddr(&peer, ADDR_C), "and the peer moves to C (by design)");
  fakeStatusSeq++;
  n0 = ncap;
  autoStatusStep(nowMs());
  CHECK(countSince(n0, AUTO_T_AUTO_STATUS) == 1 && sameAddr(&lastOf(AUTO_T_AUTO_STATUS)->to, ADDR_C),
        "the next push goes to C");
  /* The daemon's next heartbeat (a new seq) moves it back. */
  macSend(AUTO_T_HB, 0, sess, ++macSeq, (uint8_t[]){1, 2, 3, 4}, 4);
  fakeStatusSeq++;
  n0 = ncap;
  autoStatusStep(nowMs());
  CHECK(sameAddr(&peer, ADDR_A) && countSince(n0, AUTO_T_AUTO_STATUS) == 1 &&
        sameAddr(&lastOf(AUTO_T_AUTO_STATUS)->to, ADDR_A), "the daemon's next HB moves the pushes back");
  fakeState = AUTO_IDLE;
}

/* WP2-4 (accepted risk, docs "Known risk"): forged 0x13s without the key use
 * up the KILL budget, and a genuine KILL in that second is dropped. */
static void testForgedKillFlood(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  vTaskDelay(1100);   /* fresh windows */
  const uint32_t af = authFails, krd = killRateDrops;
  const int k0 = killCalls;
  uint8_t b[64];
  const int n = encodeMac(b, AUTO_T_KILL, 0, 0, 1, NULL, 0);
  b[n - 1] ^= 0x5A;   /* no key: a wrong tag */
  struct sockaddr_in from = ADDR_C;
  for (int i = 0; i < AUTO_KILL_RATE_MAX_PER_S; i++) handleDatagram(b, (size_t)n, &from);
  CHECK(authFails == af + AUTO_KILL_RATE_MAX_PER_S && killCalls == k0 && killRateDrops == krd,
        "100 forged KILLs: tag checks, none acted on");
  macSend(AUTO_T_KILL, AUTO_FLAG_ACKREQ, sess, 777, NULL, 0);
  CHECK(killCalls == k0 && killRateDrops == krd + 1, "a genuine KILL in the same second is dropped (killRateDrops)");
  vTaskDelay(1000);
  macSend(AUTO_T_KILL, AUTO_FLAG_ACKREQ, sess, 778, NULL, 0);
  CHECK(killCalls == k0 + 1, "in the next second it acts again");
}

/* WP2-5 / D2: PHOTO accepted by the state machine, but no set can start:
 * ACK OK with detail 0, FAILED reported at once, nothing held or pushed. The
 * same when the PSRAM slots cannot be allocated (slotsReady() fails). */
static void testPhotoOkNoSet(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  armSortie(sess);
  uint8_t pl[4] = {1, 0, 0, 0};
  autoPutU16(pl + 2, 150);
  const uint32_t caps0 = autoCaptures;

  /* Camera down. */
  camState = CAMERA_FLOW_FAILED;
  uint32_t s = ++macSeq;
  int n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  stabPhotoOk(40, 1, 150);
  pass();
  pkt_t *a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && autoGetU32(a->p + 8) == 0 && a->p[6] == AUTO_HOVER && ackCached(s),
        "camera down: PHOTO OK, detail 0 (no set started), cached");
  CHECK(notesFor(40, AUTO_CAP_FAILED) == 1 && notesFor(40, AUTO_CAP_BUSY) == 0 && lastNote()->photoId == 0 &&
        !photo.held && autoCaptures == caps0, "FAILED reported at once with photo id 0; nothing held");
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ | AUTO_FLAG_RETX, sess, s, pl, 4);
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && autoGetU32(a->p + 8) == 0 && !pend.active, "its RETX: the same OK detail 0, not re-posted");
  camState = CAMERA_FLOW_RUNNING;
  runPasses(300);
  CHECK(countSince(n0, AUTO_T_META) == 0 && countSince(n0, AUTO_T_CHUNK) == 0, "nothing pushed");

  /* No PSRAM slots (allocated at the first set, so free them first). */
  for (int i = 0; i < AUTO_PHOTO_SLOTS; i++) { heap_caps_free(slot[i]); slot[i] = NULL; }
  psramFail = true;
  s = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  stabPhotoOk(41, 1, 150);
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && autoGetU32(a->p + 8) == 0 && notesFor(41, AUTO_CAP_FAILED) == 1 &&
        lastNote()->photoId == 0 && !photo.held, "no PSRAM slots: PHOTO OK detail 0, FAILED");
  /* The state machine's own set (STABLE + 1 s) the same way. */
  capPending = true; capReqId = 42; capN = 2; capGap = 150;
  pass();
  CHECK(notesFor(42, AUTO_CAP_FAILED) == 1 && lastNote()->photoId == 0 && !photo.held && autoCaptures == caps0,
        "no PSRAM slots: the state machine's set FAILED at once");
  psramFail = false;
  capPending = true; capReqId = 43; capN = 1; capGap = 150;
  pass();
  CHECK(notesFor(43, AUTO_CAP_BUSY) == 1 && photo.held && slot[0] != NULL, "PSRAM back: the next set starts");
  captureAndPush();
  freeHeld(sess);

  /* A PHOTO's own set starts in the pass that answers it, with its id, even
   * when its capture request is visible before its result (the state machine
   * publishes both in one step; the link can poll in between): the per-pass
   * capture step leaves a request alone while a PHOTO is pending. */
  s = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s, pl, 4);
  pass();
  CHECK(pend.active, "PHOTO pending");
  capPending = true; capReqId = 44; capN = 1; capGap = 150;
  pass();
  CHECK(!photo.held && capPending && pend.active, "its capture request is not taken while the PHOTO is pending");
  stabStep(AUTO_R_OK, AUTO_HOVER, AUTO_HOVER);
  pass();
  a = ackOf(s, n0);
  CHECK(a && a->p[5] == AUTO_R_OK && photo.held && photo.autoReq == 44 && photo.id != 0 &&
        autoGetU32(a->p + 8) == photo.id, "PHOTO OK: its set started in the answering pass, detail = its id");
  captureAndPush();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
}

/* D1: every BUSY the link answers carries its kind; the state machine's own
 * BUSY keeps detail = the state (kind 0) and is cached. */
static void testBusyKinds(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  fakeState = AUTO_HOVER;
  macSeq = 0x12345677u;   /* a seq above 24 bits: the PENDING value is its low 24 bits */
  const uint32_t sLand = ++macSeq;
  macQueue(AUTO_T_LAND, AUTO_FLAG_ACKREQ, sess, sLand, NULL, 0);
  pass();
  const uint32_t s2 = ++macSeq;
  int n0 = ncap;
  macQueue(AUTO_T_DISARM, AUTO_FLAG_ACKREQ, sess, s2, NULL, 0);
  pass();
  pkt_t *a = ackOf(s2, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == 0x01345678u &&
        autoDetailKind(autoGetU32(a->p + 8)) == AUTO_DETAIL_BUSY_PENDING, "PENDING | (seq & 0xFFFFFF): 0x%08x",
        a ? (unsigned)autoGetU32(a->p + 8) : 0u);
  stabStep(AUTO_R_OK, AUTO_HOVER, AUTO_LAND);
  pass();
  CHECK(!pend.active, "LAND answered");

  /* The state machine's BUSY (PHOTO while its set is still captured): detail = state, kind 0, cached. */
  freeHeld(sess);
  armSortie(sess);
  uint8_t pl[4] = {1, 0, 0, 0};
  autoPutU16(pl + 2, 150);
  const uint32_t s3 = ++macSeq;
  n0 = ncap;
  macQueue(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, s3, pl, 4);
  pass();
  stabStep(AUTO_R_BUSY, AUTO_HOVER, AUTO_HOVER);
  pass();
  a = ackOf(s3, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == AUTO_HOVER && autoDetailKind(autoGetU32(a->p + 8)) == 0 &&
        ackCached(s3), "the state machine's BUSY: detail = state (kind 0), cached");
  /* CHUNKS BUSY (frame not captured yet) keeps detail 0, as in A1. */
  capPending = true; capReqId = 50; capN = 2; capGap = 1000;
  pass();
  uint8_t ch[6];
  autoPutU16(ch, photo.id); ch[2] = 1; ch[3] = 1; autoPutU16(ch + 4, 0);
  const uint32_t s4 = ++macSeq;
  n0 = ncap;
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, s4, ch, 6);
  a = ackOf(s4, n0);
  CHECK(a && a->p[5] == AUTO_R_BUSY && autoGetU32(a->p + 8) == 0, "CHUNKS BUSY: detail 0 (A1)");
  captureAndPush();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
}

/* WP2-8: while a photo set is captured or pushed the task polls every 20 ms
 * (non-blocking receives, then a 20 ms delay): a KILL arriving at any point
 * of that delay is acted on within 20 ms, and before its ACK. Idle, the
 * receive returns on arrival. */
static void testKillDuringPush(void)
{
  vTaskDelay(1100);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
  uint8_t pl[4] = {3, 0, 0, 0};
  autoPutU16(pl + 2, 300);   /* 3 frames 300 ms apart: still capturing and pushing throughout */
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK && photo.held, "a 3-frame set");
  pass();
  pass();
  const int offsets[] = {1000, 5000, 10000, 15000, 19000, 19900};
  int64_t worst = 0;
  bool allPushing = true, allBeforeAck = true;
  const int blk0 = recvBlocking, recv0 = recvCalls;
  const int64_t loop0 = fakeUs;
  for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
    const int k0 = killCalls;
    const int64_t at = fakeUs + offsets[i];
    const uint32_t seq = 600 + (uint32_t)i;
    const int n0 = ncap;
    macQueueAt(ADDR_B, at, AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, seq, NULL, 0);
    for (int j = 0; j < 10 && killCalls == k0; j++) pass();
    const int64_t lat = killCalls == k0 + 1 ? killUs - at : 1 << 30;
    worst = lat > worst ? lat : worst;
    allPushing = allPushing && pushingAtKill;
    pkt_t *a = ackOf(seq, n0);
    allBeforeAck = allBeforeAck && a && a->us == killUs && sendsAtKill < sendCalls;
  }
  CHECK(worst <= AUTO_PUSH_GAP_MS * 1000 && allPushing && allBeforeAck,
        "KILL while pushing: acted on within %lld us (<= 20 ms), before its ACK", (long long)worst);
  CHECK(recvBlocking == blk0 && recvCalls > recv0 && fakeUs - loop0 <= 12 * AUTO_PUSH_GAP_MS * 1000,
        "while pushing: non-blocking receives every 20 ms, never a blocking wait");
  CHECK(photo.held && !photo.pushDone, "the set was still being sent");
  captureAndPush();
  freeHeld(sess);
  /* Idle (no set, no command): the receive returns on arrival, active or not. */
  for (int act = 0; act < 2; act++) {
    fakeState = act ? AUTO_HOVER : AUTO_IDLE;
    pass();
    const int k0 = killCalls;
    const int64_t at = fakeUs + 7000;
    macQueueAt(ADDR_B, at, AUTO_T_KILL, 0, 0, 700 + (uint32_t)act, NULL, 0);
    for (int j = 0; j < 10 && killCalls == k0; j++) pass();
    CHECK(killCalls == k0 + 1 && killUs == at, "%s, idle: KILL acted on at arrival", act ? "active" : "IDLE");
  }
  fakeState = AUTO_IDLE;
  runPasses(3000);
}

/* Colour photos (photocol 1, docs/autonomy-link.md "Photos"): a 44-byte META with the pixel format, 38400 bytes in
 * 38 chunks (the last 512), the CRC over the whole YUYV frame; CHUNKS up to chunk 37, past it RANGE; a grey set
 * after it is the A1 META again (40 bytes); a snapshot whose length does not match its format is sent as missing. */
static void testColourPhoto(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
  camState = CAMERA_FLOW_RUNNING;
  camColour = true;
  uint8_t pl[8] = {2, 0, 0, 0};
  autoPutU16(pl + 2, 200);
  int n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  pkt_t *a = lastOf(AUTO_T_ACK);
  const uint16_t id = a ? (uint16_t)autoGetU32(a->p + 8) : 0;
  CHECK(a && a->p[5] == AUTO_R_OK && id != 0, "colour PHOTO OK");
  runLoop(3000);
  static uint8_t img[2][38400];
  uint32_t crcs[2] = {0};
  int metas = 0, chunks = 0, bad = 0;
  for (int i = n0; i < ncap; i++) {
    const uint8_t *p = cap[i].p;
    if (cap[i].h.type == AUTO_T_META) {
      metas++;
      crcs[p[2] & 1] = autoGetU32(p + 12);
      bad += !(cap[i].h.len == AUTO_META_COLOR_LEN && autoGetU16(p) == id && autoGetU16(p + 4) == 160 &&
               autoGetU16(p + 6) == 120 && autoGetU32(p + 8) == 38400 && autoGetU16(p + 34) == 1024 &&
               autoGetU16(p + 36) == 38 && p[40] == AUTO_PIXFMT_YUYV && p[41] == 0 && p[42] == 0 && p[43] == 0 &&
               p[32] == 97 && p[33] == 31 && autoGetU16(p + 38) == 3870);
    } else if (cap[i].h.type == AUTO_T_CHUNK) {
      chunks++;
      const uint16_t ci = autoGetU16(p + 4), dl = autoGetU16(p + 6);
      bad += !(ci < 38 && dl == (ci == 37 ? 512 : 1024) && cap[i].h.len == 8 + dl);
      if (ci < 38) memcpy(img[p[2] & 1] + ci * 1024, p + 8, dl);
    }
  }
  CHECK(metas == 2 && chunks == 76 && bad == 0, "2 colour METAs (44 bytes, YUYV, 38400, 38 chunks) + 76 CHUNKs "
        "(got %d, %d, %d bad)", metas, chunks, bad);
  CHECK(crc32(0, img[0], 38400) == crcs[0] && crc32(0, img[1], 38400) == crcs[1] && img[0][0] == 0 &&
        img[0][1] == 40 && img[0][3] == 200 && img[0][2 * 161] == 10, "the CRC covers the whole YUYV frame; Y and "
        "chroma as captured");
  /* CHUNKS: 34..37 of frame 1 are resent; 36 + 4 is past the end */
  autoPutU16(pl, id); pl[2] = 1; pl[3] = 4; autoPutU16(pl + 4, 34);
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_OK, "CHUNKS 34..37 of a colour frame OK");
  n0 = ncap;
  runLoop(200);
  CHECK(countSince(n0, AUTO_T_CHUNK) == 4 && autoGetU16(cap[ncap - 1].p + 4) == 37 && autoGetU16(cap[ncap - 1].p + 6) == 512,
        "4 chunks resent, the last 512 bytes");
  autoPutU16(pl + 4, 36);
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "CHUNKS past chunk 37: RANGE");
  pl[3] = 0; autoPutU16(pl + 4, 0);
  n0 = ncap;
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 6);
  runLoop(100);
  CHECK(countSince(n0, AUTO_T_META) == 1 && lastOf(AUTO_T_META)->h.len == AUTO_META_COLOR_LEN, "its META again: 44 bytes");
  freeHeld(sess);

  /* grey again (photocol 0): the A1 META, chunks past 18 RANGE */
  camColour = false;
  pl[0] = 1; pl[1] = 0; autoPutU16(pl + 2, 0);
  n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  const uint16_t id2 = (uint16_t)autoGetU32(lastOf(AUTO_T_ACK)->p + 8);
  runLoop(1500);
  pkt_t *m = lastOf(AUTO_T_META);
  CHECK(m && m->h.len == AUTO_META_LEN && autoGetU32(m->p + 8) == 19200 && autoGetU16(m->p + 36) == 19 &&
        countSince(n0, AUTO_T_CHUNK) == 19, "a grey frame: the 40-byte META, 19 chunks");
  autoPutU16(pl, id2); pl[2] = 0; pl[3] = 2; autoPutU16(pl + 4, 18);
  macSend(AUTO_T_CHUNKS, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 6);
  CHECK(lastOf(AUTO_T_ACK)->p[5] == AUTO_R_RANGE, "CHUNKS 18..19 of a grey frame: RANGE");
  freeHeld(sess);

  /* a snapshot whose length is not its format's: missing (META size 0) */
  camColour = true;
  pl[0] = 1; pl[1] = 0; autoPutU16(pl + 2, 0);
  n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  for (int t = 0; t < 400 && !photo.frame[0].done; t += AUTO_PUSH_GAP_MS) {
    if (t % 60 == 0) {
      camTick();
      if (snap == 2) snapM->bytes = 19200;   /* YUYV with a grey length */
    }
    photoStep(nowMs());
    vTaskDelay(AUTO_PUSH_GAP_MS);
  }
  runLoop(300);
  m = lastOf(AUTO_T_META);
  CHECK(countSince(n0, AUTO_T_META) == 1 && m->h.len == AUTO_META_LEN && autoGetU32(m->p + 8) == 0 &&
        autoGetU16(m->p + 36) == 0 && countSince(n0, AUTO_T_CHUNK) == 0, "inconsistent snapshot: sent as missing");
  freeHeld(sess);
  camColour = false;
}

/* Always-on colour (photocol 1, the split): the photo is written by the driver's copy task straight into the slot
 * (cam_split_core.h). A 2-frame set: the 44-byte METAs and the raw YUYV of a whole frame each, the CRC over it. Then a
 * capture stalled after 5 of its 10 chunks: the 1000 ms frame timeout cancels it (a missing frame), and the frame's 5
 * late chunks, if the driver ever delivered them, leave the slot untouched (the generation check). */
static void testSplitPhoto(void)
{
  vTaskDelay(3000);
  const uint32_t sess = newSession();
  freeHeld(sess);
  fakeState = AUTO_IDLE;
  camState = CAMERA_FLOW_RUNNING;
  camSplit = true;
  camSplitInit(&split);
  uint8_t pl[8] = {2, 0, 0, 0};
  autoPutU16(pl + 2, 150);
  int n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  pkt_t *a = lastOf(AUTO_T_ACK);
  CHECK(a && a->p[5] == AUTO_R_OK, "split: colour PHOTO OK");
  runLoop(3000);
  static uint8_t img[2][38400];
  uint32_t crcs[2] = {0};
  int metas = 0, chunks = 0, bad = 0;
  for (int i = n0; i < ncap; i++) {
    const uint8_t *p = cap[i].p;
    if (cap[i].h.type == AUTO_T_META) {
      metas++;
      crcs[p[2] & 1] = autoGetU32(p + 12);
      bad += !(cap[i].h.len == AUTO_META_COLOR_LEN && autoGetU32(p + 8) == 38400 && autoGetU16(p + 36) == 38 &&
               p[40] == AUTO_PIXFMT_YUYV);
    } else if (cap[i].h.type == AUTO_T_CHUNK) {
      chunks++;
      const uint16_t ci = autoGetU16(p + 4), dl = autoGetU16(p + 6);
      if (ci < 38) memcpy(img[p[2] & 1] + ci * 1024, p + 8, dl);
    }
  }
  int pixBad = 0;
  for (int fr = 0; fr < 2; fr++) {
    /* the frame each request latched: Y the pattern, the chroma its frame number */
    const uint8_t f = (uint8_t)(img[fr][1] - 40);
    for (uint32_t i = 0; i < 38400; i++) pixBad += img[fr][i] != splitYuyv(f, i, i & 1);
  }
  CHECK(metas == 2 && chunks == 76 && bad == 0 && pixBad == 0 && crc32(0, img[0], 38400) == crcs[0] &&
        crc32(0, img[1], 38400) == crcs[1] && img[0][1] != img[1][1],
        "split: 2 colour METAs, 76 CHUNKs, each the raw YUYV of one whole frame, the CRC over it (%d %d %d %d)",
        metas, chunks, bad, pixBad);
  CHECK(split.done == 2 && split.arm == 2 && split.cancel == 0 && !camSplitPending(&split),
        "split: 2 armed, 2 filled, idle");
  freeHeld(sess);

  /* the camera stalls after chunk 4 of the photo frame */
  pl[0] = 1; pl[1] = 0; autoPutU16(pl + 2, 0);
  n0 = ncap;
  macSend(AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, sess, ++macSeq, pl, 4);
  for (int t = 0; t < 200 && !photo.reqActive; t += AUTO_PUSH_GAP_MS) {   /* the request, no frame yet */
    photoStep(nowMs());
    vTaskDelay(AUTO_PUSH_GAP_MS);
  }
  splitStallAt = 5;
  camTick();   /* the next frame starts: 5 chunks, then the camera stalls */
  splitStallAt = -1;
  CHECK(__atomic_load_n(&split.state, __ATOMIC_ACQUIRE) == CAM_SPLIT_CAPTURING && photo.reqActive,
        "split: capturing, stalled after 5 chunks");
  uint8_t *const s0 = slot[0];
  static uint8_t before[38400];
  runLoop(1200);   /* the frame timeout: cancelled, sent as missing */
  pkt_t *m = lastOf(AUTO_T_META);
  CHECK(countSince(n0, AUTO_T_META) == 1 && m && autoGetU32(m->p + 8) == 0 && !photo.reqActive &&
        split.cancel == 1 && !camSplitPending(&split), "split: the 1000 ms timeout cancelled the capture (missing)");
  memcpy(before, s0, sizeof(before));
  splitChunks(splitStalledFrame, splitStalledIdx, 5, 10);   /* the stalled frame's late chunks */
  CHECK(memcmp(before, s0, sizeof(before)) == 0 && !camSplitPending(&split),
        "split: the late chunks after the cancel leave the slot untouched");
  camState = CAMERA_FLOW_RUNNING;
  freeHeld(sess);
  runLoop(300);
  camSplit = false;
}

static void testStats(void)
{
  autoLinkStats_t s;
  autoLinkGetStats(&s);
  CHECK(s.autoCmds == autoCmds && s.cmdTimeouts >= 1 && s.cmdTakenUnknown >= 1 && s.cmdBusy >= 2 &&
        s.autoCaptures == autoCaptures && s.photosPreempted >= 2 && s.photoHeldBusy >= 1 && s.killRateDrops >= 6,
        "stats filled: autoCmds=%u cmdTimeouts=%u cmdTakenUnknown=%u cmdBusy=%u autoCaptures=%u photosPreempted=%u "
        "photoHeldBusy=%u killRateDrops=%u", (unsigned)s.autoCmds, (unsigned)s.cmdTimeouts,
        (unsigned)s.cmdTakenUnknown, (unsigned)s.cmdBusy, (unsigned)s.autoCaptures, (unsigned)s.photosPreempted,
        (unsigned)s.photoHeldBusy, (unsigned)s.killRateDrops);
}

int main(void)
{
  for (int i = 0; i < 16; i++) testKey[i] = (uint8_t)i;
  autoMbInit(&mb);
  autoLinkInit();
  CHECK(running && keyLoaded, "init");
  CHECK(keyFp[0] == 0xbe && keyFp[1] == 0x45 && keyFp[2] == 0xcb && keyFp[3] == 0x26, "fp be45cb26");
  rxTimeoutMs = AUTO_RX_TIMEOUT_MS;   /* as openSocket() */

  testTaskAtBoot();
  testNoSession();
  testA1();
  testKill();
  testPeerRule();
  testKillBudget();
  testCommands();
  testGroundStandbyPhoto();
  testStalled();
  testTakenUnknown();
  testSessionChangePending();
  testStubLike();
  testStatusPush();
  testPhotoRouting();
  testWindow();
  testPacing();
  testKillPayloadLimit();
  testStatusDupNotPeer();
  testHelloReplayMovesPeer();
  testForgedKillFlood();
  testPhotoOkNoSet();
  testBusyKinds();
  testKillDuringPush();
  testColourPhoto();
  testSplitPhoto();
  testStats();
  testTaskFallback();
  printf("%d checks, %d failures (tx=%u nomem=%u accepted=%u dups=%u kills=%u autoCmds=%u autoStatusTx=%u)\n", checks,
         failures, (unsigned)txPackets, (unsigned)txNoMem, (unsigned)accepted, (unsigned)dups, (unsigned)kills,
         (unsigned)autoCmds, (unsigned)autoStatusTx);
  return failures != 0;
}
