/*
 * ESP-Drone Firmware
 *
 * auto_link.c - Autonomy link to the Mac on UDP 2391 (docs/autonomy-link.md)
 *
 * Stage A1: HELLO, HB, STATUS_REQ, PHOTO, CHUNKS and PHOTO_FREE. Stage A2b
 * (docs/autonomy-a2b.md section 3): ARM, TAKEOFF, LAND and DISARM (and PHOTO
 * while autonomy is active) are posted into autonomy.h's mailbox and answered
 * when the state machine has processed them, while the task goes on serving
 * every other packet; KILL calls autonomyKillFromLink() before anything else;
 * AUTO_STATUS is pushed; the photo sets the state machine starts are captured
 * and pushed in the air. One task owns the socket and all link state; the USB
 * console only reads counters. Nothing here writes the setpoint, the motors
 * (the KILL stop is autonomy.c's) or the drift hold.
 */
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"
#include "lwip/sockets.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"

#include "auto_link.h"
#include "auto_proto.h"
#include "autonomy.h"
#include "camera_flow.h"
#include "camera_jpeg.h"
#include "flight_recorder.h"
#include "imu_level_cal_store.h"
#include "motors.h"
#include "stabilizer.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "AUTOLINK"
#include "debug_cf.h"

#define AUTO_LINK_TASK_NAME     "AUTOLINK"
#define AUTO_LINK_STACK_PSRAM   8192   /* bytes, in PSRAM (see linkTaskStart) */
#define AUTO_LINK_STACK_INTERNAL 6144  /* fallback without PSRAM */
#define AUTO_LINK_TASK_PRI      1      /* below CRTP RX (2): the phone app comes first */
#define AUTO_LINK_TASK_CORE     0
#define AUTO_START_DELAY_MS     1000
#define AUTO_RX_TIMEOUT_MS      100
#define AUTO_RX_TIMEOUT_ACTIVE_MS 20   /* autonomy active: capture requests, AUTO_STATUS and HB within 20 ms */
/* Mac packets are at most 64 bytes: TAKEOFF is 44 (50 with a planned move), and a KILL may carry up
 * to AUTO_KILL_MAX_PAYLOAD (32) ignored bytes. A longer datagram is truncated
 * here and then fails the length check (authFails): it is never acted on. */
#define AUTO_RX_BUF_LEN         64
#define AUTO_KILL_MAX_PAYLOAD   (AUTO_RX_BUF_LEN - AUTO_PROTO_HDR_LEN - AUTO_PROTO_TAG_LEN)
#define AUTO_RX_PER_LOOP        4
#define AUTO_RATE_MAX_PER_S     60
/* 0x13 datagrams have their own budget, counted before the tag check, so no
 * storm of other packets can drop a KILL; a real burst is 10-20 packets.
 * Accepted risk (docs/autonomy-link.md, KILL): any station that joins the
 * softAP (stock password) can spend this budget with forged 0x13s, without
 * the key, and so drop a genuine KILL in that second. */
#define AUTO_KILL_RATE_MAX_PER_S 100
#define AUTO_TX_TRIES           3      /* control replies on a full Wi-Fi TX queue */

/* Photos: 160x120 frames from the flow camera, copied into PSRAM: grey (19200 bytes, 19 chunks) or, with NVS
 * photocol 1 (opt-in, default 0), YUYV colour (38400 bytes, 38 chunks; a 44-byte META carries the pixel format,
 * docs/autonomy-link.md "Photos"). The slots hold either. In colour the camera driver's copy task writes the raw
 * frame straight into the slot (docs/autonomy-a2b.md 28, cam_split_core.h): the same wire format, the photo ~70-190
 * ms later than a grey one. */
#define AUTO_PHOTO_SLOTS        3
#define AUTO_FRAME_W            CAMERA_NATIVE_WIDTH
#define AUTO_FRAME_H            CAMERA_NATIVE_HEIGHT
#define AUTO_FRAME_MAX_BYTES    CAMERA_FLOW_SNAPSHOT_MAX_BYTES
#define AUTO_FRAME_CHUNKS_MAX   ((AUTO_FRAME_MAX_BYTES + AUTO_CHUNK_DATA_MAX - 1) / AUTO_CHUNK_DATA_MAX)
#define AUTO_PHOTO_MAX_GAP_MS   1000
#define AUTO_FRAME_TIMEOUT_MS   1000   /* camera stalled, restarting or asleep: the frame is reported missing */
/* A PHOTO that wakes the camera from its ground standby (camera_flow.h,
 * docs/autonomy-a2b.md 17.1) is answered OK at once; its frames wait for this
 * many fresh frames after the wake (auto-exposure settles in frames 4-6 of
 * the logged wakes, 0.23-0.47 s), at most AUTO_CAM_WAKE_MAX_MS after the
 * PHOTO; after that the frame timeout reports a camera that never woke. */
#define AUTO_CAM_WAKE_FRAMES    6
#define AUTO_CAM_WAKE_MAX_MS    2500
#define AUTO_PHOTO_HOLD_MS      30000  /* auto-free this long after the push ... */
#define AUTO_PHOTO_MAX_HOLD_MS  60000  /* ... and in any case this long after the PHOTO */
#define AUTO_CHUNKS_MAX         8
#define AUTO_RESEND_QUEUE       4      /* CHUNKS requests waiting to be served */
/* Push pacing: a few packets, then a pause (~200 KB/s): keeps the Wi-Fi TX
 * queue (and the supply) from being flooded. A burst goes at most every
 * AUTO_PUSH_MIN_GAP_MS, also while the loop polls a pending command every
 * tick (a 20-tick delay can last 19 ms). */
#define AUTO_PUSH_BURST         4
#define AUTO_PUSH_GAP_MS        20
#define AUTO_PUSH_MIN_GAP_MS    (AUTO_PUSH_GAP_MS - 2)

/* AUTO_STATUS push (with a session and a peer): on every transition, every
 * 100 ms while active, every 500 ms for 2 s after returning to IDLE. */
#define AUTO_STATUS_ACTIVE_MS   100
#define AUTO_STATUS_TAIL_MS     500
#define AUTO_STATUS_TAIL_FOR_MS 2000

_Static_assert(CAMERA_FLOW_SNAPSHOT_BYTES == AUTO_FRAME_W * AUTO_FRAME_H &&
               AUTO_FRAME_MAX_BYTES == 2 * AUTO_FRAME_W * AUTO_FRAME_H, "frame sizes");
/* Colour (cam_split_core.h): a cancel (the frame timeout, photoRelease) never runs inside one of the camera driver's
 * chunk copies into a slot only while this task shares core 0 with the driver's copy task (priority 23;
 * cam_split.c checks CONFIG_CAMERA_CORE0); the snapshot core's generation check is the second line. */
_Static_assert(AUTO_LINK_TASK_CORE == 0, "auto_link shares core 0 with the camera driver's copy task");
_Static_assert(CAMERA_FLOW_PIXFMT_GRAY8 == AUTO_PIXFMT_GRAY8 && CAMERA_FLOW_PIXFMT_YUYV == AUTO_PIXFMT_YUYV,
               "the snapshot's pixel format is the META's");
_Static_assert(AUTO_CHUNK_HDR_LEN + AUTO_CHUNK_DATA_MAX <= AUTO_PROTO_MAX_PAYLOAD, "chunk fits a packet");
_Static_assert(AUTO_RX_BUF_LEN >= AUTO_PROTO_HDR_LEN + AUTO_TAKEOFF_PLAN_LEN + AUTO_PROTO_TAG_LEN &&
               AUTO_RX_BUF_LEN >= AUTO_PROTO_HDR_LEN + AUTO_CHUNKS_LEN + AUTO_PROTO_TAG_LEN, "rx buffer");
_Static_assert(AUTO_KILL_MAX_PAYLOAD == 32, "docs/autonomy-link.md states a KILL payload of 0..32 bytes");

typedef enum { TX_OK, TX_NOMEM, TX_FAIL } txResult_t;

/* Answer to an accepted command, re-sent unchanged for a duplicate seq. */
typedef struct {
  uint32_t seq;
  uint32_t detail;
  uint8_t type;
  uint8_t result;
  uint8_t state;
  bool valid;
} ackCacheEntry_t;

typedef struct {
  bool done;                  /* captured, or given up */
  bool missing;               /* no frame: META only, size 0 */
  cameraFlowSnapshot_t snap;  /* written by the camera task while requested */
  uint32_t crc;
  int16_t roll, pitch, yaw;   /* when the copy was seen, 0.01 deg */
  uint16_t thrust;
  uint16_t vbatMv;
} autoFrame_t;

typedef struct {
  uint8_t idx;
  uint8_t count;
  uint16_t first;
} resendReq_t;

typedef struct {
  bool held;
  uint16_t id;
  uint8_t n;
  uint16_t gapMs;
  uint32_t acceptedMs;
  /* a set the state machine asked for (autonomyTakeCaptureRequest) */
  uint32_t autoReq;           /* its reqId, 0: an A1 set */
  uint32_t sortie;            /* armsAccepted when it started */
  /* capture */
  uint8_t captured;           /* frames done */
  bool anyMissing;
  bool encodeActive;
  bool reqActive;             /* a camera snapshot request is outstanding */
  uint32_t reqMs;
  uint32_t nextMs;            /* next request due */
  /* push */
  uint8_t pushFrame;
  int16_t pushChunk;          /* -1: META next */
  bool pushDone;
  uint32_t pushDoneMs;
  resendReq_t resend[AUTO_RESEND_QUEUE];
  uint8_t resendHead;
  uint8_t resendCount;
  uint8_t resendNext;         /* progress within resend[resendHead] */
  autoFrame_t frame[AUTO_PHOTO_SLOTS];
} autoPhoto_t;

/* The one command given to the state machine and not answered yet (section
 * 3.1). Its ACK goes to the address and session it came from. */
typedef struct {
  bool active;
  bool taken;                 /* the state machine took it: its result is awaited to AUTONOMY_CMD_WAIT_HARD_MS */
  uint8_t type;               /* wire type */
  uint8_t flags;
  uint32_t ticket;
  uint32_t seq;
  uint32_t session;
  uint32_t t0;
  struct sockaddr_in from;
} pendCmd_t;

static uint8_t key[AUTO_PROTO_KEY_LEN];
static uint8_t keyFp[4];
static bool keyLoaded;
static uint32_t bootNonce;
static volatile bool running;
static TaskHandle_t taskHandle;
static bool stackInPsram;
static volatile bool portOpen;
static int sock = -1;
static uint32_t rxTimeoutMs;            /* SO_RCVTIMEO now */
static uint8_t *txBuf;                  /* AUTO_PROTO_MAX_PACKET, PSRAM (internal without PSRAM) */
static mbedtls_md_context_t hmacCtx;    /* set up once: no allocation per packet */

/* Link task only (the console reads a few scalars). */
static struct sockaddr_in peer;         /* set by a HELLO and by a new packet of the current session only */
static volatile bool havePeer;
static volatile uint32_t session;       /* 0 = none */
static uint32_t txSeq;
static autoProtoSeqFilter_t seqFilter;
static ackCacheEntry_t ackCache[AUTO_PROTO_SEQ_RECENT];
static uint8_t ackCacheHead;
static uint32_t helloNonce, helloSeq;   /* the HELLO that opened the session */
static uint32_t rateWindowMs, rateCount;
static uint32_t killWindowMs, killRateCount;
static volatile bool haveHb;
static volatile uint32_t lastHbMs;
static pendCmd_t pend;
static uint32_t armsAccepted;           /* ARMs the state machine accepted: each starts a sortie */
static uint32_t statusSeqSent, statusLastMs, idleSinceMs;
static bool wasActive, statusTail;

static uint8_t *slot[AUTO_PHOTO_SLOTS]; /* PSRAM, AUTO_FRAME_MAX_BYTES each, allocated at the first PHOTO, kept */
static autoPhoto_t photo;
static uint16_t photoIdLast;
static uint32_t pushLastMs;

static volatile uint32_t rxDatagrams, rateDrops, authFails, accepted, dups, oldSeqs, badSessions;
static volatile uint32_t txPackets, txErrors, txNoMem, photosAccepted;
static volatile uint32_t kills, killRateDrops, autoCmds, cmdBusy, cmdTimeouts, cmdTakenUnknown;
static volatile uint32_t autoStatusTx, autoCaptures, photosPreempted, photoHeldBusy;
static volatile uint8_t lastAckType, lastAckResult;
static volatile bool anyAck;

static uint32_t nowMs(void)
{
  return (uint32_t)(esp_timer_get_time() / 1000);
}

static void hmacSha256(const uint8_t *k, size_t kLen, const uint8_t *data, size_t len, uint8_t out[32])
{
  if (mbedtls_md_hmac_starts(&hmacCtx, k, kLen) != 0 || mbedtls_md_hmac_update(&hmacCtx, data, len) != 0 ||
      mbedtls_md_hmac_finish(&hmacCtx, out) != 0) {
    /* Never a usable tag. */
    esp_fill_random(out, 32);
  }
}

void autoLinkKeyFingerprint(const uint8_t k[16], uint8_t fp[4])
{
  uint8_t digest[32];
  if (mbedtls_sha256(k, AUTO_PROTO_KEY_LEN, digest, 0) != 0) {
    memset(digest, 0, sizeof(digest));
  }
  memcpy(fp, digest, 4);
}

const char *autoLinkResultName(uint8_t result)
{
  static const char *const names[] = {
    "OK", "BAD_STATE", "BUSY", "RANGE", "TAKEOFF_WINDOW", "CAM_UNAVAIL", "LOW_BATT", "APP_ACTIVE",
    "KILL_LATCHED", "POWER_DEFER", "NO_PHOTO", "MEM", "GUARDS", "STALE_FRAME", "BAD_SESSION",
  };
  return result < sizeof(names) / sizeof(names[0]) ? names[result] : "?";
}

/* ---- sending ---------------------------------------------------------- */

/* Payloads are built in place, right after the header. */
static uint8_t *txPayload(void)
{
  return txBuf + AUTO_PROTO_HDR_LEN;
}

/* Frames the payload already at txPayload(); returns the packet length. */
static int encode(uint8_t type, uint32_t hdrSession, uint16_t len)
{
  const autoProtoHdr_t h = {.type = type, .flags = 0, .session = hdrSession, .seq = ++txSeq, .len = len};
  return autoProtoEncode(&h, txPayload(), key, hmacSha256, txBuf, AUTO_PROTO_MAX_PACKET);
}

/* to: the sender of the packet answered, or NULL for the peer. */
static txResult_t transmit(int n, const struct sockaddr_in *to)
{
  if (to == NULL && havePeer) {
    to = &peer;
  }
  if (n < 0 || to == NULL) {
    txErrors++;
    return TX_FAIL;
  }
  if (sendto(sock, txBuf, (size_t)n, 0, (const struct sockaddr *)to, sizeof(*to)) == n) {
    txPackets++;
    return TX_OK;
  }
  if (errno == ENOMEM || errno == ENOBUFS || errno == EAGAIN) {
    txNoMem++;
    return TX_NOMEM;
  }
  txErrors++;
  return TX_FAIL;
}

/* Bulk data (META, CHUNK) to the peer: one try; the push retries later on a
 * full queue. */
static txResult_t sendData(uint8_t type, uint16_t len)
{
  return transmit(encode(type, session, len), NULL);
}

/* Replies: a few tries on a full TX queue (the Mac retries as well). */
static void sendReply(uint8_t type, uint32_t hdrSession, uint16_t len, const struct sockaddr_in *to)
{
  const int n = encode(type, hdrSession, len);
  for (int i = 0; i < AUTO_TX_TRIES; i++) {
    if (transmit(n, to) != TX_NOMEM) {
      return;
    }
    vTaskDelay(M2T(2));
  }
}

/* state: ACK byte 6, the autonomy state (of the step that processed a
 * command; autonomyState() for everything else). */
static void sendAck(const struct sockaddr_in *to, uint32_t ackSeq, uint8_t ackType, uint8_t result, uint8_t state,
                    uint32_t detail, uint32_t hdrSession)
{
  uint8_t *p = txPayload();
  autoPutU32(p, ackSeq);
  p[4] = ackType;
  p[5] = result;
  p[6] = state;
  p[7] = 0;
  autoPutU32(p + 8, detail);
  sendReply(AUTO_T_ACK, hdrSession, AUTO_ACK_LEN, to);
  lastAckType = ackType;
  lastAckResult = result;
  anyAck = true;
}

static void sendHelloAck(void)
{
  uint8_t *p = txPayload();
  memset(p, 0, AUTO_HELLO_ACK_LEN);
  autoPutU32(p, session);
  autoPutU32(p + 4, bootNonce);
  autoPutU32(p + 8, helloNonce);
  memcpy(p + 12, keyFp, 4);
  const char *fw = esp_app_get_description()->version;
  memcpy(p + 16, fw, strnlen(fw, AUTO_FW_LEN));   /* NUL padded, truncated to 24 */
  sendReply(AUTO_T_HELLO_ACK, session, AUTO_HELLO_ACK_LEN, NULL);
}

/* ---- status ----------------------------------------------------------- */

static uint8_t textureByte(float t)
{
  return t <= 0.0f ? 0 : (t >= 255.0f ? 255 : (uint8_t)(t + 0.5f));
}

/* As usb_diag.c motorsIdle(): stabilizer running, no thrust requested and
 * every motor output at zero. */
static bool motorsIdleNow(const flightRecord_t *r, bool have)
{
  if (!have || r->thrust > 0) {
    return false;
  }
  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    if (motorsGetRatio(i) != 0) {
      return false;
    }
  }
  return true;
}

static void sendStatus(const struct sockaddr_in *to)
{
  flightRecord_t r;
  const bool have = flightRecorderGetLatest(&r);
  if (!have) {
    memset(&r, 0, sizeof(r));
  }
  cameraFlowSample_t s;
  if (!cameraFlowGetLatest(&s)) {
    memset(&s, 0, sizeof(s));
  }
  uint32_t frames, timeouts, restartCount, ageMs;
  cameraFlowStats(&frames, &timeouts, &restartCount, &ageMs);
  const float fps10 = cameraFlowFps() * 10.0f;
  const uint8_t flags = (uint8_t)((motorsIdleNow(&r, have) ? 0x01 : 0) | (stabilizerTakeoffWindow() ? 0x02 : 0) |
                                  (stabilizerOnGround() ? 0x04 : 0) | (photo.held ? 0x08 : 0));

  uint8_t *p = txPayload();
  memset(p, 0, AUTO_STATUS_LEN);
  autoPutU32(p + 0, nowMs());
  autoPutU16(p + 4, r.vbatMv);   /* pmGetBatteryVoltage(), sampled by the flight recorder */
  p[6] = (uint8_t)cameraFlowState();
  p[7] = (uint8_t)autonomyState();
  autoPutU16(p + 8, fps10 <= 0.0f ? 0 : (fps10 >= 65535.0f ? 65535 : (uint16_t)(fps10 + 0.5f)));
  autoPutU16(p + 10, r.thrust);
  autoPutU32(p + 12, frames);
  autoPutU16(p + 16, (uint16_t)r.roll);
  autoPutU16(p + 18, (uint16_t)r.pitch);
  autoPutU16(p + 20, (uint16_t)r.yaw);
  p[22] = s.lum;
  p[23] = textureByte(s.texture);
  p[24] = flags;
  p[25] = photo.held ? (uint8_t)(AUTO_PHOTO_SLOTS - photo.n) : AUTO_PHOTO_SLOTS;
  autoPutU32(p + 28, authFails);
  autoPutU32(p + 32, (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  autoPutU32(p + 36, (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
  autoPutU32(p + 40, accepted);
  autoPutU32(p + 44, bootNonce);
  sendReply(AUTO_T_STATUS, session, AUTO_STATUS_LEN, to);
}

static bool isPeer(const struct sockaddr_in *a)
{
  return havePeer && a->sin_addr.s_addr == peer.sin_addr.s_addr && a->sin_port == peer.sin_port;
}

/* The state machine's snapshot with the seq autonomyGetStatus() read under
 * the same lock: never an old snapshot for a new seq. Only a send to the
 * peer counts as the push (statusSeqSent, statusLastMs): an answer at
 * another address (a duplicate STATUS_REQ replayed from elsewhere) never
 * uses up the push of a pending transition. */
static void sendAutoStatus(const struct sockaddr_in *to)
{
  autoStatusMsg_t m;
  const uint32_t seq = autonomyGetStatus(&m);
  m.bootNonce = bootNonce;
  autoProtoPackAutoStatus(&m, txPayload());
  sendReply(AUTO_T_AUTO_STATUS, session, AUTO_AUTO_STATUS_LEN, to);
  if (to == NULL || isPeer(to)) {
    statusSeqSent = seq;
    statusLastMs = nowMs();
  }
  autoStatusTx++;
}

/* A STATUS_REQ: STATUS, then AUTO_STATUS. */
static void statusReply(const struct sockaddr_in *to)
{
  sendStatus(to);
  sendAutoStatus(to);
}

static void autoStatusStep(uint32_t now)
{
  const bool active = autonomyActive();
  if (active) {
    wasActive = true;
  } else if (wasActive) {
    wasActive = false;
    statusTail = true;   /* so the Mac sees how the sortie ended */
    idleSinceMs = now;
  }
  if (session == 0 || !havePeer) {
    return;
  }
  bool due = autonomyStatusSeq() != statusSeqSent;   /* a transition */
  if (active) {
    due = due || now - statusLastMs >= AUTO_STATUS_ACTIVE_MS;
  } else if (statusTail) {
    if (now - idleSinceMs > AUTO_STATUS_TAIL_FOR_MS) {
      statusTail = false;
    } else {
      due = due || now - statusLastMs >= AUTO_STATUS_TAIL_MS;
    }
  }
  if (due) {
    sendAutoStatus(NULL);
  }
}

/* ---- photos ----------------------------------------------------------- */

static bool slotsReady(void)
{
  for (int i = 0; i < AUTO_PHOTO_SLOTS; i++) {
    if (slot[i] == NULL) {
      slot[i] = heap_caps_malloc(AUTO_FRAME_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (slot[i] == NULL) {
      return false;
    }
  }
  return true;
}

static void photoRelease(void)
{
  cameraJpegCancel();
  if (photo.reqActive) {
    /* Only refused while the camera task is copying (grey: the frame; colour: the meta only), which always
     * completes within a fraction of a millisecond. */
    while (!cameraFlowSnapshotCancel()) {
      vTaskDelay(1);
    }
  }
  if (photo.autoReq != 0 && photo.captured < photo.n) {
    autonomyNoteCapture(photo.autoReq, AUTO_CAP_FAILED, photo.id);   /* released before its last frame */
  }
  memset(&photo, 0, sizeof(photo));
}

/* A state-machine set of this sortie is held: the frames Claude has not
 * received yet (or re-requests with CHUNKS) must not be released. */
static bool autoSetHeld(void)
{
  return photo.held && photo.autoReq != 0 && photo.sortie == armsAccepted;
}

static void photoBegin(uint8_t n, uint16_t gapMs, uint32_t now)
{
  memset(&photo, 0, sizeof(photo));
  if (++photoIdLast == 0) {
    photoIdLast = 1;
  }
  photo.id = photoIdLast;
  photo.n = n;
  photo.gapMs = gapMs;
  photo.acceptedMs = now;
  photo.nextMs = now;
  photo.pushChunk = -1;
  photo.held = true;
}

/* PHOTO in IDLE (stage A1). */
static uint8_t photoStart(const uint8_t *pl, uint16_t len, uint32_t *detail)
{
  if (len != AUTO_PHOTO_LEN) {
    return AUTO_R_RANGE;
  }
  const uint8_t n = pl[0];
  const uint16_t gapMs = autoGetU16(pl + 2);
  if (n < 1 || n > AUTO_PHOTO_SLOTS || gapMs > AUTO_PHOTO_MAX_GAP_MS) {
    return AUTO_R_RANGE;
  }
  if (cameraJpegBusy()) return AUTO_R_BUSY;
  if (photo.held) {
    *detail = autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, photo.id);   /* so the Mac can free a set it lost track of */
    return AUTO_R_BUSY;
  }
  if (stabilizerTakeoffWindow()) {
    return AUTO_R_TAKEOFF_WINDOW;
  }
  /* Asleep on the ground: accepted, the wake below takes ~0.4-0.6 s to the
   * first settled frame (captureStep waits). The take-off standby, a stopped
   * or restarting camera: CAM_UNAVAIL as before. */
  if (!cameraFlowGroundStandby() && cameraFlowState() != CAMERA_FLOW_RUNNING) {
    return AUTO_R_CAM_UNAVAIL;
  }
  if (!slotsReady()) {
    return AUTO_R_MEM;
  }
  cameraFlowGroundWake(cameraFlowGroundHoldMs());   /* wakes it, or extends the hold of an awake one */
  photoBegin(n, gapMs, nowMs());
  photosAccepted++;
  *detail = photo.id;
  return AUTO_R_OK;
}

/* A set the state machine asked for (at STABLE + 1 s, or an accepted PHOTO
 * while waiting). Any set still held is released first: an A1 set, or one
 * left over from an earlier sortie (a PHOTO is refused while this sortie's
 * set is held, so it is never this sortie's). Progress goes back with the
 * same reqId: BUSY now, DONE or FAILED after the last frame. true: a set was
 * started (its id is photo.id); false: no request, or it could not start
 * (camera not running, no PSRAM slots: FAILED at once). */
static bool autoCaptureStep(uint32_t now)
{
  uint32_t reqId;
  uint8_t n;
  uint16_t gapMs;
  if (!autonomyTakeCaptureRequest(&reqId, &n, &gapMs)) {
    return false;
  }
  if (photo.held) {
    photoRelease();
    photosPreempted++;
  }
  if (n < 1 || n > AUTO_PHOTO_SLOTS || cameraFlowState() != CAMERA_FLOW_RUNNING || cameraJpegBusy() || !slotsReady()) {
    autonomyNoteCapture(reqId, AUTO_CAP_FAILED, 0);
    return false;
  }
  photoBegin(n, gapMs > AUTO_PHOTO_MAX_GAP_MS ? AUTO_PHOTO_MAX_GAP_MS : gapMs, now);
  photo.autoReq = reqId;
  photo.sortie = armsAccepted;
  autoCaptures++;
  autonomyNoteCapture(reqId, AUTO_CAP_BUSY, photo.id);
  return true;
}

/* A captured frame's bytes (0: missing) and chunks, from its snapshot's pixel format. */
static uint32_t frameBytes(const autoFrame_t *f)
{
  return f->missing ? 0u : f->snap.bytes;
}

static uint16_t frameChunks(const autoFrame_t *f)
{
  return autoChunkCount(frameBytes(f));
}

static uint8_t photoChunks(const uint8_t *pl, uint16_t len)
{
  if (len != AUTO_CHUNKS_LEN) {
    return AUTO_R_RANGE;
  }
  const uint16_t id = autoGetU16(pl);
  const uint8_t idx = pl[2];
  const uint8_t count = pl[3];
  const uint16_t first = autoGetU16(pl + 4);
  if (!photo.held || id != photo.id) {
    return AUTO_R_NO_PHOTO;
  }
  /* count 0: the frame's META again (a lost META is otherwise never re-sent). Past a colour frame's chunks: RANGE;
   * past a grey frame's (19) once it is captured. */
  if (idx >= photo.n || count > AUTO_CHUNKS_MAX ||
      (count == 0 ? first != 0 : (uint32_t)first + count > AUTO_FRAME_CHUNKS_MAX)) {
    return AUTO_R_RANGE;
  }
  if (!photo.frame[idx].done) {
    return AUTO_R_BUSY;   /* not captured yet; its push follows */
  }
  if (photo.frame[idx].missing && count != 0) {
    return AUTO_R_NO_PHOTO;
  }
  if (count != 0 && (uint32_t)first + count > frameChunks(&photo.frame[idx])) {
    return AUTO_R_RANGE;
  }
  if (photo.resendCount >= AUTO_RESEND_QUEUE) {
    return AUTO_R_BUSY;
  }
  resendReq_t *q = &photo.resend[(photo.resendHead + photo.resendCount) % AUTO_RESEND_QUEUE];
  q->idx = idx;
  q->count = count;
  q->first = first;
  photo.resendCount++;
  return AUTO_R_OK;
}

static uint8_t photoFree(const uint8_t *pl, uint16_t len, uint32_t *detail)
{
  if (len != AUTO_PHOTO_FREE_LEN) {
    return AUTO_R_RANGE;
  }
  const uint16_t id = autoGetU16(pl);
  if (!photo.held || id != photo.id) {
    return AUTO_R_NO_PHOTO;
  }
  photoRelease();
  *detail = id;
  return AUTO_R_OK;
}

static void frameDone(autoFrame_t *f, uint32_t now, bool missing)
{
  /* the camera task's copy names its format and length; anything else is not sent */
  missing = missing || f->snap.bytes == 0u || f->snap.bytes > AUTO_FRAME_MAX_BYTES ||
            (f->snap.pixFmt != AUTO_PIXFMT_JPEG &&
             f->snap.bytes != autoPixfmtBytesPerPixel(f->snap.pixFmt) * AUTO_FRAME_W * AUTO_FRAME_H);
  if (!missing && f->snap.pixFmt != AUTO_PIXFMT_JPEG &&
      cameraJpegStart(slot[photo.captured], f->snap.bytes, f->snap.pixFmt)) {
    photo.encodeActive = true;
    return;
  }
  f->missing = missing;
  if (missing) {
    memset(&f->snap, 0, sizeof(f->snap));
  } else {
    /* Attitude when the copy is seen: one flow-processing time plus at most
     * one loop period after the capture. */
    flightRecord_t r;
    if (flightRecorderGetLatest(&r)) {
      f->roll = r.roll;
      f->pitch = r.pitch;
      f->yaw = r.yaw;
      f->thrust = r.thrust;
      f->vbatMv = r.vbatMv;
    }
    f->crc = esp_rom_crc32_le(0, slot[photo.captured], f->snap.bytes);
  }
  f->done = true;
  photo.captured++;
  photo.nextMs = now + photo.gapMs;
  photo.anyMissing = photo.anyMissing || missing;
  if (photo.autoReq != 0 && photo.captured == photo.n) {
    autonomyNoteCapture(photo.autoReq, photo.anyMissing ? AUTO_CAP_FAILED : AUTO_CAP_DONE, photo.id);
  }
}

/* Next frame of the set: request a copy from the camera task, wait for it
 * (never blocking), then wait gapMs. */
static void captureStep(uint32_t now)
{
  autoFrame_t *f = &photo.frame[photo.captured];
  if (photo.encodeActive) {
    uint32_t bytes = 0;
    int encoded = cameraJpegTake(slot[photo.captured], AUTO_FRAME_MAX_BYTES, &bytes);
    if (encoded == 0) return;
    photo.encodeActive = false;
    if (encoded > 0) { f->snap.bytes = bytes; f->snap.pixFmt = AUTO_PIXFMT_JPEG; }
    frameDone(f, now, encoded < 0);
    return;
  }
  /* A copy made as a take-off began: stamp it now, not when the window ends
   * (taking it sends nothing). */
  if (photo.reqActive && cameraFlowSnapshotTake()) {
    photo.reqActive = false;
    frameDone(f, now, false);
    return;
  }
  if (stabilizerTakeoffWindow() && !(photo.autoReq != 0 && autonomyCameraAwake())) {
    /* The camera sleeps through a manual take-off: wait for the window to
     * end instead of timing the frame out. An autonomous take-off keeps it
     * awake, so the state machine's set is captured at once (its push
     * still waits for the window to end). */
    if (photo.reqActive) {
      photo.reqMs = now;
    } else if ((int32_t)(now - photo.nextMs) > 0) {
      photo.nextMs = now;
    }
    return;
  }
  const cameraFlowState_t cs = cameraFlowState();
  if ((cs == CAMERA_FLOW_STANDBY || cs == CAMERA_FLOW_RUNNING) && !cameraFlowAwakeFresh(AUTO_CAM_WAKE_FRAMES) &&
      now - photo.acceptedMs < AUTO_CAM_WAKE_MAX_MS) {
    /* Waking from the ground standby: wait for settled frames the same way;
     * past AUTO_CAM_WAKE_MAX_MS the timeout below reports the frame missing.
     * A stopped or restarting camera times out as before. */
    if (photo.reqActive) {
      photo.reqMs = now;
    } else if ((int32_t)(now - photo.nextMs) > 0) {
      photo.nextMs = now;
    }
    return;
  }
  if (!photo.reqActive) {
    if ((int32_t)(now - photo.nextMs) < 0) {
      return;
    }
    if (cameraFlowSnapshotRequest(slot[photo.captured], &f->snap)) {
      photo.reqActive = true;
      photo.reqMs = now;
    } else if (now - photo.nextMs >= AUTO_FRAME_TIMEOUT_MS) {
      frameDone(f, now, true);   /* camera not running */
    }
    return;
  }
  if (cameraFlowSnapshotTake()) {
    photo.reqActive = false;
    frameDone(f, now, false);
  } else if (now - photo.reqMs >= AUTO_FRAME_TIMEOUT_MS && cameraFlowSnapshotCancel()) {
    photo.reqActive = false;
    frameDone(f, now, true);
  }
}

static txResult_t sendMeta(uint8_t idx)
{
  const autoFrame_t *f = &photo.frame[idx];
  /* a grey or missing frame: the 40-byte META of A1, byte for byte; a colour one: 44 bytes with its pixel format */
  const uint8_t fmt = f->missing ? AUTO_PIXFMT_GRAY8 : f->snap.pixFmt;
  const uint16_t metaLen = autoMetaLen(fmt);
  uint8_t *p = txPayload();
  memset(p, 0, AUTO_META_COLOR_LEN);
  autoPutU16(p, photo.id);
  p[2] = idx;
  p[3] = photo.n;
  autoPutU16(p + 4, AUTO_FRAME_W);
  autoPutU16(p + 6, AUTO_FRAME_H);
  autoPutU32(p + 8, frameBytes(f));
  autoPutU32(p + 12, f->crc);
  autoPutU32(p + 16, f->snap.seq);
  autoPutU32(p + 20, (uint32_t)(f->snap.frameTimeUs / 1000));
  autoPutU16(p + 24, (uint16_t)f->roll);
  autoPutU16(p + 26, (uint16_t)f->pitch);
  autoPutU16(p + 28, (uint16_t)f->yaw);
  autoPutU16(p + 30, f->thrust);
  p[32] = f->snap.lum;
  p[33] = textureByte(f->snap.texture);
  autoPutU16(p + 34, AUTO_CHUNK_DATA_MAX);
  autoPutU16(p + 36, frameChunks(f));
  autoPutU16(p + 38, f->vbatMv);
  p[AUTO_META_PIXFMT_OFF] = fmt;   /* sent with a colour frame only (metaLen 44) */
  return sendData(AUTO_T_META, metaLen);
}

static txResult_t sendChunk(uint8_t idx, uint16_t chunk)
{
  const uint32_t bytes = frameBytes(&photo.frame[idx]);   /* chunk < frameChunks(): the callers */
  const uint32_t off = (uint32_t)chunk * AUTO_CHUNK_DATA_MAX;
  const uint16_t dataLen = (uint16_t)(bytes - off < AUTO_CHUNK_DATA_MAX ? bytes - off : AUTO_CHUNK_DATA_MAX);
  uint8_t *p = txPayload();
  autoPutU16(p, photo.id);
  p[2] = idx;
  p[3] = 0;
  autoPutU16(p + 4, chunk);
  autoPutU16(p + 6, dataLen);
  memcpy(p + AUTO_CHUNK_HDR_LEN, slot[idx] + off, dataLen);
  return sendData(AUTO_T_CHUNK, (uint16_t)(AUTO_CHUNK_HDR_LEN + dataLen));
}

/* One paced burst: re-requested chunks first, then the next frame's META and
 * chunks in order. A full TX queue ends the burst; the same packet is tried
 * again next time. */
static void pushStep(uint32_t now)
{
  for (int sent = 0; sent < AUTO_PUSH_BURST; sent++) {
    if (photo.resendCount > 0) {
      const resendReq_t *q = &photo.resend[photo.resendHead];
      const txResult_t r = q->count == 0 ? sendMeta(q->idx)
                                         : sendChunk(q->idx, (uint16_t)(q->first + photo.resendNext));
      if (r == TX_NOMEM) {
        return;
      }
      if (++photo.resendNext >= q->count) {
        photo.resendNext = 0;
        photo.resendHead = (uint8_t)((photo.resendHead + 1) % AUTO_RESEND_QUEUE);
        photo.resendCount--;
      }
    } else if (!photo.pushDone && photo.pushFrame < photo.captured) {
      const autoFrame_t *f = &photo.frame[photo.pushFrame];
      const txResult_t r = photo.pushChunk < 0 ? sendMeta(photo.pushFrame)
                                                : sendChunk(photo.pushFrame, (uint16_t)photo.pushChunk);
      if (r == TX_NOMEM) {
        return;
      }
      photo.pushChunk++;
      if (f->missing || photo.pushChunk >= frameChunks(f)) {
        photo.pushChunk = -1;
        if (++photo.pushFrame >= photo.n) {
          photo.pushDone = true;
          photo.pushDoneMs = now;
        }
      }
    } else {
      return;
    }
  }
}

/* Capturing or sending: the task then polls instead of blocking on receive. */
static bool photoActive(void)
{
  return photo.held && (!photo.pushDone || photo.resendCount > 0);
}

static void photoStep(uint32_t now)
{
  if (!photo.held) {
    return;
  }
  if (photo.captured < photo.n) {
    captureStep(now);
  }
  /* No Wi-Fi bursts during a take-off window (supply margin at lift-off). */
  if (!stabilizerTakeoffWindow() && now - pushLastMs >= AUTO_PUSH_MIN_GAP_MS) {
    pushLastMs = now;
    pushStep(now);
  }
  if ((photo.pushDone && photo.resendCount == 0 && now - photo.pushDoneMs >= AUTO_PHOTO_HOLD_MS) ||
      now - photo.acceptedMs >= AUTO_PHOTO_MAX_HOLD_MS) {
    photoRelease();
  }
}

/* ---- commands for the state machine (section 3.1) ---------------------- */

static void ackCacheStore(uint32_t seq, uint8_t type, uint8_t result, uint32_t detail, uint8_t state)
{
  ackCacheEntry_t *e = &ackCache[ackCacheHead];
  e->seq = seq;
  e->type = type;
  e->result = result;
  e->detail = detail;
  e->state = state;
  e->valid = true;
  ackCacheHead = (uint8_t)((ackCacheHead + 1) % AUTO_PROTO_SEQ_RECENT);
}

/* Answers the pending command and frees the one-command slot. BUSY answers
 * are never cached (a RETX of that seq then gets nothing: the Mac sends
 * again with a new seq); a result is cached only in the session whose seq
 * filter it belongs to. */
static void pendAck(uint8_t result, uint32_t detail, uint8_t state, bool cache)
{
  pend.active = false;
  if (!(pend.flags & AUTO_FLAG_ACKREQ)) {
    return;
  }
  if (cache && pend.session == session) {
    ackCacheStore(pend.seq, pend.type, result, detail, state);
  }
  sendAck(&pend.from, pend.seq, pend.type, result, state, detail, pend.session);
}

static void pendResult(const autoCmdResult_t *r, uint32_t now)
{
  uint32_t detail = r->detail;
  if (pend.type == AUTO_T_ARM && r->result == AUTO_R_OK) {
    armsAccepted++;
  }
  if (pend.type == AUTO_T_PHOTO && r->result == AUTO_R_OK) {
    /* The state machine asked for the set in the step that answered: start
     * it now and answer with its id (the Mac matches META/CHUNK by it).
     * Detail 0: accepted, but no set started (camera not running, no PSRAM
     * slots): AUTO_STATUS capture reads FAILED and the state machine lands
     * (CAPTURE_FAIL); the Mac waits for no photo. */
    detail = autoCaptureStep(now) ? photo.id : 0;
  }
  pendAck(r->result, detail, r->state, true);
}

/* Every loop pass: the result normally arrives one stabilizer loop after the
 * post. Not taken by AUTONOMY_CMD_WAIT_MS: withdrawn, guaranteed never
 * processed. Taken but not answered by AUTONOMY_CMD_WAIT_HARD_MS (a stalled
 * stabilizer): BUSY with AUTO_DETAIL_TAKEN_UNKNOWN, it may have been acted on. */
static void autoCmdStep(uint32_t now)
{
  if (!pend.active) {
    return;
  }
  autoCmdResult_t r;
  if (autonomyTakeResult(pend.ticket, &r)) {
    pendResult(&r, now);
    return;
  }
  const uint32_t age = now - pend.t0;
  if (!pend.taken && age >= AUTONOMY_CMD_WAIT_MS) {
    switch (autonomyCancelCommand(pend.ticket)) {
      case AUTO_MB_CANCELLED: {
        const autoState_t st = autonomyState();
        cmdTimeouts++;
        pendAck(AUTO_R_BUSY, autoDetailBusy(AUTO_DETAIL_BUSY_UNTAKEN, (uint32_t)st), (uint8_t)st, false);
        return;
      }
      case AUTO_MB_DONE:
        if (autonomyTakeResult(pend.ticket, &r)) {
          pendResult(&r, now);
          return;
        }
        pend.taken = true;
        break;
      default:   /* TAKEN, or UNKNOWN (treated as taken) */
        pend.taken = true;
        break;
    }
  }
  if (pend.taken && age >= AUTONOMY_CMD_WAIT_HARD_MS) {
    cmdTakenUnknown++;
    pendAck(AUTO_R_BUSY, AUTO_DETAIL_TAKEN_UNKNOWN, autonomyState(), false);
  }
}

/* ARM, TAKEOFF, LAND, DISARM, MOVE, YAW, STOP, SET_HEIGHT, and PHOTO while autonomy is active. true:
 * posted, autoCmdStep answers it. false: answered now with *result and
 * *detail (RANGE: bad length or pad, detail 0; BUSY, never cached, its kind
 * in the detail's top byte (auto_proto.h): HELD_SET, a state-machine set
 * still held (its id); PENDING, a command already pending (its seq);
 * UNTAKEN, the mailbox slot still held by a command the stabilizer took (the
 * state)). */
static bool autoCommand(const autoProtoHdr_t *h, const uint8_t *pl, const struct sockaddr_in *from, uint8_t *result,
                        uint32_t *detail)
{
  autoCmd_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  bool ok;
  switch (h->type) {
    case AUTO_T_ARM: {
      autoProtoArm_t a;
      ok = autoProtoParseArm(pl, h->len, &a);
      if (ok) {
        cmd.mode = a.mode;
        cmd.airMaxMs = a.airMaxMs;
      }
      cmd.type = AUTO_CMD_ARM;
      break;
    }
    case AUTO_T_TAKEOFF: {
      autoProtoTakeoff_t t;
      ok = autoProtoParseTakeoff(pl, h->len, &t);
      if (ok) {
        cmd.token = t.token;
        cmd.heightCm = t.heightCm;
        cmd.photoN = t.photoN;
        cmd.photoGapMs = t.photoGapMs;
        cmd.actWaitMs = t.actWaitMs;
        cmd.moveRightCm = t.planRightCm;   /* the planned move (auto_fsm.h autoCmd_t) */
        cmd.moveFwdCm = t.planFwdCm;
      }
      cmd.type = AUTO_CMD_TAKEOFF;
      break;
    }
    case AUTO_T_LAND:
      ok = h->len == 0;
      cmd.type = AUTO_CMD_LAND;
      break;
    case AUTO_T_DISARM:
      ok = h->len == 0;
      cmd.type = AUTO_CMD_DISARM;
      break;
    case AUTO_T_MOVE: {   /* interactive hover */
      autoProtoMove_t m;
      ok = autoProtoParseMove(pl, h->len, &m);
      if (ok) {
        cmd.moveRightCm = m.rightCm;
        cmd.moveFwdCm = m.fwdCm;
        cmd.moveUpCm = m.upCm;
      }
      cmd.type = AUTO_CMD_MOVE;
      break;
    }
    case AUTO_T_YAW: {
      autoProtoYaw_t y;
      ok = autoProtoParseYaw(pl, h->len, &y);
      if (ok) {
        cmd.yawDeg = y.deg;
      }
      cmd.type = AUTO_CMD_YAW;
      break;
    }
    case AUTO_T_STOP:
      ok = h->len == 0;
      cmd.type = AUTO_CMD_STOP;
      break;
    case AUTO_T_SET_HEIGHT: {   /* "you are at X m" (2026-10-15, docs/autonomy-a2b.md 23) */
      autoProtoSetHeight_t sh;
      ok = autoProtoParseSetHeight(pl, h->len, &sh) && sh.xCm >= 0;
      if (ok) {
        cmd.heightCm = (uint16_t)sh.xCm;   /* X (auto_fsm.h autoCmd_t) */
        cmd.moveUpCm = sh.atCm;            /* h_at (AUTO_SH_AT_NOW: now) */
        if (sh.toForm) {                   /* "go to the target" (2026-10-16, docs/autonomy-a2b.md 27) */
          cmd.mode = AUTO_SH_FORM_TO;
          cmd.moveRightCm = sh.toCm;       /* the target (AUTO_SH_TO_REF: the commanded height) */
        }
      }
      cmd.type = AUTO_CMD_SET_HEIGHT;
      break;
    }
    default: {   /* AUTO_T_PHOTO */
      autoProtoPhoto_t ph;
      ok = autoProtoParsePhoto(pl, h->len, &ph);
      if (ok) {
        cmd.photoN = ph.n;
        cmd.photoGapMs = ph.gapMs;
      }
      cmd.type = AUTO_CMD_PHOTO;
      break;
    }
  }
  if (!ok) {
    *result = AUTO_R_RANGE;
    *detail = 0;
    return false;
  }
  if (cmd.type == AUTO_CMD_ARM || cmd.type == AUTO_CMD_TAKEOFF) {
    /* Wake the camera from its ground standby (docs/autonomy-a2b.md 17.1): a
     * FLIGHT ARM that arrives while it sleeps is refused GUARDS CAMERA (G4
     * needs two fresh frames, ~0.15-0.25 s after the wake), and the Mac sends
     * it again 0.5 s later. */
    cameraFlowGroundWake(cameraFlowGroundHoldMs());
  }
  if (cmd.type == AUTO_CMD_PHOTO && autoSetHeld()) {
    photoHeldBusy++;   /* the Mac frees the held set first (PHOTO_FREE), then asks again */
    *result = AUTO_R_BUSY;
    *detail = autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, photo.id);
    return false;
  }
  if (pend.active) {
    cmdBusy++;
    *result = AUTO_R_BUSY;
    *detail = autoDetailBusy(AUTO_DETAIL_BUSY_PENDING, pend.seq);
    return false;
  }
  uint32_t ticket;
  if (!autonomyPostCommand(&cmd, &ticket)) {
    cmdBusy++;
    *result = AUTO_R_BUSY;
    *detail = autoDetailBusy(AUTO_DETAIL_BUSY_UNTAKEN, (uint32_t)autonomyState());
    return false;
  }
  memset(&pend, 0, sizeof(pend));
  pend.active = true;
  pend.type = h->type;
  pend.flags = h->flags;
  pend.ticket = ticket;
  pend.seq = h->seq;
  pend.session = h->session;
  pend.t0 = nowMs();
  pend.from = *from;
  autoCmds++;
  return true;
}

/* ---- receiving -------------------------------------------------------- */

/* A repeat of a recently accepted seq: answer again, do not act again. */
static void replayAnswer(const autoProtoHdr_t *h, const struct sockaddr_in *from)
{
  if (h->type == AUTO_T_STATUS_REQ) {
    statusReply(from);   /* a query: a fresh answer is the right one */
  }
  for (int i = 0; i < AUTO_PROTO_SEQ_RECENT; i++) {
    const ackCacheEntry_t *e = &ackCache[i];
    if (e->valid && e->seq == h->seq) {
      sendAck(from, e->seq, e->type, e->result, e->state, e->detail, session);
      return;
    }
  }
}

static void setPeer(const struct sockaddr_in *from)
{
  peer = *from;
  havePeer = true;
}

static void handleHello(const autoProtoHdr_t *h, const uint8_t *pl, const struct sockaddr_in *from)
{
  if (h->session != 0) {
    badSessions++;
    sendAck(from, h->seq, h->type, AUTO_R_BAD_SESSION, (uint8_t)autonomyState(), 0, h->session);
    return;
  }
  if (h->len != AUTO_HELLO_LEN) {
    if (h->flags & AUTO_FLAG_ACKREQ) {
      sendAck(from, h->seq, h->type, AUTO_R_RANGE, (uint8_t)autonomyState(), 0, 0);
    }
    return;
  }
  const uint32_t nonce = autoGetU32(pl);
  if (session != 0 && nonce == helloNonce && h->seq == helloSeq) {
    /* The HELLO that opened this session again (its HELLO_ACK was lost): the
     * session stays and the peer moves to the sender, by design (plan 3.2;
     * the HELLO_ACK goes there). So a captured copy of this HELLO, replayed
     * from another station, moves the pushes (AUTO_STATUS, META, CHUNK)
     * there until the daemon's next new-seq packet (its 100 ms HB) moves
     * them back; CHUNKS recovers the chunks lost meanwhile. It cannot change
     * the session, act, or redirect any reply. (A HELLO of an earlier session
     * has another nonce: it opens a new session below, which lands an active
     * sortie, LAND SESSION.) */
    dups++;
    setPeer(from);
    sendHelloAck();
    return;
  }
  uint32_t s;
  do {
    s = esp_random();
  } while (s == 0 || s == session);
  session = s;
  txSeq = 0;
  autoProtoSeqReset(&seqFilter);
  autoProtoSeqCheck(&seqFilter, h->seq);
  memset(ackCache, 0, sizeof(ackCache));
  ackCacheHead = 0;
  helloNonce = nonce;
  helloSeq = h->seq;
  accepted++;
  setPeer(from);
  autonomyNoteSession();   /* while active, the state machine lands (LAND SESSION) */
  sendHelloAck();
}

/* M9: a valid tag alone, any session, any seq, a payload of 0 to
 * AUTO_KILL_MAX_PAYLOAD (32) ignored bytes (a longer one is truncated on
 * receive and fails the length check); acted on before anything else,
 * answered at the sender's address in the sender's session, never cached,
 * never a peer change (the kill client sends session 0 from its own
 * socket). */
static void handleKill(const autoProtoHdr_t *h, const struct sockaddr_in *from)
{
  const autoState_t before = autonomyState();
  autonomyKillFromLink();
  kills++;
  if (h->flags & AUTO_FLAG_ACKREQ) {
    sendAck(from, h->seq, AUTO_T_KILL, AUTO_R_OK, (uint8_t)autonomyState(), (uint32_t)before, h->session);
  }
}

static void handlePacket(const autoProtoHdr_t *h, const uint8_t *pl, const struct sockaddr_in *from)
{
  if (h->type == AUTO_T_KILL) {
    handleKill(h, from);
    return;
  }
  if (h->type == AUTO_T_HELLO) {
    handleHello(h, pl, from);
    return;
  }
  if (session == 0 || h->session != session) {
    /* Sent in the Mac's (stale) session so it passes the Mac's own filter. */
    badSessions++;
    sendAck(from, h->seq, h->type, AUTO_R_BAD_SESSION, (uint8_t)autonomyState(), 0, h->session);
    return;
  }
  switch (autoProtoSeqCheck(&seqFilter, h->seq)) {
    case AUTO_SEQ_OLD:
      oldSeqs++;
      return;
    case AUTO_SEQ_DUP:
      dups++;
      replayAnswer(h, from);
      return;
    default:
      break;
  }
  accepted++;
  /* A new packet of the current session moves the peer. A duplicate or old
   * seq, another session or a KILL never does, so replaying one of these
   * cannot redirect AUTO_STATUS or the photo push; the one replay that can
   * is the session's own HELLO (handleHello). Replies always go to the
   * sender. */
  setPeer(from);

  uint8_t result = AUTO_R_OK;
  uint32_t detail = 0;
  bool cache = true;
  /* The flight commands, and PHOTO in a sortie, go to the state machine. */
  const bool forFsm = h->type == AUTO_T_ARM || h->type == AUTO_T_TAKEOFF || h->type == AUTO_T_LAND ||
                      h->type == AUTO_T_DISARM || h->type == AUTO_T_MOVE || h->type == AUTO_T_YAW ||
                      h->type == AUTO_T_STOP || h->type == AUTO_T_SET_HEIGHT ||
                      (h->type == AUTO_T_PHOTO && autonomyActive());
  if (forFsm) {
    if (autoCommand(h, pl, from, &result, &detail)) {
      return;   /* posted: autoCmdStep answers */
    }
    cache = result != AUTO_R_BUSY;
  } else {
    switch (h->type) {
      case AUTO_T_HB:
        if (h->len != AUTO_HB_LEN) {
          result = AUTO_R_RANGE;
          break;
        }
        lastHbMs = nowMs();
        haveHb = true;
        autonomyNoteHeartbeat();
        break;
      case AUTO_T_STATUS_REQ:
        statusReply(from);
        break;
      case AUTO_T_PHOTO:
        result = photoStart(pl, h->len, &detail);
        break;
      case AUTO_T_CHUNKS:
        result = photoChunks(pl, h->len);
        break;
      case AUTO_T_PHOTO_FREE:
        result = photoFree(pl, h->len, &detail);
        break;
      default:
        /* unknown types: refused */
        result = AUTO_R_BAD_STATE;
        break;
    }
  }
  if (h->flags & AUTO_FLAG_ACKREQ) {
    const uint8_t state = (uint8_t)autonomyState();
    if (cache) {
      ackCacheStore(h->seq, h->type, result, detail, state);
    }
    sendAck(from, h->seq, h->type, result, state, detail, session);
  }
}

static bool rateOk(uint32_t now, uint32_t *windowMs, uint32_t *count, uint32_t max)
{
  if (now - *windowMs >= 1000) {
    *windowMs = now;
    *count = 0;
  }
  return ++*count <= max;
}

static void handleDatagram(const uint8_t *buf, size_t len, const struct sockaddr_in *from)
{
  rxDatagrams++;
  const uint32_t now = nowMs();
  /* The clear-text type picks the budget; the tag is checked after it. */
  if (len >= AUTO_PROTO_HDR_LEN + AUTO_PROTO_TAG_LEN && buf[AUTO_PROTO_TYPE_OFF] == AUTO_T_KILL) {
    if (!rateOk(now, &killWindowMs, &killRateCount, AUTO_KILL_RATE_MAX_PER_S)) {
      killRateDrops++;
      return;
    }
  } else if (!rateOk(now, &rateWindowMs, &rateCount, AUTO_RATE_MAX_PER_S)) {
    rateDrops++;
    return;
  }
  autoProtoHdr_t h;
  const uint8_t *pl = NULL;
  if (autoProtoDecode(buf, len, key, hmacSha256, &h, &pl) != AUTO_DEC_OK) {
    authFails++;
    return;
  }
  handlePacket(&h, pl, from);
}

static bool openSocket(void)
{
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) {
    return false;
  }
  struct timeval tv = {.tv_sec = 0, .tv_usec = AUTO_RX_TIMEOUT_MS * 1000};
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(AUTO_PROTO_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0 ||
      bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    DEBUG_PRINTW("UDP %d: socket setup failed (errno %d)", AUTO_PROTO_PORT, errno);
    close(s);
    return false;
  }
  sock = s;
  rxTimeoutMs = AUTO_RX_TIMEOUT_MS;
  return true;
}

static void setRxTimeout(uint32_t ms)
{
  if (ms == rxTimeoutMs) {
    return;
  }
  struct timeval tv = {.tv_sec = 0, .tv_usec = (int)(ms * 1000)};
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0) {
    rxTimeoutMs = ms;
  }
}

/* One pass of the task loop. Nothing in it waits for the state machine: a
 * pending command is polled each pass, so KILL, HB, STATUS and the photo
 * push are served throughout. */
static void linkPass(uint8_t rx[AUTO_RX_BUF_LEN])
{
  setRxTimeout(autonomyActive() ? AUTO_RX_TIMEOUT_ACTIVE_MS : AUTO_RX_TIMEOUT_MS);
  const bool pushing = photoActive();
  for (int i = 0; i < AUTO_RX_PER_LOOP; i++) {
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);
    /* Block (up to the receive timeout) only when idle, and only once. */
    const int flags = (pushing || pend.active || i > 0) ? MSG_DONTWAIT : 0;
    const int len = recvfrom(sock, rx, AUTO_RX_BUF_LEN, flags, (struct sockaddr *)&from, &fromLen);
    if (len < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        vTaskDelay(M2T(10));
      }
      break;
    }
    if (fromLen >= sizeof(from) && from.sin_family == AF_INET) {
      handleDatagram(rx, (size_t)len, &from);
    }
  }
  const uint32_t now = nowMs();
  autoCmdStep(now);
  /* A request outside a PHOTO's answer (the set at STABLE + 1 s). While a
   * PHOTO is pending, its own request is left to pendResult, which answers
   * with that set's id: the state machine publishes the request and the
   * result in one step, and this pass can fall between the two. */
  if (!(pend.active && pend.type == AUTO_T_PHOTO)) {
    (void)autoCaptureStep(now);
  }
  photoStep(now);
  autoStatusStep(now);
  if (pend.active) {
    vTaskDelay(1);   /* the result is polled every tick, for at most 200 ms */
  } else if (pushing) {
    vTaskDelay(M2T(AUTO_PUSH_GAP_MS));
  }
}

static void autoLinkTask(void *param)
{
  vTaskDelay(M2T(AUTO_START_DELAY_MS));
  while (!openSocket()) {
    vTaskDelay(M2T(2000));
  }
  portOpen = true;
  DEBUG_PRINTI("listening on UDP %d, key fp %02x%02x%02x%02x, stack in %s", AUTO_PROTO_PORT, keyFp[0], keyFp[1],
               keyFp[2], keyFp[3], stackInPsram ? "PSRAM" : "internal RAM");

  uint8_t rx[AUTO_RX_BUF_LEN];
  while (1) {
    linkPass(rx);
  }
}

/* The task's stack in PSRAM (CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY), its
 * control block internal; internal RAM (the A1 task) only without PSRAM.
 *
 * A task whose stack is in PSRAM must never start a flash operation: it
 * disables the cache, and PSRAM with it (spi_flash/cache_utils.c asserts
 * esp_task_stack_is_sane_cache_disabled()). This task never does:
 *   - its only NVS access, autoKeyStoreLoad(), runs in autoLinkInit(), in the
 *     init task, before this task exists; no other imu_level_cal_store.h
 *     call is made in this file;
 *   - everything else it calls is RAM work: lwIP sockets (lwip_sendto copies
 *     the payload into a pbuf, LWIP_NETIF_TX_SINGLE_PBUF), the HMAC (the SHA
 *     DMA driver writes PSRAM input back from the cache first, mbedtls
 *     port/sha/dma/sha.c), esp_timer, esp_random, heap_caps, the camera and
 *     recorder getters and snapshot calls (spinlocks and atomics),
 *     stabilizerTakeoffWindow/OnGround (reads), motorsGetRatio (driver mutex,
 *     LEDC register read), esp_app_get_description (a constant read through
 *     the cache), console prints, and autonomy.h's link-task calls (copies
 *     under a spinlock; the KILL stop writes GPIO and LEDC registers);
 *   - a flash operation of another task suspends this one like any task.
 * The TX buffer (HMAC input, then copied by lwIP) is in PSRAM too. */
static bool linkTaskStart(void)
{
#ifdef CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY
  static StaticTask_t tcb;
  StackType_t *stack = heap_caps_malloc(AUTO_LINK_STACK_PSRAM, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (stack != NULL) {
    taskHandle = xTaskCreateStaticPinnedToCore(autoLinkTask, AUTO_LINK_TASK_NAME, AUTO_LINK_STACK_PSRAM, NULL,
                                               AUTO_LINK_TASK_PRI, stack, &tcb, AUTO_LINK_TASK_CORE);
    if (taskHandle != NULL) {
      stackInPsram = true;
      return true;
    }
    heap_caps_free(stack);
  }
#endif
  stackInPsram = false;
  return xTaskCreatePinnedToCore(autoLinkTask, AUTO_LINK_TASK_NAME, AUTO_LINK_STACK_INTERNAL, NULL,
                                 AUTO_LINK_TASK_PRI, &taskHandle, AUTO_LINK_TASK_CORE) == pdPASS;
}

static uint8_t *txBufAlloc(void)
{
  uint8_t *b = heap_caps_malloc(AUTO_PROTO_MAX_PACKET, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return b != NULL ? b : heap_caps_malloc(AUTO_PROTO_MAX_PACKET, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void autoLinkInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  isInit = true;
  esp_err_t err = autoKeyStoreLoad(key);
  if (err != ESP_OK) {
    memset(key, 0, sizeof(key));
    DEBUG_PRINTI("autonomy link off: no key (%s); install one over USB with 'autokey'", esp_err_to_name(err));
    return;
  }
  keyLoaded = true;
  autoLinkKeyFingerprint(key, keyFp);
  bootNonce = esp_random();

  txBuf = txBufAlloc();
  mbedtls_md_init(&hmacCtx);
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (txBuf == NULL || info == NULL || mbedtls_md_setup(&hmacCtx, info, 1) != 0 || !linkTaskStart()) {
    mbedtls_md_free(&hmacCtx);
    heap_caps_free(txBuf);
    txBuf = NULL;
    DEBUG_PRINTW("autonomy link off: no memory");
    return;
  }
  running = true;
}

void autoLinkGetStats(autoLinkStats_t *out)
{
  memset(out, 0, sizeof(*out));
  out->keyLoaded = keyLoaded;
  memcpy(out->keyFp, keyFp, sizeof(out->keyFp));
  out->running = running;
  /* The task is never deleted once started. */
  out->stackFree = (running && taskHandle) ? (uint32_t)uxTaskGetStackHighWaterMark(taskHandle) : 0;
  out->portOpen = portOpen;
  out->session = session;
  out->havePeer = havePeer;
  if (havePeer) {
    const uint32_t a = peer.sin_addr.s_addr;   /* network order: a.b.c.d in memory */
    memcpy(out->peerIp, &a, sizeof(out->peerIp));
    out->peerPort = ntohs(peer.sin_port);
  }
  out->rxDatagrams = rxDatagrams;
  out->rateDrops = rateDrops;
  out->authFails = authFails;
  out->accepted = accepted;
  out->dups = dups;
  out->oldSeq = oldSeqs;
  out->badSession = badSessions;
  out->txPackets = txPackets;
  out->txErrors = txErrors;
  out->txNoMem = txNoMem;
  out->photos = photosAccepted;
  out->photoBusy = photo.held;
  out->photoId = photo.held ? photo.id : 0;
  out->hbAgeMs = haveHb ? nowMs() - lastHbMs : UINT32_MAX;
  out->lastAckType = lastAckType;
  out->lastAckResult = lastAckResult;
  out->anyAck = anyAck;
  out->stackInPsram = running && stackInPsram;
  out->kills = kills;
  out->autoCmds = autoCmds;
  out->cmdTimeouts = cmdTimeouts;
  out->autoStatusTx = autoStatusTx;
  out->autoCaptures = autoCaptures;
  out->photosPreempted = photosPreempted;
  out->cmdBusy = cmdBusy;
  out->cmdTakenUnknown = cmdTakenUnknown;
  out->photoHeldBusy = photoHeldBusy;
  out->killRateDrops = killRateDrops;
}
