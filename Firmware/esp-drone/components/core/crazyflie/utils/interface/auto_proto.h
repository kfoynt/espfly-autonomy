/*
 * ESP-Drone Firmware
 *
 * auto_proto.h - Autonomy link packets (docs/autonomy-link.md)
 *
 * Framing, HMAC tag, session sequence filter, little-endian helpers and
 * CRC-32 for the Mac <-> drone autonomy link on UDP 2391. Pure code: the
 * HMAC-SHA256 is passed in (mbedtls on the drone, CommonCrypto in the host
 * test), so tools/tests/test_auto_proto.c checks it byte for byte against
 * tools/tests/auto_proto_golden.json (stage A1) and
 * tools/tests/auto_proto_golden_a2.json (stage A2b: ARM, TAKEOFF, LAND,
 * KILL, DISARM, AUTO_STATUS).
 *
 * Stage A2b (docs/autonomy-a2b.md section 3) adds the flight commands
 * 0x10-0x14 (until then rejected with ACK BAD_STATE) and AUTO_STATUS (0x84).
 * Protocol version stays 1: every A1 packet is unchanged; STATUS byte 7 and
 * ACK byte 6 now carry the autonomy state (autoState_t, auto_fsm.h).
 *
 * The interactive hover (user decision 2026-10-12, late; docs/autonomy-a2b.md
 * section 14) uses the three types stage C had reserved: YAW 0x30, MOVE 0x31
 * (stage C's STEP) and STOP 0x32 (its ABORT_MOVE), and appends 32 bytes to
 * AUTO_STATUS (96 bytes; the first 64 unchanged). "Claude decides"
 * (2026-10-15, docs/autonomy-a2b.md 22) appends 40 more (136 bytes: what the
 * drone did on its own and the evidence; the first 96 unchanged). Fast moves
 * (2026-10-16, docs/autonomy-a2b.md 31) append 4 (140 bytes: the speed step).
 */
#ifndef AUTO_PROTO_H_
#define AUTO_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AUTO_PROTO_PORT         2391
#define AUTO_PROTO_MAGIC        0xE5
#define AUTO_PROTO_VERSION      1
#define AUTO_PROTO_HDR_LEN      16
#define AUTO_PROTO_TYPE_OFF     2      /* clear-text type byte (A2b: the link's KILL budget peeks it before the tag check) */
#define AUTO_PROTO_TAG_LEN      16
#define AUTO_PROTO_MAX_PAYLOAD  1100
#define AUTO_PROTO_MAX_PACKET   (AUTO_PROTO_HDR_LEN + AUTO_PROTO_MAX_PAYLOAD + AUTO_PROTO_TAG_LEN)
#define AUTO_PROTO_KEY_LEN      16
#define AUTO_PROTO_SEQ_RECENT   8

#define AUTO_FLAG_ACKREQ        0x01
#define AUTO_FLAG_RETX          0x02

/* Mac -> drone */
#define AUTO_T_HELLO            0x01
#define AUTO_T_HB               0x02
#define AUTO_T_STATUS_REQ       0x03   /* answered with STATUS, then AUTO_STATUS (A2b) */
#define AUTO_T_ARM              0x10   /* A2b: u8 mode, u8 0, u16 airMaxMs */
#define AUTO_T_TAKEOFF          0x11   /* A2b: u32 armToken, u16 heightCm, u8 photoN, u8 actWaitMs bits 16-23 (a pad that
                                          had to be 0 until 2026-10-14), u16 photoGapMs, u16 actWaitMs bits 0-15; 18
                                          bytes with a planned move (2026-10-14): + i16 planRightCm, i16 planFwdCm,
                                          u16 0 */
#define AUTO_T_LAND             0x12   /* A2b: empty; never refused */
#define AUTO_T_KILL             0x13   /* A2b: empty; accepted with a valid tag alone (any session, any seq) */
#define AUTO_T_DISARM           0x14   /* A2b: empty; never refused */
#define AUTO_T_PHOTO            0x20   /* A1; while autonomy is active it goes to the state machine (AUTO_CMD_PHOTO) */
#define AUTO_T_CHUNKS           0x21
#define AUTO_T_PHOTO_FREE       0x22
#define AUTO_T_YAW              0x30   /* interactive hover: i16 deg (+ clockwise), u16 0 */
#define AUTO_T_MOVE             0x31   /* interactive hover: i16 rightCm, i16 fwdCm, i16 upCm, u16 0 */
#define AUTO_T_STOP             0x32   /* interactive hover: empty; never refused */
#define AUTO_T_SET_HEIGHT       0x33   /* "you are at X m" (2026-10-15, docs/autonomy-a2b.md 23): i16 xCm (the judged true
                                          height, 10..250), i16 atCm (the drone's estimate when the judged photo was
                                          captured; INT16_MIN: the estimate now). An older drone answers BAD_STATE detail
                                          0 (an unknown type); this one never gives detail 0 for it. The "go to the
                                          target" form (2026-10-16, docs/autonomy-a2b.md 27) appends i16 toCm (the
                                          height to fly to, 25..200; INT16_MIN: the commanded height): 6 bytes; a
                                          drone before it answers RANGE detail 0 (the link's length check, not
                                          processed) */
/* drone -> Mac */
#define AUTO_T_HELLO_ACK        0x81
#define AUTO_T_ACK              0x82
#define AUTO_T_STATUS           0x83
#define AUTO_T_AUTO_STATUS      0x84   /* A2b: autonomy state machine and vertical controller, see below */
#define AUTO_T_META             0xA0
#define AUTO_T_CHUNK            0xA1

typedef enum {
  AUTO_R_OK = 0,
  AUTO_R_BAD_STATE = 1,
  AUTO_R_BUSY = 2,
  AUTO_R_RANGE = 3,
  AUTO_R_TAKEOFF_WINDOW = 4,
  AUTO_R_CAM_UNAVAIL = 5,
  AUTO_R_LOW_BATT = 6,
  AUTO_R_APP_ACTIVE = 7,
  AUTO_R_KILL_LATCHED = 8,
  AUTO_R_POWER_DEFER = 9,
  AUTO_R_NO_PHOTO = 10,
  AUTO_R_MEM = 11,
  AUTO_R_GUARDS = 12,
  AUTO_R_STALE_FRAME = 13,
  AUTO_R_BAD_SESSION = 14,
} autoResult_t;

/* ACK detail of a BUSY the link itself answers (A2b; docs/autonomy-link.md
 * "BUSY"). Bits 24-31 hold the kind, the low bits its value:
 *   AUTO_DETAIL_BUSY_PENDING   another command is pending; value = its seq & 0xFFFFFF.
 *                              Not processed: send again (new seq) after that one's ACK.
 *   AUTO_DETAIL_BUSY_UNTAKEN   not processed, guaranteed (the mailbox slot was held, or
 *                              the command was withdrawn untaken at 100 ms); value = the
 *                              autonomy state.
 *   AUTO_DETAIL_BUSY_HELD_SET  PHOTO refused while a photo set is held (IDLE: any set;
 *                              active: a set the state machine started this sortie);
 *                              value = that set's photo id (16 bits).
 *   0xFF                       AUTO_DETAIL_TAKEN_UNKNOWN (auto_mailbox.h, 0xFFFFFFFF): taken,
 *                              no result within 200 ms; it may have been acted on.
 * A BUSY the state machine answers keeps its documented detail (the state:
 * kind 0), and a CHUNKS answered BUSY (frame not captured yet, resend queue
 * full) keeps detail 0, as in A1. A detail is a photo id only when its kind
 * is HELD_SET. */
#define AUTO_DETAIL_KIND_MASK       0xFF000000u
#define AUTO_DETAIL_VALUE_MASK      0x00FFFFFFu
#define AUTO_DETAIL_BUSY_PENDING    0x01000000u
#define AUTO_DETAIL_BUSY_UNTAKEN    0x02000000u
#define AUTO_DETAIL_BUSY_HELD_SET   0x03000000u

/* kind: one of AUTO_DETAIL_BUSY_*; value: the pending seq, the state or the
 * photo id (masked to 24 bits, or 16 for HELD_SET). */
static inline uint32_t autoDetailBusy(uint32_t kind, uint32_t value)
{
  return (kind & AUTO_DETAIL_KIND_MASK) |
         (value & (kind == AUTO_DETAIL_BUSY_HELD_SET ? 0xFFFFu : AUTO_DETAIL_VALUE_MASK));
}

static inline uint32_t autoDetailKind(uint32_t detail)
{
  return detail & AUTO_DETAIL_KIND_MASK;
}

/* Payload sizes */
#define AUTO_HELLO_LEN          4
#define AUTO_HELLO_ACK_LEN      40
#define AUTO_HB_LEN             4
#define AUTO_ARM_LEN            4      /* A2b */
#define AUTO_TAKEOFF_LEN        12     /* A2b */
#define AUTO_TAKEOFF_PLAN_LEN   18     /* with a planned move (docs/autonomy-a2b.md 21.4): an older drone answers RANGE,
                                          detail 0 (the link's length check: not processed, no take-off) */
#define AUTO_YAW_LEN            4      /* interactive hover */
#define AUTO_MOVE_LEN           8      /* interactive hover */
#define AUTO_SET_HEIGHT_LEN     4      /* SET_HEIGHT (2026-10-15): "you are at X", flown back to the old estimate */
#define AUTO_SET_HEIGHT_TO_LEN  6      /* SET_HEIGHT with toCm (2026-10-16): "you are at X, go to the target" */
#define AUTO_PHOTO_LEN          4
#define AUTO_CHUNKS_LEN         6
#define AUTO_PHOTO_FREE_LEN     2
#define AUTO_ACK_LEN            12
#define AUTO_STATUS_LEN         48
#define AUTO_AUTO_STATUS_LEN    140    /* A2b 64, the interactive hover appends 32, hold and report 40 (2026-10-15), the
                                          speed step 4 (2026-10-16) */
#define AUTO_AUTO_STATUS_HR_LEN 136    /* a hold-and-report drone's AUTO_STATUS (2026-10-15 to the fast-move build) */
#define AUTO_AUTO_STATUS_IH_LEN 96     /* an interactive-hover drone's AUTO_STATUS (before 2026-10-15) */
#define AUTO_AUTO_STATUS_A2B_LEN 64    /* an A2b drone's AUTO_STATUS (the parser takes all four) */
#define AUTO_META_LEN           40     /* a grey frame's META: unchanged since A1 */
#define AUTO_META_COLOR_LEN     44     /* a colour frame's (2026-10-15, docs/autonomy-link.md "Photos"): + u8 pixFmt,
                                          3 bytes 0. An older Mac reads only 40-byte METAs: NVS photocol 0 */
#define AUTO_META_PIXFMT_OFF    40
#define AUTO_CHUNK_HDR_LEN      8
#define AUTO_CHUNK_DATA_MAX     1024
#define AUTO_FW_LEN             24

typedef struct {
  uint8_t type;
  uint8_t flags;
  uint32_t session;
  uint32_t seq;
  uint16_t len;   /* payload length */
} autoProtoHdr_t;

/* HMAC-SHA256 of data with key; writes 32 bytes. */
typedef void (*autoProtoHmacFn)(const uint8_t *key, size_t keyLen, const uint8_t *data, size_t len,
                                uint8_t out[32]);

typedef enum {
  AUTO_DEC_OK = 0,
  AUTO_DEC_SHORT = -1,
  AUTO_DEC_MAGIC = -2,
  AUTO_DEC_VERSION = -3,
  AUTO_DEC_LENGTH = -4,
  AUTO_DEC_TAG = -5,
} autoDecodeResult_t;

/* Builds a packet in out; returns its length, or -1 if it does not fit or
 * the payload is too long. */
int autoProtoEncode(const autoProtoHdr_t *h, const uint8_t *payload, const uint8_t key[AUTO_PROTO_KEY_LEN],
                    autoProtoHmacFn hmac, uint8_t *out, size_t outCap);

/* Checks framing and tag (constant-time compare); on success fills h and
 * points payload into buf. */
autoDecodeResult_t autoProtoDecode(const uint8_t *buf, size_t len, const uint8_t key[AUTO_PROTO_KEY_LEN],
                                   autoProtoHmacFn hmac, autoProtoHdr_t *h, const uint8_t **payload);

/* Mac sequence numbers within one session: strictly increasing; a repeat of
 * one of the last AUTO_PROTO_SEQ_RECENT accepted values is a duplicate (the
 * caller re-sends its cached ack and does not act again). */
typedef enum { AUTO_SEQ_NEW, AUTO_SEQ_DUP, AUTO_SEQ_OLD } autoSeqResult_t;

typedef struct {
  uint32_t lastSeq;
  bool any;
  uint32_t recent[AUTO_PROTO_SEQ_RECENT];
  uint8_t recentCount;
  uint8_t recentHead;
} autoProtoSeqFilter_t;

void autoProtoSeqReset(autoProtoSeqFilter_t *f);
autoSeqResult_t autoProtoSeqCheck(autoProtoSeqFilter_t *f, uint32_t seq);

/* CRC-32 (IEEE 802.3, as zlib.crc32). */
uint32_t autoProtoCrc32(const uint8_t *data, size_t len);

/* ---- A2b command payloads ------------------------------------------------
 * The parsers check the exact length and that every pad byte is 0; they do
 * not range-check values (the state machine does: RANGE). TAKEOFF byte 7 is
 * no pad since 2026-10-14 (docs/autonomy-a2b.md 18.2): it carries bits 16-23
 * of actWaitMs, so a wait of up to 120 s fits the same 12 bytes. An older
 * Mac sends 0 there (a wait <= 65535 ms reads exactly as before); an older
 * drone refuses a nonzero byte 7 (RANGE, detail 0), never misreading it. */

typedef struct {
  uint8_t mode;            /* autoMode_t: 0 FLIGHT, 1 DRY, 2 BENCH */
  uint16_t airMaxMs;       /* 0 = the NVS value */
} autoProtoArm_t;

typedef struct {
  uint32_t token;          /* the armToken from the ARM ack's detail */
  uint16_t heightCm;       /* 20..200 (AUTO_HEIGHT_CM_MIN..MAX; 20..120 until 2026-10-15) */
  uint8_t photoN;          /* 0..3 */
  uint16_t photoGapMs;     /* 150..1000 (also when photoN is 0) */
  uint32_t actWaitMs;      /* this sortie's hover wait, u24 on the wire (byte 7 high, bytes 10-11 low): 0 = NVS
                              autowait, else 1000..120000 (60000 until 2026-10-14; A2b: must be 0) */
  /* The planned move from take-off (2026-10-14, docs/autonomy-a2b.md 21.4): flown as a MOVE at the start of the
   * hover wait, in the take-off heading's frame; 0, 0: none (the 12-byte TAKEOFF). The state machine range-checks
   * it as a MOVE (auto_motion.h autoMvRangeOk) and needs a nonzero hover wait for it (RANGE). */
  int16_t planRightCm;     /* + right, - left */
  int16_t planFwdCm;       /* + forward (toward the scene ahead), - back */
} autoProtoTakeoff_t;

typedef struct {
  int16_t rightCm;         /* + right, - left: |.| <= 150 */
  int16_t fwdCm;           /* + forward, - back: -150..400 (150 until 2026-10-14), sqrt(right^2 + fwd^2) <= 400 */
  int16_t upCm;            /* + up, - down: |.| <= 50 (with up, |fwd| <= 150) */
} autoProtoMove_t;

typedef struct {
  int16_t deg;             /* + clockwise (right), - left: |.| <= 180 */
} autoProtoYaw_t;

typedef struct {
  int16_t xCm;             /* the judged true height, cm (the link refuses < 0: RANGE detail 0; the FSM 10..250) */
  int16_t atCm;            /* the drone's estimate at the judged photo, cm; INT16_MIN (AUTO_SH_AT_NOW): now */
  int16_t toCm;            /* toForm: the height to fly to, cm (the FSM 25..200); INT16_MIN (AUTO_SH_TO_REF): the
                              commanded height (take-off height + the moves); 0 without toForm */
  bool toForm;             /* the 6-byte form (AUTO_SET_HEIGHT_TO_LEN): fly to the target, not back to the old
                              estimate (auto_fsm.h AUTO_SH_FORM_TO) */
} autoProtoSetHeight_t;

typedef struct {
  uint8_t n;               /* 1..3 */
  uint16_t gapMs;          /* A1: 0..1000; in the hover wait: 150..1000 */
} autoProtoPhoto_t;

bool autoProtoParseArm(const uint8_t *p, uint16_t len, autoProtoArm_t *out);
/* TAKEOFF: AUTO_TAKEOFF_LEN (plan 0, 0) or AUTO_TAKEOFF_PLAN_LEN (bytes 16-17 must be 0); any other length is
 * malformed (false: no take-off). */
bool autoProtoParseTakeoff(const uint8_t *p, uint16_t len, autoProtoTakeoff_t *out);
bool autoProtoParsePhoto(const uint8_t *p, uint16_t len, autoProtoPhoto_t *out);
bool autoProtoParseMove(const uint8_t *p, uint16_t len, autoProtoMove_t *out);
bool autoProtoParseYaw(const uint8_t *p, uint16_t len, autoProtoYaw_t *out);
/* SET_HEIGHT: AUTO_SET_HEIGHT_LEN (toForm false, toCm 0) or AUTO_SET_HEIGHT_TO_LEN (toForm true); any other length is
 * malformed (false). */
bool autoProtoParseSetHeight(const uint8_t *p, uint16_t len, autoProtoSetHeight_t *out);
void autoProtoPackArm(const autoProtoArm_t *a, uint8_t out[AUTO_ARM_LEN]);
/* Writes AUTO_TAKEOFF_PLAN_LEN bytes only when a plan is set (else the A2b 12, so a sortie without a plan still
 * flies on an older drone); returns the length written. out holds at least AUTO_TAKEOFF_PLAN_LEN bytes. */
uint16_t autoProtoPackTakeoff(const autoProtoTakeoff_t *t, uint8_t *out);
void autoProtoPackMove(const autoProtoMove_t *m, uint8_t out[AUTO_MOVE_LEN]);
void autoProtoPackYaw(const autoProtoYaw_t *y, uint8_t out[AUTO_YAW_LEN]);
void autoProtoPackSetHeight(const autoProtoSetHeight_t *sh, uint8_t out[AUTO_SET_HEIGHT_LEN]);   /* the 4-byte form */
void autoProtoPackSetHeightTo(const autoProtoSetHeight_t *sh, uint8_t out[AUTO_SET_HEIGHT_TO_LEN]);   /* the 6-byte */

/* ---- AUTO_STATUS (0x84), 64 bytes, little-endian ---------------------------
 *  off size field
 *    0  4  upMs            drone FreeRTOS tick, ms
 *    4  4  bootNonce       as HELLO_ACK / STATUS (a change: the drone reset)
 *    8  4  transitions     state machine transitions since boot
 *   12  4  failedGuards    IDLE: the FLIGHT guard bitmap (AUTO_G_*), 10 Hz; else the bitmap last checked
 *   16  1  state           autoState_t
 *   17  1  mode            autoMode_t of the current or last ARM
 *   18  1  reason          autoReason_t of the latest transition
 *   19  1  vzLandReason    vzReason_t (when reason is VZ, or LAND ended)
 *   20  1  landTrip        autoReason_t (TOUCHDOWN_TRIP: TILT or RATE)
 *   21  1  vzPhase         vzPhase_t (VZ_IDLE outside a sortie)
 *   22  1  vzReason        vzReason_t of the controller's latest phase change
 *   23  1  flags           AUTO_STF_*
 *   24  1  trips           AUTO_TRF_*: trip/rest conditions true now
 *   25  1  capture         autoCapture_t of the set the state machine started
 *   26  2  photoId         that set's photo id (META/CHUNK id), 0 none
 *   28  2  stateMs         time in the state, saturated at 65535
 *   30  2  airMs           air clock (from the first RAMP step), saturated
 *   32  2  stableMs        age of the STABLE streak (auto_fsm.h), saturated
 *   34  2  waitLeftMs      hover wait left (0 unless waiting)
 *   36  2  hoverWaitMs     the wait latched at ARM (NVS autowait)
 *   38  2  vbatMv          10 Hz battery
 *   40  2  vbatFastMinMv   lowest 1 kHz battery over the last 320 ms
 *   42  2  hMm      (i16)  controller height estimate, mm
 *   44  2  vzMms    (i16)  controller vertical speed estimate, mm/s, up
 *   46  2  thrust          controller command (before the mode clamp)
 *   48  2  tLo             thrust at lift-off (0 before)
 *   50  2  tHold           average thrust in HOLD (0 before)
 *   52  2  tRef            LAND reference (0 before LAND)
 *   54  2  kPriorDiv10     the sortie's stored hover gain / 10 (NVS vzK)
 *   56  2  khatDiv10       the controller's hover gain estimate / 10
 *   58  1  lastCmdType     autoCmdType_t of the last command the FSM processed (SET_HEIGHT 10)
 *   59  1  lastCmdResult   its autoResult_t
 *   60  4  killCount       Mac KILLs received since boot
 * The interactive hover (appended; an A2b drone sends the first 64 bytes only):
 *   64  1  mvState         autoMvState_t (auto_motion.h)
 *   65  1  mvKind          autoMvKind_t of the latest motion
 *   66  1  mvWhy           autoMvWhy_t of the latest refused MOVE / YAW / SET_HEIGHT
 *   67  1  mvEnd           autoMvEnd_t of the latest motion
 *   68  1  mvFlags         AUTO_MVF_*: the scene distance valid / stale / sim / from a forward move, the camera
 *                          height guess allowed, the re-track bob; 0x40 with mvKind MOVE: a SET_HEIGHT's correction
 *   69  1  battLow         bit 0 (AUTO_BATTLOW_HELD): the BATT_LOW condition (the 10 Hz battery below
 *                          AUTO_VBAT_LOW_LAND_V, 3.00 V, or the hover thrust T_hold above AUTO_HEADROOM_LAND_T,
 *                          57000) has held 0.25 s (MOVE / YAW refused; BATT_LOW after 0.5 s in HOVER); bit 1
 *                          (AUTO_BATTLOW_NOW): it holds now (single readings scatter with an sd of 29-39 mV).
 *                          A MOVE / YAW refused for T_hold above AUTO_MV_REFUSE_T (53400) alone does not set
 *                          bit 0: the reply LOW_BATT tells it
 *   70  2  mvSeq           motions accepted this sortie
 *   72  2  progRightCm (i16) the latest motion's measured progress (FLIGHT: the camera hold; DRY/BENCH: ideal)
 *   74  2  progFwdCm   (i16)
 *   76  2  progUpCm    (i16) the height estimate's change
 *   78  2  tgtRightCm  (i16) its command
 *   80  2  tgtFwdCm    (i16)
 *   82  2  tgtUpCm     (i16)
 *   84  2  sceneDCm        the scene distance D a MOVE would use now (0: none, see mvFlags)
 *   86  2  hTargetCm   (i16) take-off height + the vertical moves (commanded; the estimate is hMm)
 *   88  2  yawOffCdeg  (i16) heading from the take-off heading, 0.01 deg, + clockwise
 *   90  2  yawTgtDeg   (i16) the latest YAW's command
 *   92  2  mvPhaseMs       time in mvState, saturated
 *   94  2  airLeftDs       0 since 2026-10-15 (no air clock; until then the 120 s air limit's, 0.1 s)
 * Hold and report (appended 2026-10-15, docs/autonomy-a2b.md 22; an interactive-hover drone sends 96 bytes):
 *   96  1  evCode          autoEvent_t of the latest event (auto_fsm.h): what the drone did on its own
 *   97  1  evDetail        its detail (camBits, a vzReason_t, an autoMvEnd_t, ms, ...)
 *   98  2  evCount         events this sortie (wraps): a change is a new event
 *  100  2  evAgeMs         since it, saturated
 *  102  1  holdBits        AUTO_HB_*: CAM_LOST, SINKING, HUSH, K_CLAMP, CAPPED, NO_VIEW, NOT_STABLE, OPEN_LOOP
 *  103  1  camBits         AUTO_CB_*: why the camera height is out (latched at CAM_LOST, ORed while out)
 *  104  2  camOffMs        the camera height out this long (with CAM_LOST), saturated
 *  106  2  sinkInMs        the SINK in this long (with CAM_LOST; 0 at once), saturated
 *  108  2  peakAccCg       the latest contact's peak |acc|, 0.01 g
 *  110  2  peakRateDps     its peak max(|gx|, |gy|), dps
 *  112  2  contactAxCg (i16) its body-frame force at the peak, x forward, 0.01 g (the obstacle is opposite)
 *  114  2  contactAyCg (i16) y left
 *  116  2  sinceCmdDs      since Claude's latest accepted MOVE, YAW, STOP or PHOTO, 0.1 s (65535: none)
 *  118  2  airDs           the air clock, 0.1 s (exact past 65.5 s)
 *  120  2  khatPm          the controller's Khat / kPrior, per mille
 *  122  2  capQ            the controller's open-loop thrust bound (0: none yet)
 *  124  2  contactYawCdeg (i16) the heading offset at the latest contact, 0.01 deg (+ clockwise)
 *  126  1  contacts        this sortie, saturating
 *  127  1  loopGaps        this sortie, saturating
 *  128  1  ev1Code         the event before the latest ...
 *  129  1  ev1Detail
 *  130  2  ev1AgeMs
 *  132  1  ev2Code         ... and the one before that (a 100 ms push may carry three)
 *  133  1  ev2Detail
 *  134  2  ev2AgeMs
 * The speed step (appended 2026-10-16, docs/autonomy-a2b.md 31; a hold-and-report drone sends 136 bytes):
 *  136  2  mvSpeedPct      the motion speed step, % (100..300): the NVS mvspeed in IDLE, the sortie's latched value
 *                          from ARM on (0 when parsed from an older drone's status: it flies 100)
 *  138  2  reserved        0
 * Python: struct "<IIII10B8H2h6H2BI" + "<6BH6hHhhhHH" + "<BBHHBBHHHHhhHHHHhBBBBHBBH" + "<HH". Pushed every 100 ms
 * while autonomy is active, at once on every transition and event, every 500 ms for 2 s after returning to IDLE,
 * and after each STATUS reply. */
#define AUTO_BATTLOW_HELD       0x01   /* AUTO_STATUS battLow bits */
#define AUTO_BATTLOW_NOW        0x02
#define AUTO_STF_WAITING        0x01   /* HOVER: photos done, waiting for the Mac */
#define AUTO_STF_STABLE         0x02   /* STABLE conditions hold now */
#define AUTO_STF_CAM_HEIGHT     0x04   /* the controller uses the camera height observer this step (vz camHealthy) */
#define AUTO_STF_CAM_AWAKE      0x08   /* camera held out of its take-off standby (camNoStandby) */
#define AUTO_STF_RESET_LOCKED   0x10   /* reset-lock: FLIGHT and BENCH refused until a battery power-on */
#define AUTO_STF_PRESPIN        0x20   /* pre-spin warning (motors start within 1 s) */
#define AUTO_STF_SIM            0x40   /* the controller flies the sim plant (DRY, BENCH-sim) */
#define AUTO_STF_HOLD           0x80   /* camera drift hold applied (engaged or coasting) */

#define AUTO_TRF_TILT           0x01   /* tilt > 30 deg */
#define AUTO_TRF_RATE           0x02   /* |gx| or |gy| > 500 dps */
#define AUTO_TRF_FALL           0x04   /* |acc| < 0.4 g */
#define AUTO_TRF_ATTERR         0x08   /* attitude tracking error > 10 deg */
#define AUTO_TRF_VBAT_LOW       0x10   /* 10 Hz battery < AUTO_VBAT_LAND_V (2.60 V) */
#define AUTO_TRF_MOVED          0x20   /* ARMED/pre-spin stillness lost */
#define AUTO_TRF_REST           0x40   /* at rest (gyro < 10 dps, | |acc| - 1 | < 0.15 g) */
#define AUTO_TRF_VZ_CAP         0x80   /* the controller's thrust is at a ceiling */

typedef struct {
  uint32_t upMs, bootNonce, transitions, failedGuards;
  uint8_t state, mode, reason, vzLandReason, landTrip, vzPhase, vzReason, flags, trips, capture;
  uint16_t photoId, stateMs, airMs, stableMs, waitLeftMs, hoverWaitMs, vbatMv, vbatFastMinMv;
  int16_t hMm, vzMms;
  uint16_t thrust, tLo, tHold, tRef, kPriorDiv10, khatDiv10;
  uint8_t lastCmdType, lastCmdResult;
  uint32_t killCount;
  /* interactive hover (bytes 64..95) */
  uint8_t mvState, mvKind, mvWhy, mvEnd, mvFlags, battLow;
  uint16_t mvSeq;
  int16_t progRightCm, progFwdCm, progUpCm, tgtRightCm, tgtFwdCm, tgtUpCm;
  uint16_t sceneDCm;
  int16_t hTargetCm, yawOffCdeg, yawTgtDeg;
  uint16_t mvPhaseMs, airLeftDs;
  /* hold and report (bytes 96..135) */
  uint8_t evCode, evDetail;
  uint16_t evCount, evAgeMs;
  uint8_t holdBits, camBits;
  uint16_t camOffMs, sinkInMs, peakAccCg, peakRateDps;
  int16_t contactAxCg, contactAyCg;
  uint16_t sinceCmdDs, airDs, khatPm, capQ;
  int16_t contactYawCdeg;
  uint8_t contacts, loopGaps;
  uint8_t ev1Code, ev1Detail;
  uint16_t ev1AgeMs;
  uint8_t ev2Code, ev2Detail;
  uint16_t ev2AgeMs;
  /* the speed step (bytes 136..139) */
  uint16_t mvSpeedPct;
} autoStatusMsg_t;

void autoProtoPackAutoStatus(const autoStatusMsg_t *m, uint8_t out[AUTO_AUTO_STATUS_LEN]);
/* Takes AUTO_AUTO_STATUS_LEN bytes, a hold-and-report drone's AUTO_AUTO_STATUS_HR_LEN, an interactive-hover drone's
 * AUTO_AUTO_STATUS_IH_LEN or an A2b drone's AUTO_AUTO_STATUS_A2B_LEN (the rest 0). */
bool autoProtoParseAutoStatus(const uint8_t *p, uint16_t len, autoStatusMsg_t *out);

static inline void autoPutU16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static inline void autoPutU32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static inline uint16_t autoGetU16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t autoGetU32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint16_t autoChunkCount(uint32_t size)
{
  return (uint16_t)((size + AUTO_CHUNK_DATA_MAX - 1) / AUTO_CHUNK_DATA_MAX);
}

/* ---- photo pixel formats (META byte 40 of a 44-byte META) ---------------------------------------------------
 * GRAY8: 8-bit luma, row-major, width bytes a row (a 40-byte META: every grey frame, as since A1). YUYV: YUV 4:2:2
 * as the OV3660 sends it (esp32-camera PIXFORMAT_YUV422), a pixel pair in 4 bytes Y0 U Y1 V, 2 x width bytes a
 * row, full-range BT.601 (Y is the grey frame's value). Any other code: a frame the Mac cannot decode. */
#define AUTO_PIXFMT_GRAY8       0
#define AUTO_PIXFMT_YUYV        1
#define AUTO_PIXFMT_JPEG        2

/* Bytes per pixel of a pixel format (0: unknown). */
static inline uint32_t autoPixfmtBytesPerPixel(uint8_t fmt)
{
  return fmt == AUTO_PIXFMT_GRAY8 ? 1u : (fmt == AUTO_PIXFMT_YUYV ? 2u : 0u);
}

/* The META of a frame in this format: AUTO_META_LEN for grey (byte for byte the A1 META), AUTO_META_COLOR_LEN with
 * the format at AUTO_META_PIXFMT_OFF otherwise. */
static inline uint16_t autoMetaLen(uint8_t fmt)
{
  return fmt == AUTO_PIXFMT_GRAY8 ? AUTO_META_LEN : AUTO_META_COLOR_LEN;
}

#endif /* AUTO_PROTO_H_ */
