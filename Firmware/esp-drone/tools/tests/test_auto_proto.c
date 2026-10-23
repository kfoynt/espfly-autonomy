/*
 * Host unit tests for components/core/crazyflie/utils/src/auto_proto.c
 * against tools/tests/auto_proto_golden.json (stage A1) and
 * tools/tests/auto_proto_golden_a2.json (stage A2b: ARM, TAKEOFF, LAND, KILL,
 * DISARM, PHOTO in the hover, their ACKs and AUTO_STATUS) and
 * tools/tests/auto_proto_golden_ih.json (the interactive hover: MOVE, YAW,
 * STOP, TAKEOFF with a wait, the 96-byte AUTO_STATUS; SET_HEIGHT and a colour
 * photo's 44-byte META since 2026-10-15), all made by
 * gen_auto_proto_golden.py. The A2 and interactive packets are rebuilt from
 * their fields with the packers and parsed back.
 *
 * Build and run from Firmware/esp-drone (macOS: HMAC from CommonCrypto):
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_auto_proto.c components/core/crazyflie/utils/src/auto_proto.c \
 *      -o /tmp/test_auto_proto && /tmp/test_auto_proto tools/tests/auto_proto_golden.json \
 *      tools/tests/auto_proto_golden_a2.json tools/tests/auto_proto_golden_ih.json
 */
#include <CommonCrypto/CommonDigest.h>
#include <CommonCrypto/CommonHMAC.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_proto.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static void hmacSha256(const uint8_t *key, size_t keyLen, const uint8_t *data, size_t len, uint8_t out[32])
{
  CCHmac(kCCHmacAlgSHA256, key, keyLen, data, len, out);
}

static char *json;      /* the golden file being read: A1, then A2 */

/* Hex string value of "name": "..." in the golden file (flat, our own format). */
static size_t golden(const char *name, uint8_t *out, size_t cap)
{
  char pat[64];
  snprintf(pat, sizeof(pat), "\"%s\": \"", name);
  const char *p = strstr(json, pat);
  if (!p) {
    return 0;
  }
  p += strlen(pat);
  size_t n = 0;
  while (p[0] && p[0] != '"' && n < cap) {
    unsigned v;
    sscanf(p, "%2x", &v);
    out[n++] = (uint8_t)v;
    p += 2;
  }
  return n;
}

static uint8_t key[16];

/* Re-encode a golden packet from its own header fields and payload. */
static void checkPacket(const char *name)
{
  uint8_t g[AUTO_PROTO_MAX_PACKET];
  size_t glen = golden(name, g, sizeof(g));
  CHECK(glen >= AUTO_PROTO_HDR_LEN + AUTO_PROTO_TAG_LEN, "golden %s present", name);
  if (glen < AUTO_PROTO_HDR_LEN + AUTO_PROTO_TAG_LEN) {
    return;
  }
  autoProtoHdr_t h;
  const uint8_t *payload = NULL;
  CHECK(autoProtoDecode(g, glen, key, hmacSha256, &h, &payload) == AUTO_DEC_OK, "%s decodes", name);
  uint8_t out[AUTO_PROTO_MAX_PACKET];
  int n = autoProtoEncode(&h, payload, key, hmacSha256, out, sizeof(out));
  CHECK(n == (int)glen && memcmp(out, g, glen) == 0, "%s re-encodes byte for byte", name);

  uint8_t bad[AUTO_PROTO_MAX_PACKET];
  memcpy(bad, g, glen);
  bad[AUTO_PROTO_HDR_LEN - 1] ^= 1;   /* reserved byte: covered by the tag */
  CHECK(autoProtoDecode(bad, glen, key, hmacSha256, &h, &payload) == AUTO_DEC_TAG, "%s: header change breaks tag", name);
  memcpy(bad, g, glen);
  bad[glen - 1] ^= 0x80;
  CHECK(autoProtoDecode(bad, glen, key, hmacSha256, &h, &payload) == AUTO_DEC_TAG, "%s: tag change detected", name);
  uint8_t otherKey[16];
  memcpy(otherKey, key, 16);
  otherKey[0] ^= 1;
  CHECK(autoProtoDecode(g, glen, otherKey, hmacSha256, &h, &payload) == AUTO_DEC_TAG, "%s: wrong key rejected", name);
  /* An empty payload truncated is shorter than header + tag. */
  const autoDecodeResult_t truncated = glen - 1 < AUTO_PROTO_HDR_LEN + AUTO_PROTO_TAG_LEN ? AUTO_DEC_SHORT : AUTO_DEC_LENGTH;
  CHECK(autoProtoDecode(g, glen - 1, key, hmacSha256, &h, &payload) == truncated, "%s: truncated rejected", name);
}

static void testFields(void)
{
  uint8_t g[AUTO_PROTO_MAX_PACKET];
  size_t glen = golden("photo", g, sizeof(g));
  autoProtoHdr_t h;
  const uint8_t *p;
  CHECK(autoProtoDecode(g, glen, key, hmacSha256, &h, &p) == AUTO_DEC_OK, "photo decodes");
  CHECK(h.type == AUTO_T_PHOTO && h.flags == AUTO_FLAG_ACKREQ && h.session == 0xA1B2C3D4u && h.seq == 2 &&
        h.len == AUTO_PHOTO_LEN, "photo header fields");
  CHECK(p[0] == 2 && autoGetU16(p + 2) == 300, "photo payload fields");

  glen = golden("meta", g, sizeof(g));
  CHECK(autoProtoDecode(g, glen, key, hmacSha256, &h, &p) == AUTO_DEC_OK && h.len == AUTO_META_LEN, "meta length");
  CHECK(autoGetU16(p + 4) == 160 && autoGetU16(p + 6) == 120 && autoGetU32(p + 8) == 19200 &&
        (int16_t)autoGetU16(p + 24) == -150 && autoGetU16(p + 36) == 19, "meta fields");

  glen = golden("hello_ack", g, sizeof(g));
  CHECK(autoProtoDecode(g, glen, key, hmacSha256, &h, &p) == AUTO_DEC_OK && h.len == AUTO_HELLO_ACK_LEN,
        "hello_ack length");
  uint8_t digest[32];
  CC_SHA256(key, 16, digest);
  CHECK(memcmp(p + 12, digest, 4) == 0, "hello_ack key fingerprint = SHA-256(key)[0:4]");
}

static void testCrc(void)
{
  static uint8_t frame[19200];
  for (int y = 0; y < 120; y++) {
    for (int x = 0; x < 160; x++) {
      frame[y * 160 + x] = (uint8_t)((x * 7 + y * 3) & 0xFF);
    }
  }
  const char *p = strstr(json, "\"frame_crc32\": ");
  unsigned long want = p ? strtoul(p + strlen("\"frame_crc32\": "), NULL, 10) : 0;
  CHECK(autoProtoCrc32(frame, sizeof(frame)) == (uint32_t)want, "CRC-32 matches zlib (%lu)", want);
  CHECK(autoProtoCrc32((const uint8_t *)"123456789", 9) == 0xCBF43926u, "CRC-32 check value");
  CHECK(autoChunkCount(19200) == 19 && autoChunkCount(1024) == 1 && autoChunkCount(1025) == 2, "chunk count");
}

static void testSeq(void)
{
  autoProtoSeqFilter_t f;
  autoProtoSeqReset(&f);
  CHECK(autoProtoSeqCheck(&f, 5) == AUTO_SEQ_NEW, "first seq accepted");
  CHECK(autoProtoSeqCheck(&f, 5) == AUTO_SEQ_DUP, "repeat is a duplicate");
  CHECK(autoProtoSeqCheck(&f, 4) == AUTO_SEQ_OLD, "older unseen seq dropped");
  CHECK(autoProtoSeqCheck(&f, 9) == AUTO_SEQ_NEW, "gap forward accepted");
  for (uint32_t s = 10; s < 20; s++) {
    autoProtoSeqCheck(&f, s);
  }
  CHECK(autoProtoSeqCheck(&f, 12) == AUTO_SEQ_DUP, "recent repeat still a duplicate");
  CHECK(autoProtoSeqCheck(&f, 9) == AUTO_SEQ_OLD, "repeat beyond the recent window is old");
}

static void testEncodeLimits(void)
{
  autoProtoHdr_t h = {.type = AUTO_T_CHUNK, .len = AUTO_PROTO_MAX_PAYLOAD + 1};
  uint8_t buf[AUTO_PROTO_MAX_PACKET + 8];
  static uint8_t payload[AUTO_PROTO_MAX_PAYLOAD + 1];
  CHECK(autoProtoEncode(&h, payload, key, hmacSha256, buf, sizeof(buf)) < 0, "payload over the limit refused");
  h.len = 10;
  CHECK(autoProtoEncode(&h, payload, key, hmacSha256, buf, 41) < 0, "too small an output refused");
  CHECK(autoProtoEncode(&h, payload, key, hmacSha256, buf, 42) == 42, "exact fit encodes");
}

/* ---- stage A2b (auto_proto_golden_a2.json) ------------------------------ */

#define A2_SESSION 0xA1B2C3D4u

/* Encodes a packet from header fields and payload and compares it with the
 * golden packet of that name, byte for byte. */
static void checkBuilt(const char *name, uint8_t type, uint8_t flags, uint32_t session, uint32_t seq,
                       const uint8_t *payload, uint16_t len)
{
  uint8_t g[AUTO_PROTO_MAX_PACKET], out[AUTO_PROTO_MAX_PACKET];
  const size_t glen = golden(name, g, sizeof(g));
  const autoProtoHdr_t h = {.type = type, .flags = flags, .session = session, .seq = seq, .len = len};
  const int n = autoProtoEncode(&h, payload, key, hmacSha256, out, sizeof(out));
  CHECK(n > 0 && (size_t)n == glen && memcmp(out, g, glen) == 0, "%s built from its fields equals the golden packet",
        name);
}

/* Payload of a golden packet (decoded, tag checked). */
static const uint8_t *goldenPayload(const char *name, uint8_t *buf, size_t cap, autoProtoHdr_t *h)
{
  const uint8_t *p = NULL;
  const size_t n = golden(name, buf, cap);
  if (autoProtoDecode(buf, n, key, hmacSha256, h, &p) != AUTO_DEC_OK) {
    memset(h, 0, sizeof(*h));
    return NULL;
  }
  return p;
}

static void ackPayload(uint8_t p[AUTO_ACK_LEN], uint32_t ackSeq, uint8_t ackType, uint8_t result, uint8_t state,
                       uint32_t detail)
{
  autoPutU32(p, ackSeq);
  p[4] = ackType;
  p[5] = result;
  p[6] = state;
  p[7] = 0;
  autoPutU32(p + 8, detail);
}

/* The AUTO_STATUS fields of gen_auto_proto_golden.py. */
static autoStatusMsg_t goldenStatusFields(void)
{
  autoStatusMsg_t m;
  memset(&m, 0, sizeof(m));
  m.upMs = 123456;
  m.bootNonce = 0x0BADF00D;
  m.transitions = 7;
  m.failedGuards = 4;
  m.state = 3;
  m.mode = 0;
  m.reason = 3;
  m.vzLandReason = 0;
  m.landTrip = 0;
  m.vzPhase = 4;
  m.vzReason = 4;
  m.flags = 0x8D;
  m.trips = 0x40;
  m.capture = 2;
  m.photoId = 5;
  m.stateMs = 2100;
  m.airMs = 4300;
  m.stableMs = 1050;
  m.waitLeftMs = 3950;
  m.hoverWaitMs = 5000;
  m.vbatMv = 3800;
  m.vbatFastMinMv = 3420;
  m.hMm = 305;
  m.vzMms = -12;
  m.thrust = 47100;
  m.tLo = 35400;
  m.tHold = 46800;
  m.tRef = 0;
  m.kPriorDiv10 = 16500;
  m.khatDiv10 = 16620;
  m.lastCmdType = 6;
  m.lastCmdResult = 0;
  m.killCount = 1;
  return m;
}

static void testA2Golden(void)
{
  const char *names[] = {"arm", "arm_air", "takeoff", "land", "disarm", "kill", "kill_any_session", "photo_hover",
                         "ack_arm", "ack_guards", "auto_status"};
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
    checkPacket(names[i]);
  }
  CHECK(strstr(json, "\"auto_status_fmt\": \"<IIII10B8H2h6H2BI\"") != NULL, "AUTO_STATUS format string");

  /* Every packet from its fields, with the packers where there is one. */
  uint8_t p[AUTO_PROTO_MAX_PAYLOAD];
  const autoProtoArm_t arm = {.mode = 1, .airMaxMs = 0}, armAir = {.mode = 0, .airMaxMs = 3000};
  autoProtoPackArm(&arm, p);
  checkBuilt("arm", AUTO_T_ARM, AUTO_FLAG_ACKREQ, A2_SESSION, 10, p, AUTO_ARM_LEN);
  autoProtoPackArm(&armAir, p);
  checkBuilt("arm_air", AUTO_T_ARM, AUTO_FLAG_ACKREQ, A2_SESSION, 11, p, AUTO_ARM_LEN);
  const autoProtoTakeoff_t to = {.token = 0x5EED0001u, .heightCm = 40, .photoN = 2, .photoGapMs = 300, .actWaitMs = 0};
  autoProtoPackTakeoff(&to, p);
  checkBuilt("takeoff", AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, A2_SESSION, 12, p, AUTO_TAKEOFF_LEN);
  checkBuilt("land", AUTO_T_LAND, AUTO_FLAG_ACKREQ, A2_SESSION, 13, NULL, 0);
  checkBuilt("disarm", AUTO_T_DISARM, AUTO_FLAG_ACKREQ, A2_SESSION, 14, NULL, 0);
  checkBuilt("kill", AUTO_T_KILL, AUTO_FLAG_ACKREQ, A2_SESSION, 15, NULL, 0);
  checkBuilt("kill_any_session", AUTO_T_KILL, AUTO_FLAG_ACKREQ, 0, 0x7FFFFFFFu, NULL, 0);
  p[0] = 1;
  p[1] = 0;
  autoPutU16(p + 2, 150);
  checkBuilt("photo_hover", AUTO_T_PHOTO, AUTO_FLAG_ACKREQ, A2_SESSION, 16, p, AUTO_PHOTO_LEN);
  ackPayload(p, 10, AUTO_T_ARM, AUTO_R_OK, 1 /* ARMED */, 0x5EED0001u);
  checkBuilt("ack_arm", AUTO_T_ACK, 0, A2_SESSION, 20, p, AUTO_ACK_LEN);
  ackPayload(p, 11, AUTO_T_ARM, AUTO_R_GUARDS, 0 /* IDLE */, 0x0000040Cu);
  checkBuilt("ack_guards", AUTO_T_ACK, 0, A2_SESSION, 21, p, AUTO_ACK_LEN);
  const autoStatusMsg_t st = goldenStatusFields();
  memset(p, 0xEE, AUTO_AUTO_STATUS_LEN + 4);
  autoProtoPackAutoStatus(&st, p);
  CHECK(p[AUTO_AUTO_STATUS_LEN] == 0xEE && p[AUTO_AUTO_STATUS_LEN + 3] == 0xEE && AUTO_AUTO_STATUS_LEN == 140 &&
        AUTO_AUTO_STATUS_HR_LEN == 136, "AUTO_STATUS packs exactly 140 bytes (2026-10-16: + the speed step)");
  /* an A2b drone's AUTO_STATUS is the first 64 bytes, unchanged */
  checkBuilt("auto_status", AUTO_T_AUTO_STATUS, 0, A2_SESSION, 22, p, AUTO_AUTO_STATUS_A2B_LEN);

  /* ... and parsed back to the same fields. */
  uint8_t g[AUTO_PROTO_MAX_PACKET];
  autoProtoHdr_t h;
  const uint8_t *pl = goldenPayload("arm", g, sizeof(g), &h);
  autoProtoArm_t a;
  CHECK(pl && autoProtoParseArm(pl, h.len, &a) && a.mode == 1 && a.airMaxMs == 0, "arm parses back");
  pl = goldenPayload("arm_air", g, sizeof(g), &h);
  CHECK(pl && autoProtoParseArm(pl, h.len, &a) && a.mode == 0 && a.airMaxMs == 3000, "arm_air parses back");
  pl = goldenPayload("takeoff", g, sizeof(g), &h);
  autoProtoTakeoff_t t;
  CHECK(pl && autoProtoParseTakeoff(pl, h.len, &t) && t.token == 0x5EED0001u && t.heightCm == 40 && t.photoN == 2 &&
        t.photoGapMs == 300 && t.actWaitMs == 0, "takeoff parses back");
  pl = goldenPayload("photo_hover", g, sizeof(g), &h);
  autoProtoPhoto_t ph;
  CHECK(pl && autoProtoParsePhoto(pl, h.len, &ph) && ph.n == 1 && ph.gapMs == 150, "photo_hover parses back");
  pl = goldenPayload("kill_any_session", g, sizeof(g), &h);
  CHECK(pl && h.type == AUTO_T_KILL && h.session == 0 && h.seq == 0x7FFFFFFFu && h.len == 0,
        "kill_any_session: session 0, any seq, empty");
  pl = goldenPayload("ack_arm", g, sizeof(g), &h);
  CHECK(pl && h.len == AUTO_ACK_LEN && autoGetU32(pl) == 10 && pl[4] == AUTO_T_ARM && pl[5] == AUTO_R_OK &&
        pl[6] == 1 && autoGetU32(pl + 8) == 0x5EED0001u, "ack_arm: state byte 6 and armToken detail");
  pl = goldenPayload("auto_status", g, sizeof(g), &h);
  autoStatusMsg_t back;
  memset(&back, 0, sizeof(back));
  CHECK(pl && h.len == AUTO_AUTO_STATUS_A2B_LEN && autoProtoParseAutoStatus(pl, h.len, &back) &&
        memcmp(&back, &st, sizeof(st)) == 0, "auto_status (an A2b drone's 64 bytes) parses back to the same fields, "
        "the interactive hover's 0");
}

/* ---- the interactive hover (auto_proto_golden_ih.json) ------------------ */

static void testIhGolden(void)
{
  const char *names[] = {"move", "move_left_down", "yaw", "yaw_left", "stop", "takeoff_wait", "ack_move_refused",
                         "auto_status_ih", "takeoff_wait_65536", "takeoff_wait_120000", "auto_status_ih120",
                         "auto_status_hr",
                         "takeoff_plan", "set_height", "set_height_now", "ack_set_height", "ack_set_height_refused",
                         "set_height_to", "set_height_to_120", "meta_color"};
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
    checkPacket(names[i]);
  }
  CHECK(strstr(json, "\"auto_status_fmt\": \"<IIII10B8H2h6H2BI6BH6hHhhhHH\"") != NULL, "AUTO_STATUS format string");
  uint8_t p[AUTO_PROTO_MAX_PAYLOAD];
  const autoProtoMove_t mv = {.rightCm = 50, .fwdCm = -120, .upCm = 20}, mv2 = {.rightCm = -150, .fwdCm = 0, .upCm = -50};
  autoProtoPackMove(&mv, p);
  checkBuilt("move", AUTO_T_MOVE, AUTO_FLAG_ACKREQ, A2_SESSION, 30, p, AUTO_MOVE_LEN);
  autoProtoPackMove(&mv2, p);
  checkBuilt("move_left_down", AUTO_T_MOVE, AUTO_FLAG_ACKREQ, A2_SESSION, 31, p, AUTO_MOVE_LEN);
  const autoProtoYaw_t y = {.deg = 90}, y2 = {.deg = -180};
  autoProtoPackYaw(&y, p);
  checkBuilt("yaw", AUTO_T_YAW, AUTO_FLAG_ACKREQ, A2_SESSION, 32, p, AUTO_YAW_LEN);
  autoProtoPackYaw(&y2, p);
  checkBuilt("yaw_left", AUTO_T_YAW, AUTO_FLAG_ACKREQ, A2_SESSION, 33, p, AUTO_YAW_LEN);
  checkBuilt("stop", AUTO_T_STOP, AUTO_FLAG_ACKREQ, A2_SESSION, 34, NULL, 0);
  const autoProtoTakeoff_t to = {.token = 0x5EED0002u, .heightCm = 100, .photoN = 1, .photoGapMs = 300,
                                 .actWaitMs = 60000};
  autoProtoPackTakeoff(&to, p);
  checkBuilt("takeoff_wait", AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, A2_SESSION, 35, p, AUTO_TAKEOFF_LEN);
  ackPayload(p, 30, AUTO_T_MOVE, AUTO_R_CAM_UNAVAIL, 3 /* HOVER */, 3u | 5u << 8 | 3u << 16);
  checkBuilt("ack_move_refused", AUTO_T_ACK, 0, A2_SESSION, 36, p, AUTO_ACK_LEN);
  autoStatusMsg_t st;
  memset(&st, 0, sizeof(st));
  st.upMs = 654321;
  st.bootNonce = 0x0BADF00D;
  st.transitions = 9;
  st.state = 3;
  st.reason = 3;
  st.vzPhase = 4;
  st.vzReason = 4;
  st.flags = 0x8D;
  st.trips = 0x40;
  st.capture = 2;
  st.photoId = 6;
  st.stateMs = 30000;
  st.airMs = 41200;
  st.stableMs = 20000;
  st.waitLeftMs = 52000;
  st.hoverWaitMs = 60000;
  st.vbatMv = 3460;
  st.vbatFastMinMv = 3390;
  st.hMm = 1012;
  st.vzMms = 8;
  st.thrust = 46900;
  st.tLo = 35100;
  st.tHold = 46700;
  st.kPriorDiv10 = 16500;
  st.khatDiv10 = 16710;
  st.lastCmdType = 7;
  st.mvState = 1;
  st.mvKind = 1;
  st.mvFlags = 0x01;
  st.mvSeq = 3;
  st.progRightCm = 21;
  st.progFwdCm = -48;
  st.tgtRightCm = 50;
  st.tgtFwdCm = -120;
  st.sceneDCm = 212;
  st.hTargetCm = 100;
  st.yawOffCdeg = -9000;
  st.yawTgtDeg = -90;
  st.mvPhaseMs = 2400;
  st.airLeftDs = 388;
  autoProtoPackAutoStatus(&st, p);
  /* (an interactive-hover drone's 96 bytes: the first 96 of today's 136, unchanged) */
  checkBuilt("auto_status_ih", AUTO_T_AUTO_STATUS, 0, A2_SESSION, 37, p, AUTO_AUTO_STATUS_IH_LEN);

  uint8_t g[AUTO_PROTO_MAX_PACKET];
  autoProtoHdr_t h;
  const uint8_t *pl = goldenPayload("move", g, sizeof(g), &h);
  autoProtoMove_t m;
  CHECK(pl && autoProtoParseMove(pl, h.len, &m) && m.rightCm == 50 && m.fwdCm == -120 && m.upCm == 20,
        "move parses back (signed)");
  pl = goldenPayload("yaw_left", g, sizeof(g), &h);
  autoProtoYaw_t yy;
  CHECK(pl && autoProtoParseYaw(pl, h.len, &yy) && yy.deg == -180, "yaw_left parses back (-180)");
  pl = goldenPayload("takeoff_wait", g, sizeof(g), &h);
  autoProtoTakeoff_t t;
  CHECK(pl && autoProtoParseTakeoff(pl, h.len, &t) && t.heightCm == 100 && t.actWaitMs == 60000,
        "takeoff_wait parses back (100 cm, wait 60000 ms)");
  pl = goldenPayload("auto_status_ih", g, sizeof(g), &h);
  autoStatusMsg_t back;
  memset(&back, 0xA5, sizeof(back));
  CHECK(pl && h.len == AUTO_AUTO_STATUS_IH_LEN && autoProtoParseAutoStatus(pl, h.len, &back) &&
        back.yawOffCdeg == -9000 && back.progFwdCm == -48 && back.sceneDCm == 212 && back.airLeftDs == 388 &&
        back.mvSeq == 3 && back.mvFlags == 1 && back.hTargetCm == 100,
        "auto_status_ih parses back (yaw -90.00 deg, progress -48 cm, D 2.12 m)");
  autoStatusMsg_t cmp = st;
  cmp.bootNonce = back.bootNonce;
  CHECK(memcmp(&back, &cmp, sizeof(back)) == 0, "every field round-trips");

  /* Waits over 65.5 s (2026-10-14, docs/autonomy-a2b.md 18.2): TAKEOFF byte 7, a pad until then, carries actWaitMs
   * bits 16-23 (an older Mac sends 0 there: the 60000 vector above is unchanged) */
  const autoProtoTakeoff_t t65 = {.token = 0x5EED0003u, .heightCm = 105, .photoN = 1, .photoGapMs = 300,
                                  .actWaitMs = 65536};
  autoProtoPackTakeoff(&t65, p);
  CHECK(p[7] == 0x01 && p[10] == 0 && p[11] == 0, "65536 ms: byte 7 0x01, bytes 10-11 0x0000");
  checkBuilt("takeoff_wait_65536", AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, A2_SESSION, 38, p, AUTO_TAKEOFF_LEN);
  const autoProtoTakeoff_t t120 = {.token = 0x5EED0004u, .heightCm = 105, .photoN = 1, .photoGapMs = 300,
                                   .actWaitMs = 120000};
  autoProtoPackTakeoff(&t120, p);
  CHECK(p[7] == 0x01 && autoGetU16(p + 10) == 0xD4C0, "120000 ms: byte 7 0x01, bytes 10-11 0xD4C0");
  checkBuilt("takeoff_wait_120000", AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, A2_SESSION, 39, p, AUTO_TAKEOFF_LEN);
  pl = goldenPayload("takeoff_wait_120000", g, sizeof(g), &h);
  CHECK(pl && autoProtoParseTakeoff(pl, h.len, &t) && t.actWaitMs == 120000 && t.heightCm == 105 && t.photoN == 1 &&
        t.photoGapMs == 300 && t.token == 0x5EED0004u, "takeoff_wait_120000 parses back (%u ms)",
        (unsigned)t.actWaitMs);
  pl = goldenPayload("takeoff_wait_65536", g, sizeof(g), &h);
  CHECK(pl && autoProtoParseTakeoff(pl, h.len, &t) && t.actWaitMs == 65536, "takeoff_wait_65536 parses back (%u)",
        (unsigned)t.actWaitMs);
  /* u24 round trips; bits 24-31 are not sent (the FSM's range is far below them) */
  static const uint32_t waits[] = {0u, 1000u, 60000u, 65535u, 65536u, 99999u, 120000u, 120001u, 0xFFFFFFu, 0x1FFFFFFu};
  int rt = 0;
  for (size_t i = 0; i < sizeof(waits) / sizeof(waits[0]); i++) {
    autoProtoTakeoff_t a = t120, b;
    a.actWaitMs = waits[i];
    autoProtoPackTakeoff(&a, p);
    rt += autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN, &b) && b.actWaitMs == (waits[i] & 0xFFFFFFu) &&
          b.token == a.token && b.heightCm == a.heightCm && b.photoN == a.photoN && b.photoGapMs == a.photoGapMs;
  }
  CHECK(rt == (int)(sizeof(waits) / sizeof(waits[0])), "TAKEOFF actWaitMs round-trips as u24 (%d)", rt);
  /* the long sortie's AUTO_STATUS: the u16 ms fields saturated at 65535, the air clock from airLeftDs */
  autoStatusMsg_t s120 = st;
  s120.upMs = 765432;
  s120.transitions = 11;
  s120.photoId = 7;
  s120.stateMs = s120.airMs = s120.stableMs = s120.waitLeftMs = s120.hoverWaitMs = 65535;
  s120.vbatMv = 3410;
  s120.vbatFastMinMv = 3350;
  s120.hMm = 1050;
  s120.vzMms = -3;
  s120.thrust = 47300;
  s120.tHold = 47200;
  s120.khatDiv10 = 16630;
  s120.lastCmdType = 9;
  s120.mvState = 4;
  s120.mvEnd = 2;
  s120.mvSeq = 2;
  s120.progRightCm = s120.progFwdCm = s120.tgtRightCm = s120.tgtFwdCm = 0;
  s120.sceneDCm = 230;
  s120.hTargetCm = 105;
  s120.yawOffCdeg = s120.yawTgtDeg = 0;
  s120.mvPhaseMs = 65535;
  s120.airLeftDs = 150;
  autoProtoPackAutoStatus(&s120, p);
  checkBuilt("auto_status_ih120", AUTO_T_AUTO_STATUS, 0, A2_SESSION, 40, p, AUTO_AUTO_STATUS_IH_LEN);

  /* hold and report (2026-10-15, docs/autonomy-a2b.md 22): 136 bytes, the first 96 as before */
  CHECK(strstr(json, "\"auto_status_fmt_hr\": \"<IIII10B8H2h6H2BI6BH6hHhhhHHBBHHBBHHHHhhHHHHhBBBBHBBH\"") != NULL,
        "the 136-byte AUTO_STATUS format string");
  autoStatusMsg_t hr = st;
  hr.waitLeftMs = 0;
  hr.mvState = 5;
  hr.mvEnd = 8;
  hr.airLeftDs = 0;
  hr.evCode = 1;
  hr.evDetail = 0x02;
  hr.evCount = 7;
  hr.evAgeMs = 1800;
  hr.holdBits = 0x81;
  hr.camBits = 0x02;
  hr.camOffMs = 1800;
  hr.sinkInMs = 3200;
  hr.peakAccCg = 231;
  hr.peakRateDps = 310;
  hr.contactAxCg = -205;
  hr.contactAyCg = 98;
  hr.sinceCmdDs = 124;
  hr.airDs = 412;
  hr.khatPm = 1030;
  hr.capQ = 47120;
  hr.contactYawCdeg = -9000;
  hr.contacts = 1;
  hr.ev1Code = 5;
  hr.ev1Detail = 0x84;
  hr.ev1AgeMs = 2300;
  hr.ev2Code = 10;
  hr.ev2Detail = 8;
  hr.ev2AgeMs = 2300;
  autoProtoPackAutoStatus(&hr, p);
  /* a hold-and-report drone's 136 bytes: the first 136 of today's 140 */
  checkBuilt("auto_status_hr", AUTO_T_AUTO_STATUS, 0, A2_SESSION, 42, p, AUTO_AUTO_STATUS_HR_LEN);
  pl = goldenPayload("auto_status_hr", g, sizeof(g), &h);
  autoStatusMsg_t hb;
  memset(&hb, 0xA5, sizeof(hb));
  CHECK(pl && h.len == AUTO_AUTO_STATUS_HR_LEN && autoProtoParseAutoStatus(pl, h.len, &hb) && hb.evCode == 1 &&
            hb.contactAxCg == -205 && hb.contactYawCdeg == -9000 && hb.capQ == 47120 && hb.ev2Detail == 8,
        "auto_status_hr parses back (CAM_LOST, the contact's force -2.05 g x, heading -90 deg)");
  autoStatusMsg_t hc = hr;
  hc.bootNonce = hb.bootNonce;
  CHECK(memcmp(&hb, &hc, sizeof(hb)) == 0 && hb.mvSpeedPct == 0, "every 136-byte field round-trips (no speed step: "
        "0, an older drone flies 100)");
  /* the speed step (2026-10-16, docs/autonomy-a2b.md 31): 140 bytes, mvSpeedPct 200 at 136, 0 at 138 */
  autoStatusMsg_t ms = hr;
  ms.mvSpeedPct = 200;
  memset(p, 0xEE, AUTO_AUTO_STATUS_LEN + 4);
  autoProtoPackAutoStatus(&ms, p);
  CHECK(p[136] == 200 && p[137] == 0 && p[138] == 0 && p[139] == 0 && p[140] == 0xEE, "mvSpeedPct at bytes 136-137, "
        "138-139 reserved 0");
  checkBuilt("auto_status_ms", AUTO_T_AUTO_STATUS, 0, A2_SESSION, 50, p, AUTO_AUTO_STATUS_LEN);
  pl = goldenPayload("auto_status_ms", g, sizeof(g), &h);
  memset(&hb, 0xA5, sizeof(hb));
  CHECK(pl && h.len == AUTO_AUTO_STATUS_LEN && autoProtoParseAutoStatus(pl, h.len, &hb) && hb.mvSpeedPct == 200 &&
            hb.evCode == 1 && hb.ev2AgeMs == 2300, "auto_status_ms (140 bytes) parses back: mvSpeedPct 200");
  hc = ms;
  hc.bootNonce = hb.bootNonce;
  CHECK(memcmp(&hb, &hc, sizeof(hb)) == 0, "every 140-byte field round-trips");
  /* a 96-byte status (an interactive-hover drone) parses with the new block 0 */
  memset(&hb, 0xA5, sizeof(hb));
  CHECK(autoProtoParseAutoStatus(p, AUTO_AUTO_STATUS_IH_LEN, &hb) && hb.evCode == 0 && hb.evCount == 0 &&
        hb.capQ == 0 && hb.ev2AgeMs == 0 && hb.mvEnd == 8, "a 96-byte AUTO_STATUS: the hold and report fields 0");

  /* the take-off's planned move (2026-10-14, docs/autonomy-a2b.md 21.4): 18 bytes only with a plan */
  {
    const autoProtoTakeoff_t tp = {.token = 0x5EED0005u, .heightCm = 100, .photoN = 1, .photoGapMs = 300,
                                   .actWaitMs = 90000, .planRightCm = -80, .planFwdCm = 250};
    uint8_t pp[AUTO_TAKEOFF_PLAN_LEN + 4];
    memset(pp, 0xEE, sizeof(pp));
    const uint16_t n = autoProtoPackTakeoff(&tp, pp);
    CHECK(n == AUTO_TAKEOFF_PLAN_LEN && pp[18] == 0xEE && autoGetU16(pp + 12) == (uint16_t)-80 &&
          autoGetU16(pp + 14) == 250 && pp[16] == 0 && pp[17] == 0, "takeoff with a plan: 18 bytes (%u), right -80 "
          "forward 250, reserved 0, nothing written past them", n);
    checkBuilt("takeoff_plan", AUTO_T_TAKEOFF, AUTO_FLAG_ACKREQ, A2_SESSION, 41, pp, n);
    pl = goldenPayload("takeoff_plan", g, sizeof(g), &h);
    autoProtoTakeoff_t b;
    memset(&b, 0xA5, sizeof(b));
    CHECK(pl && h.len == AUTO_TAKEOFF_PLAN_LEN && autoProtoParseTakeoff(pl, h.len, &b) && b.planRightCm == -80 &&
          b.planFwdCm == 250 && b.actWaitMs == 90000 && b.heightCm == 100 && b.photoN == 1 && b.photoGapMs == 300 &&
          b.token == 0x5EED0005u, "takeoff_plan parses back (left 80, forward 250, wait 90000 ms)");
    autoProtoTakeoff_t np = tp;
    np.planRightCm = np.planFwdCm = 0;
    memset(pp, 0xEE, sizeof(pp));
    CHECK(autoProtoPackTakeoff(&np, pp) == AUTO_TAKEOFF_LEN && pp[12] == 0xEE, "no plan: the 12 bytes of before "
          "(an older drone flies it), nothing written past them");
    CHECK(autoProtoParseTakeoff(pp, AUTO_TAKEOFF_LEN, &b) && b.planRightCm == 0 && b.planFwdCm == 0 &&
          b.actWaitMs == 90000, "12 bytes parse with plan 0, 0");
    /* lengths and the reserved field */
    autoProtoPackTakeoff(&tp, pp);
    CHECK(autoProtoParseTakeoff(pp, AUTO_TAKEOFF_PLAN_LEN, &b) && !autoProtoParseTakeoff(pp, 17, &b) &&
          !autoProtoParseTakeoff(pp, 19, &b) && !autoProtoParseTakeoff(pp, 13, &b) &&
          !autoProtoParseTakeoff(pp, 11, &b) && !autoProtoParseTakeoff(pp, 0, &b), "takeoff: 12 or 18 bytes, never "
          "11, 13, 17, 19 or 0");
    pp[16] = 1;
    CHECK(!autoProtoParseTakeoff(pp, AUTO_TAKEOFF_PLAN_LEN, &b), "takeoff: reserved byte 16 nonzero: malformed");
    pp[16] = 0;
    pp[17] = 0x80;
    CHECK(!autoProtoParseTakeoff(pp, AUTO_TAKEOFF_PLAN_LEN, &b), "takeoff: reserved byte 17 nonzero: malformed");
    /* round trips, signed */
    static const int16_t pv[][2] = {{0, 400}, {150, 370}, {-150, -150}, {1, 0}, {0, -1}, {-32768, 32767}, {7, -9}};
    int rt = 0;
    for (size_t i = 0; i < sizeof(pv) / sizeof(pv[0]); i++) {
      autoProtoTakeoff_t a = tp;
      a.planRightCm = pv[i][0];
      a.planFwdCm = pv[i][1];
      const uint16_t len = autoProtoPackTakeoff(&a, pp);
      rt += len == AUTO_TAKEOFF_PLAN_LEN && autoProtoParseTakeoff(pp, len, &b) && b.planRightCm == pv[i][0] &&
            b.planFwdCm == pv[i][1] && b.token == a.token && b.actWaitMs == a.actWaitMs;
    }
    CHECK(rt == (int)(sizeof(pv) / sizeof(pv[0])), "takeoff plans round-trip (%d)", rt);
  }

  /* SET_HEIGHT (0x33, 2026-10-15): i16 xCm, i16 atCm (INT16_MIN: now); its ACK detail state | why << 8 | old cm */
  {
    const autoProtoSetHeight_t sh = {.xCm = 110, .atCm = 95}, shNow = {.xCm = 80, .atCm = INT16_MIN};
    uint8_t sp[AUTO_ACK_LEN + 4];   /* (the ACKs below are built in it too) */
    memset(sp, 0xEE, sizeof(sp));
    autoProtoPackSetHeight(&sh, sp);
    CHECK(sp[AUTO_SET_HEIGHT_LEN] == 0xEE && AUTO_T_SET_HEIGHT == 0x33 && AUTO_SET_HEIGHT_LEN == 4,
          "set_height: 4 bytes, type 0x33, nothing written past them");
    checkBuilt("set_height", AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, A2_SESSION, 43, sp, AUTO_SET_HEIGHT_LEN);
    autoProtoPackSetHeight(&shNow, sp);
    checkBuilt("set_height_now", AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, A2_SESSION, 44, sp, AUTO_SET_HEIGHT_LEN);
    ackPayload(sp, 43, AUTO_T_SET_HEIGHT, AUTO_R_OK, 3, 3u | 0u << 8 | 100u << 16);
    checkBuilt("ack_set_height", AUTO_T_ACK, 0, A2_SESSION, 45, sp, AUTO_ACK_LEN);
    ackPayload(sp, 44, AUTO_T_SET_HEIGHT, AUTO_R_RANGE, 3, 3u | 13u << 8 | 98u << 16);
    checkBuilt("ack_set_height_refused", AUTO_T_ACK, 0, A2_SESSION, 46, sp, AUTO_ACK_LEN);
    pl = goldenPayload("set_height_now", g, sizeof(g), &h);
    autoProtoSetHeight_t b;
    memset(&b, 0xA5, sizeof(b));
    CHECK(pl && h.len == AUTO_SET_HEIGHT_LEN && autoProtoParseSetHeight(pl, h.len, &b) && b.xCm == 80 &&
          b.atCm == INT16_MIN, "set_height_now parses back (X 80, at INT16_MIN)");
    CHECK(!b.toForm && b.toCm == 0, "the 4-byte form: no target");
    CHECK(!autoProtoParseSetHeight(pl, 3, &b) && !autoProtoParseSetHeight(pl, 5, &b) &&
              !autoProtoParseSetHeight(pl, 0, &b) && !autoProtoParseSetHeight(pl, 7, &b) &&
              !autoProtoParseSetHeight(pl, 8, &b),
          "set_height: exactly 4 or 6 bytes");
    /* the "go to the target" form (2026-10-16): i16 toCm appended, INT16_MIN the commanded height */
    const autoProtoSetHeight_t to = {.xCm = 110, .atCm = 90, .toCm = INT16_MIN, .toForm = true},
                               to120 = {.xCm = 90, .atCm = INT16_MIN, .toCm = 120, .toForm = true};
    memset(sp, 0xEE, sizeof(sp));
    autoProtoPackSetHeightTo(&to, sp);
    CHECK(sp[AUTO_SET_HEIGHT_TO_LEN] == 0xEE && AUTO_SET_HEIGHT_TO_LEN == 6, "set_height_to: 6 bytes, nothing past them");
    checkBuilt("set_height_to", AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, A2_SESSION, 48, sp, AUTO_SET_HEIGHT_TO_LEN);
    autoProtoPackSetHeightTo(&to120, sp);
    checkBuilt("set_height_to_120", AUTO_T_SET_HEIGHT, AUTO_FLAG_ACKREQ, A2_SESSION, 49, sp, AUTO_SET_HEIGHT_TO_LEN);
    pl = goldenPayload("set_height_to_120", g, sizeof(g), &h);
    memset(&b, 0xA5, sizeof(b));
    CHECK(pl && h.len == AUTO_SET_HEIGHT_TO_LEN && autoProtoParseSetHeight(pl, h.len, &b) && b.toForm &&
              b.xCm == 90 && b.atCm == INT16_MIN && b.toCm == 120,
          "set_height_to_120 parses back (X 90 now, to 120)");
    pl = goldenPayload("set_height_to", g, sizeof(g), &h);
    CHECK(pl && autoProtoParseSetHeight(pl, h.len, &b) && b.toForm && b.toCm == INT16_MIN && b.atCm == 90,
          "set_height_to parses back (to the commanded height)");
    static const int16_t sv[][2] = {{0, 0}, {250, -32768}, {-1, 32767}, {10, -5}, {32767, 100}};
    int rt = 0;
    for (size_t i = 0; i < sizeof(sv) / sizeof(sv[0]); i++) {
      const autoProtoSetHeight_t a = {.xCm = sv[i][0], .atCm = sv[i][1]};
      autoProtoPackSetHeight(&a, sp);
      rt += autoProtoParseSetHeight(sp, AUTO_SET_HEIGHT_LEN, &b) && b.xCm == a.xCm && b.atCm == a.atCm;
    }
    CHECK(rt == (int)(sizeof(sv) / sizeof(sv[0])), "set_height round-trips, signed (%d)", rt);
  }

  /* A colour photo's META (2026-10-15): 44 bytes, the 40 of A1 then pixFmt YUYV and 3 zero bytes; the size and the
   * chunk count of the YUYV frame; the CRC of the golden rule's frame. A grey META stays the 40 of A1. */
  {
    static uint8_t cf[2 * 160 * 120];
    for (int y = 0; y < 120; y++) {
      for (int x = 0; x < 160; x++) {
        cf[2 * (y * 160 + x)] = (uint8_t)((x * 7 + y * 3) & 0xFF);
        cf[2 * (y * 160 + x) + 1] = (uint8_t)(x % 2 == 0 ? 64 + y : 192 - y);
      }
    }
    pl = goldenPayload("meta_color", g, sizeof(g), &h);
    CHECK(pl && h.type == AUTO_T_META && h.len == AUTO_META_COLOR_LEN && AUTO_META_COLOR_LEN == 44 &&
          AUTO_META_PIXFMT_OFF == AUTO_META_LEN, "meta_color: 44 bytes, the format after the A1 META");
    CHECK(pl && pl[AUTO_META_PIXFMT_OFF] == AUTO_PIXFMT_YUYV && pl[41] == 0 && pl[42] == 0 && pl[43] == 0 &&
          autoGetU16(pl + 4) == 160 && autoGetU16(pl + 6) == 120 &&
          autoGetU32(pl + 8) == autoPixfmtBytesPerPixel(AUTO_PIXFMT_YUYV) * 160u * 120u &&
          autoGetU16(pl + 36) == autoChunkCount(38400) && autoChunkCount(38400) == 38 &&
          autoGetU32(pl + 12) == autoProtoCrc32(cf, sizeof(cf)), "meta_color fields: YUYV, 38400 bytes, 38 chunks, "
          "the frame's CRC");
    CHECK(autoMetaLen(AUTO_PIXFMT_GRAY8) == AUTO_META_LEN && autoMetaLen(AUTO_PIXFMT_YUYV) == AUTO_META_COLOR_LEN &&
          autoPixfmtBytesPerPixel(AUTO_PIXFMT_GRAY8) == 1 && autoPixfmtBytesPerPixel(AUTO_PIXFMT_YUYV) == 2 &&
          autoPixfmtBytesPerPixel(2) == 0 && autoPixfmtBytesPerPixel(0xFF) == 0, "pixel formats: grey 1 byte (the "
          "40-byte META), YUYV 2 (44), others unknown");
    CHECK(strstr(json, "\"frame_color_crc32\": ") != NULL, "the colour frame's CRC is in the golden file");
  }

  /* the parsers' framing rules */
  uint8_t q[16] = {0};
  CHECK(autoProtoParseMove(q, AUTO_MOVE_LEN, &m) && !autoProtoParseMove(q, AUTO_MOVE_LEN - 1, &m) &&
        !autoProtoParseMove(q, AUTO_MOVE_LEN + 1, &m), "move: exact length");
  q[6] = 1;
  CHECK(!autoProtoParseMove(q, AUTO_MOVE_LEN, &m), "move: nonzero pad rejected");
  q[6] = 0;
  q[7] = 0x80;
  CHECK(!autoProtoParseMove(q, AUTO_MOVE_LEN, &m), "move: nonzero pad (byte 7) rejected");
  memset(q, 0, sizeof(q));
  autoPutU16(q, (uint16_t)(int16_t)-32768);
  CHECK(autoProtoParseYaw(q, AUTO_YAW_LEN, &yy) && yy.deg == -32768, "yaw: any value parses (the FSM ranges it)");
  q[3] = 1;
  CHECK(!autoProtoParseYaw(q, AUTO_YAW_LEN, &yy) && !autoProtoParseYaw(q, 0, &yy), "yaw: pad and length");
}

static void testA2Parsers(void)
{
  uint8_t p[16] = {0};
  autoProtoArm_t a;
  autoProtoTakeoff_t t;
  autoProtoPhoto_t ph;
  autoStatusMsg_t m;

  p[0] = 2;
  CHECK(autoProtoParseArm(p, AUTO_ARM_LEN, &a) && a.mode == 2, "arm: valid");
  CHECK(!autoProtoParseArm(p, AUTO_ARM_LEN - 1, &a) && !autoProtoParseArm(p, AUTO_ARM_LEN + 1, &a) &&
        !autoProtoParseArm(p, 0, &a), "arm: wrong length rejected");
  p[1] = 1;
  CHECK(!autoProtoParseArm(p, AUTO_ARM_LEN, &a), "arm: nonzero pad rejected");
  p[1] = 0;
  p[0] = 200;
  CHECK(autoProtoParseArm(p, AUTO_ARM_LEN, &a) && a.mode == 200, "arm: values are not range-checked (the FSM does)");

  memset(p, 0, sizeof(p));
  CHECK(autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN, &t), "takeoff: valid");
  CHECK(!autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN - 1, &t) && !autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN + 1, &t),
        "takeoff: wrong length rejected");
  /* byte 7 was a pad that had to be 0 until 2026-10-14: now actWaitMs bits 16-23 (any value parses; the FSM
   * answers RANGE above 120000) */
  p[7] = 1;
  autoPutU16(p + 10, 0xD4C0);
  CHECK(autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN, &t) && t.actWaitMs == 120000, "takeoff: byte 7 is bits 16-23 "
        "(0x01 + 0xD4C0 = 120000, %u)", (unsigned)t.actWaitMs);
  p[7] = 0x80;
  CHECK(autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN, &t) && t.actWaitMs == 0x80D4C0u, "takeoff: byte 7 0x80 parses "
        "(%u ms; the FSM refuses it)", (unsigned)t.actWaitMs);
  p[7] = 0;
  autoPutU16(p + 10, 7);
  CHECK(autoProtoParseTakeoff(p, AUTO_TAKEOFF_LEN, &t) && t.actWaitMs == 7,
        "takeoff: actWaitMs is a field, not a pad (the FSM answers RANGE)");

  memset(p, 0, sizeof(p));
  p[0] = 3;
  autoPutU16(p + 2, 1000);
  CHECK(autoProtoParsePhoto(p, AUTO_PHOTO_LEN, &ph) && ph.n == 3 && ph.gapMs == 1000, "photo: valid");
  CHECK(!autoProtoParsePhoto(p, AUTO_PHOTO_LEN - 1, &ph) && !autoProtoParsePhoto(p, AUTO_PHOTO_LEN + 1, &ph),
        "photo: wrong length rejected");
  p[1] = 1;
  CHECK(!autoProtoParsePhoto(p, AUTO_PHOTO_LEN, &ph), "photo: nonzero pad rejected");

  uint8_t s[AUTO_AUTO_STATUS_LEN + 1] = {0};
  CHECK(!autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_LEN - 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_LEN + 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_A2B_LEN - 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_A2B_LEN + 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_IH_LEN - 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_IH_LEN + 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_HR_LEN - 1, &m) &&
        !autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_HR_LEN + 1, &m) &&
        autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_LEN, &m) &&
        autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_HR_LEN, &m) &&
        autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_IH_LEN, &m) &&
        autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_A2B_LEN, &m),
        "auto_status: 140, a hold-and-report drone's 136, an interactive-hover drone's 96 or an A2b drone's 64 bytes "
        "only");

  /* Pack then parse at the extremes. */
  autoStatusMsg_t lo, hi, back;
  memset(&lo, 0, sizeof(lo));
  memset(&hi, 0, sizeof(hi));
  hi.upMs = hi.bootNonce = hi.transitions = hi.failedGuards = hi.killCount = 0xFFFFFFFFu;
  hi.state = hi.mode = hi.reason = hi.vzLandReason = hi.landTrip = hi.vzPhase = hi.vzReason = hi.flags = hi.trips =
      hi.capture = hi.lastCmdType = hi.lastCmdResult = 0xFF;
  hi.photoId = hi.stateMs = hi.airMs = hi.stableMs = hi.waitLeftMs = hi.hoverWaitMs = hi.vbatMv = hi.vbatFastMinMv =
      hi.thrust = hi.tLo = hi.tHold = hi.tRef = hi.kPriorDiv10 = hi.khatDiv10 = 0xFFFF;
  hi.hMm = INT16_MIN;
  hi.vzMms = INT16_MAX;
  hi.mvState = hi.mvKind = hi.mvWhy = hi.mvEnd = hi.mvFlags = hi.battLow = 0xFF;
  hi.mvSeq = hi.sceneDCm = hi.mvPhaseMs = hi.airLeftDs = 0xFFFF;
  hi.progRightCm = hi.tgtFwdCm = hi.yawOffCdeg = INT16_MIN;
  hi.progFwdCm = hi.progUpCm = hi.tgtRightCm = hi.tgtUpCm = hi.hTargetCm = hi.yawTgtDeg = INT16_MAX;
  hi.evCode = hi.evDetail = hi.holdBits = hi.camBits = hi.contacts = hi.loopGaps = hi.ev1Code = hi.ev1Detail =
      hi.ev2Code = hi.ev2Detail = 0xFF;
  hi.evCount = hi.evAgeMs = hi.camOffMs = hi.sinkInMs = hi.peakAccCg = hi.peakRateDps = hi.sinceCmdDs = hi.airDs =
      hi.khatPm = hi.capQ = hi.ev1AgeMs = hi.ev2AgeMs = 0xFFFF;
  hi.contactAxCg = INT16_MIN;
  hi.contactAyCg = hi.contactYawCdeg = INT16_MAX;
  hi.mvSpeedPct = 0xFFFF;
  const autoStatusMsg_t *cases[] = {&lo, &hi};
  for (int i = 0; i < 2; i++) {
    autoProtoPackAutoStatus(cases[i], s);
    memset(&back, 0, sizeof(back));
    CHECK(autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_LEN, &back) && memcmp(&back, cases[i], sizeof(back)) == 0,
          "auto_status round trip at the %s extreme", i ? "upper" : "lower");
  }
  autoProtoPackAutoStatus(&hi, s);
  CHECK(s[42] == 0x00 && s[43] == 0x80 && s[44] == 0xFF && s[45] == 0x7F, "hMm/vzMms little-endian two's complement");
  hi.hMm = -1;
  hi.vzMms = INT16_MIN;
  autoProtoPackAutoStatus(&hi, s);
  CHECK(autoProtoParseAutoStatus(s, AUTO_AUTO_STATUS_LEN, &back) && back.hMm == -1 && back.vzMms == INT16_MIN,
        "negative heights and speeds round-trip");
}

/* Integrator decision D1: the kind of a link BUSY in the detail's top byte. */
static void testBusyDetail(void)
{
  CHECK(autoDetailBusy(AUTO_DETAIL_BUSY_PENDING, 5) == 0x01000005u &&
        autoDetailBusy(AUTO_DETAIL_BUSY_PENDING, 0x12345678u) == 0x01345678u, "PENDING: 0x01 | seq & 0xFFFFFF");
  CHECK(autoDetailBusy(AUTO_DETAIL_BUSY_UNTAKEN, 3) == 0x02000003u, "UNTAKEN: 0x02 | state");
  CHECK(autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, 0x1234) == 0x03001234u &&
        autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, 0xABCDEFu) == 0x0300CDEFu, "HELD_SET: 0x03 | id & 0xFFFF");
  CHECK(autoDetailKind(0x01345678u) == AUTO_DETAIL_BUSY_PENDING && autoDetailKind(0x02000005u) == AUTO_DETAIL_BUSY_UNTAKEN &&
        autoDetailKind(0x03000001u) == AUTO_DETAIL_BUSY_HELD_SET && autoDetailKind(5) == 0 &&
        autoDetailKind(0xFFFFFFFFu) == 0xFF000000u, "kinds: 1, 2, 3; a state-machine detail 0; TAKEN_UNKNOWN 0xFF");
  /* A small seq, a state and a photo id no longer look alike on the wire. */
  CHECK(autoDetailBusy(AUTO_DETAIL_BUSY_PENDING, 3) != autoDetailBusy(AUTO_DETAIL_BUSY_UNTAKEN, 3) &&
        autoDetailBusy(AUTO_DETAIL_BUSY_UNTAKEN, 3) != autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, 3) &&
        autoDetailBusy(AUTO_DETAIL_BUSY_HELD_SET, 3) != 3u, "same small value, different kinds");
  /* On the wire: ACK detail is little-endian at payload offset 8, so the kind is payload byte 11. */
  uint8_t p[AUTO_ACK_LEN] = {0};
  autoPutU32(p, 9);
  p[4] = AUTO_T_LAND;
  p[5] = AUTO_R_BUSY;
  autoPutU32(p + 8, autoDetailBusy(AUTO_DETAIL_BUSY_PENDING, 0x00ABCDEFu));
  uint8_t buf[AUTO_PROTO_MAX_PACKET];
  const autoProtoHdr_t h = {.type = AUTO_T_ACK, .session = A2_SESSION, .seq = 30, .len = AUTO_ACK_LEN};
  const int n = autoProtoEncode(&h, p, key, hmacSha256, buf, sizeof(buf));
  autoProtoHdr_t back;
  const uint8_t *pl = NULL;
  CHECK(n > 0 && autoProtoDecode(buf, (size_t)n, key, hmacSha256, &back, &pl) == AUTO_DEC_OK && pl[11] == 0x01 &&
        pl[8] == 0xEF && pl[9] == 0xCD && pl[10] == 0xAB, "ACK BUSY PENDING: kind in payload byte 11");
}

static char *readFile(const char *path)
{
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    printf("cannot open %s\n", path);
    return NULL;
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  char *buf = calloc(1, (size_t)sz + 1);
  if (buf && fread(buf, 1, (size_t)sz, fp) != (size_t)sz) {
    free(buf);
    buf = NULL;
  }
  fclose(fp);
  return buf;
}

int main(int argc, char **argv)
{
  const char *path = argc > 1 ? argv[1] : "tools/tests/auto_proto_golden.json";
  const char *pathA2 = argc > 2 ? argv[2] : "tools/tests/auto_proto_golden_a2.json";
  const char *pathIh = argc > 3 ? argv[3] : "tools/tests/auto_proto_golden_ih.json";
  char *a1 = readFile(path);
  char *a2 = readFile(pathA2);
  char *ih = readFile(pathIh);
  if (!a1 || !a2 || !ih) {
    return 2;
  }
  for (int i = 0; i < 16; i++) {
    key[i] = (uint8_t)i;
  }
  json = a1;
  const char *names[] = {"hello", "hello_ack", "photo", "ack", "meta", "chunk", "chunks", "hb"};
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
    checkPacket(names[i]);
  }
  testFields();
  testCrc();
  testSeq();
  testEncodeLimits();
  json = a2;
  testA2Golden();
  testA2Parsers();
  testBusyDetail();
  json = ih;
  testIhGolden();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
