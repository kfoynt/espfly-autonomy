/*
 * ESP-Drone Firmware
 *
 * auto_proto.c - Autonomy link packets (docs/autonomy-link.md): framing, tag,
 * sequence filter, CRC-32, and the stage A2b command payloads and AUTO_STATUS.
 */
#include "auto_proto.h"

#include <string.h>

int autoProtoEncode(const autoProtoHdr_t *h, const uint8_t *payload, const uint8_t key[AUTO_PROTO_KEY_LEN],
                    autoProtoHmacFn hmac, uint8_t *out, size_t outCap)
{
  if (h->len > AUTO_PROTO_MAX_PAYLOAD || (h->len > 0 && payload == NULL)) {
    return -1;
  }
  const size_t total = AUTO_PROTO_HDR_LEN + (size_t)h->len + AUTO_PROTO_TAG_LEN;
  if (outCap < total) {
    return -1;
  }
  out[0] = AUTO_PROTO_MAGIC;
  out[1] = AUTO_PROTO_VERSION;
  out[2] = h->type;
  out[3] = h->flags;
  autoPutU32(out + 4, h->session);
  autoPutU32(out + 8, h->seq);
  autoPutU16(out + 12, h->len);
  autoPutU16(out + 14, 0);
  if (h->len > 0) {
    memmove(out + AUTO_PROTO_HDR_LEN, payload, h->len);
  }
  uint8_t mac[32];
  hmac(key, AUTO_PROTO_KEY_LEN, out, AUTO_PROTO_HDR_LEN + h->len, mac);
  memcpy(out + AUTO_PROTO_HDR_LEN + h->len, mac, AUTO_PROTO_TAG_LEN);
  return (int)total;
}

autoDecodeResult_t autoProtoDecode(const uint8_t *buf, size_t len, const uint8_t key[AUTO_PROTO_KEY_LEN],
                                   autoProtoHmacFn hmac, autoProtoHdr_t *h, const uint8_t **payload)
{
  if (len < AUTO_PROTO_HDR_LEN + AUTO_PROTO_TAG_LEN) {
    return AUTO_DEC_SHORT;
  }
  if (buf[0] != AUTO_PROTO_MAGIC) {
    return AUTO_DEC_MAGIC;
  }
  if (buf[1] != AUTO_PROTO_VERSION) {
    return AUTO_DEC_VERSION;
  }
  const uint16_t plen = autoGetU16(buf + 12);
  if (plen > AUTO_PROTO_MAX_PAYLOAD || len != AUTO_PROTO_HDR_LEN + (size_t)plen + AUTO_PROTO_TAG_LEN) {
    return AUTO_DEC_LENGTH;
  }
  uint8_t mac[32];
  hmac(key, AUTO_PROTO_KEY_LEN, buf, AUTO_PROTO_HDR_LEN + plen, mac);
  uint8_t diff = 0;
  for (int i = 0; i < AUTO_PROTO_TAG_LEN; i++) {
    diff |= (uint8_t)(mac[i] ^ buf[AUTO_PROTO_HDR_LEN + plen + i]);
  }
  if (diff != 0) {
    return AUTO_DEC_TAG;
  }
  h->type = buf[2];
  h->flags = buf[3];
  h->session = autoGetU32(buf + 4);
  h->seq = autoGetU32(buf + 8);
  h->len = plen;
  *payload = buf + AUTO_PROTO_HDR_LEN;
  return AUTO_DEC_OK;
}

void autoProtoSeqReset(autoProtoSeqFilter_t *f)
{
  memset(f, 0, sizeof(*f));
}

autoSeqResult_t autoProtoSeqCheck(autoProtoSeqFilter_t *f, uint32_t seq)
{
  for (int i = 0; i < f->recentCount; i++) {
    if (f->recent[i] == seq) {
      return AUTO_SEQ_DUP;
    }
  }
  if (f->any && seq <= f->lastSeq) {
    return AUTO_SEQ_OLD;
  }
  f->any = true;
  f->lastSeq = seq;
  f->recent[f->recentHead] = seq;
  f->recentHead = (uint8_t)((f->recentHead + 1) % AUTO_PROTO_SEQ_RECENT);
  if (f->recentCount < AUTO_PROTO_SEQ_RECENT) {
    f->recentCount++;
  }
  return AUTO_SEQ_NEW;
}

uint32_t autoProtoCrc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

/* ---- A2b command payloads ---------------------------------------------- */

bool autoProtoParseArm(const uint8_t *p, uint16_t len, autoProtoArm_t *out)
{
  if (len != AUTO_ARM_LEN || p[1] != 0) {
    return false;
  }
  out->mode = p[0];
  out->airMaxMs = autoGetU16(p + 2);
  return true;
}

/* Byte 7 is actWaitMs bits 16-23 (auto_proto.h): no pad left to check in
 * the first 12; the state machine range-checks the wait and the plan
 * (RANGE). 18 bytes: the planned move, bytes 16-17 reserved (0). */
bool autoProtoParseTakeoff(const uint8_t *p, uint16_t len, autoProtoTakeoff_t *out)
{
  if (len != AUTO_TAKEOFF_LEN && !(len == AUTO_TAKEOFF_PLAN_LEN && p[16] == 0 && p[17] == 0)) {
    return false;
  }
  out->token = autoGetU32(p);
  out->heightCm = autoGetU16(p + 4);
  out->photoN = p[6];
  out->photoGapMs = autoGetU16(p + 8);
  out->actWaitMs = (uint32_t)autoGetU16(p + 10) | ((uint32_t)p[7] << 16);
  out->planRightCm = len == AUTO_TAKEOFF_PLAN_LEN ? (int16_t)autoGetU16(p + 12) : 0;
  out->planFwdCm = len == AUTO_TAKEOFF_PLAN_LEN ? (int16_t)autoGetU16(p + 14) : 0;
  return true;
}

bool autoProtoParsePhoto(const uint8_t *p, uint16_t len, autoProtoPhoto_t *out)
{
  if (len != AUTO_PHOTO_LEN || p[1] != 0) {
    return false;
  }
  out->n = p[0];
  out->gapMs = autoGetU16(p + 2);
  return true;
}

bool autoProtoParseMove(const uint8_t *p, uint16_t len, autoProtoMove_t *out)
{
  if (len != AUTO_MOVE_LEN || p[6] != 0 || p[7] != 0) {
    return false;
  }
  out->rightCm = (int16_t)autoGetU16(p);
  out->fwdCm = (int16_t)autoGetU16(p + 2);
  out->upCm = (int16_t)autoGetU16(p + 4);
  return true;
}

bool autoProtoParseYaw(const uint8_t *p, uint16_t len, autoProtoYaw_t *out)
{
  if (len != AUTO_YAW_LEN || p[2] != 0 || p[3] != 0) {
    return false;
  }
  out->deg = (int16_t)autoGetU16(p);
  return true;
}

/* SET_HEIGHT: every field signed, no pad (the link refuses xCm < 0; the state machine range-checks); 4 bytes, or 6
 * with toCm (2026-10-16). */
bool autoProtoParseSetHeight(const uint8_t *p, uint16_t len, autoProtoSetHeight_t *out)
{
  if (len != AUTO_SET_HEIGHT_LEN && len != AUTO_SET_HEIGHT_TO_LEN) {
    return false;
  }
  out->xCm = (int16_t)autoGetU16(p);
  out->atCm = (int16_t)autoGetU16(p + 2);
  out->toForm = len == AUTO_SET_HEIGHT_TO_LEN;
  out->toCm = out->toForm ? (int16_t)autoGetU16(p + 4) : 0;
  return true;
}

void autoProtoPackSetHeight(const autoProtoSetHeight_t *sh, uint8_t out[AUTO_SET_HEIGHT_LEN])
{
  autoPutU16(out, (uint16_t)sh->xCm);
  autoPutU16(out + 2, (uint16_t)sh->atCm);
}

void autoProtoPackSetHeightTo(const autoProtoSetHeight_t *sh, uint8_t out[AUTO_SET_HEIGHT_TO_LEN])
{
  autoProtoPackSetHeight(sh, out);
  autoPutU16(out + 4, (uint16_t)sh->toCm);
}

void autoProtoPackMove(const autoProtoMove_t *m, uint8_t out[AUTO_MOVE_LEN])
{
  autoPutU16(out, (uint16_t)m->rightCm);
  autoPutU16(out + 2, (uint16_t)m->fwdCm);
  autoPutU16(out + 4, (uint16_t)m->upCm);
  out[6] = 0;
  out[7] = 0;
}

void autoProtoPackYaw(const autoProtoYaw_t *y, uint8_t out[AUTO_YAW_LEN])
{
  autoPutU16(out, (uint16_t)y->deg);
  out[2] = 0;
  out[3] = 0;
}

void autoProtoPackArm(const autoProtoArm_t *a, uint8_t out[AUTO_ARM_LEN])
{
  out[0] = a->mode;
  out[1] = 0;
  autoPutU16(out + 2, a->airMaxMs);
}

uint16_t autoProtoPackTakeoff(const autoProtoTakeoff_t *t, uint8_t *out)
{
  autoPutU32(out, t->token);
  autoPutU16(out + 4, t->heightCm);
  out[6] = t->photoN;
  out[7] = (uint8_t)((t->actWaitMs >> 16) & 0xFFu);   /* bits 24-31 are not sent: the FSM's range is far below */
  autoPutU16(out + 8, t->photoGapMs);
  autoPutU16(out + 10, (uint16_t)(t->actWaitMs & 0xFFFFu));
  if (t->planRightCm == 0 && t->planFwdCm == 0) {
    return AUTO_TAKEOFF_LEN;
  }
  autoPutU16(out + 12, (uint16_t)t->planRightCm);
  autoPutU16(out + 14, (uint16_t)t->planFwdCm);
  autoPutU16(out + 16, 0);
  return AUTO_TAKEOFF_PLAN_LEN;
}

/* ---- AUTO_STATUS (0x84): the layout table in auto_proto.h --------------- */

void autoProtoPackAutoStatus(const autoStatusMsg_t *m, uint8_t out[AUTO_AUTO_STATUS_LEN])
{
  autoPutU32(out + 0, m->upMs);
  autoPutU32(out + 4, m->bootNonce);
  autoPutU32(out + 8, m->transitions);
  autoPutU32(out + 12, m->failedGuards);
  out[16] = m->state;
  out[17] = m->mode;
  out[18] = m->reason;
  out[19] = m->vzLandReason;
  out[20] = m->landTrip;
  out[21] = m->vzPhase;
  out[22] = m->vzReason;
  out[23] = m->flags;
  out[24] = m->trips;
  out[25] = m->capture;
  autoPutU16(out + 26, m->photoId);
  autoPutU16(out + 28, m->stateMs);
  autoPutU16(out + 30, m->airMs);
  autoPutU16(out + 32, m->stableMs);
  autoPutU16(out + 34, m->waitLeftMs);
  autoPutU16(out + 36, m->hoverWaitMs);
  autoPutU16(out + 38, m->vbatMv);
  autoPutU16(out + 40, m->vbatFastMinMv);
  autoPutU16(out + 42, (uint16_t)m->hMm);
  autoPutU16(out + 44, (uint16_t)m->vzMms);
  autoPutU16(out + 46, m->thrust);
  autoPutU16(out + 48, m->tLo);
  autoPutU16(out + 50, m->tHold);
  autoPutU16(out + 52, m->tRef);
  autoPutU16(out + 54, m->kPriorDiv10);
  autoPutU16(out + 56, m->khatDiv10);
  out[58] = m->lastCmdType;
  out[59] = m->lastCmdResult;
  autoPutU32(out + 60, m->killCount);
  out[64] = m->mvState;
  out[65] = m->mvKind;
  out[66] = m->mvWhy;
  out[67] = m->mvEnd;
  out[68] = m->mvFlags;
  out[69] = m->battLow;
  autoPutU16(out + 70, m->mvSeq);
  autoPutU16(out + 72, (uint16_t)m->progRightCm);
  autoPutU16(out + 74, (uint16_t)m->progFwdCm);
  autoPutU16(out + 76, (uint16_t)m->progUpCm);
  autoPutU16(out + 78, (uint16_t)m->tgtRightCm);
  autoPutU16(out + 80, (uint16_t)m->tgtFwdCm);
  autoPutU16(out + 82, (uint16_t)m->tgtUpCm);
  autoPutU16(out + 84, m->sceneDCm);
  autoPutU16(out + 86, (uint16_t)m->hTargetCm);
  autoPutU16(out + 88, (uint16_t)m->yawOffCdeg);
  autoPutU16(out + 90, (uint16_t)m->yawTgtDeg);
  autoPutU16(out + 92, m->mvPhaseMs);
  autoPutU16(out + 94, m->airLeftDs);
  out[96] = m->evCode;
  out[97] = m->evDetail;
  autoPutU16(out + 98, m->evCount);
  autoPutU16(out + 100, m->evAgeMs);
  out[102] = m->holdBits;
  out[103] = m->camBits;
  autoPutU16(out + 104, m->camOffMs);
  autoPutU16(out + 106, m->sinkInMs);
  autoPutU16(out + 108, m->peakAccCg);
  autoPutU16(out + 110, m->peakRateDps);
  autoPutU16(out + 112, (uint16_t)m->contactAxCg);
  autoPutU16(out + 114, (uint16_t)m->contactAyCg);
  autoPutU16(out + 116, m->sinceCmdDs);
  autoPutU16(out + 118, m->airDs);
  autoPutU16(out + 120, m->khatPm);
  autoPutU16(out + 122, m->capQ);
  autoPutU16(out + 124, (uint16_t)m->contactYawCdeg);
  out[126] = m->contacts;
  out[127] = m->loopGaps;
  out[128] = m->ev1Code;
  out[129] = m->ev1Detail;
  autoPutU16(out + 130, m->ev1AgeMs);
  out[132] = m->ev2Code;
  out[133] = m->ev2Detail;
  autoPutU16(out + 134, m->ev2AgeMs);
  autoPutU16(out + 136, m->mvSpeedPct);
  autoPutU16(out + 138, 0);
}

bool autoProtoParseAutoStatus(const uint8_t *p, uint16_t len, autoStatusMsg_t *out)
{
  if (len != AUTO_AUTO_STATUS_LEN && len != AUTO_AUTO_STATUS_HR_LEN && len != AUTO_AUTO_STATUS_IH_LEN &&
      len != AUTO_AUTO_STATUS_A2B_LEN) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->upMs = autoGetU32(p + 0);
  out->bootNonce = autoGetU32(p + 4);
  out->transitions = autoGetU32(p + 8);
  out->failedGuards = autoGetU32(p + 12);
  out->state = p[16];
  out->mode = p[17];
  out->reason = p[18];
  out->vzLandReason = p[19];
  out->landTrip = p[20];
  out->vzPhase = p[21];
  out->vzReason = p[22];
  out->flags = p[23];
  out->trips = p[24];
  out->capture = p[25];
  out->photoId = autoGetU16(p + 26);
  out->stateMs = autoGetU16(p + 28);
  out->airMs = autoGetU16(p + 30);
  out->stableMs = autoGetU16(p + 32);
  out->waitLeftMs = autoGetU16(p + 34);
  out->hoverWaitMs = autoGetU16(p + 36);
  out->vbatMv = autoGetU16(p + 38);
  out->vbatFastMinMv = autoGetU16(p + 40);
  out->hMm = (int16_t)autoGetU16(p + 42);
  out->vzMms = (int16_t)autoGetU16(p + 44);
  out->thrust = autoGetU16(p + 46);
  out->tLo = autoGetU16(p + 48);
  out->tHold = autoGetU16(p + 50);
  out->tRef = autoGetU16(p + 52);
  out->kPriorDiv10 = autoGetU16(p + 54);
  out->khatDiv10 = autoGetU16(p + 56);
  out->lastCmdType = p[58];
  out->lastCmdResult = p[59];
  out->killCount = autoGetU32(p + 60);
  if (len < AUTO_AUTO_STATUS_IH_LEN) {
    return true;   /* an A2b drone: no interactive hover fields */
  }
  out->mvState = p[64];
  out->mvKind = p[65];
  out->mvWhy = p[66];
  out->mvEnd = p[67];
  out->mvFlags = p[68];
  out->battLow = p[69];
  out->mvSeq = autoGetU16(p + 70);
  out->progRightCm = (int16_t)autoGetU16(p + 72);
  out->progFwdCm = (int16_t)autoGetU16(p + 74);
  out->progUpCm = (int16_t)autoGetU16(p + 76);
  out->tgtRightCm = (int16_t)autoGetU16(p + 78);
  out->tgtFwdCm = (int16_t)autoGetU16(p + 80);
  out->tgtUpCm = (int16_t)autoGetU16(p + 82);
  out->sceneDCm = autoGetU16(p + 84);
  out->hTargetCm = (int16_t)autoGetU16(p + 86);
  out->yawOffCdeg = (int16_t)autoGetU16(p + 88);
  out->yawTgtDeg = (int16_t)autoGetU16(p + 90);
  out->mvPhaseMs = autoGetU16(p + 92);
  out->airLeftDs = autoGetU16(p + 94);
  if (len < AUTO_AUTO_STATUS_HR_LEN) {
    return true;   /* an interactive-hover drone: no hold and report fields */
  }
  out->evCode = p[96];
  out->evDetail = p[97];
  out->evCount = autoGetU16(p + 98);
  out->evAgeMs = autoGetU16(p + 100);
  out->holdBits = p[102];
  out->camBits = p[103];
  out->camOffMs = autoGetU16(p + 104);
  out->sinkInMs = autoGetU16(p + 106);
  out->peakAccCg = autoGetU16(p + 108);
  out->peakRateDps = autoGetU16(p + 110);
  out->contactAxCg = (int16_t)autoGetU16(p + 112);
  out->contactAyCg = (int16_t)autoGetU16(p + 114);
  out->sinceCmdDs = autoGetU16(p + 116);
  out->airDs = autoGetU16(p + 118);
  out->khatPm = autoGetU16(p + 120);
  out->capQ = autoGetU16(p + 122);
  out->contactYawCdeg = (int16_t)autoGetU16(p + 124);
  out->contacts = p[126];
  out->loopGaps = p[127];
  out->ev1Code = p[128];
  out->ev1Detail = p[129];
  out->ev1AgeMs = autoGetU16(p + 130);
  out->ev2Code = p[132];
  out->ev2Detail = p[133];
  out->ev2AgeMs = autoGetU16(p + 134);
  if (len < AUTO_AUTO_STATUS_LEN) {
    return true;   /* a hold-and-report drone: no speed step (it flies 100 %) */
  }
  out->mvSpeedPct = autoGetU16(p + 136);
  return true;
}
