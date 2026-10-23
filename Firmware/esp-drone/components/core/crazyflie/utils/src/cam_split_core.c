/*
 * ESP-Drone Firmware
 *
 * cam_split_core.c - Always-on colour: the photo frame's raw YUYV out of the camera driver's copy (cam_split_core.h;
 * pure, host-tested by tools/tests/test_cam_split.c). Flash code.
 */
#include <stddef.h>
#include <string.h>

#include "cam_split_core.h"

/* The host test's publish-order check: called just before the latched buffer and time are written. */
#ifdef CAM_SPLIT_TEST_HOOK
void camSplitTestHook(const camSplit_t *s);
#define CAM_SPLIT_HOOK(s) camSplitTestHook(s)
#else
#define CAM_SPLIT_HOOK(s) ((void)0)
#endif

static uint32_t loadState(const camSplit_t *s)
{
  return __atomic_load_n(&s->state, __ATOMIC_ACQUIRE);
}

static bool casState(camSplit_t *s, uint32_t from, uint32_t to)
{
  uint32_t expect = from;
  return __atomic_compare_exchange_n(&s->state, &expect, to, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

void camSplitInit(camSplit_t *s)
{
  memset(s, 0, sizeof(*s));
}

bool camSplitGeometryOk(uint32_t fbSize, uint32_t recvSize, uint32_t halfSize, uint32_t halfCnt)
{
  return fbSize == CAM_SPLIT_FB_BYTES && recvSize == CAM_SPLIT_RECV_BYTES && halfSize == CAM_SPLIT_HALF_BYTES &&
         halfCnt == CAM_SPLIT_HALF_CNT;
}

bool camSplitFrameOf(const uint8_t *const bufs[], uint32_t n, const uint8_t *out, const uint8_t *in, uint32_t fbSize,
                     uint32_t *idx, uint32_t *off)
{
  if (out == NULL || out == in || bufs == NULL) {
    return false;
  }
  const uintptr_t o = (uintptr_t)out;
  for (uint32_t i = 0; i < n && i < CAM_SPLIT_MAX_FB; i++) {
    const uintptr_t b = (uintptr_t)bufs[i];
    if (bufs[i] != NULL && o >= b && o - b < fbSize) {
      *idx = i;
      *off = (uint32_t)(o - b);
      return true;
    }
  }
  return false;
}

bool camSplitRequest(camSplit_t *s, uint8_t *slot)
{
  if (slot == NULL || loadState(s) != CAM_SPLIT_IDLE) {
    return false;
  }
  s->slot = slot;
  __atomic_store_n(&s->state, CAM_SPLIT_REQUESTED, __ATOMIC_RELEASE);
  return true;
}

bool camSplitTake(camSplit_t *s)
{
  return casState(s, CAM_SPLIT_FILLED, CAM_SPLIT_IDLE);
}

bool camSplitCancel(camSplit_t *s)
{
  for (;;) {
    const uint32_t st = loadState(s);
    switch (st) {
      case CAM_SPLIT_IDLE:
        return true;
      case CAM_SPLIT_COPYING:
        return false;
      case CAM_SPLIT_FILLED:
        if (casState(s, st, CAM_SPLIT_IDLE)) {
          return true;
        }
        break;
      case CAM_SPLIT_REQUESTED:
      case CAM_SPLIT_CAPTURING:
      case CAM_SPLIT_CAPTURED:
        /* the generation first: a chunk that still sees CAPTURING then fails its generation check */
        __atomic_fetch_add(&s->gen, 1u, __ATOMIC_ACQ_REL);
        if (casState(s, st, CAM_SPLIT_IDLE)) {
          s->cancel++;
          return true;
        }
        break;
      default:
        return false;
    }
  }
}

bool camSplitPending(const camSplit_t *s)
{
  return loadState(s) != CAM_SPLIT_IDLE;
}

bool camSplitWants(const camSplit_t *s)
{
  const uint32_t st = __atomic_load_n(&s->state, __ATOMIC_RELAXED);
  return st == CAM_SPLIT_REQUESTED || st == CAM_SPLIT_CAPTURING;
}

/* The copy task's latch at a frame start, the state already CAPTURING (not ready). */
static void latch(camSplit_t *s, const uint8_t *fbBuf, uint64_t tsUs)
{
  s->capGen = __atomic_load_n(&s->gen, __ATOMIC_ACQUIRE);
  CAM_SPLIT_HOOK(s);
  s->capBuf = fbBuf;
  s->capTsUs = tsUs;
  s->nextOff = 0;
}

uint8_t *camSplitChunk(camSplit_t *s, uint32_t idx, const uint8_t *fbBuf, uint32_t off, uint32_t len, uint64_t tsUs)
{
  /* only the split geometry's chunks: 3840 in, 1920 out, at a whole chunk's offset inside the Y frame */
  if (fbBuf == NULL || len != CAM_SPLIT_HALF_BYTES || off % CAM_SPLIT_CHUNK_OUT != 0u ||
      off > CAM_SPLIT_FB_BYTES - CAM_SPLIT_CHUNK_OUT) {
    return NULL;
  }
  (void)idx;   /* the buffer itself identifies the frame */
  const uint32_t st = loadState(s);
  if (off == 0u) {
    if (st == CAM_SPLIT_REQUESTED) {
      if (!casState(s, CAM_SPLIT_REQUESTED, CAM_SPLIT_CAPTURING)) {
        return NULL;   /* cancelled meanwhile */
      }
      latch(s, fbBuf, tsUs);
      s->arm++;
    } else if (st == CAM_SPLIT_CAPTURING) {
      /* the frame being captured was aborted (EV-OVF, FB-SIZE): the capture moves to this one */
      latch(s, fbBuf, tsUs);
      s->restart++;
    } else {
      return NULL;
    }
  } else {
    if (st != CAM_SPLIT_CAPTURING) {
      return NULL;
    }
    if (fbBuf != s->capBuf || off != s->nextOff) {
      /* out of sequence (never seen from cam_hal.c): re-arm for the next frame start */
      if (casState(s, CAM_SPLIT_CAPTURING, CAM_SPLIT_REQUESTED)) {
        s->restart++;
      }
      return NULL;
    }
  }
  if (__atomic_load_n(&s->gen, __ATOMIC_ACQUIRE) != s->capGen) {
    return NULL;   /* cancelled since the latch */
  }
  s->nextOff = off + len / 2u;
  return s->slot + 2u * off;
}

void camSplitChunkCopied(camSplit_t *s, uint32_t off, uint32_t len)
{
  if (off + len / 2u != CAM_SPLIT_FB_BYTES || __atomic_load_n(&s->gen, __ATOMIC_ACQUIRE) != s->capGen) {
    return;
  }
  (void)casState(s, CAM_SPLIT_CAPTURING, CAM_SPLIT_CAPTURED);   /* release: the slot and the latch are complete */
}

bool camSplitCopyChunk(camSplit_t *s, uint32_t idx, const uint8_t *fbBuf, uint32_t off, const uint8_t *in,
                       uint32_t len, uint64_t tsUs)
{
  uint8_t *dst = camSplitChunk(s, idx, fbBuf, off, len, tsUs);
  if (dst == NULL) {
    return false;
  }
  memcpy(dst, in, len);
  camSplitChunkCopied(s, off, len);
  return true;
}

bool camSplitStep(camSplit_t *s, const uint8_t *const bufs[], const uint64_t tsUs[], uint32_t n, uint32_t fbSize,
                  const uint8_t *out, const uint8_t *in, uint32_t len)
{
  if (!camSplitWants(s)) {
    return false;
  }
  uint32_t idx, off;
  if (!camSplitFrameOf(bufs, n, out, in, fbSize, &idx, &off)) {
    return false;
  }
  return camSplitCopyChunk(s, idx, bufs[idx], off, in, len, tsUs[idx]);
}

camSplitFlow_t camSplitFlowFrame(camSplit_t *s, const uint8_t *buf, uint64_t tsUs)
{
  if (__atomic_load_n(&s->state, __ATOMIC_RELAXED) != CAM_SPLIT_CAPTURED) {
    return CAM_SPLIT_FLOW_NONE;   /* the photo frame's whole cost on every other frame: one load */
  }
  (void)loadState(s);   /* acquire: the latch below was published by CAPTURED */
  if (buf == s->capBuf && tsUs == s->capTsUs) {
    return casState(s, CAM_SPLIT_CAPTURED, CAM_SPLIT_COPYING) ? CAM_SPLIT_FLOW_FILL : CAM_SPLIT_FLOW_NONE;
  }
  if (tsUs > s->capTsUs) {
    /* the frames come in capture order: the captured one was dropped (GRAB_LATEST) or skipped */
    if (casState(s, CAM_SPLIT_CAPTURED, CAM_SPLIT_REQUESTED)) {
      s->rearm++;
      return CAM_SPLIT_FLOW_REARM;
    }
  }
  return CAM_SPLIT_FLOW_NONE;
}

void camSplitFilled(camSplit_t *s)
{
  s->done++;
  __atomic_store_n(&s->state, CAM_SPLIT_FILLED, __ATOMIC_RELEASE);
}

void camSplitReset(camSplit_t *s)
{
  __atomic_fetch_add(&s->gen, 1u, __ATOMIC_ACQ_REL);
  for (;;) {
    const uint32_t st = loadState(s);
    if (st != CAM_SPLIT_CAPTURING && st != CAM_SPLIT_CAPTURED) {
      break;
    }
    if (casState(s, st, CAM_SPLIT_REQUESTED)) {
      s->resets++;
      break;
    }
  }
  CAM_SPLIT_HOOK(s);
  s->capBuf = NULL;
  s->capTsUs = 0;
  s->capGen = 0;
  s->nextOff = 0;
}
