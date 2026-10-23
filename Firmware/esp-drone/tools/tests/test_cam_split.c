/*
 * Host test of always-on colour's pure core (components/core/crazyflie/utils/src/cam_split_core.c,
 * docs/autonomy-a2b.md section 28): a model of esp32-camera 2.1.0's cam_hal.c copy sequence in the split geometry
 * (2 PSRAM frame buffers of 19200 B, the 15360 B DMA ring of 4 half buffers, 10 chunks of 3840 B in and 1920 B out a
 * frame, the driver's own YUV-to-grey ll_cam_memcpy, ll_cam.c:536-550, copied below), the ll_cam_memcpy wrapper's
 * step (camSplitStep, as cam_split.c calls it), the flow task's side (camSplitFlowFrame / camSplitFilled, as
 * camera_flow.c snapshotFill does) and the requester's (request / take / cancel, as auto_link.c).
 *
 * Cases (the plan's C1 list): 1 the Y bytes equal ofLumaFromYuyv's (optical_flow.c, the host reference) on unarmed
 * and armed frames; 2 the armed slot is the raw YUYV frame, byte for byte; 3 a request in the middle of a frame
 * latches the next frame; 4 a cancel in the middle of a frame changes no slot byte afterwards (poisoned slot), IDLE;
 * 5 an aborted frame restarts the capture; 6 a GRAB_LATEST drop (the flow sees a newer frame while CAPTURED)
 * re-arms; 7 a camera re-init resets (CAPTURING / CAPTURED -> REQUESTED, the latch cleared); 8 pass-through for an
 * unknown pointer and for out == in; 9 the geometry check; 10 the latched buffer and time never change while
 * CAPTURED (hook); plus the handshake's refusals, a chunk out of sequence, and a randomised soak (requests, cancels,
 * aborts, drops and re-inits over 20000 frames: every filled photo is its frame's YUYV, every Y frame the luma).
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=gnu99 -O2 -Wall -Wextra -DCAM_SPLIT_TEST_HOOK -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_cam_split.c components/core/crazyflie/utils/src/cam_split_core.c \
 *      components/core/crazyflie/utils/src/optical_flow.c -lm -o /tmp/test_cam_split && /tmp/test_cam_split
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cam_split_core.h"
#include "optical_flow.h"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

_Static_assert(OF_WIDTH * OF_HEIGHT == CAM_SPLIT_FB_BYTES, "the flow frame is the split's Y frame");

/* ---- the publish-order hook (cam_split_core.c, CAM_SPLIT_TEST_HOOK) ---- */
static int hookCalls, hookBad;
void camSplitTestHook(const camSplit_t *s)
{
  hookCalls++;
  if (__atomic_load_n(&s->state, __ATOMIC_ACQUIRE) == CAM_SPLIT_CAPTURED) {
    hookBad++;   /* the latch would change under a published CAPTURED */
  }
}

/* ---- the driver model ---- */
static camSplit_t S;
static uint8_t fb[2][CAM_SPLIT_FB_BYTES];
static uint64_t fbTs[2];
static uint8_t ring[CAM_SPLIT_HALF_BYTES * CAM_SPLIT_HALF_CNT];
static uint8_t slotA[CAM_SPLIT_RECV_BYTES], slotB[CAM_SPLIT_RECV_BYTES];
static uint64_t nowUs = 1000000;

/* esp32-camera 2.1.0 target/esp32s3/ll_cam.c ll_cam_memcpy, its YUV-to-grayscale branch (in 2, fb 1) */
static size_t realMemcpy(uint8_t *out, const uint8_t *in, size_t len)
{
  size_t end = len / 8;
  for (size_t i = 0; i < end; ++i) {
    out[0] = in[0];
    out[1] = in[2];
    out[2] = in[4];
    out[3] = in[6];
    out += 4;
    in += 8;
  }
  return len / 2;
}

/* the wrapper (cam_split.c __wrap_ll_cam_memcpy): the driver's copy, then the split step */
static int stepCopies;
static size_t wrapMemcpy(uint8_t *out, const uint8_t *in, size_t len)
{
  const size_t n = realMemcpy(out, in, len);
  const uint8_t *bufs[2] = {fb[0], fb[1]};
  stepCopies += camSplitStep(&S, bufs, fbTs, 2, CAM_SPLIT_FB_BYTES, out, in, (uint32_t)len);
  return n;
}

/* A sensor frame: Y a pattern of the frame number, U and V others (never equal to Y's). */
static void makeFrame(uint8_t *yuyv, uint32_t num)
{
  for (uint32_t p = 0; p < CAM_SPLIT_FB_BYTES; p++) {
    const uint32_t x = p % CAM_SPLIT_W, y = p / CAM_SPLIT_W;
    yuyv[2 * p] = (uint8_t)(x * 3 + y * 5 + num * 7);
    yuyv[2 * p + 1] = (uint8_t)((p & 1) ? 200 - y + num : 40 + x + num * 3);   /* U on even pixels, V on odd */
  }
}

typedef void (*chunkCb_t)(int k);
static uint32_t ringPos;   /* cam_hal's cnt % dma_half_buffer_cnt runs on across frames: the ring position */

/* One frame into buffer idx (cam_hal.c cam_task: the timestamp at the frame start, len 0, one copy per EOF event);
 * stops after `chunks` chunks (an aborted frame); cb after each chunk. Returns the Y length written. */
static size_t runFrame(int idx, const uint8_t *yuyv, int chunks, chunkCb_t cb)
{
  nowUs += 67500;
  fbTs[idx] = nowUs;
  size_t len = 0;
  for (int k = 0; k < chunks; k++) {
    uint8_t *half = ring + (ringPos++ % CAM_SPLIT_HALF_CNT) * CAM_SPLIT_HALF_BYTES;
    memcpy(half, yuyv + k * CAM_SPLIT_HALF_BYTES, CAM_SPLIT_HALF_BYTES);   /* the GDMA */
    len += wrapMemcpy(&fb[idx][len], half, CAM_SPLIT_HALF_BYTES);
    memset(half, 0xEE, CAM_SPLIT_HALF_BYTES);   /* the DMA reuses the half buffer */
    if (cb) {
      cb(k);
    }
  }
  return len;
}

/* the flow's side for a processed frame (camera_flow.c snapshotFill, split branch) */
static uint64_t metaTs;
static const uint8_t *metaBuf;
static camSplitFlow_t flowFrame(int idx)
{
  const camSplitFlow_t r = camSplitFlowFrame(&S, fb[idx], fbTs[idx]);
  if (r == CAM_SPLIT_FLOW_FILL) {
    metaTs = fbTs[idx];
    metaBuf = fb[idx];
    camSplitFilled(&S);
  }
  return r;
}

static void reset(void)
{
  camSplitInit(&S);
  memset(fb, 0, sizeof(fb));
  memset(slotA, 0xA5, sizeof(slotA));
  memset(slotB, 0x5A, sizeof(slotB));
  metaTs = 0;
  metaBuf = NULL;
}

static uint32_t st(void)
{
  return __atomic_load_n(&S.state, __ATOMIC_ACQUIRE);
}

static int lumaEq(int idx, const uint8_t *yuyv)
{
  static uint8_t ref[CAM_SPLIT_FB_BYTES];
  ofLumaFromYuyv(yuyv, ref);
  return memcmp(ref, fb[idx], CAM_SPLIT_FB_BYTES) == 0;
}

static uint8_t frA[CAM_SPLIT_RECV_BYTES], frB[CAM_SPLIT_RECV_BYTES], frC[CAM_SPLIT_RECV_BYTES];

/* 1, 2: Y on unarmed and armed frames; the armed slot */
static void testIdentity(void)
{
  reset();
  makeFrame(frA, 1);
  makeFrame(frB, 2);
  CHECK(runFrame(0, frA, CAM_SPLIT_CHUNKS, NULL) == CAM_SPLIT_FB_BYTES && lumaEq(0, frA), "unarmed frame: 19200 Y bytes, the luma");
  CHECK(st() == CAM_SPLIT_IDLE && stepCopies == 0, "unarmed: nothing copied");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE, "the flow: nothing to do");
  CHECK(camSplitRequest(&S, slotA) && st() == CAM_SPLIT_REQUESTED, "request");
  runFrame(1, frB, CAM_SPLIT_CHUNKS, NULL);
  CHECK(lumaEq(1, frB), "armed frame: the Y bytes are the luma too");
  CHECK(st() == CAM_SPLIT_CAPTURED && stepCopies == CAM_SPLIT_CHUNKS && S.arm == 1, "armed: 10 chunks copied, CAPTURED");
  CHECK(memcmp(slotA, frB, CAM_SPLIT_RECV_BYTES) == 0, "the slot is the raw YUYV frame, byte for byte (38400 B)");
  CHECK(!camSplitTake(&S), "no take before the flow filled the meta");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE && st() == CAM_SPLIT_CAPTURED, "the flow's older frame: wait");
  CHECK(flowFrame(1) == CAM_SPLIT_FLOW_FILL && st() == CAM_SPLIT_FILLED && metaTs == fbTs[1] && metaBuf == fb[1] &&
        S.done == 1, "the flow's photo frame: meta, FILLED");
  CHECK(camSplitTake(&S) && st() == CAM_SPLIT_IDLE && !camSplitPending(&S), "taken: IDLE");
  /* the YUYV Y equals the Y8 identity at the extremes (0 and 255 pass through the driver's copy) */
  for (uint32_t p = 0; p < CAM_SPLIT_FB_BYTES; p++) {
    frC[2 * p] = (uint8_t)(p % 3 == 0 ? 0 : (p % 3 == 1 ? 255 : 16));
    frC[2 * p + 1] = 128;
  }
  runFrame(0, frC, CAM_SPLIT_CHUNKS, NULL);
  CHECK(lumaEq(0, frC) && fb[0][0] == 0 && fb[0][1] == 255 && fb[0][2] == 16, "Y 0 / 255 / 16 kept as sent");
}

/* 3: a request in the middle of a frame latches the next frame */
static void midRequest(int k)
{
  if (k == 4) {
    CHECK(camSplitRequest(&S, slotA), "request after chunk 4");
  }
}
static void testMidRequest(void)
{
  reset();
  makeFrame(frA, 11);
  makeFrame(frB, 12);
  runFrame(0, frA, CAM_SPLIT_CHUNKS, midRequest);
  CHECK(st() == CAM_SPLIT_REQUESTED && S.arm == 0, "the frame already started is not captured");
  CHECK(slotA[0] == 0xA5 && slotA[CAM_SPLIT_RECV_BYTES - 1] == 0xA5, "the slot untouched");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE, "the flow's frame is no photo");
  runFrame(1, frB, CAM_SPLIT_CHUNKS, NULL);
  CHECK(st() == CAM_SPLIT_CAPTURED && memcmp(slotA, frB, CAM_SPLIT_RECV_BYTES) == 0, "the next frame is the photo");
  CHECK(flowFrame(1) == CAM_SPLIT_FLOW_FILL && metaTs == fbTs[1], "filled from it");
}

/* 4: a cancel in the middle of a frame: no slot byte changes afterwards; IDLE */
static uint8_t slotAtCancel[CAM_SPLIT_RECV_BYTES];
static void midCancel(int k)
{
  if (k == 3) {
    CHECK(st() == CAM_SPLIT_CAPTURING, "capturing at chunk 3");
    CHECK(camSplitCancel(&S) && st() == CAM_SPLIT_IDLE, "cancel while capturing: IDLE");
    memcpy(slotAtCancel, slotA, sizeof(slotA));
  }
}
static void testMidCancel(void)
{
  reset();
  makeFrame(frA, 21);
  makeFrame(frB, 22);
  CHECK(camSplitRequest(&S, slotA), "request");
  const uint32_t g0 = S.gen;
  runFrame(0, frA, CAM_SPLIT_CHUNKS, midCancel);
  CHECK(memcmp(slotAtCancel, slotA, sizeof(slotA)) == 0, "no slot byte changed after the cancel");
  CHECK(memcmp(slotA, frA, 4 * CAM_SPLIT_HALF_BYTES) == 0 && slotA[4 * CAM_SPLIT_HALF_BYTES] == 0xA5 &&
        slotA[CAM_SPLIT_RECV_BYTES - 1] == 0xA5, "only chunks 0-3 (before the cancel) were written");
  CHECK(st() == CAM_SPLIT_IDLE && S.cancel == 1 && S.gen == g0 + 1, "IDLE, one cancel, the generation +1");
  CHECK(lumaEq(0, frA), "the flow's Y frame complete regardless");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE, "the flow: nothing");
  /* a new request into another slot: a clean full capture, the old slot never touched again */
  memcpy(slotAtCancel, slotA, sizeof(slotA));
  CHECK(camSplitRequest(&S, slotB), "a new request (slot B)");
  runFrame(1, frB, CAM_SPLIT_CHUNKS, NULL);
  CHECK(memcmp(slotB, frB, CAM_SPLIT_RECV_BYTES) == 0 && memcmp(slotAtCancel, slotA, sizeof(slotA)) == 0,
        "slot B is the next frame; slot A unchanged");
  /* the generation check alone (the second line of defence): a cancel that a chunk could not see in the state */
  reset();
  CHECK(camSplitRequest(&S, slotA), "request");
  uint8_t *d0 = camSplitChunk(&S, 0, fb[0], 0, CAM_SPLIT_HALF_BYTES, 5);
  CHECK(d0 == slotA && st() == CAM_SPLIT_CAPTURING, "chunk 0 latched");
  __atomic_fetch_add(&S.gen, 1u, __ATOMIC_ACQ_REL);   /* the cancel's first step, its CAS not yet seen */
  CHECK(camSplitChunk(&S, 0, fb[0], CAM_SPLIT_CHUNK_OUT, CAM_SPLIT_HALF_BYTES, 5) == NULL,
        "a chunk after the generation moved: not copied");
  camSplitChunkCopied(&S, CAM_SPLIT_FB_BYTES - CAM_SPLIT_CHUNK_OUT, CAM_SPLIT_HALF_BYTES);
  CHECK(st() == CAM_SPLIT_CAPTURING, "and never published CAPTURED");
  CHECK(camSplitCancel(&S) && st() == CAM_SPLIT_IDLE, "the cancel completes");
}

/* 5: an aborted frame (EV-OVF / FB-SIZE: the driver restarts at offset 0) restarts the capture */
static void testAbort(void)
{
  reset();
  makeFrame(frA, 31);
  makeFrame(frB, 32);
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(0, frA, 6, NULL);
  CHECK(st() == CAM_SPLIT_CAPTURING && S.capTsUs == fbTs[0], "6 chunks: capturing frame A");
  const uint64_t tsA = fbTs[0];
  runFrame(0, frB, CAM_SPLIT_CHUNKS, NULL);   /* the same buffer again (cam_hal picks the still-free frame), a new time */
  CHECK(st() == CAM_SPLIT_CAPTURED && S.restart == 1 && S.arm == 1 && S.capTsUs == fbTs[0] && fbTs[0] != tsA,
        "restarted on the new frame");
  CHECK(memcmp(slotA, frB, CAM_SPLIT_RECV_BYTES) == 0, "the slot holds the complete new frame");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_FILL && metaTs == fbTs[0], "filled with the new frame's time");
  /* a chunk out of sequence (never from cam_hal): re-armed, nothing published */
  reset();
  CHECK(camSplitRequest(&S, slotA), "request");
  CHECK(camSplitChunk(&S, 0, fb[0], 0, CAM_SPLIT_HALF_BYTES, 7) != NULL, "chunk 0");
  CHECK(camSplitChunk(&S, 0, fb[0], 2 * CAM_SPLIT_CHUNK_OUT, CAM_SPLIT_HALF_BYTES, 7) == NULL &&
        st() == CAM_SPLIT_REQUESTED && S.restart == 1, "chunk 2 after chunk 0: re-armed");
  CHECK(camSplitChunk(&S, 1, fb[1], CAM_SPLIT_CHUNK_OUT, CAM_SPLIT_HALF_BYTES, 7) == NULL &&
        st() == CAM_SPLIT_REQUESTED, "a mid-frame chunk does not latch");
  CHECK(camSplitChunk(&S, 0, fb[0], 0, 7680, 7) == NULL && camSplitChunk(&S, 0, fb[0], 100, CAM_SPLIT_HALF_BYTES, 7)
        == NULL && camSplitChunk(&S, 0, fb[0], CAM_SPLIT_FB_BYTES, CAM_SPLIT_HALF_BYTES, 7) == NULL &&
        st() == CAM_SPLIT_REQUESTED, "other geometries (7680 B halves, odd offsets, past the end): never latched");
}

/* 6: GRAB_LATEST dropped the captured frame: the flow sees a newer one while CAPTURED and re-arms */
static void testDrop(void)
{
  reset();
  makeFrame(frA, 41);
  makeFrame(frB, 42);
  makeFrame(frC, 43);
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(0, frA, CAM_SPLIT_CHUNKS, NULL);
  CHECK(st() == CAM_SPLIT_CAPTURED, "frame A captured");
  runFrame(1, frB, CAM_SPLIT_CHUNKS, NULL);   /* the queue was full: cam_hal popped A (cam_give), pushed B */
  CHECK(st() == CAM_SPLIT_CAPTURED && memcmp(slotA, frA, CAM_SPLIT_RECV_BYTES) == 0,
        "CAPTURED is never overwritten by later frames");
  CHECK(flowFrame(1) == CAM_SPLIT_FLOW_REARM && st() == CAM_SPLIT_REQUESTED && S.rearm == 1,
        "the flow got B, newer than the capture: re-armed");
  runFrame(0, frC, CAM_SPLIT_CHUNKS, NULL);   /* buffer 0 reused, at the same address as A */
  CHECK(st() == CAM_SPLIT_CAPTURED && memcmp(slotA, frC, CAM_SPLIT_RECV_BYTES) == 0 && S.arm == 2,
        "the next frame captured into the same slot");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_FILL && metaTs == fbTs[0] && metaBuf == fb[0], "and filled");
  /* the same buffer address with A's old time does not match the new capture */
  reset();
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(0, frA, CAM_SPLIT_CHUNKS, NULL);
  CHECK(camSplitFlowFrame(&S, fb[0], fbTs[0] - 67500) == CAM_SPLIT_FLOW_NONE && st() == CAM_SPLIT_CAPTURED,
        "an older frame in the same buffer: no match, no re-arm");
  CHECK(camSplitFlowFrame(&S, fb[1], fbTs[0]) == CAM_SPLIT_FLOW_NONE && st() == CAM_SPLIT_CAPTURED,
        "the same time in another buffer: no match");
}

/* 7: a camera (re)initialisation resets the latch */
static void testReinit(void)
{
  reset();
  makeFrame(frA, 51);
  makeFrame(frB, 52);
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(0, frA, 5, NULL);
  CHECK(st() == CAM_SPLIT_CAPTURING, "capturing when the camera stalls");
  const uint32_t g0 = S.gen;
  camSplitReset(&S);   /* esp_camera_deinit + cameraStart: the sample-mode wrapper */
  CHECK(st() == CAM_SPLIT_REQUESTED && S.capBuf == NULL && S.capTsUs == 0 && S.capGen == 0 && S.gen == g0 + 1 &&
        S.resets == 1, "CAPTURING -> REQUESTED, the latch cleared, the generation moved");
  runFrame(0, frB, CAM_SPLIT_CHUNKS, NULL);   /* the re-allocated buffer at the same address */
  CHECK(st() == CAM_SPLIT_CAPTURED && memcmp(slotA, frB, CAM_SPLIT_RECV_BYTES) == 0, "the first new frame captured");
  camSplitReset(&S);
  CHECK(st() == CAM_SPLIT_REQUESTED && S.resets == 2, "CAPTURED -> REQUESTED (the frame died with the driver)");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE, "the old frame is no longer the photo");
  camSplitInit(&S);
  camSplitReset(&S);
  CHECK(st() == CAM_SPLIT_IDLE && S.resets == 0, "IDLE kept");
  CHECK(camSplitRequest(&S, slotA), "request");
  camSplitReset(&S);
  CHECK(st() == CAM_SPLIT_REQUESTED && S.resets == 0, "REQUESTED kept");
  runFrame(1, frA, CAM_SPLIT_CHUNKS, NULL);
  CHECK(flowFrame(1) == CAM_SPLIT_FLOW_FILL, "filled");
  camSplitReset(&S);
  CHECK(st() == CAM_SPLIT_FILLED && camSplitTake(&S), "FILLED kept: still taken");
}

/* 8: pass-through */
static void testPassThrough(void)
{
  reset();
  makeFrame(frA, 61);
  static uint8_t other[CAM_SPLIT_FB_BYTES];
  const uint8_t *bufs[2] = {fb[0], fb[1]};
  uint32_t idx = 99, off = 99;
  CHECK(!camSplitFrameOf(bufs, 2, other, ring, CAM_SPLIT_FB_BYTES, &idx, &off) && idx == 99,
        "an unknown pointer: pass-through");
  CHECK(!camSplitFrameOf(bufs, 2, fb[0], fb[0], CAM_SPLIT_FB_BYTES, &idx, &off), "out == in (cam_take's in-place "
        "call, PSRAM DMA mode): pass-through");
  CHECK(!camSplitFrameOf(bufs, 2, NULL, ring, CAM_SPLIT_FB_BYTES, &idx, &off), "NULL: pass-through");
  CHECK(!camSplitFrameOf(bufs, 1, fb[0] + CAM_SPLIT_FB_BYTES, ring, CAM_SPLIT_FB_BYTES, &idx, &off) &&
        camSplitFrameOf(bufs, 1, fb[0] + CAM_SPLIT_FB_BYTES - 1, ring, CAM_SPLIT_FB_BYTES, &idx, &off) && idx == 0 &&
        off == CAM_SPLIT_FB_BYTES - 1, "one past a buffer's end is not inside it, its last byte is");
  CHECK(camSplitFrameOf(bufs, 2, fb[1] + 3840, ring, CAM_SPLIT_FB_BYTES, &idx, &off) && idx == 1 && off == 3840,
        "inside buffer 1: its index and offset");
  CHECK(camSplitRequest(&S, slotA), "request");
  const uint8_t *bufsX[2] = {fb[0], fb[1]};
  CHECK(!camSplitStep(&S, bufsX, fbTs, 2, CAM_SPLIT_FB_BYTES, other, ring, CAM_SPLIT_HALF_BYTES) &&
        st() == CAM_SPLIT_REQUESTED, "a copy into an unknown buffer: no latch");
  CHECK(!camSplitStep(&S, bufsX, fbTs, 2, CAM_SPLIT_FB_BYTES, fb[0], fb[0], CAM_SPLIT_HALF_BYTES) &&
        st() == CAM_SPLIT_REQUESTED && slotA[0] == 0xA5, "an in-place copy: no latch, slot untouched");
  camSplitInit(&S);
  CHECK(!camSplitWants(&S) && !camSplitStep(&S, bufsX, fbTs, 2, CAM_SPLIT_FB_BYTES, fb[0], ring,
                                            CAM_SPLIT_HALF_BYTES), "IDLE: the step returns at its first load");
}

/* 9: the geometry */
static void testGeometry(void)
{
  CHECK(camSplitGeometryOk(CAM_SPLIT_FB_BYTES, CAM_SPLIT_RECV_BYTES, CAM_SPLIT_HALF_BYTES, CAM_SPLIT_HALF_CNT), "the split geometry");
  CHECK(!camSplitGeometryOk(19200, 19200, 3840, 4), "grey (Y8 from the sensor): not split");
  CHECK(!camSplitGeometryOk(38400, 38400, 7680, 2), "the driver's YUYV default (fb 2 bytes a pixel)");
  CHECK(!camSplitGeometryOk(19200, 38400, 7680, 2), "fb forced, the half buffers not");
  CHECK(!camSplitGeometryOk(19200, 38400, 3840, 2), "a wrong count");
  CHECK(!camSplitGeometryOk(CAM_SPLIT_FB_BYTES+4, CAM_SPLIT_RECV_BYTES+8, 3840, 4), "another frame size");
}

/* the handshake's refusals */
static void testHandshake(void)
{
  reset();
  CHECK(!camSplitRequest(&S, NULL), "no slot: refused");
  CHECK(camSplitRequest(&S, slotA) && !camSplitRequest(&S, slotB) && S.slot == slotA, "one request at a time");
  CHECK(camSplitPending(&S) && camSplitWants(&S), "pending, wanted");
  CHECK(!camSplitTake(&S), "nothing to take");
  CHECK(camSplitCancel(&S) && st() == CAM_SPLIT_IDLE && S.cancel == 1, "cancel REQUESTED");
  CHECK(camSplitCancel(&S) && S.cancel == 1, "cancel when IDLE: true, not counted");
  makeFrame(frA, 71);
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(0, frA, CAM_SPLIT_CHUNKS, NULL);
  CHECK(!camSplitWants(&S), "CAPTURED: the copy task's step does nothing");
  CHECK(camSplitCancel(&S) && st() == CAM_SPLIT_IDLE && S.cancel == 2, "cancel CAPTURED");
  CHECK(flowFrame(0) == CAM_SPLIT_FLOW_NONE, "the flow after it: nothing");
  CHECK(camSplitRequest(&S, slotA), "request");
  runFrame(1, frA, CAM_SPLIT_CHUNKS, NULL);
  CHECK(camSplitFlowFrame(&S, fb[1], fbTs[1]) == CAM_SPLIT_FLOW_FILL && st() == CAM_SPLIT_COPYING, "COPYING");
  CHECK(!camSplitCancel(&S) && st() == CAM_SPLIT_COPYING, "cancel while COPYING: refused (the caller retries)");
  camSplitFilled(&S);
  CHECK(camSplitCancel(&S) && st() == CAM_SPLIT_IDLE && S.cancel == 2, "then it drops the filled photo");
}

/* a randomised soak of the driver, the flow and the requester */
static uint32_t rng = 12345;
static uint32_t rnd(uint32_t n)
{
  rng = rng * 1103515245u + 12345u;
  return (rng >> 8) % n;
}
static int soakCancelAt = -1;
static void soakCb(int k)
{
  if (k == soakCancelAt) {
    camSplitCancel(&S);
  }
}
static void testSoak(void)
{
  reset();
  static uint8_t frames[2][CAM_SPLIT_RECV_BYTES];
  static uint8_t yuyvOf[2][CAM_SPLIT_RECV_BYTES];   /* the frame each buffer holds */
  int filled = 0, badSlot = 0, badLuma = 0, stuck = 0, held = -1;
  uint32_t lastNum = 0;
  uint8_t *slot = slotA;
  int pendingFrames = 0;
  for (uint32_t f = 0; f < 20000; f++) {
    const int idx = (held == 0) ? 1 : (held == 1 ? 0 : (int)rnd(2));
    makeFrame(frames[f & 1], ++lastNum);
    if (!camSplitPending(&S) && rnd(4) == 0) {
      slot = rnd(2) ? slotA : slotB;
      camSplitRequest(&S, slot);
      pendingFrames = 0;
    }
    soakCancelAt = rnd(40) == 0 ? (int)rnd(CAM_SPLIT_CHUNKS) : -1;
    const int chunks = rnd(50) == 0 ? (int)rnd(CAM_SPLIT_CHUNKS) : CAM_SPLIT_CHUNKS;
    runFrame(idx, frames[f & 1], chunks, soakCb);
    if (chunks < CAM_SPLIT_CHUNKS) {
      continue;   /* aborted: the driver restarts in the same buffer at the next frame start */
    }
    memcpy(yuyvOf[idx], frames[f & 1], CAM_SPLIT_RECV_BYTES);
    badLuma += !lumaEq(idx, frames[f & 1]);
    if (rnd(30) == 0) {
      camSplitReset(&S);   /* a stall re-init */
    }
    if (rnd(8) == 0) {
      held = -1;
      continue;   /* GRAB_LATEST dropped it, or a stale frame the flow skips */
    }
    held = idx;
    if (flowFrame(idx) == CAM_SPLIT_FLOW_FILL) {
      filled++;
      badSlot += memcmp(slot, yuyvOf[idx], CAM_SPLIT_RECV_BYTES) != 0 || metaBuf != fb[idx] || metaTs != fbTs[idx];
    }
    if (camSplitTake(&S)) {
      pendingFrames = 0;
    } else if (camSplitPending(&S) && ++pendingFrames > 15) {   /* auto_link's 1 s timeout */
      stuck++;
      while (!camSplitCancel(&S)) {
      }
      pendingFrames = 0;
    }
  }
  CHECK(badLuma == 0, "every complete frame's Y is the luma (%d bad)", badLuma);
  CHECK(filled > 3000 && badSlot == 0, "every filled photo is its frame's YUYV with its buffer and time (%d filled, "
        "%d bad)", filled, badSlot);
  CHECK(stuck < filled / 20, "requests timing out are rare (%d of %d)", stuck, filled);
  CHECK(S.arm >= (uint32_t)filled && S.rearm > 0 && S.restart > 0 && S.resets > 0 && S.cancel > 0,
        "the soak reached every path (arm %u restart %u rearm %u cancel %u resets %u)", (unsigned)S.arm,
        (unsigned)S.restart, (unsigned)S.rearm, (unsigned)S.cancel, (unsigned)S.resets);
}

int main(void)
{
  testIdentity();
  testMidRequest();
  testMidCancel();
  testAbort();
  testDrop();
  testReinit();
  testPassThrough();
  testGeometry();
  testHandshake();
  hookCalls = 0;
  hookBad = 0;
  testSoak();
  /* 10: the latch (buffer and time) never changed under a published CAPTURED */
  CHECK(hookCalls > 1000 && hookBad == 0, "the latched buffer and time are written only while not CAPTURED (%d "
        "writes, %d under CAPTURED)", hookCalls, hookBad);
  printf("test_cam_split: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
