/*
 * ESP-Drone Firmware
 *
 * cam_split_core.h - Always-on colour: the photo frame's raw YUYV copied out of the camera driver's own copy
 * (docs/autonomy-a2b.md section 28; the plan colour-sink-plan-2026-10-16.md section 2)
 *
 * With NVS photocol 1 the OV3660 streams YUV 4:2:2 (YUYV, 0x4300 = 0x30) on every frame, and the camera driver
 * (esp32-camera 2.1.0) is told its frame buffers hold one byte a pixel (cam_split.c, a link-time wrapper of
 * ll_cam_set_sample_mode): its own IRAM ll_cam_memcpy then writes out[k] = in[2k], the Y bytes, into the same 19200 B
 * PSRAM frame buffers the grey camera fills, so the optical flow reads exactly the frame layout and code path of grey.
 * The DMA ring keeps grey's geometry (4 half buffers of 3840 B, an event queue 3 deep: the ll_cam_dma_sizes wrapper),
 * so each frame is 10 chunks of 3840 B in (12 YUYV lines) and 1920 B out.
 *
 * Only for the frame a photo request arms, the ll_cam_memcpy wrapper also copies each raw 3840 B chunk from the
 * internal DMA ring straight into the requester's PSRAM slot at slot + 2 x off (off: the chunk's offset in the Y
 * frame), so the slot ends up holding the whole 38400 B YUYV frame as the sensor sent it. No chroma plane, no
 * re-interleave, no luma buffer, no copy in the flow task: the flow only fills the photo's meta (sequence, time,
 * texture, brightness) when it processes that same frame (matched by its buffer and capture time).
 *
 * The snapshot states (split mode only; grey keeps camera_flow.c's own handshake, unchanged):
 *
 *   IDLE --request--> REQUESTED --frame start (off 0)--> CAPTURING --last chunk--> CAPTURED
 *   CAPTURED --the flow processes the frame (buf + time match)--> COPYING (meta only) --> FILLED --take--> IDLE
 *
 *   - a new frame start while CAPTURING (the frame was aborted: EV-OVF, FB-SIZE) restarts the capture on it;
 *     a chunk out of sequence re-arms (REQUESTED);
 *   - the flow sees a newer frame while CAPTURED (the captured one was dropped by GRAB_LATEST, or skipped as stale):
 *     CAPTURED -> REQUESTED (re-arm, counted);
 *   - cancel: REQUESTED, CAPTURING or CAPTURED -> IDLE, and the generation counter is incremented first; every chunk
 *     copies only while CAPTURING with the generation it latched (the second line of defence: the requester and the
 *     driver's copy task are on core 0, the copy task at priority 23, so a cancel cannot run in the middle of a
 *     chunk); FILLED -> IDLE; COPYING refuses (a few microseconds; the caller retries);
 *   - every camera (re)initialisation (camSplitReset, from the sample-mode wrapper): CAPTURING or CAPTURED ->
 *     REQUESTED (the request stays armed, so the requester's 1 s timeout still applies), the latched buffer, time
 *     and generation invalidated (the stall re-init frees and re-allocates the frame buffers, likely at the same
 *     addresses).
 *
 * Publish order: the latched buffer and time are written only while the state is CAPTURING (never CAPTURED), and
 * CAPTURED is stored with release after the last chunk's copy; the flow reads them only after an acquire load of
 * CAPTURED. Pure C (memcpy and the GCC __atomic builtins), host-tested by tools/tests/test_cam_split.c. Flash code:
 * the driver's copy task (cam_task) and the flow task are ordinary flash tasks.
 */
#ifndef CAM_SPLIT_CORE_H_
#define CAM_SPLIT_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "camera_profile.h"
#define CAM_SPLIT_W           CAMERA_NATIVE_WIDTH
#define CAM_SPLIT_H           CAMERA_NATIVE_HEIGHT
#define CAM_SPLIT_FB_BYTES    (CAM_SPLIT_W * CAM_SPLIT_H)       /* 19200: the Y8 frame the flow reads */
#define CAM_SPLIT_RECV_BYTES  (2u * CAM_SPLIT_W * CAM_SPLIT_H)  /* 38400: the YUYV frame = the photo */
#define CAM_SPLIT_HALF_BYTES  3840u                            /* one DMA half buffer, 12 YUYV lines (grey: 24 Y8) */
#define CAM_SPLIT_HALF_CNT    4u                               /* the 15360 B ring, an event queue 3 deep */
#define CAM_SPLIT_CHUNK_OUT   (CAM_SPLIT_HALF_BYTES / 2u)      /* 1920 Y bytes a chunk */
#define CAM_SPLIT_CHUNKS      (CAM_SPLIT_FB_BYTES / CAM_SPLIT_CHUNK_OUT)   /* 10 a frame */
#define CAM_SPLIT_MAX_FB      4u                               /* frame buffers looked at (fb_count 2) */

/* The states. IDLE, REQUESTED, FILLED and COPYING have the values of camera_flow.c's grey SNAP_*. */
#define CAM_SPLIT_IDLE        0u
#define CAM_SPLIT_REQUESTED   1u
#define CAM_SPLIT_FILLED      2u
#define CAM_SPLIT_COPYING     3u
#define CAM_SPLIT_CAPTURING   4u
#define CAM_SPLIT_CAPTURED    5u

typedef struct {
  uint32_t state;             /* CAM_SPLIT_*: atomic */
  uint32_t gen;               /* atomic: +1 at every cancel of a pending request and at every reset */
  uint32_t capGen;            /* copy task: gen when the capture latched */
  uint8_t *slot;              /* the requester's buffer (CAM_SPLIT_RECV_BYTES), written before REQUESTED */
  const uint8_t *capBuf;      /* the latched frame buffer (meaningful while CAPTURED) */
  uint32_t nextOff;           /* the next chunk's offset in the Y frame while CAPTURING */
  uint64_t capTsUs;           /* the latched frame's capture (start of frame) time, the flow's frame time */
  /* counters (u16: internal RAM; they wrap), each with one writer: arm and restart the copy task, done and rearm
   * the flow, cancel the requester, resets the (re)initialising task */
  uint16_t arm;               /* captures latched at a frame start */
  uint16_t restart;           /* captures restarted (a frame start while capturing) or dropped (a chunk out of
                               * sequence) */
  uint16_t done;              /* photos filled */
  uint16_t rearm;             /* CAPTURED -> REQUESTED: the flow saw a newer frame */
  uint16_t cancel;            /* pending requests cancelled */
  uint16_t resets;            /* captures re-armed by a camera (re)initialisation */
} camSplit_t;

/* Flow side of a processed frame (camSplitFlowFrame). */
typedef enum {
  CAM_SPLIT_FLOW_NONE = 0,    /* nothing to do */
  CAM_SPLIT_FLOW_FILL,        /* this is the photo frame: the caller now owns COPYING, fills the meta, then calls
                               * camSplitFilled() */
  CAM_SPLIT_FLOW_REARM,       /* the captured frame was not this one and this one is newer: re-armed */
} camSplitFlow_t;

void camSplitInit(camSplit_t *s);

/* The split geometry of one camera init: the driver's fb_size, recv_size, dma_half_buffer_size and
 * dma_half_buffer_cnt must be 19200 / 38400 / 3840 / 4. */
bool camSplitGeometryOk(uint32_t fbSize, uint32_t recvSize, uint32_t halfSize, uint32_t halfCnt);

/* Which frame buffer a driver copy writes into: true with its index and the offset of out in it when out lies inside
 * bufs[i] .. bufs[i] + fbSize - 1 for one of the n buffers; false (pass-through) for any other pointer, NULL, or out
 * == in (the in-place call cam_take makes in the driver's PSRAM DMA mode). */
bool camSplitFrameOf(const uint8_t *const bufs[], uint32_t n, const uint8_t *out, const uint8_t *in, uint32_t fbSize,
                     uint32_t *idx, uint32_t *off);

/* Requester: arms the next frame to start for slot (CAM_SPLIT_RECV_BYTES). False unless IDLE. Anything else the
 * requester hands over (camera_flow.c's meta pointer) must be written before this call (its release store publishes
 * it). */
bool camSplitRequest(camSplit_t *s, uint8_t *slot);
/* Requester: true (and IDLE again) once FILLED. */
bool camSplitTake(camSplit_t *s);
/* Requester: withdraws a pending request (REQUESTED, CAPTURING, CAPTURED: the generation first, then IDLE) or drops a
 * filled one; true when IDLE afterwards, false while COPYING (call again). */
bool camSplitCancel(camSplit_t *s);
/* Any task: a request is outstanding (not IDLE). */
bool camSplitPending(const camSplit_t *s);

/* Copy task: true when a chunk can matter (REQUESTED or CAPTURING): a plain load, the unarmed frames' whole cost. */
bool camSplitWants(const camSplit_t *s);
/* Copy task, a chunk of len input bytes written as len / 2 Y bytes at offset off of frame buffer idx (fbBuf, captured
 * at tsUs): the slot address to copy the raw chunk to (slot + 2 x off), or NULL. Latches a capture at off 0 when
 * REQUESTED (restarts one when CAPTURING); checks the generation. */
uint8_t *camSplitChunk(camSplit_t *s, uint32_t idx, const uint8_t *fbBuf, uint32_t off, uint32_t len, uint64_t tsUs);
/* Copy task, after a chunk camSplitChunk() named was copied: CAPTURED (release) when it was the frame's last. */
void camSplitChunkCopied(camSplit_t *s, uint32_t off, uint32_t len);
/* Copy task: camSplitChunk, the copy (memcpy from in), camSplitChunkCopied. True when this chunk was copied. */
bool camSplitCopyChunk(camSplit_t *s, uint32_t idx, const uint8_t *fbBuf, uint32_t off, const uint8_t *in,
                       uint32_t len, uint64_t tsUs);
/* Copy task, the ll_cam_memcpy wrapper's whole step after the driver's own copy of len input bytes from in to out:
 * nothing unless camSplitWants(); else camSplitFrameOf() over the n buffers (tsUs[i]: buffer i's capture time) and
 * camSplitCopyChunk(). */
bool camSplitStep(camSplit_t *s, const uint8_t *const bufs[], const uint64_t tsUs[], uint32_t n, uint32_t fbSize,
                  const uint8_t *out, const uint8_t *in, uint32_t len);

/* Flow task, after processing the frame buf captured at tsUs: CAPTURED and the same frame -> COPYING (FILL); CAPTURED
 * and a newer frame -> REQUESTED (REARM); else NONE. */
camSplitFlow_t camSplitFlowFrame(camSplit_t *s, const uint8_t *buf, uint64_t tsUs);
/* Flow task, after FILL and the meta: FILLED (release). */
void camSplitFilled(camSplit_t *s);

/* The (re)initialising task, at every camera init before the driver's copy task starts: CAPTURING or CAPTURED ->
 * REQUESTED, the latched capture invalidated (the generation incremented, buffer, time and latched generation
 * cleared). IDLE, REQUESTED and FILLED are kept. */
void camSplitReset(camSplit_t *s);

#endif /* CAM_SPLIT_CORE_H_ */
