/*
 * ESP-Drone Firmware
 *
 * cam_split.h - Always-on colour: the link-time wrappers of the camera driver (cam_split.c; the pure core and its
 * design: cam_split_core.h; docs/autonomy-a2b.md section 28)
 *
 * camera_flow.c calls camSplitEnable() before every esp_camera_init() (true only with NVS photocol 1 and the
 * PIXFORMAT_YUV422 init it then makes), and camSplitInitInfo() after it: the boot check. The snapshot handshake of
 * the split mode is camSplitSnap (cam_split_core.h), shared by the driver's copy task (the ll_cam_memcpy wrapper),
 * the flow task and the requester.
 */
#ifndef CAM_SPLIT_H_
#define CAM_SPLIT_H_

#include <stdbool.h>
#include <stdint.h>

#include "cam_split_core.h"

extern camSplit_t camSplitSnap;

/* The split for the next camera init(s): with it, a YUV422 init on the OV3660 (outside the driver's PSRAM DMA mode)
 * gets 1-byte frame buffers and grey's DMA geometry. */
void camSplitEnable(bool on);

typedef struct {
  bool active;           /* the latest init was split (YUV422, OV3660, fb_bytes_per_pixel forced to 1) */
  bool geometryOk;       /* ... and its sizes are the split's (camSplitGeometryOk) */
  uint32_t fbSize;       /* the latest init's driver sizes, for the log (0 before any) */
  uint32_t recvSize;
  uint32_t halfSize;
  uint32_t halfCnt;
  uint32_t inits;        /* sample-mode calls (every esp_camera_init) */
} camSplitInitInfo_t;
void camSplitInitInfo(camSplitInitInfo_t *info);

#endif /* CAM_SPLIT_H_ */
