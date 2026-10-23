#ifndef CAMERA_JPEG_H
#define CAMERA_JPEG_H
#include <stdbool.h>
#include <stdint.h>
/* Single job; input slot must remain allocated and unchanged until !Busy().
 * Cancel retains ownership until the worker stops reading the input. */
bool cameraJpegBusy(void);
bool cameraJpegStart(uint8_t *src, uint32_t bytes, uint8_t format);
/* 0 pending, 1 copied JPEG, -1 failed; only the link task calls Start/Take/Cancel. */
int cameraJpegTake(uint8_t *dst, uint32_t capacity, uint32_t *bytes);
void cameraJpegCancel(void);
#endif
