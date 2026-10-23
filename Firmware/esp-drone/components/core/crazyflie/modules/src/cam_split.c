/*
 * ESP-Drone Firmware
 *
 * cam_split.c - Always-on colour: three link-time wrappers (-Wl,--wrap, CMakeLists.txt) of esp32-camera 2.1.0's
 * ll_cam functions, which cam_hal.c calls from another object file (cam_split.h, cam_split_core.h,
 * docs/autonomy-a2b.md section 28). Flash code: the driver's copy task (cam_task) is an ordinary flash task, and
 * the other two run in esp_camera_init (the flow task). Each wrapper calls the real function first; without the
 * split (photocol 0, or another sensor or format) they change nothing.
 *
 *   __wrap_ll_cam_set_sample_mode: with the split enabled, PIXFORMAT_YUV422 on the OV3660 outside the PSRAM DMA
 *     mode (XCLK 16 MHz): fb_bytes_per_pixel = 1, the driver's own "YUV to grayscale" geometry (cam_config then
 *     computes fb_size 19200, recv_size 38400). Then, on every init (also the 4 s stall re-init), the snapshot core
 *     is reset (camSplitReset).
 *   __wrap_ll_cam_dma_sizes: grey's DMA geometry for the split: dma_half_buffer_size 3840, dma_half_buffer_cnt
 *     dma_buffer_size / 3840 = 4 (node 3840 = 12 YUYV lines, 120 % 12 == 0; the driver's YUYV default is 2 x 7680
 *     with an event queue 1 deep), and the boot check's sizes.
 *   __wrap_ll_cam_memcpy: always the driver's own copy (__real_, IRAM; with fb 1 its out[k] = in[2k] Y path), then
 *     the split step (camSplitStep): only for the armed photo frame, the raw chunk to the requester's PSRAM slot.
 */
#include <string.h>

#include "sdkconfig.h"
#include "ll_cam.h"   /* esp32-camera's private target/private_include (CMakeLists.txt): cam_obj_t */

#include "cam_split.h"

#if CONFIG_CAMERA_CONVERTER_ENABLED
#error "cam_split.c needs the camera converter off: cam_obj_t's bytes-per-pixel fields are then uint8_t"
#endif
#if !CONFIG_CAMERA_CORE0
#error "cam_split.c: the driver's copy task must share core 0 with auto_link (a cancel never interleaves a chunk)"
#endif
#if !CONFIG_IDF_TARGET_ESP32S3
#error "cam_split.c wraps the ESP32-S3 ll_cam (target/esp32s3/ll_cam.c)"
#endif

_Static_assert(CAM_SPLIT_MAX_FB >= 2, "fb_count 2 (camera_flow.c)");

size_t __real_ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len);
esp_err_t __real_ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz,
                                        uint16_t sensor_pid);
bool __real_ll_cam_dma_sizes(cam_obj_t *cam);
size_t __wrap_ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len);
esp_err_t __wrap_ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz,
                                        uint16_t sensor_pid);
bool __wrap_ll_cam_dma_sizes(cam_obj_t *cam);

#define CAM_SPLIT_PSRAM_MODE_HZ 16000000u   /* cam_hal.c cam_config: psram_mode = (xclk == 16 MHz) */

camSplit_t camSplitSnap;

/* internal RAM, 12 bytes: the switch, this init's flags and sizes (u16: all under 65536) */
static struct {
  bool wanted;      /* camSplitEnable() */
  bool active;      /* this init: fb forced to 1 byte a pixel */
  bool on;          /* ... and the geometry passed: the copy wrapper's step runs */
  uint8_t pad;
  uint32_t fb, recv, half, cnt;
} split;
static uint16_t splitInits;

void camSplitEnable(bool on)
{
  split.wanted = on;
}

void camSplitInitInfo(camSplitInitInfo_t *info)
{
  info->active = split.active;
  info->geometryOk = split.on;
  info->fbSize = split.fb;
  info->recvSize = split.recv;
  info->halfSize = split.half;
  info->halfCnt = split.cnt;
  info->inits = splitInits;
}

esp_err_t __wrap_ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz,
                                        uint16_t sensor_pid)
{
  const esp_err_t err = __real_ll_cam_set_sample_mode(cam, pix_format, xclk_freq_hz, sensor_pid);
  /* the copy task does not exist yet (cam_config creates it later; a re-init deleted the old one first) */
  split.on = false;
  split.active = false;
  split.fb = split.recv = split.half = split.cnt = 0;
  if (err == ESP_OK && split.wanted && pix_format == PIXFORMAT_YUV422 && sensor_pid == OV3660_PID &&
      xclk_freq_hz != CAM_SPLIT_PSRAM_MODE_HZ && cam->in_bytes_per_pixel == 2 && cam->fb_bytes_per_pixel == 2) {
    cam->fb_bytes_per_pixel = 1;
    split.active = true;
  }
  splitInits++;
  camSplitReset(&camSplitSnap);
  return err;
}

bool __wrap_ll_cam_dma_sizes(cam_obj_t *cam)
{
  const bool ok = __real_ll_cam_dma_sizes(cam);
  if (!split.active) {
    return ok;
  }
  if (ok && !cam->jpeg_mode && !cam->psram_mode && cam->dma_bytes_per_item == 1 &&
      cam->dma_node_buffer_size == CAM_SPLIT_HALF_BYTES && cam->dma_buffer_size % CAM_SPLIT_HALF_BYTES == 0) {
    cam->dma_half_buffer_size = CAM_SPLIT_HALF_BYTES;
    cam->dma_half_buffer_cnt = cam->dma_buffer_size / CAM_SPLIT_HALF_BYTES;
  }
  split.fb = (uint32_t)cam->fb_size;
  split.recv = (uint32_t)cam->recv_size;
  split.half = (uint32_t)cam->dma_half_buffer_size;
  split.cnt = (uint32_t)cam->dma_half_buffer_cnt;
  split.on = ok && !cam->psram_mode &&
             camSplitGeometryOk(cam->fb_size, cam->recv_size, cam->dma_half_buffer_size, cam->dma_half_buffer_cnt);
  return ok;
}

/* The armed path, out of line: its arrays live on the copy task's stack only while a photo is being captured (the
 * unarmed and grey path's frame stays the size of the driver's own copy's). */
static void __attribute__((noinline)) splitArmedChunk(const cam_obj_t *cam, const uint8_t *out, const uint8_t *in,
                                                      size_t len)
{
  const uint8_t *bufs[CAM_SPLIT_MAX_FB];
  uint64_t ts[CAM_SPLIT_MAX_FB];
  const uint32_t cnt = cam->frame_cnt < CAM_SPLIT_MAX_FB ? cam->frame_cnt : CAM_SPLIT_MAX_FB;
  for (uint32_t i = 0; i < cnt; i++) {
    const camera_fb_t *fb = &cam->frames[i].fb;
    bufs[i] = fb->buf;
    ts[i] = (uint64_t)fb->timestamp.tv_sec * 1000000ULL + (uint64_t)fb->timestamp.tv_usec;   /* camera_flow.c's t */
  }
  (void)camSplitStep(&camSplitSnap, bufs, ts, cnt, cam->fb_size, out, in, (uint32_t)len);
}

size_t __wrap_ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len)
{
  const size_t n = __real_ll_cam_memcpy(cam, out, in, len);
  if (split.on && camSplitWants(&camSplitSnap)) {
    splitArmedChunk(cam, out, in, len);   /* a photo requested or being captured */
  }
  return n;   /* grey, and every unarmed split frame: two loads */
}
