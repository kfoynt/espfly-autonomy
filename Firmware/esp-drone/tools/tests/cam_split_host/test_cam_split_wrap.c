/*
 * Host harness for components/core/crazyflie/modules/src/cam_split.c (docs/autonomy-a2b.md section 28): the three
 * link-time wrappers against host copies of esp32-camera 2.1.0's own functions (target/esp32s3/ll_cam.c
 * ll_cam_set_sample_mode, ll_cam_dma_sizes / ll_cam_calc_rgb_dma and ll_cam_memcpy, the converter off; stub/ll_cam.h
 * has cam_obj_t's fields as in 2.1.0) and a model of driver/cam_hal.c's cam_config sizes and cam_task's copy loop
 * (its FB-OVF and FB-SIZE checks).
 *
 * Checks: grey (Y8 from the OV3660) keeps the driver's geometry (fb 19200, 4 x 3840, queue 3) and never activates the
 * split, also with the split enabled; the driver's YUYV default is fb 38400 with 2 x 7680 halves (queue 1); with the
 * split enabled, YUV422 on the OV3660 at 13.33 MHz gets fb 19200, recv 38400, 4 x 3840 halves, queue 3, 10 copies a
 * frame, and a whole frame passes the driver's FB-OVF and FB-SIZE checks with the Y bytes in the frame buffer; the
 * OV2640, the PSRAM DMA mode (16 MHz) and the split disabled stay the driver's; every init resets the snapshot core
 * (a capture in progress re-armed); the copy wrapper writes the slot only for an armed frame.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=gnu99 -Wall -Wextra -Itools/tests/cam_split_host/stub -Icomponents/core/crazyflie/modules/interface \
 *      -Icomponents/core/crazyflie/utils/interface -Icomponents/core/crazyflie/modules/src \
 *      tools/tests/cam_split_host/test_cam_split_wrap.c components/core/crazyflie/utils/src/cam_split_core.c \
 *      -o /tmp/test_cam_split_wrap && /tmp/test_cam_split_wrap
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cam_split.c"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

/* ---- esp32-camera 2.1.0 target/esp32s3/ll_cam.c, the converter off (copied; ESP_LOG calls left out) ---- */
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE (4092)

esp_err_t __real_ll_cam_set_sample_mode(cam_obj_t *cam, pixformat_t pix_format, uint32_t xclk_freq_hz,
                                        uint16_t sensor_pid)
{
  (void)xclk_freq_hz;
  if (pix_format == PIXFORMAT_GRAYSCALE) {
    if (sensor_pid == OV3660_PID) {
      cam->in_bytes_per_pixel = 1;       // camera sends Y8
    } else {
      cam->in_bytes_per_pixel = 2;       // camera sends YU/YV
    }
    cam->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
  } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
    cam->in_bytes_per_pixel = 2;       // for DMA receive
    cam->fb_bytes_per_pixel = 2;       // frame buffer stores YU/YV/RGB565
  } else if (pix_format == PIXFORMAT_JPEG) {
    cam->in_bytes_per_pixel = 1;
    cam->fb_bytes_per_pixel = 1;
  } else {
    return ESP_ERR_NOT_SUPPORTED;
  }
  return ESP_OK;
}

static bool ll_cam_calc_rgb_dma(cam_obj_t *cam)
{
  size_t node_max = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / cam->dma_bytes_per_item;
  size_t line_width = cam->width * cam->in_bytes_per_pixel;
  size_t node_size = node_max;
  size_t nodes_per_line = 1;
  size_t lines_per_node = 1;
  if (line_width >= node_max) {
    for (size_t i = node_max; i > 0; i = i - 1) {
      if ((line_width % i) == 0) {
        node_size = i;
        nodes_per_line = line_width / node_size;
        break;
      }
    }
  } else {
    for (size_t i = node_max; i > 0; i = i - 1) {
      if ((i % line_width) == 0) {
        node_size = i;
        lines_per_node = node_size / line_width;
        while ((cam->height % lines_per_node) != 0) {
          lines_per_node = lines_per_node - 1;
          node_size = lines_per_node * line_width;
        }
        break;
      }
    }
  }
  cam->dma_node_buffer_size = node_size * cam->dma_bytes_per_item;
  size_t dma_half_buffer_max = CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX / 2 / cam->dma_bytes_per_item;
  if (line_width > dma_half_buffer_max) {
    return 0;
  }
  size_t dma_half_buffer_min = node_size * nodes_per_line;
  size_t dma_half_buffer = (dma_half_buffer_max / dma_half_buffer_min) * dma_half_buffer_min;
  size_t lines_per_half_buffer = dma_half_buffer / line_width;
  while ((cam->height % lines_per_half_buffer) != 0) {
    dma_half_buffer = dma_half_buffer - dma_half_buffer_min;
    lines_per_half_buffer = dma_half_buffer / line_width;
  }
  size_t dma_buffer_max = 2 * dma_half_buffer_max;
  if (cam->psram_mode) {
    dma_buffer_max = cam->recv_size / cam->dma_bytes_per_item;
  }
  size_t dma_buffer_size = dma_buffer_max;
  if (!cam->psram_mode) {
    dma_buffer_size = (dma_buffer_max / dma_half_buffer) * dma_half_buffer;
  }
  cam->dma_buffer_size = dma_buffer_size * cam->dma_bytes_per_item;
  cam->dma_half_buffer_size = dma_half_buffer * cam->dma_bytes_per_item;
  cam->dma_half_buffer_cnt = cam->dma_buffer_size / cam->dma_half_buffer_size;
  return 1;
}

bool __real_ll_cam_dma_sizes(cam_obj_t *cam)
{
  cam->dma_bytes_per_item = 1;
  return ll_cam_calc_rgb_dma(cam);   /* not JPEG */
}

static int realCopies;
size_t __real_ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len)
{
  realCopies++;
  if (cam->in_bytes_per_pixel == 2 && cam->fb_bytes_per_pixel == 1) {
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
  memcpy(out, in, len);
  return len;
}

/* ---- driver/cam_hal.c: cam_config's sizes and cam_task's copy loop ---- */
static cam_obj_t cam;
static cam_frame_t frames[2];
static uint8_t fbMem[2][38400];
static uint8_t ring[16384];
static uint32_t queueLen;

static esp_err_t camConfig(pixformat_t fmt, uint32_t xclk, uint16_t pid)
{
  memset(&cam, 0, sizeof(cam));
  esp_err_t err = __wrap_ll_cam_set_sample_mode(&cam, fmt, xclk, pid);   /* cam_hal.c calls the wrapped symbol */
  if (err != ESP_OK) {
    return err;
  }
  cam.jpeg_mode = fmt == PIXFORMAT_JPEG;
  cam.psram_mode = xclk == 16000000;
  cam.frame_cnt = 2;
  cam.width = 160;
  cam.height = 120;
  cam.recv_size = cam.width * cam.height * cam.in_bytes_per_pixel;
  cam.fb_size = cam.width * cam.height * cam.fb_bytes_per_pixel;
  if (!__wrap_ll_cam_dma_sizes(&cam)) {
    return -1;
  }
  cam.dma_node_cnt = cam.dma_buffer_size / cam.dma_node_buffer_size;
  cam.frame_copy_cnt = cam.recv_size / cam.dma_half_buffer_size;
  memset(frames, 0, sizeof(frames));
  for (int x = 0; x < 2; x++) {
    frames[x].fb.buf = fbMem[x];
  }
  cam.frames = frames;
  queueLen = cam.dma_half_buffer_cnt - 1 ? cam.dma_half_buffer_cnt - 1 : 1;
  return ESP_OK;
}

/* One frame into buffer idx as cam_task copies it; returns 0 when it would be queued, else the driver's error. */
static int camFrame(int idx, const uint8_t *src, long tsUs)
{
  camera_fb_t *fb = &frames[idx].fb;
  fb->timestamp.tv_sec = tsUs / 1000000;
  fb->timestamp.tv_usec = tsUs % 1000000;
  fb->len = 0;
  const size_t pixels_per_dma = (cam.dma_half_buffer_size * cam.fb_bytes_per_pixel) /
                                (cam.dma_bytes_per_item * cam.in_bytes_per_pixel);
  for (uint32_t cnt = 0; cnt < cam.frame_copy_cnt; cnt++) {
    if (cam.fb_size < fb->len + pixels_per_dma) {
      return 1;   /* FB-OVF */
    }
    uint8_t *half = &ring[(cnt % cam.dma_half_buffer_cnt) * cam.dma_half_buffer_size];
    memcpy(half, src + cnt * cam.dma_half_buffer_size, cam.dma_half_buffer_size);   /* the GDMA */
    fb->len += __wrap_ll_cam_memcpy(&cam, &fb->buf[fb->len], half, cam.dma_half_buffer_size);
  }
  return fb->len != cam.fb_size ? 2 : 0;   /* FB-SIZE */
}

static uint8_t yuyv[38400], slot[38400];

static void makeYuyv(int seed)
{
  for (int i = 0; i < 38400; i++) {
    yuyv[i] = (uint8_t)(i & 1 ? 100 + seed + i / 640 : (i / 2) * 3 + seed);
  }
}

int main(void)
{
  camSplitInitInfo_t info;
  makeYuyv(1);

  /* grey, the split off (photocol 0) and on */
  for (int on = 0; on < 2; on++) {
    camSplitEnable(on);
    CHECK(camConfig(PIXFORMAT_GRAYSCALE, 13333333, OV3660_PID) == ESP_OK, "grey init");
    camSplitInitInfo(&info);
    CHECK(cam.in_bytes_per_pixel == 1 && cam.fb_size == 19200 && cam.recv_size == 19200 &&
          cam.dma_half_buffer_size == 3840 && cam.dma_half_buffer_cnt == 4 && queueLen == 3 &&
          cam.frame_copy_cnt == 5 && !info.active && !info.geometryOk && !split.on,
          "grey (split %d): the driver's geometry (19200, 4 x 3840, queue 3, 5 copies), no split", on);
  }
  /* the driver's own YUYV geometry: the split disabled */
  camSplitEnable(false);
  CHECK(camConfig(PIXFORMAT_YUV422, 13333333, OV3660_PID) == ESP_OK, "YUYV init");
  camSplitInitInfo(&info);
  CHECK(cam.fb_size == 38400 && cam.dma_half_buffer_size == 7680 && cam.dma_half_buffer_cnt == 2 && queueLen == 1 &&
        !info.active, "the driver's YUYV default: fb 38400, 2 x 7680, queue 1 (the review's risk); no split");

  /* the split: YUV422 on the OV3660 at 13.33 MHz */
  camSplitEnable(true);
  CHECK(camConfig(PIXFORMAT_YUV422, 13333333, OV3660_PID) == ESP_OK, "split init");
  camSplitInitInfo(&info);
  CHECK(info.active && info.geometryOk && split.on && info.fbSize == 19200 && info.recvSize == 38400 &&
        info.halfSize == 3840 && info.halfCnt == 4, "the boot check's sizes: 19200 38400 3840 4");
  CHECK(cam.in_bytes_per_pixel == 2 && cam.fb_bytes_per_pixel == 1 && cam.dma_node_buffer_size == 3840 &&
        cam.dma_buffer_size == 15360 && cam.dma_node_cnt == 4 && cam.frame_copy_cnt == 10 && queueLen == 3,
        "fb 1 byte a pixel, node 3840, the 15360 B ring in 4 halves, 10 copies a frame, the queue 3 deep");
  realCopies = 0;
  memset(slot, 0xA5, sizeof(slot));
  CHECK(camFrame(0, yuyv, 1000000) == 0 && realCopies == 10, "a whole frame passes FB-OVF and FB-SIZE, 10 copies");
  int yBad = 0;
  for (int i = 0; i < 19200; i++) {
    yBad += fbMem[0][i] != yuyv[2 * i];
  }
  CHECK(yBad == 0 && slot[0] == 0xA5 && slot[38399] == 0xA5, "the frame buffer holds the Y bytes; unarmed: the slot "
        "untouched");
  /* armed: the next frame's raw YUYV in the slot */
  CHECK(camSplitRequest(&camSplitSnap, slot), "request");
  makeYuyv(7);
  CHECK(camFrame(1, yuyv, 1067500) == 0, "the photo frame");
  CHECK(memcmp(slot, yuyv, sizeof(slot)) == 0 && camSplitSnap.state == CAM_SPLIT_CAPTURED &&
        camSplitSnap.capTsUs == 1067500 && camSplitSnap.capBuf == fbMem[1], "armed: the slot is the raw frame, "
        "CAPTURED with its buffer and capture time (camera_flow.c's t)");
  CHECK(camSplitFlowFrame(&camSplitSnap, fbMem[1], 1067500) == CAM_SPLIT_FLOW_FILL, "the flow matches it");
  camSplitFilled(&camSplitSnap);
  CHECK(camSplitTake(&camSplitSnap), "taken");

  /* every init resets the core: a capture in progress is re-armed (the 4 s stall re-init) */
  CHECK(camSplitRequest(&camSplitSnap, slot), "request");
  CHECK(camSplitCopyChunk(&camSplitSnap, 0, fbMem[0], 0, ring, 3840, 5) && camSplitSnap.state == CAM_SPLIT_CAPTURING,
        "capturing");
  const uint32_t inits0 = splitInits;
  CHECK(camConfig(PIXFORMAT_YUV422, 13333333, OV3660_PID) == ESP_OK, "re-init");
  CHECK(camSplitSnap.state == CAM_SPLIT_REQUESTED && camSplitSnap.capBuf == NULL && splitInits == inits0 + 1,
        "the re-init re-armed the capture and cleared the latch");
  CHECK(camSplitCancel(&camSplitSnap), "cancel");
  camSplitEnable(false);
  CHECK(camConfig(PIXFORMAT_GRAYSCALE, 13333333, OV3660_PID) == ESP_OK && splitInits == inits0 + 2,
        "a grey init resets too");

  /* not split: another sensor, the PSRAM DMA mode */
  camSplitEnable(true);
  CHECK(camConfig(PIXFORMAT_YUV422, 13333333, OV2640_PID) == ESP_OK, "OV2640 init");
  camSplitInitInfo(&info);
  CHECK(!info.active && !split.on && cam.fb_size == 38400, "the OV2640: the driver's YUYV, no split");
  CHECK(camConfig(PIXFORMAT_YUV422, 16000000, OV3660_PID) == ESP_OK, "16 MHz init");
  camSplitInitInfo(&info);
  CHECK(!info.active && !split.on && cam.fb_size == 38400, "the PSRAM DMA mode (16 MHz): no split");
  /* an unarmed copy with the split off never touches the core */
  CHECK(camSplitRequest(&camSplitSnap, slot), "request");
  memset(slot, 0x5A, sizeof(slot));
  camSplitEnable(false);
  CHECK(camConfig(PIXFORMAT_GRAYSCALE, 13333333, OV3660_PID) == ESP_OK, "grey");
  uint8_t y8[19200];
  memset(y8, 9, sizeof(y8));
  CHECK(camFrame(0, y8, 2000000) == 0 && slot[0] == 0x5A && camSplitSnap.state == CAM_SPLIT_REQUESTED,
        "grey with a split request pending: no copy, no latch");
  camSplitCancel(&camSplitSnap);

  printf("test_cam_split_wrap: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
