#pragma once
/* Host stub of esp32-camera 2.1.0's target/private_include/ll_cam.h: the cam_obj_t fields cam_split.c and the test's
 * model of cam_hal.c use, with their 2.1.0 names and types (the converter off: the bytes-per-pixel fields uint8_t); the
 * queues, task and interrupt handles are left out, so the layout is not the drone's (the build reads the real header). */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_NOT_SUPPORTED 0x106
typedef enum { PIXFORMAT_RGB565, PIXFORMAT_YUV422, PIXFORMAT_YUV420, PIXFORMAT_GRAYSCALE, PIXFORMAT_JPEG } pixformat_t;
#define OV2640_PID 0x26
#define OV3660_PID 0x3660
typedef struct {
  uint8_t *buf;
  size_t len;
  size_t width, height;
  pixformat_t format;
  struct timeval timestamp;
} camera_fb_t;
typedef struct {
  camera_fb_t fb;
  uint8_t en;
  void *dma;
  size_t fb_offset;
} cam_frame_t;
typedef struct {
  uint32_t dma_bytes_per_item;
  uint32_t dma_buffer_size;
  uint32_t dma_half_buffer_size;
  uint32_t dma_half_buffer_cnt;
  uint32_t dma_node_buffer_size;
  uint32_t dma_node_cnt;
  uint32_t frame_copy_cnt;
  void *dma;
  uint8_t *dma_buffer;
  cam_frame_t *frames;
  uint8_t jpeg_mode;
  uint8_t vsync_pin;
  uint8_t vsync_invert;
  uint32_t frame_cnt;
  uint32_t recv_size;
  bool swap_data;
  bool psram_mode;
  uint16_t width;
  uint16_t height;
  uint8_t in_bytes_per_pixel;
  uint8_t fb_bytes_per_pixel;
  uint32_t fb_size;
} cam_obj_t;
