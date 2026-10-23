/* Native QVGA uses the driver's standard raw 20 MHz profile (~11.1 fps).
 * The PLL, capture edge, saved NVS and flight-control gains are untouched. */
#include <stdbool.h>
#include "esp_camera.h"
void __real_cam_start(void);
void __wrap_cam_start(void);
static bool timingOk;
bool cameraVgaTimingOk(void) { return timingOk; }
void __wrap_cam_start(void) {
  sensor_t *s = esp_camera_sensor_get();
  timingOk = s && s->id.PID == OV3660_PID && s->status.framesize == FRAMESIZE_QVGA;
  __real_cam_start();
}
