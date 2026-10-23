/*
 * ESP-Drone Firmware
 *
 * usb_diag.c - Line-based diagnostics console on the USB-C port
 *
 * Commands (one per line):
 *   help
 *   status
 *   stream <hz>            0 stops; 1..100 sends 'S' records (and keeps the camera out of its ground standby)
 *   measure <ms>           stationary statistics, motors inhibited
 *   cal show | cal set <x_g> <y_g> | cal save | cal clear
 *   trim show | trim set <roll> <pitch> | trim save | trim clear
 *   atrim show | atrim set <roll_deg> <pitch_deg> | atrim save | atrim clear
 *   facc show | facc set <x_g> <y_g> | facc save | facc clear   (in-flight acc offset)
 *   gtc show | gtc set <kx> <ky> <kz> [<knee_C>] | gtc on|off | gtc save | gtc clear | gtc hold <s>
 *   atc show | atc set <kx> <ky> <kz> [<knee_C>] | atc on|off | atc save | atc clear | atc latch
 *   cool [<s> [<pwm>]] | cool stop
 *                          ground prop-wash cooling spin (ground_cool.h, docs/autonomy-a2b.md 36.3): all four
 *                          motors at one PWM (default 12000, 1000..20000; lift-off needs >= 30000) for s seconds
 *                          (default 20, max 40); IDLE, resting on the ground, battery >= vmin; aborts on tilt,
 *                          gyro, acc, kill, an ARM; 'cool' alone prints the state and the IMU temperature
 *                          accelerometer zero temperature compensation (accel_tempco.h, docs/autonomy-a2b.md 36):
 *                          k in mg per C, the same contract as gtc; 'latch' takes the resting reading as the
 *                          check's reference now (bench; the ARM and the cooling spin do it on their own)
 *                          gyro temperature compensation (gyro_tempco.h, docs/autonomy-a2b.md 19.1): live
 *                          until 'gtc save'; motors must be stopped; 'hold' (RAM, <= 300 s) keeps the ground
 *                          re-zero off for a bench cooling test, ended by any thrust or an active autonomy;
 *                          'show' prints the post-flight re-zero (A) and the logged ones (Z) before its OK line
 *   flowfit [robust|lite|plain]
 *                          the flow fit's variant (optical_flow.h, docs/autonomy-a2b.md 19.4), saved, used from the
 *                          next frame: robust (default), lite and plain are the CPU fallback's steps; the reply
 *                          shows the exposure steps refused / corrected and the camera loop's cost
 *   flowfit prof [reset]   the camera loop's cost per stage (camera_flow.h cameraFlowStageStats, docs/autonomy-a2b.md
 *                          30): avg / min / max us of the downsample, coarse and fine fit, copies, snapshot, rest
 *                          and total over the frames since the boot or 'reset'
 *   mki show | mki set <0..0.2> | mki save | mki clear
 *                          the attitude filter's in-flight gyro-bias learning gain twoKiAir (sensfusion6.h,
 *                          docs/autonomy-a2b.md 19.2; default 0.040, 0 = the filter before it); motors stopped
 *   bypass [gtc|mki|rec|flow|all on|off]
 *                          loop-rate bisect toggles (bench_bypass.h, docs/autonomy-bench.md A4b): each switches one
 *                          per-loop addition of a2b section 19 back to 14bcfc6's code; RAM only (0 at boot), motors
 *                          stopped, autonomy idle; shown in status, gtc and auto prof as bypass=0x..
 *   yawlock [on|off]       heading hold, saved; motors must be stopped to change
 *   hold [on|off]          camera drift hold, saved; motors must be stopped to change
 *   holdk show | holdk set <1..3> | holdk save | holdk clear
 *                          the drift hold's largest gain scale with the scene distance (camera_hold.h,
 *                          docs/autonomy-a2b.md 20; default 2.5, 1 = the hold before it): 'set' is live until
 *                          'holdk save'; motors stopped, autonomy idle; 'show' prints the live scale, its target and
 *                          the latched G
 *   cam [on|off]           start the camera at boot (default off), saved; takes effect after reboot
 *   crash [clear]          last panic (PC, backtrace, abort message) and recent warnings
 *   flights [clear]        per-flight summaries and the pre-reset trace (RTC memory, survive brownouts)
 *   camvz                  camera height observer in shadow mode (cam_vz_glue.h): live estimate, counters,
 *                          the current sortie's flight-summary fields and the stabilizer-task cost
 *   live                   one line: thrust, motors, battery and 3.3 V rail (1 kHz monitor), Wi-Fi counters
 *   gcap [<thrust>|off]    ground-test thrust cap, saved; only lowers thrust; motors must be stopped
 *   reboot                 software restart (RTC logs survive); motors must be stopped
 *   rail                   bench check of the GPIO9 rail meter (drives the pad low, then high again)
 *   net                    phone join / app timing, UDP counters, packets per CRTP port
 *   tkoff [camsb|camgnd|txlo|wifiq on|off] | tkoff camms|camcold|camgndms|camlead <ms> | tkoff camwake <s>
 *                          take-off load shedding and the camera's ground standby (docs/autonomy-a2b.md 17.1),
 *                          saved; camwake (RAM) holds the camera awake for <s> s, 0 ends the hold
 *   tkoff camclk 20|13     the camera's input clock (17.2), saved, used from the next boot
 *   pwm [<hz>|stock]       motor PWM frequency (10000..100000, stock 15000), saved; motors must be stopped
 *   soft [on|off]          take-off thrust ramp after a ground stop (default on), saved; motors must be stopped
 *   plock [on|off]         restart lock: no thrust after a non-power-on reset until the battery is replugged
 *                          (default on), saved; 'off' also releases a current lock
 *   rec info | rec dump | rec clear
 *   autokey <32 hex digits> | autokey clear | autokey
 *                          autonomy-link key (docs/autonomy-link.md), saved; never echoed (the
 *                          reply shows its fingerprint); takes effect after reboot; motors must be stopped
 *   auto                   autonomy link: key, port, session, counters, photo state; autonomy state
 *   auto status | auto get | auto rtc | auto prof [reset]
 *                          autonomy (docs/autonomy-a2b.md): state machine and inputs, settings with their
 *                          source, the reset-lock RTC record, the 1 kHz loop's cost
 *   auto set <key> <value> | auto reset <key> | auto on|off | auto bench sim|imu
 *                          autonomy settings (NVS, never over Wi-Fi) and the BENCH input source (RAM); IDLE only
 *   auto unlock            clears the autonomy reset-lock as a battery power-on does (user decision
 *                          2026-10-12, evening); USB only, IDLE with the motors stopped; no other lock changes
 *   mguard [status|on|off] motor watchdog (motor_guard.h): cuts the motors when the stabilizer stops
 *                          with any duty > 0; default on, saved; motors must be stopped to change
 *   mguard test hang|imu <ms> | mguard test panic
 *                          bench fault injection (props off; refused unless 1 <= gcap <= 20000):
 *                          the stabilizer busy-waits / gets no IMU data for <ms> (1..500), or abort()
 *
 * While autonomy is active (any state but IDLE) every command that writes
 * NVS, inhibits the motors or starts a bench mode answers "ERR busy: autonomy
 * active" (docs/autonomy-a2b.md section 7.3); the store's flash interlock
 * refuses the write in any case.
 *
 * Replies start with "OK " or "ERR ". Record lines ('S' stream, 'R' dump)
 * end with "*XX", the XOR of all preceding characters in the line.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "hal/usb_serial_jtag_ll.h"

#include "config.h"
#include "stm32_legacy.h"
#include "static_mem.h"
#include "usb_diag.h"
#include "flight_recorder.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "imu_level_cal.h"
#include "imu_level_cal_store.h"
#include "torque_trim.h"
#include "controller_pid.h"
#include "motors.h"
#include "crtp_commander.h"
#include "commander.h"
#include "system.h"
#include "pm_esplane.h"
#include "wifi_esp32.h"
#include "camera_flow.h"
#include "camera_hold.h"
#include "crash_log.h"
#include "flight_rtc_log.h"
#include "stabilizer.h"
#include "esp_heap_caps.h"
#include "crtp.h"
#include "adc_esp32.h"
#include "reg_snap.h"
#include "takeoff_aux.h"
#include "att_trim_auto.h"
#include "auto_link.h"
#include "autonomy.h"
#include "ground_cool_glue.h"
#include "motor_guard.h"
#include "cam_vz_glue.h"
#include "esp_sleep.h"
#include "sensfusion6.h"
#include "bench_bypass.h"
#include "hal/wdt_hal.h"
#include "soc/rtc_cntl_reg.h"
#define DEBUG_MODULE "USBDIAG"
#include "debug_cf.h"

#define LINE_MAX_LEN       128
#define OUT_MAX_LEN        1536   /* PSRAM; 'status' reached ~950 characters before the camera fields of 17.1-17.2 */
#define TX_TIMEOUT_US      20000
#define LOOP_PERIOD_MS     5
#define MEASURE_MIN_MS     200
#define MEASURE_MAX_MS     30000
/* Stillness thresholds for a drone resting on a table (MPU6050, DLPF 42 Hz). */
#define STILL_GYRO_STD_DPS 1.0f
#define STILL_ACC_STD_G    0.02f
/* Motor watchdog fault tests (docs/autonomy-a2.md 7.3): bench only. */
#define MGUARD_TEST_MAX_MS 500
#define MGUARD_TEST_GCAP   20000

/* Every command handler is its own call: inlined into the dispatcher (GCC
 * inlines static functions called once), their locals and argument blocks
 * merged into usbDiagTask's frame (1552 B at 82bcefb), live under every
 * command - under 'status' that frame plus status's own (1696 B) and
 * vfprintf's left no room for an interrupt frame. */
#define OUT_OF_LINE __attribute__((noinline))

#define RECORD_FIELDS "idx,tickMs,flightId,flags,thrust,rollDes_cdeg,pitchDes_cdeg,roll_cdeg,pitch_cdeg,yaw_cdeg," \
                      "gx_ddps,gy_ddps,gz_ddps,ax_mg,ay_mg,az_mg,m1,m2,m3,m4,rollI_cdps,pitchI_cdps,rollRateI,pitchRateI," \
                      "yawRateDes_ddps,vbat_mv,flowSeq,flowOk,flowLum,flowTexture,flowResidual_dg,flowDt_100us,flowAge_ms," \
                      "flowDx_cpx,flowDy_cpx,flowScale_e4,flowRot_e4,flowGx_e4,flowGy_e4,flowGz_e4," \
                      "holdFlags,holdRoll_cdeg,holdPitch_cdeg,rawThrust,linkAge_ms,linkFlags," \
                      "cvH_mm,cvV_mms,cvBias_cmg,cvGain_dpm,cvFlags,cvSigV_mms," \
                      "autoState,autoReason,vzPhase,vzH_mm,vzV_mms,vzThrust,autoFlags," \
                      "mvState,mvRefLat_e4,mvRefFwd_e4,mvVRef_mms,mvYawOff_cdeg," \
                      "imuTemp_cC,gtcCorrX_mdps,gtcCorrY_mdps,mkiBiasX_mdps,mkiBiasY_mdps,flowGain_e3,flowSat_pct," \
                      "flowExpGate,atcCorrZ_cmg"

STATIC_MEM_TASK_ALLOC(usbDiagTask, USBDIAG_TASK_STACKSIZE);

/* The output line: PSRAM (allocated by usbDiagInit(); internal RAM pays for
 * the interactive hover's IRAM, docs/autonomy-a2b.md 14.7). Only this
 * console's task writes it, never with the cache off. NULL: no output. */
static char *outBuf;
static char line[LINE_MAX_LEN];
static size_t lineLen;
static bool lineOverflow;
static bool hostGone;
static uint32_t droppedLines;
static uint32_t streamPeriodMs;
static uint32_t nextStreamMs;
static uint32_t streamSeq;
/* NULL until the level offset is changed from this console. */
static const char *levelSourceOverride;

/* Non-blocking for the caller's purposes: waits at most TX_TIMEOUT_US for a
 * reading host, then drops the rest of the line and fails fast until the
 * host reads again. */
static bool usbWrite(const char *buf, size_t len)
{
  size_t sent = 0;
  int64_t lastProgress = esp_timer_get_time();

  while (sent < len) {
    uint32_t n = usb_serial_jtag_ll_write_txfifo((const uint8_t *)buf + sent, len - sent);
    if (n > 0) {
      sent += n;
      lastProgress = esp_timer_get_time();
      hostGone = false;
      continue;
    }
    usb_serial_jtag_ll_txfifo_flush();
    if (hostGone || esp_timer_get_time() - lastProgress > TX_TIMEOUT_US) {
      hostGone = true;
      droppedLines++;
      return false;
    }
    vTaskDelay(1);
  }
  usb_serial_jtag_ll_txfifo_flush();
  return true;
}

/* Formats onto the output line at outBuf[n] and returns the new length
 * (n unchanged on a format error, at most OUT_MAX_LEN - 2); nothing is sent
 * until outEnd(). */
static int outCatV(int n, const char *fmt, va_list ap)
{
  if (outBuf == NULL || n < 0 || n >= OUT_MAX_LEN - 2) {
    return n;
  }
  int k = vsnprintf(outBuf + n, OUT_MAX_LEN - 1 - n, fmt, ap);
  if (k < 0) {
    return n;
  }
  return (k > OUT_MAX_LEN - 2 - n) ? OUT_MAX_LEN - 2 : n + k;
}

/* Sends the line built by outCat() calls, with its newline. */
static void outEnd(int n)
{
  if (outBuf == NULL || n < 0) {
    return;
  }
  outBuf[n++] = '\n';
  usbWrite(outBuf, n);
}

/* A long reply ('status') is built in several outCat() calls, one line and
 * one write as before: a single call with ~100 arguments puts its whole
 * argument block (~0.5 KB) in the caller's frame on this task's 5 KB stack,
 * under newlib's vfprintf (~1.5 KB deep with %f). */
static int outCat(int n, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  n = outCatV(n, fmt, ap);
  va_end(ap);
  return n;
}

static void out(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  const int n = outCatV(0, fmt, ap);
  va_end(ap);
  if (n > 0) {
    outEnd(n);
  }
}

static void outRecord(char tag, uint32_t idx, const flightRecord_t *r)
{
  if (outBuf == NULL) {
    return;
  }
  int n = snprintf(outBuf, OUT_MAX_LEN,
                   "%c,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%d,%d,%d,%d,"
                   "%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%u,%u,%u,%d,%d,%d,%u,%u,%u,"
                   "%u,%u,%u,%d,%d,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d",
                   tag, (unsigned)idx, (unsigned)r->tickMs, (unsigned)r->flightId, (unsigned)r->flags,
                   (unsigned)r->thrust, r->rollDes, r->pitchDes, r->roll, r->pitch, r->yaw,
                   r->gyro[0], r->gyro[1], r->gyro[2], r->acc[0], r->acc[1], r->acc[2],
                   (unsigned)r->motor[0], (unsigned)r->motor[1], (unsigned)r->motor[2], (unsigned)r->motor[3],
                   r->rollI, r->pitchI, r->rollRateI, r->pitchRateI,
                   r->yawRateDes, (unsigned)r->vbatMv, (unsigned)r->flowSeq, (unsigned)r->flowOk,
                   (unsigned)r->flowLum, (unsigned)r->flowTexture, (unsigned)r->flowResidual,
                   (unsigned)r->flowDt, (unsigned)r->flowAgeMs, r->flowDx, r->flowDy, r->flowScale,
                   r->flowRot, r->flowGyro[0], r->flowGyro[1], r->flowGyro[2],
                   (unsigned)r->holdFlags, r->holdRoll, r->holdPitch,
                   (unsigned)r->rawThrust, (unsigned)r->linkAgeMs, (unsigned)r->linkFlags,
                   r->cvH, r->cvV, r->cvBias, (unsigned)r->cvGain, (unsigned)r->cvFlags, (unsigned)r->cvSigV,
                   (unsigned)r->autoState, (unsigned)r->autoReason, (unsigned)r->vzPhase, r->vzHmm, r->vzVmms,
                   (unsigned)r->vzThrust, (unsigned)r->autoFlags, (unsigned)r->mvState, r->mvRefLat, r->mvRefFwd,
                   r->mvVRef, r->mvYawOff, r->imuTemp, r->gtcCorrX, r->gtcCorrY, r->mkiBiasX, r->mkiBiasY,
                   (int)r->flowGain, (unsigned)(r->flowSat & 0x7Fu), (unsigned)(r->flowSat >> 7), r->atcCorrZ);
  if (n < 0 || n > OUT_MAX_LEN - 5) {
    return;
  }
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) {
    cs ^= (uint8_t)outBuf[i];
  }
  n += snprintf(outBuf + n, OUT_MAX_LEN - n, "*%02X\n", cs);
  usbWrite(outBuf, n);
}

static const char *levelSource(void)
{
  if (levelSourceOverride) {
    return levelSourceOverride;
  }
  float x, y;
  int loadResult;
  sensorsMpu6050GetLevelOffset(&x, &y, &loadResult);
  if (loadResult == ESP_OK) {
    return "nvs";
  }
  return (loadResult == ESP_ERR_NVS_NOT_FOUND) ? "none" : "rejected";
}

/* While autonomy is active: no NVS write (a flash write stops both cores),
 * no motor inhibit, no bench mode (docs/autonomy-a2b.md section 7.3). */
static bool autoBusy(void)
{
  if (autonomyActive()) {
    out("ERR busy: autonomy active");
    return true;
  }
  return false;
}

/* A console change a sortie could read (a live setting, a bench mode, an NVS
 * write) is made inside the flash interlock, taken before anything changes
 * (docs/autonomy-a2b.md section 7.3): an ARM that came first (taken, or
 * waiting in the mailbox) makes it answer busy with nothing changed, and an
 * ARM taken while it runs is refused by G1, so no sortie starts with half a
 * change. Inside, the NVS write comes first and the live change only after it
 * succeeded. Every true is paired with one changeEnd(). */
static bool changeBegin(void)
{
  if (autoBusy()) {
    return false;
  }
  if (!autonomyFlashBegin()) {
    out("ERR busy: autonomy active");
    return false;
  }
  return true;
}

static void changeEnd(void)
{
  autonomyFlashEnd();
}

/* The store's own interlock refused the write (an ARM was posted meanwhile;
 * that ARM is refused, and nothing was written). */
static bool storeBusy(esp_err_t err)
{
  if (err == ESP_ERR_INVALID_STATE) {
    out("ERR busy: autonomy active");
    return true;
  }
  return false;
}

static bool motorsIdle(const char **why)
{
  if (autonomyActive()) {
    *why = "autonomy active";
    return false;
  }
  flightRecord_t r;
  if (!flightRecorderGetLatest(&r)) {
    *why = "stabilizer not running yet";
    return false;
  }
  if (r.thrust > 0) {
    *why = "thrust is requested; set throttle to zero / disconnect the app";
    return false;
  }
  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    if (motorsGetRatio(i) != 0) {
      *why = "a motor output is nonzero";
      return false;
    }
  }
  return true;
}

static bool motorsAllZero(void)
{
  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    if (motorsGetRatio(i) != 0) {
      return false;
    }
  }
  return true;
}

/* Hold every motor at zero; true once the outputs read back as zero. */
static bool inhibitBegin(void)
{
  motorsSetInhibit(true);
  vTaskDelay(M2T(20));
  return motorsAllZero();
}

/* Release the inhibit only after RPYT thrust is locked, so motors can only
 * restart after the pilot sends a fresh zero-throttle command. */
static void inhibitEnd(void)
{
  crtpCommanderRpytLockThrust();
  motorsSetInhibit(false);
}

OUT_OF_LINE static void cmdHelp(void)
{
  out("OK commands: help | status | stream <hz 0..100> | measure <ms %d..%d> | "
      "cal show | cal set <x_g> <y_g> | cal save | cal clear | "
      "trim show | trim set <roll> <pitch> | trim save | trim clear | atrim show|set|save|clear|learn on|off | facc show|set|save|clear | gtc show|set <kx> <ky> <kz> [<knee>]|on|off|save|clear|hold <s> | atc show|set <kx> <ky> <kz> [<knee>]|on|off|save|clear|latch | cool [<s> [<pwm>]]|stop | mki show|set <0..0.2>|save|clear | flowfit [robust|lite|plain] | flowfit prof [reset] | bypass [gtc|mki|rec|flow|all on|off] | yawlock [on|off] | hold [on|off] | holdk show|set <1..3>|save|clear | cam [on|off] | crash [clear] | flights [clear] | camvz | live | gcap [<thrust>|off] | reboot [sleep|rwdt|swd] | regs | tkoff [camsb|camgnd|txlo|wifiq on|off] | tkoff camms|camcold|camgndms|camlead <ms> | tkoff camwake <s> | tkoff camclk 20|13 | rail | net | pwm [<hz>|stock] | soft [on|off] | plock [on|off] | "
      "rec info | rec dump | rec clear | autokey <32 hex>|clear | auto | auto status|get|rtc|prof [reset] | "
      "auto set <key> <value> | auto reset <key> | auto on|off | auto bench sim|imu | auto unlock | "
      "mguard [on|off] | mguard test hang|imu <ms>|panic",
      MEASURE_MIN_MS, MEASURE_MAX_MS);
}

static const char *resetReasonName(esp_reset_reason_t reason)
{
  switch (reason) {
    case ESP_RST_POWERON:  return "poweron";
    case ESP_RST_EXT:      return "external";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT:      return "wdt";
    case ESP_RST_DEEPSLEEP:return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "other";
  }
}

/* One line, its fields in the order tools/espfly_diag.py and the bench
 * notes know, built in short pieces (outCat) with the getters' structs in
 * block scopes (82bcefb's 97-argument out() and the inlined handlers above
 * it overflowed this stack, docs/autonomy-a2b.md 26). diagStackFree, last: the least free
 * stack this console task has had since boot, bytes. */
OUT_OF_LINE static void cmdStatus(void)
{
  const esp_app_desc_t *app = esp_app_get_description();
  const esp_reset_reason_t reset = esp_reset_reason();
  int n;
  {
    uint16_t vNow, vMinNow, railNow, railMinNow;
    flightRtcLogLive(&vNow, &vMinNow, &railNow, &railMinNow);
    n = outCat(0, "OK status fw=%s idf=%s built=%s_%s up_ms=%u reset=%s(%d) vbat=%.2f vbatMin=%.2f vbatFastMin=%.2f "
               "railFastMin=%.2f gcap=%u pwm=%u soft=%d plock=%d rlocked=%d ",
               app->version, app->idf_ver, app->date, app->time, (unsigned)T2M(xTaskGetTickCount()),
               resetReasonName(reset), (int)reset, (double)pmGetBatteryVoltage(), (double)pmGetBatteryVoltageMin(),
               flightRtcLogLiveVbatMin() / 1000.0, railMinNow / 1000.0, stabilizerGetThrustCap(),
               (unsigned)motorsGetPwmFreq(), stabilizerSoftStartEnabled(), stabilizerRestartLockEnabled(),
               stabilizerRestartLocked());
  }
  {
    float x, y;
    int loadResult;
    sensorsMpu6050GetLevelOffset(&x, &y, &loadResult);
    uint32_t rezeros;
    float gDrift[3];
    sensorsMpu6050GetGyroRezero(&rezeros, gDrift);
    n = outCat(n, "armed=%d inhibit=%d calibrated=%d accScale=%.5f level=%.5f,%.5f src=%s gyroRezeros=%u "
               "gyroDrift=%.2f,%.2f,%.2f ",
               systemIsArmed(), motorsIsInhibited(), sensorsAreCalibrated(), (double)sensorsMpu6050GetAccScale(),
               (double)x, (double)y, levelSource(), (unsigned)rezeros, (double)gDrift[0], (double)gDrift[1],
               (double)gDrift[2]);
  }
  {
    float trimRoll, trimPitch, attR, attP;
    int trimLoad, attLoad;
    controllerPidGetTorqueTrim(&trimRoll, &trimPitch, &trimLoad);
    controllerPidGetAttitudeTrim(&attR, &attP, &attLoad);
    float faccX, faccY;
    sensorsMpu6050GetFlightAccOffset(&faccX, &faccY);
    float holdR, holdP, holdCorrR, holdCorrP;
    bool holdEng;
    uint64_t holdT;
    cameraFlowGetHold(&holdR, &holdP, &holdEng, &holdT);
    controllerPidGetCameraCorrection(&holdCorrR, &holdCorrP);
    n = outCat(n, "trim=%.0f,%.0f atrim=%.2f,%.2f facc=%.4f,%.4f yawLock=%d hold=%d holdEng=%d holdCorr=%.2f,%.2f ",
               (double)trimRoll, (double)trimPitch, (double)attR, (double)attP, (double)faccX, (double)faccY,
               controllerPidGetYawLock(), cameraFlowHoldEnabled(), holdEng, (double)holdCorrR, (double)holdCorrP);
  }
  unsigned flightId;
  {
    flightRecord_t r;
    const bool haveState = flightRecorderGetLatest(&r);
    flightId = haveState ? (unsigned)r.flightId : 0u;
    n = outCat(n, "thrust=%u motors=%d,%d,%d,%d roll=%.2f pitch=%.2f yaw=%.2f ",
               haveState ? (unsigned)r.thrust : 0u,
               motorsGetRatio(0), motorsGetRatio(1), motorsGetRatio(2), motorsGetRatio(3),
               haveState ? r.roll / 100.0 : NAN, haveState ? r.pitch / 100.0 : NAN, haveState ? r.yaw / 100.0 : NAN);
  }
  {
    crashInfo_t crash;
    const bool haveCrash = crashLogGet(&crash);
    n = outCat(n, "prio=%d rec=%u/%u flight=%u dropped=%u crash=%d cam=%s camGnd=%d camPid=0x%x camFps=%.1f ",
               commanderGetActivePriority(), (unsigned)flightRecorderCount(), (unsigned)flightRecorderCapacity(),
               flightId, (unsigned)droppedLines, haveCrash,
               cameraFlowStateName(), cameraFlowGroundStandby(), (unsigned)cameraFlowSensorPid(),
               (double)cameraFlowFps());
  }
  {
    uint32_t camFrames, camTimeouts, camRestarts, camAgeMs;
    cameraFlowStats(&camFrames, &camTimeouts, &camRestarts, &camAgeMs);
    uint32_t camBad, camStack, flowStack;
    cameraFlowHealth(&camBad, &camStack, &flowStack);
    n = outCat(n, "camFrames=%u camTimeouts=%u camRestarts=%u camAgeMs=%u camBad=%u camStackFree=%d flowStackFree=%d ",
               (unsigned)camFrames, (unsigned)camTimeouts, (unsigned)camRestarts, (unsigned)camAgeMs,
               (unsigned)camBad, camStack == UINT32_MAX ? -1 : (int)camStack,
               flowStack == UINT32_MAX ? -1 : (int)flowStack);
  }
  {
    cameraFlowClockStats_t clk;
    cameraFlowClockStats(&clk);
    n = outCat(n, "camClk=%.2f camClkFallback=%d camClkChk=%s camProcAvgUs=%u camProcMaxUs=%u camSkips=%u "
               "camWinFps=%.2f camGyroOffUs=%u camNavLagUs=%u ",
               clk.xclkHz / 1e6, clk.fellBack, clk.tooSlow ? "slow" : (clk.pending ? "wait" : "ok"),
               (unsigned)clk.procAvgUs, (unsigned)clk.procMaxUs, (unsigned)clk.skips,
               (double)clk.windowFps, (unsigned)clk.gyroOffsetUs, (unsigned)cameraFlowMotionLagUsV);
  }
  {
    stabilizerBenchTelemetry_t tel;
    const bool haveTel = stabilizerGetBenchTelemetry(&tel);
    n = outCat(n, "camPix=%s loops=%u maxGapUs=%u gaps1500=%u gaps2500=%u maxExecUs=%u heapInt=%u ",
               cameraFlowPhotoPixFmt() == CAMERA_FLOW_PIXFMT_JPEG ? "jpeg-vga" :
               (cameraFlowPhotoPixFmt() == CAMERA_FLOW_PIXFMT_YUYV ? "yuyv-split" : "gray"),
               haveTel ? (unsigned)tel.loop_count : 0u, haveTel ? (unsigned)tel.max_loop_gap_us : 0u,
               haveTel ? (unsigned)tel.loop_gap_over_1500us : 0u, haveTel ? (unsigned)tel.loop_gap_over_2500us : 0u,
               haveTel ? (unsigned)tel.max_loop_exec_us : 0u,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  }
  {
    /* the colour split (docs/autonomy-a2b.md 28): its boot check and the photo handshake's counters */
    cameraFlowSplitStats_t sp;
    cameraFlowSplitStats(&sp);
    n = outCat(n, "camSplit=%s camSplitArm=%u camSplitDone=%u camSplitRearm=%u camSplitCancel=%u camSplitRestart=%u ",
               cameraFlowSplitName(sp.split), (unsigned)sp.arm, (unsigned)sp.done, (unsigned)sp.rearm,
               (unsigned)sp.cancel, (unsigned)(sp.restart + sp.resets));
  }
  {
    rpytLinkStats_t link;
    crtpCommanderRpytGetLinkStats(&link);
    const uint32_t nowMs = T2M(xTaskGetTickCount());
    n = outCat(n, "pkts=%u pktInvalid=%u pktAgeMs=%u pktMaxGapMs=%u pktGaps300=%u relocks=%u rawThrust=%u locked=%d ",
               (unsigned)link.packets, (unsigned)link.invalidPackets,
               link.packets ? (unsigned)(nowMs - link.lastPacketMs) : 0u, (unsigned)link.maxGapMs,
               (unsigned)link.gapsOver300ms, (unsigned)link.relocksWhileThrust, (unsigned)link.lastRawThrust,
               link.thrustLocked);
  }
  {
    wifiLinkStats_t wifi;
    wifiGetLinkStats(&wifi);
    crashBoot_t boots[1];
    uint32_t totalBoots = 0;
    crashLogBoots(boots, 1, &totalBoots);
    n = outCat(n, "staJoins=%u staLeaves=%u rxPkts=%u rxCksumErr=%u rxQDrops=%u rxErr=%u wifiTxDbm=%.2f boots=%u "
               "mguard=%s ",
               (unsigned)wifi.staJoins, (unsigned)wifi.staLeaves, (unsigned)wifi.rxPackets,
               (unsigned)wifi.cksumErrors, (unsigned)wifi.queueDrops, (unsigned)wifi.rxErrors, wifi.txPowerQdbm / 4.0,
               (unsigned)totalBoots, motorGuardTripped() ? "tripped" : (motorGuardEnabled() ? "on" : "off"));
  }
  {
    /* the camera height observer's offset drift rate T2 (docs/autonomy-a2b.md 29): its setting and this sortie */
    camVzGlueT2_t t2;
    camVzGlueT2(&t2);
    n = outCat(n, "t2k=%.2f t2r=%+.3f t2Cfg=%s t2=%s t2OnS=%.1f t2bR=%+.3f ", (double)t2.kR, (double)t2.priorMgS,
               t2.cfgBad ? "bad" : "ok", t2.kR <= 0.0f ? "off" : (!t2.t2.on ? "wait" : (t2.t2.drifting ? "on" : "frozen")),
               t2.t2.onUs / 1e6, (double)t2.t2.t2RateMgS);
  }
  {
    /* the accelerometer compensation (docs/autonomy-a2b.md 36): on, the z correction subtracted now (mg) */
    float ta[3];
    sensorsMpu6050GetAccTcCorr(ta);
    n = outCat(n, "atc=%d atcZ=%+.2f ", (int)sensorsMpu6050AccTcOn(), (double)(ta[2] * 1000.0f));
  }
  n = outCat(n, "imuTempC=%.2f bypass=0x%02x diagStackFree=%u", (double)sensorsMpu6050GetTempC(),
             (unsigned)benchBypassMask, (unsigned)uxTaskGetStackHighWaterMark(NULL));
  outEnd(n);
}

OUT_OF_LINE static void cmdStream(int argc, char **argv)
{
  int hz = (argc >= 2) ? atoi(argv[1]) : 0;
  if (hz < 0 || hz > FLIGHT_RECORDER_RATE_HZ) {
    out("ERR stream: rate must be 0..%d Hz", FLIGHT_RECORDER_RATE_HZ);
    return;
  }
  streamPeriodMs = (hz == 0) ? 0 : (1000 / hz);
  nextStreamMs = T2M(xTaskGetTickCount());
  cameraFlowGroundBenchHold(hz > 0);   /* the stream's flow fields need the camera awake (docs/autonomy-a2b.md 17.1) */
  out("OK stream hz=%d fields=%s", hz, RECORD_FIELDS);
}

OUT_OF_LINE static void cmdMeasure(int argc, char **argv)
{
  int ms = (argc >= 2) ? atoi(argv[1]) : 3000;
  const char *why;

  if (ms < MEASURE_MIN_MS || ms > MEASURE_MAX_MS) {
    out("ERR measure: duration must be %d..%d ms", MEASURE_MIN_MS, MEASURE_MAX_MS);
    return;
  }
  if (autoBusy()) {
    return;
  }
  if (!motorsIdle(&why)) {
    out("ERR measure: %s", why);
    return;
  }
  if (!inhibitBegin()) {
    inhibitEnd();
    out("ERR measure: motor outputs did not read back as zero");
    return;
  }

  sensorsMpu6050AccumStart();

  double sumR = 0, sumP = 0, sumR2 = 0, sumP2 = 0;
  uint32_t n = 0;
  unsigned maxThrust = 0;
  bool motorsStayedZero = true;
  TickType_t start = xTaskGetTickCount();
  TickType_t lastWake = start;
  while ((xTaskGetTickCount() - start) < M2T(ms)) {
    vTaskDelayUntil(&lastWake, M2T(10));
    flightRecord_t r;
    if (flightRecorderGetLatest(&r)) {
      double roll = r.roll / 100.0, pitch = r.pitch / 100.0;
      sumR += roll;
      sumP += pitch;
      sumR2 += roll * roll;
      sumP2 += pitch * pitch;
      n++;
      if (r.thrust > maxThrust) {
        maxThrust = r.thrust;
      }
    }
    motorsStayedZero &= motorsAllZero();
  }

  sensorsImuStats_t st;
  bool haveStats = sensorsMpu6050AccumStop(&st);
  inhibitEnd();

  if (!haveStats || n == 0) {
    out("ERR measure: no sensor samples");
    return;
  }

  float ox, oy;
  int loadResult;
  sensorsMpu6050GetLevelOffset(&ox, &oy, &loadResult);

  /* Attitude the estimator converges to for this mean acceleration (level
   * offset applied, Kconfig alignment assumed zero): same formulas as
   * sensfusion6GetEulerRPY(). */
  float cx = st.accMean[0] - ox, cy = st.accMean[1] - oy, cz = st.accMean[2];
  float cnorm = sqrtf(cx * cx + cy * cy + cz * cz);
  float accRoll = atan2f(cy, cz) * 180.0f / (float)M_PI;
  float accPitch = asinf(cx / cnorm) * 180.0f / (float)M_PI;

  double meanR = sumR / n, meanP = sumP / n;
  double stdR = sqrt(fmax(sumR2 / n - meanR * meanR, 0.0));
  double stdP = sqrt(fmax(sumP2 / n - meanP * meanP, 0.0));

  bool still = true;
  for (int i = 0; i < 3; i++) {
    still &= st.gyroStd[i] <= STILL_GYRO_STD_DPS && st.accStd[i] <= STILL_ACC_STD_G;
  }

  out("OK measure n=%u ms=%d still=%d motorsZero=%d maxThrust=%u "
      "acc=%.5f,%.5f,%.5f accStd=%.5f,%.5f,%.5f "
      "gyro=%.4f,%.4f,%.4f gyroStd=%.4f,%.4f,%.4f gyroBias=%.4f,%.4f,%.4f calibrated=%d "
      "level=%.5f,%.5f accScale=%.5f accRoll=%.3f accPitch=%.3f "
      "estRoll=%.3f estPitch=%.3f estRollStd=%.3f estPitchStd=%.3f",
      (unsigned)st.count, ms, still, motorsStayedZero, maxThrust,
      (double)st.accMean[0], (double)st.accMean[1], (double)st.accMean[2],
      (double)st.accStd[0], (double)st.accStd[1], (double)st.accStd[2],
      (double)st.gyroMean[0], (double)st.gyroMean[1], (double)st.gyroMean[2],
      (double)st.gyroStd[0], (double)st.gyroStd[1], (double)st.gyroStd[2],
      (double)st.gyroBias[0], (double)st.gyroBias[1], (double)st.gyroBias[2], st.gyroBiasFound,
      (double)ox, (double)oy, (double)sensorsMpu6050GetAccScale(), (double)accRoll, (double)accPitch,
      meanR, meanP, stdR, stdP);
}

OUT_OF_LINE static void cmdCal(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  float x, y;
  int loadResult;

  if (strcmp(sub, "show") == 0) {
    sensorsMpu6050GetLevelOffset(&x, &y, &loadResult);
    out("OK cal level=%.5f,%.5f src=%s boot=%s pitchEq=%.3f rollEq=%.3f",
        (double)x, (double)y, levelSource(), esp_err_to_name(loadResult),
        (double)(asinf(x) * 180.0f / (float)M_PI), (double)(atanf(y) * 180.0f / (float)M_PI));
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    bool clear = (strcmp(sub, "clear") == 0);
    if (clear) {
      x = 0.0f;
      y = 0.0f;
    } else {
      if (argc < 4) {
        out("ERR cal set: usage cal set <x_g> <y_g>");
        return;
      }
      char *end1, *end2;
      x = strtof(argv[2], &end1);
      y = strtof(argv[3], &end2);
      if (*end1 != '\0' || *end2 != '\0' || !imuLevelCalOffsetsValid(x, y)) {
        out("ERR cal set: offsets must be numbers within +/-%.2f g", (double)IMU_LEVEL_CAL_MAX_OFFSET_G);
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR cal %s: %s", sub, why);
      return;
    }
    if (!inhibitBegin()) {
      inhibitEnd();
      changeEnd();
      out("ERR cal %s: motor outputs did not read back as zero", sub);
      return;
    }
    const esp_err_t err = clear ? imuLevelCalStoreErase() : ESP_OK;
    if (err != ESP_ERR_INVALID_STATE) {
      sensorsMpu6050SetLevelOffset(x, y);
    }
    inhibitEnd();
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      levelSourceOverride = "live";
      out("ERR cal clear: live offsets zeroed but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    levelSourceOverride = clear ? "none" : "live";
    out("OK cal %s level=%.5f,%.5f src=%s note=estimator_settles_in_10s_power_cycle_before_flight",
        sub, (double)x, (double)y, levelSourceOverride);
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    imuLevelCalRecord_t rec;
    sensorsMpu6050GetLevelOffset(&x, &y, &loadResult);
    imuLevelCalRecordInit(&rec, x, y);
    esp_err_t err = imuLevelCalStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR cal save: %s (live offsets unchanged, stored record may be the previous one)", esp_err_to_name(err));
      return;
    }
    levelSourceOverride = "nvs";
    out("OK cal save level=%.5f,%.5f src=nvs", (double)x, (double)y);
  } else {
    out("ERR cal: unknown subcommand '%s'", sub);
  }
}

OUT_OF_LINE static void cmdTrim(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  float roll, pitch;
  int loadResult;

  if (strcmp(sub, "show") == 0) {
    controllerPidGetTorqueTrim(&roll, &pitch, &loadResult);
    out("OK trim roll=%.0f pitch=%.0f boot=%s", (double)roll, (double)pitch, esp_err_to_name(loadResult));
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    bool clear = (strcmp(sub, "clear") == 0);
    if (clear) {
      roll = 0.0f;
      pitch = 0.0f;
    } else {
      if (argc < 4) {
        out("ERR trim set: usage trim set <roll> <pitch>");
        return;
      }
      char *end1, *end2;
      roll = strtof(argv[2], &end1);
      pitch = strtof(argv[3], &end2);
      if (*end1 != '\0' || *end2 != '\0' || !torqueTrimValuesValid(roll, pitch)) {
        out("ERR trim set: values must be numbers within +/-%.0f", (double)TORQUE_TRIM_MAX);
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR trim %s: %s", sub, why);
      return;
    }
    if (!inhibitBegin()) {
      inhibitEnd();
      changeEnd();
      out("ERR trim %s: motor outputs did not read back as zero", sub);
      return;
    }
    const esp_err_t err = clear ? torqueTrimStoreErase() : ESP_OK;
    if (err != ESP_ERR_INVALID_STATE) {
      controllerPidSetTorqueTrim(roll, pitch);
    }
    inhibitEnd();
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR trim clear: live trim zeroed but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    out("OK trim %s roll=%.0f pitch=%.0f%s", sub, (double)roll, (double)pitch,
        clear ? " stored=erased" : " stored=unchanged");
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    torqueTrimRecord_t rec;
    controllerPidGetTorqueTrim(&roll, &pitch, &loadResult);
    torqueTrimRecordInit(&rec, roll, pitch);
    esp_err_t err = torqueTrimStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR trim save: %s (live trim unchanged, stored record may be the previous one)", esp_err_to_name(err));
      return;
    }
    out("OK trim save roll=%.0f pitch=%.0f stored=nvs", (double)roll, (double)pitch);
  } else {
    out("ERR trim: unknown subcommand '%s'", sub);
  }
}

OUT_OF_LINE static void cmdFlightAcc(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  float x, y;

  if (strcmp(sub, "show") == 0) {
    sensorsMpu6050GetFlightAccOffset(&x, &y);
    out("OK facc x=%.4f y=%.4f", (double)x, (double)y);
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    bool clear = (strcmp(sub, "clear") == 0);
    x = 0.0f;
    y = 0.0f;
    if (!clear) {
      char *e1, *e2;
      if (argc < 4) {
        out("ERR facc set: usage facc set <x_g> <y_g>");
        return;
      }
      x = strtof(argv[2], &e1);
      y = strtof(argv[3], &e2);
      if (*e1 != '\0' || *e2 != '\0') {
        out("ERR facc set: not numbers");
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR facc %s: %s", sub, why);
      return;
    }
    const esp_err_t err = clear ? flightAccStoreErase() : ESP_OK;
    const bool ok = err != ESP_ERR_INVALID_STATE && sensorsMpu6050SetFlightAccOffset(x, y);
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (!ok) {
      out("ERR facc set: values must be within +/-0.05 g");
      return;
    }
    if (err != ESP_OK) {
      out("ERR facc clear: live offset zeroed but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    out("OK facc %s x=%.4f y=%.4f%s", sub, (double)x, (double)y, clear ? " stored=erased" : " stored=unchanged");
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    imuLevelCalRecord_t rec;
    sensorsMpu6050GetFlightAccOffset(&x, &y);
    imuLevelCalRecordInit(&rec, x, y);
    esp_err_t err = flightAccStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR facc save: %s", esp_err_to_name(err));
      return;
    }
    out("OK facc save x=%.4f y=%.4f stored=nvs", (double)x, (double)y);
  } else {
    out("ERR facc: unknown subcommand '%s'", sub);
  }
}

OUT_OF_LINE static void cmdAttTrim(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  float r, p;
  int load;

  if (strcmp(sub, "show") == 0) {
    controllerPidGetAttitudeTrim(&r, &p, &load);
    uint32_t learned;
    float mr, mp;
    bool pending;
    attTrimAutoStats(&learned, &mr, &mp, &pending);
    out("OK atrim roll=%.2f pitch=%.2f boot=%s learn=%d learned=%u lastHoldMean=%.2f,%.2f savePending=%d "
        "manualUnsaved=%d", (double)r, (double)p, esp_err_to_name(load), attTrimAutoEnabled(), (unsigned)learned,
        (double)mr, (double)mp, pending, attTrimAutoManualLiveOnly());
  } else if (strcmp(sub, "learn") == 0) {
    if (argc < 3 || (strcmp(argv[2], "on") != 0 && strcmp(argv[2], "off") != 0)) {
      out("ERR atrim learn: usage atrim learn on|off");
      return;
    }
    const bool on = strcmp(argv[2], "on") == 0;
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR atrim learn: %s", why);
      return;
    }
    const esp_err_t err = attTrimLearnStoreSave(on);
    if (err == ESP_OK) {
      attTrimAutoSetEnabled(on);
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR atrim learn: NVS save failed (%s); unchanged", esp_err_to_name(err));
      return;
    }
    out("OK atrim learn %s stored=nvs", on ? "on" : "off");
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    bool clear = (strcmp(sub, "clear") == 0);
    r = 0.0f;
    p = 0.0f;
    if (!clear) {
      char *e1, *e2;
      if (argc < 4) {
        out("ERR atrim set: usage atrim set <roll_deg> <pitch_deg>");
        return;
      }
      r = strtof(argv[2], &e1);
      p = strtof(argv[3], &e2);
      if (*e1 != '\0' || *e2 != '\0' || !attTrimValuesValid(r, p)) {
        out("ERR atrim set: values must be numbers within +/-%.1f deg", (double)ATT_TRIM_MAX_DEG);
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR atrim %s: %s", sub, why);
      return;
    }
    const esp_err_t err = clear ? attTrimStoreErase() : ESP_OK;
    if (err != ESP_ERR_INVALID_STATE) {
      controllerPidSetAttitudeTrim(r, p);
      /* A set is a trial: the learner neither builds on it nor stores it until
       * 'atrim save'; a clear is a fresh start it may learn from. */
      attTrimAutoNoteManual(!clear);
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR atrim clear: live trim zeroed but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    out("OK atrim %s roll=%.2f pitch=%.2f%s", sub, (double)r, (double)p,
        clear ? " stored=erased" : " stored=unchanged learning=paused_until_save");
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    attTrimRecord_t rec;
    controllerPidGetAttitudeTrim(&r, &p, &load);
    attTrimRecordInit(&rec, r, p);
    esp_err_t err = attTrimStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR atrim save: %s", esp_err_to_name(err));
      return;
    }
    attTrimAutoNoteManual(false);
    out("OK atrim save roll=%.2f pitch=%.2f stored=nvs", (double)r, (double)p);
  } else {
    out("ERR atrim: unknown subcommand '%s'", sub);
  }
}

static const char *gtcSource(int loadResult)
{
  if (loadResult == ESP_OK) {
    return "nvs";
  }
  if (loadResult == ESP_ERR_NVS_NOT_FOUND) {
    return "default";
  }
  if (loadResult == ESP_ERR_INVALID_STATE) {
    return "live";   /* changed from this console, not saved */
  }
  return "rejected";
}

/* One logged re-zero's fields: Tref, T, dTeff (with this config's knee), step, corr and r = step - corr (what
 * the compensation left: a still gyro's reading just before the re-zero), dps. */
#define GTC_RZ_FMT "Tref=%.2f T=%.2f dTeff=%+.2f step=%+.3f,%+.3f,%+.3f corr=%+.3f,%+.3f,%+.3f r=%+.3f,%+.3f,%+.3f"
#define GTC_RZ_ARGS(cfg, e)                                                                                         \
  (e)->tRefOldCc / 100.0, (e)->tNowCc / 100.0,                                                                      \
      (double)gyroTcEffDelta((cfg), (e)->tNowCc / 100.0f, (e)->tRefOldCc / 100.0f), (e)->stepMdps[0] / 1000.0,      \
      (e)->stepMdps[1] / 1000.0, (e)->stepMdps[2] / 1000.0, (e)->corrMdps[0] / 1000.0, (e)->corrMdps[1] / 1000.0,   \
      (e)->corrMdps[2] / 1000.0, (double)gyroTcRezeroResidual((e), 0), (double)gyroTcRezeroResidual((e), 1),        \
      (double)gyroTcRezeroResidual((e), 2)

/* The detail lines come first and the OK line last, so that a reader that
 * stops at OK (espfly_diag raw) has printed them all. */
static void gtcShow(void)
{
  sensorsGyroTcStatus_t st;
  sensorsMpu6050GetGyroTc(&st);
  /* The post-flight check (docs/autonomy-bench.md I5): the first re-zero after the latest airborne stretch, kept
   * until the next one (quiet re-zeros once a second do not push it out). */
  if (st.airSeq > 0) {
    const sensorsGyroTcRezero_t *e = &st.air;
    out("A seq=%u ageS=%.0f landS=%.1f airS=%.1f " GTC_RZ_FMT " on=%d", (unsigned)st.airSeq, st.airAgeMs / 1000.0,
        st.airLandMs / 1000.0, st.airSpanMs / 1000.0, GTC_RZ_ARGS(&st.cfg, e), !!(e->flags & SENSORS_GTC_RZ_ON));
  } else {
    out("A seq=0 (no re-zero after a flight yet this boot)");
  }
  /* The logged re-zeros, newest last: after thrust (flight=1; air=1 above lift-off), after a 'gtc hold' (hold=1:
   * the cooling test's residual), or with a step >= 0.05 dps. */
  for (int i = 0; i < st.n; i++) {
    const sensorsGyroTcRezero_t *e = &st.last[i];
    out("Z i=%d " GTC_RZ_FMT " flight=%d air=%d hold=%d cool=%d on=%d", i - st.n, GTC_RZ_ARGS(&st.cfg, e),
        !!(e->flags & SENSORS_GTC_RZ_FLIGHT), !!(e->flags & SENSORS_GTC_RZ_AIR), !!(e->flags & SENSORS_GTC_RZ_HOLD),
        !!(e->flags & SENSORS_GTC_RZ_COOL), !!(e->flags & SENSORS_GTC_RZ_ON));
  }
  out("OK gtc on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f clamp=%.2f src=%s T=%.2f Tlp=%.2f Tref=%.2f refSet=%d "
      "latchAgeMs=%u dTeff=%+.2f corr=%+.3f,%+.3f,%+.3f model=%+.3f,%+.3f,%+.3f rezeros=%u logged=%u n=%d airSeq=%u "
      "thrustSinceLatch=%d airSinceLatch=%d hold=%d holdLeftS=%.0f bypass=%d",
      (int)st.cfg.on, (double)st.cfg.k[0], (double)st.cfg.k[1], (double)st.cfg.k[2], (double)st.cfg.kneeC,
      (double)st.cfg.clampDps, gtcSource(st.loadResult), (double)st.tempC, (double)st.tLp, (double)st.tRef,
      (int)st.refSet, (unsigned)st.latchAgeMs,
      st.refSet ? (double)gyroTcEffDelta(&st.cfg, st.tLp, st.tRef) : 0.0, (double)st.corr[0],
      (double)st.corr[1], (double)st.corr[2], (double)st.model[0], (double)st.model[1], (double)st.model[2],
      (unsigned)st.rezeros, (unsigned)st.logged, st.n, (unsigned)st.airSeq, (int)st.thrustSinceLatch,
      (int)st.airSinceLatch, (int)st.holdOn, st.holdLeftMs / 1000.0, (int)st.bypassed);
}

OUT_OF_LINE static void cmdGyroTc(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;

  if (strcmp(sub, "show") == 0) {
    gtcShow();
    return;
  }
  if (strcmp(sub, "hold") == 0) {
    char *e = NULL;
    const long sec = argc >= 3 ? strtol(argv[2], &e, 10) : -1;
    if (argc < 3 || *e != '\0' || sec < 0 || sec > 300) {
      out("ERR gtc hold: usage gtc hold <0..300 s>");
      return;
    }
    if (autoBusy()) {
      return;
    }
    if (!motorsIdle(&why)) {
      out("ERR gtc hold: %s", why);
      return;
    }
    sensorsMpu6050GyroTcHold((uint32_t)sec * 1000u);
    out("OK gtc hold %lds (no ground re-zero; any thrust or an ARM ends it)", sec);
    return;
  }
  if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    sensorsGyroTcStatus_t st;
    sensorsMpu6050GetGyroTc(&st);
    gyroTcRecord_t rec;
    gyroTcRecordInit(&rec, &st.cfg);
    const esp_err_t err = gyroTcStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR gtc save: %s", esp_err_to_name(err));
      return;
    }
    sensorsMpu6050SetGyroTcConfig(&st.cfg, ESP_OK);
    out("OK gtc save on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f stored=nvs", (int)st.cfg.on, (double)st.cfg.k[0],
        (double)st.cfg.k[1], (double)st.cfg.k[2], (double)st.cfg.kneeC);
    return;
  }
  const bool isSet = strcmp(sub, "set") == 0, isClear = strcmp(sub, "clear") == 0;
  const bool isOn = strcmp(sub, "on") == 0, isOff = strcmp(sub, "off") == 0;
  if (!isSet && !isClear && !isOn && !isOff) {
    out("ERR gtc: unknown subcommand '%s'", sub);
    return;
  }
  sensorsGyroTcStatus_t st;
  sensorsMpu6050GetGyroTc(&st);
  gyroTcConfig_t c = st.cfg;
  if (isClear) {
    gyroTcDefaultConfig(&c);
  } else if (isOn || isOff) {
    c.on = isOn;
  } else {
    if (argc < 5) {
      out("ERR gtc set: usage gtc set <kx> <ky> <kz> [<knee_C>] (dps per C)");
      return;
    }
    char *e[4];
    for (int i = 0; i < 3; i++) {
      c.k[i] = strtof(argv[2 + i], &e[i]);
    }
    c.kneeC = argc >= 6 ? strtof(argv[5], &e[3]) : c.kneeC;
    if (*e[0] != '\0' || *e[1] != '\0' || *e[2] != '\0' || (argc >= 6 && *e[3] != '\0') ||
        !gyroTcConfigValid(&c)) {
      out("ERR gtc set: |k| <= %.2f dps/C, knee %.0f..%.0f C", (double)GYRO_TC_K_MAX, (double)GYRO_TC_KNEE_MIN,
          (double)GYRO_TC_KNEE_MAX);
      return;
    }
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR gtc %s: %s", sub, why);
    return;
  }
  const esp_err_t err = isClear ? gyroTcStoreErase() : ESP_OK;
  if (err != ESP_ERR_INVALID_STATE) {
    sensorsMpu6050SetGyroTcConfig(&c, isClear ? ESP_ERR_NVS_NOT_FOUND : ESP_ERR_INVALID_STATE);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR gtc clear: defaults applied but NVS erase failed (%s)", esp_err_to_name(err));
    return;
  }
  out("OK gtc %s on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f%s", sub, (int)c.on, (double)c.k[0], (double)c.k[1],
      (double)c.k[2], (double)c.kneeC, isClear ? " stored=erased" : " stored=unchanged");
}

/* The accelerometer compensation's check (accel_tempco.h, docs/autonomy-a2b.md 36.2): the same lines as gtc's in
 * mg: step = the still reading after minus the reference before, corr = the model at that moment, r = step - corr
 * what the compensation left (also the offset the controller flew with, up to the idle spin's own offset). */
#define ATC_RZ_FMT "Tref=%.2f T=%.2f dTeff=%+.2f step=%+.2f,%+.2f,%+.2f corr=%+.2f,%+.2f,%+.2f r=%+.2f,%+.2f,%+.2f"
#define ATC_RZ_ARGS(cfg, e)                                                                                         \
  (e)->tRefCc / 100.0, (e)->tNowCc / 100.0,                                                                         \
      (double)accTcEffDelta((cfg), (e)->tNowCc / 100.0f, (e)->tRefCc / 100.0f), (e)->stepCmg[0] / 100.0,           \
      (e)->stepCmg[1] / 100.0, (e)->stepCmg[2] / 100.0, (e)->corrCmg[0] / 100.0, (e)->corrCmg[1] / 100.0,          \
      (e)->corrCmg[2] / 100.0, (double)accTcRezeroResidual((e), 0), (double)accTcRezeroResidual((e), 1),           \
      (double)accTcRezeroResidual((e), 2)

static void atcShow(void)
{
  sensorsAccTcStatus_t st;
  sensorsMpu6050GetAccTc(&st);
  if (st.airSeq > 0) {
    const sensorsAccTcRezero_t *e = &st.air;
    out("A seq=%u ageS=%.0f landS=%.1f " ATC_RZ_FMT " on=%d", (unsigned)st.airSeq, st.airAgeMs / 1000.0,
        st.airLandMs / 1000.0, ATC_RZ_ARGS(&st.cfg, e), !!(e->flags & ACC_TC_RZ_ON));
  } else {
    out("A seq=0 (no re-zero after a flight yet this boot)");
  }
  /* The logged entries, newest last: after thrust (flight=1; air=1 above lift-off) or a cooling spin (cool=1). */
  for (int i = 0; i < st.n; i++) {
    const sensorsAccTcRezero_t *e = &st.last[i];
    out("Z i=%d " ATC_RZ_FMT " flight=%d air=%d cool=%d on=%d", i - st.n, ATC_RZ_ARGS(&st.cfg, e),
        !!(e->flags & ACC_TC_RZ_FLIGHT), !!(e->flags & ACC_TC_RZ_AIR), !!(e->flags & ACC_TC_RZ_COOL),
        !!(e->flags & ACC_TC_RZ_ON));
  }
  out("OK atc on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f clamp=%.1f src=%s T=%.2f Tlp=%.2f Tref=%.2f refSet=%d refValid=%d "
      "latchAgeMs=%u pending=%d dTeff=%+.2f corr=%+.2f,%+.2f,%+.2f model=%+.2f,%+.2f,%+.2f ref=%+.1f,%+.1f,%+.1f "
      "still=%+.1f,%+.1f,%+.1f stillValid=%d logged=%u n=%d airSeq=%u thrustSinceLatch=%d airSinceLatch=%d "
      "coolSince=%d bypass=%d",
      (int)st.cfg.on, (double)st.cfg.k[0], (double)st.cfg.k[1], (double)st.cfg.k[2], (double)st.cfg.kneeC,
      (double)st.cfg.clampMg, gtcSource(st.loadResult), (double)st.tempC, (double)st.tLp, (double)st.tRef,
      (int)st.refSet, (int)st.refValid, (unsigned)st.latchAgeMs, (int)st.latchPending,
      st.refSet ? (double)accTcEffDelta(&st.cfg, st.tLp, st.tRef) : 0.0, (double)st.corr[0], (double)st.corr[1],
      (double)st.corr[2], (double)st.model[0], (double)st.model[1], (double)st.model[2], (double)st.refMg[0],
      (double)st.refMg[1], (double)st.refMg[2], (double)st.stillMg[0], (double)st.stillMg[1], (double)st.stillMg[2],
      (int)st.stillValid, (unsigned)st.logged, st.n, (unsigned)st.airSeq, (int)st.thrustSinceLatch,
      (int)st.airSinceLatch, (int)st.coolSince, (int)st.bypassed);
}

OUT_OF_LINE static void cmdAccTc(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;

  if (strcmp(sub, "show") == 0) {
    atcShow();
    return;
  }
  if (strcmp(sub, "latch") == 0) {
    if (autoBusy()) {
      return;
    }
    if (!motorsIdle(&why)) {
      out("ERR atc latch: %s", why);
      return;
    }
    sensorsMpu6050AccTcLatch(false);
    out("OK atc latch (the resting reading of the next 50 ms block is the reference; the correction restarts at 0)");
    return;
  }
  if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    sensorsAccTcStatus_t st;
    sensorsMpu6050GetAccTc(&st);
    accTcRecord_t rec;
    accTcRecordInit(&rec, &st.cfg);
    const esp_err_t err = accTcStoreSave(&rec);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR atc save: %s", esp_err_to_name(err));
      return;
    }
    sensorsMpu6050SetAccTcConfig(&st.cfg, ESP_OK);
    out("OK atc save on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f stored=nvs", (int)st.cfg.on, (double)st.cfg.k[0],
        (double)st.cfg.k[1], (double)st.cfg.k[2], (double)st.cfg.kneeC);
    return;
  }
  const bool isSet = strcmp(sub, "set") == 0, isClear = strcmp(sub, "clear") == 0;
  const bool isOn = strcmp(sub, "on") == 0, isOff = strcmp(sub, "off") == 0;
  if (!isSet && !isClear && !isOn && !isOff) {
    out("ERR atc: unknown subcommand '%s'", sub);
    return;
  }
  sensorsAccTcStatus_t st;
  sensorsMpu6050GetAccTc(&st);
  accTcConfig_t c = st.cfg;
  if (isClear) {
    accTcDefaultConfig(&c);
  } else if (isOn || isOff) {
    c.on = isOn;
  } else {
    if (argc < 5) {
      out("ERR atc set: usage atc set <kx> <ky> <kz> [<knee_C>] (mg per C)");
      return;
    }
    char *e[4];
    for (int i = 0; i < 3; i++) {
      c.k[i] = strtof(argv[2 + i], &e[i]);
    }
    c.kneeC = argc >= 6 ? strtof(argv[5], &e[3]) : c.kneeC;
    if (*e[0] != '\0' || *e[1] != '\0' || *e[2] != '\0' || (argc >= 6 && *e[3] != '\0') ||
        !accTcConfigValid(&c)) {
      out("ERR atc set: |k| <= %.1f mg/C, knee %.0f..%.0f C", (double)ACC_TC_K_MAX, (double)ACC_TC_KNEE_MIN,
          (double)ACC_TC_KNEE_MAX);
      return;
    }
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR atc %s: %s", sub, why);
    return;
  }
  const esp_err_t err = isClear ? accTcStoreErase() : ESP_OK;
  if (err != ESP_ERR_INVALID_STATE) {
    sensorsMpu6050SetAccTcConfig(&c, isClear ? ESP_ERR_NVS_NOT_FOUND : ESP_ERR_INVALID_STATE);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR atc clear: defaults applied but NVS erase failed (%s)", esp_err_to_name(err));
    return;
  }
  out("OK atc %s on=%d k=%+.3f,%+.3f,%+.3f knee=%.1f%s", sub, (int)c.on, (double)c.k[0], (double)c.k[1],
      (double)c.k[2], (double)c.kneeC, isClear ? " stored=erased" : " stored=unchanged");
}

/* The ground cooling spin (ground_cool_glue.h). The state line: run / the latest end, the PWM and seconds, the
 * time left, the IMU temperature at the start and now, the tilt and gyro maxima of the run. */
static void coolShow(const char *tag)
{
  groundCoolGlueStatus_t st;
  groundCoolGlueStatus(&st);
  out("OK cool%s state=%s pwm=%u s=%u leftS=%.1f T0=%.2f T=%.2f dT=%+.2f elapsedS=%.1f tiltMax=%.1f gyroMax=%.0f "
      "runs=%u", tag, st.running ? "run" : groundCoolEndName((groundCoolEnd_t)st.gc.end), (unsigned)st.gc.pwm,
      (unsigned)st.gc.seconds, st.leftMs / 1000.0, (double)st.gc.tStartC, (double)st.tempC,
      (double)(st.tempC - st.gc.tStartC), st.gc.elapsedMs / 1000.0, (double)st.gc.tiltMaxDeg, (double)st.gc.gyroMaxDps,
      (unsigned)st.gc.runs);
}

OUT_OF_LINE static void cmdCool(int argc, char **argv)
{
  if (argc < 2) {
    coolShow("");
    return;
  }
  if (strcmp(argv[1], "stop") == 0) {
    groundCoolGlueStop();
    out("OK cool stop (ends at the next loop)");
    return;
  }
  char *e1 = NULL, *e2 = NULL;
  const long sec = strtol(argv[1], &e1, 10);
  const long pwm = argc >= 3 ? strtol(argv[2], &e2, 10) : (long)GROUND_COOL_PWM_DEFAULT;
  if (*e1 != '\0' || (argc >= 3 && *e2 != '\0') || sec < 1 || sec > (long)GROUND_COOL_S_MAX ||
      pwm < (long)GROUND_COOL_PWM_MIN || pwm > (long)GROUND_COOL_PWM_MAX) {
    out("ERR cool: usage cool [<1..%u s> [<%u..%u pwm>]] | cool stop", (unsigned)GROUND_COOL_S_MAX,
        (unsigned)GROUND_COOL_PWM_MIN, (unsigned)GROUND_COOL_PWM_MAX);
    return;
  }
  /* The console's own guards first (the same words as every motor command), then the spin's. */
  const char *why;
  if (autoBusy()) {
    return;
  }
  if (!motorsIdle(&why)) {
    out("ERR cool: %s", why);
    return;
  }
  if (!groundCoolGlueStart((uint32_t)pwm, (uint32_t)sec, &why)) {
    out("ERR cool: %s", why);
    return;
  }
  coolShow(" start");
}

static const char *const flowFitNames[] = { "robust", "lite", "plain" };

OUT_OF_LINE static void cmdFlowFit(int argc, char **argv)
{
  const char *why;
  if (argc >= 2 && strcmp(argv[1], "prof") == 0) {
    /* the camera task's per-stage times (camera_flow.h cameraFlowStageStats; docs/autonomy-a2b.md 30) */
    if (argc >= 3 && strcmp(argv[2], "reset") == 0) {
      cameraFlowStageReset();
      out("OK flowfit prof reset");
      return;
    }
    cameraFlowStageStats_t st;
    if (!cameraFlowStageStats(&st)) {
      out("OK flowfit prof frames=0");
      return;
    }
    const uint32_t *v[3] = {st.avgUs, st.minUs, st.maxUs};
    int n = outCat(0, "OK flowfit prof frames=%u stages=ds/coarse/fine/copy/snap/post/total", (unsigned)st.frames);
    for (int k = 0; k < 3; k++) {
      n = outCat(n, " %sUs=%u/%u/%u/%u/%u/%u/%u", k == 0 ? "avg" : (k == 1 ? "min" : "max"), (unsigned)v[k][0],
                 (unsigned)v[k][1], (unsigned)v[k][2], (unsigned)v[k][3], (unsigned)v[k][4], (unsigned)v[k][5],
                 (unsigned)v[k][6]);
    }
    outEnd(n);
    return;
  }
  if (argc >= 2) {
    int mode = -1;
    for (int i = 0; i < 3; i++) {
      if (strcmp(argv[1], flowFitNames[i]) == 0) {
        mode = i;
      }
    }
    if (mode < 0) {
      out("ERR flowfit: usage flowfit [robust|lite|plain] | flowfit prof [reset]");
      return;
    }
    if (autoBusy()) {
      return;
    }
    if (!motorsIdle(&why)) {
      out("ERR flowfit: %s", why);
      return;
    }
    const esp_err_t err = flowFitStoreSave((uint8_t)mode);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR flowfit: NVS save failed (%s); unchanged", esp_err_to_name(err));
      return;
    }
    cameraFlowSetFitMode((uint8_t)mode);
  }
  uint32_t gated, corrected;
  cameraFlowExposureStats(&gated, &corrected);
  cameraFlowClockStats_t clk;
  cameraFlowClockStats(&clk);
  cameraFlowSample_t c;
  const bool have = cameraFlowGetLatest(&c);
  out("OK flowfit mode=%s%s expGated=%u expCorrected=%u gainE3=%d satPct=%d camProcAvgUs=%u camProcMaxUs=%u "
      "camFps=%.1f camClk=%.2f bypass=%d", flowFitNames[cameraFlowFitMode() <= 2 ? cameraFlowFitMode() : 0],
      argc >= 2 ? " stored=nvs" : "", (unsigned)gated, (unsigned)corrected, have ? c.gainE3 : 0,
      have ? (c.satPct & 0x7F) : -1, (unsigned)clk.procAvgUs, (unsigned)clk.procMaxUs, (double)cameraFlowFps(),
      clk.xclkHz / 1e6, (int)benchBypassOn(BENCH_BYPASS_FLOW));
}

/* NULL: the boot value (NVS or the default); else what this console last did. */
static const char *mkiSourceOverride;

OUT_OF_LINE static void cmdMki(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  static bool bootChecked;
  static const char *bootSource = "?";
  if (!bootChecked && !autonomyActive()) {   /* an NVS read stops the flash cache: not while autonomy is active */
    float v;
    const esp_err_t e = mahonyKiAirStoreLoad(&v);
    bootSource = e == ESP_OK ? "nvs" : (e == ESP_ERR_NVS_NOT_FOUND ? "default" : "rejected");
    bootChecked = true;
  }
  if (strcmp(sub, "show") == 0) {
    float b[3];
    sensfusion6GetLearnedBiasDps(b);
    out("OK mki twoKiAir=%.4f src=%s airborne=%d learned=%+.3f,%+.3f,%+.3f dps bypass=%d",
        (double)sensfusion6GetKiAir(), mkiSourceOverride ? mkiSourceOverride : bootSource, (int)sensfusion6Airborne(),
        (double)b[0], (double)b[1], (double)b[2], (int)benchBypassOn(BENCH_BYPASS_MKI));
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    const bool clear = strcmp(sub, "clear") == 0;
    float v = 0.040f;
    if (!clear) {
      char *e = NULL;
      v = argc >= 3 ? strtof(argv[2], &e) : NAN;
      if (argc < 3 || *e != '\0' || !(v >= 0.0f && v <= SENSFUSION6_KI_AIR_MAX)) {
        out("ERR mki set: usage mki set <0..%.1f> (2 x Ki in flight; 0.040 default, 0 = off)",
            (double)SENSFUSION6_KI_AIR_MAX);
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR mki %s: %s", sub, why);
      return;
    }
    const esp_err_t err = clear ? mahonyKiAirStoreErase() : ESP_OK;
    if (err != ESP_ERR_INVALID_STATE) {
      sensfusion6SetKiAir(v);
      mkiSourceOverride = clear ? "default" : "live";
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR mki clear: default applied but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    out("OK mki %s twoKiAir=%.4f%s", sub, (double)v, clear ? " stored=erased" : " stored=unchanged");
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    const float v = sensfusion6GetKiAir();
    const esp_err_t err = mahonyKiAirStoreSave(v);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR mki save: %s", esp_err_to_name(err));
      return;
    }
    mkiSourceOverride = "nvs";
    out("OK mki save twoKiAir=%.4f stored=nvs", (double)v);
  } else {
    out("ERR mki: unknown subcommand '%s'", sub);
  }
}

/* The loop-rate bisect toggles (bench_bypass.h, docs/autonomy-bench.md A4b). */
static const struct {
  const char *name;
  uint32_t bit;
} bypassNames[] = {
  { "gtc", BENCH_BYPASS_GTC },
  { "mki", BENCH_BYPASS_MKI },
  { "rec", BENCH_BYPASS_REC },
  { "flow", BENCH_BYPASS_FLOW },
};

/* Every toggle that was on at any time since the last 'auto prof reset' (or the boot): the mask the loop
 * counters of 'auto prof' were measured under, which the live mask cannot tell once a bit is cleared before the
 * read, or across sorties without a reset in between. This task only; nothing on the loop reads it. */
static uint32_t benchBypassSeen;

static void bypassApply(uint32_t bits, bool on)
{
  if (bits & BENCH_BYPASS_GTC) {
    sensorsMpu6050GyroTcBypass(on);   /* the driver resets the compensation when it comes back */
  }
  const uint32_t rest = bits & (BENCH_BYPASS_MKI | BENCH_BYPASS_REC | BENCH_BYPASS_FLOW);
  if (on) {
    benchBypassMask |= rest;   /* this task is the mask's only writer */
  } else {
    benchBypassMask &= ~rest;
  }
  benchBypassSeen |= benchBypassMask;
}

OUT_OF_LINE static void cmdBypass(int argc, char **argv)
{
  const char *why;
  if (argc >= 2) {
    uint32_t bits = 0;
    if (strcmp(argv[1], "all") == 0) {
      bits = BENCH_BYPASS_ALL;
    }
    for (size_t i = 0; i < sizeof(bypassNames) / sizeof(bypassNames[0]); i++) {
      if (strcmp(argv[1], bypassNames[i].name) == 0) {
        bits = bypassNames[i].bit;
      }
    }
    const bool on = argc >= 3 && strcmp(argv[2], "on") == 0;
    if (bits == 0 || argc < 3 || (!on && strcmp(argv[2], "off") != 0)) {
      out("ERR bypass: usage bypass [gtc|mki|rec|flow|all on|off]");
      return;
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR bypass: %s", why);
      return;
    }
    bypassApply(bits, on);
    changeEnd();
  }
  const uint32_t m = benchBypassMask;
  out("OK bypass=0x%02x gtc=%d mki=%d rec=%d flow=%d (RAM only, 0 at boot; %s)", (unsigned)m,
      !!(m & BENCH_BYPASS_GTC), !!(m & BENCH_BYPASS_MKI), !!(m & BENCH_BYPASS_REC), !!(m & BENCH_BYPASS_FLOW),
      m ? "a bisect build state: not a go/no-go" : "the build as designed");
}

OUT_OF_LINE static void cmdCam(int argc, char **argv)
{
  const char *why;
  bool stored = false;
  /* An NVS read also stops the flash cache: not while autonomy is active. */
  const bool known = !autonomyActive() && camEnableStoreLoad(&stored) == ESP_OK;
  if (argc < 2) {
    out("OK cam boot=%s now=%s", known ? (stored ? "on" : "off") : (autonomyActive() ? "?" : "off"),
        cameraFlowStateName());
    return;
  }
  bool on;
  if (strcmp(argv[1], "on") == 0) {
    on = true;
  } else if (strcmp(argv[1], "off") == 0) {
    on = false;
  } else {
    out("ERR cam: usage cam [on|off]");
    return;
  }
  if (autoBusy()) {
    return;
  }
  if (!motorsIdle(&why)) {
    out("ERR cam: %s", why);
    return;
  }
  esp_err_t err = camEnableStoreSave(on);
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR cam: NVS save failed (%s)", esp_err_to_name(err));
    return;
  }
  out("OK cam %s stored=nvs note=takes_effect_after_reboot", on ? "on" : "off");
}

OUT_OF_LINE static void cmdHold(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    uint32_t rr, rd, re;
    cameraFlowHoldSceneStats(&rr, &rd, &re);
    out("OK hold %s sceneRes=%u sceneDx=%u sceneExp=%u", cameraFlowHoldEnabled() ? "on" : "off", (unsigned)rr,
        (unsigned)rd, (unsigned)re);
    return;
  }
  bool on;
  if (strcmp(argv[1], "on") == 0) {
    on = true;
  } else if (strcmp(argv[1], "off") == 0) {
    on = false;
  } else {
    out("ERR hold: usage hold [on|off]");
    return;
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR hold: %s", why);
    return;
  }
  const esp_err_t err = camHoldStoreSave(on);
  if (err == ESP_OK) {
    cameraFlowSetHoldEnabled(on);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR hold: NVS save failed (%s); unchanged", esp_err_to_name(err));
    return;
  }
  out("OK hold %s stored=nvs", on ? "on" : "off");
}

/* NULL: the boot value (NVS or the default); else what this console last did. */
static const char *holdkSourceOverride;

OUT_OF_LINE static void cmdHoldK(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "show";
  const char *why;
  static bool bootChecked;
  static const char *bootSource = "?";
  if (!bootChecked && !autonomyActive()) {   /* an NVS read stops the flash cache: not while autonomy is active */
    float v;
    const esp_err_t e = camHoldKmaxStoreLoad(&v);
    bootSource = e == ESP_OK ? "nvs" : (e == ESP_ERR_NVS_NOT_FOUND ? "default" : "rejected");
    bootChecked = true;
  }
  if (strcmp(sub, "show") == 0) {
    float sc, tgt, g;
    bool tr;
    cameraFlowHoldGainInfo(&sc, &tgt, &g, &tr);
    out("OK holdk kmax=%.2f src=%s s=%.2f target=%.2f G=%.1f D=%.2f translated=%d hold=%s",
        (double)cameraFlowHoldKmax(), holdkSourceOverride ? holdkSourceOverride : bootSource, (double)sc, (double)tgt,
        (double)g, g > 0.0f ? (double)(AUTO_MV_FY_PX / g) : 0.0, (int)tr, cameraFlowHoldEnabled() ? "on" : "off");
  } else if (strcmp(sub, "set") == 0 || strcmp(sub, "clear") == 0) {
    const bool clear = strcmp(sub, "clear") == 0;
    float v = CAM_HOLD_GAIN_KMAX_DEFAULT;
    if (!clear) {
      char *e = NULL;
      v = argc >= 3 ? strtof(argv[2], &e) : NAN;
      if (argc < 3 || *e != '\0' || !(v >= 1.0f && v <= CAM_HOLD_GAIN_SCALE_MAX)) {
        out("ERR holdk set: usage holdk set <1..%.0f> (%.1f default, 1 = the hold before the gain scale)",
            (double)CAM_HOLD_GAIN_SCALE_MAX, (double)CAM_HOLD_GAIN_KMAX_DEFAULT);
        return;
      }
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR holdk %s: %s", sub, why);
      return;
    }
    const esp_err_t err = clear ? camHoldKmaxStoreErase() : ESP_OK;
    if (err != ESP_ERR_INVALID_STATE) {
      cameraFlowSetHoldKmax(v);
      holdkSourceOverride = clear ? "default" : "live";
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR holdk clear: default applied but NVS erase failed (%s)", esp_err_to_name(err));
      return;
    }
    out("OK holdk %s kmax=%.2f%s", sub, (double)v, clear ? " stored=erased" : " stored=unchanged");
  } else if (strcmp(sub, "save") == 0) {
    if (autoBusy()) {
      return;
    }
    const float v = cameraFlowHoldKmax();
    const esp_err_t err = camHoldKmaxStoreSave(v);
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR holdk save: %s", esp_err_to_name(err));
      return;
    }
    holdkSourceOverride = "nvs";
    out("OK holdk save kmax=%.2f stored=nvs", (double)v);
  } else {
    out("ERR holdk: unknown subcommand '%s'", sub);
  }
}

OUT_OF_LINE static void cmdYawLock(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    out("OK yawlock %s", controllerPidGetYawLock() ? "on" : "off");
    return;
  }
  bool on;
  if (strcmp(argv[1], "on") == 0) {
    on = true;
  } else if (strcmp(argv[1], "off") == 0) {
    on = false;
  } else {
    out("ERR yawlock: usage yawlock [on|off]");
    return;
  }
  /* Saving writes flash, which stalls the CPU briefly: only while stopped. */
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR yawlock: %s", why);
    return;
  }
  const esp_err_t err = yawLockStoreSave(on);
  if (err == ESP_OK) {
    controllerPidSetYawLock(on);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR yawlock: NVS save failed (%s); unchanged", esp_err_to_name(err));
    return;
  }
  out("OK yawlock %s stored=nvs", on ? "on" : "off");
}

OUT_OF_LINE static void cmdCrash(int argc, char **argv)
{
  if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
    crashLogClear();
    out("OK crash cleared");
    return;
  }
  if (argc >= 2 && strcmp(argv[1], "test") == 0) {
    /* Bench check of the capture path: deliberate abort, then reboot. */
    const char *why;
    if (autoBusy()) {
      return;
    }
    if (!motorsIdle(&why)) {
      out("ERR crash test: %s", why);
      return;
    }
    out("OK crash test: aborting now, the drone will reboot");
    vTaskDelay(M2T(100));
    abort();
  }
  crashInfo_t c;
  bool valid = crashLogGet(&c);
  char bt[CRASH_LOG_BACKTRACE_DEPTH * 11 + 1];
  int k = 0;
  bt[0] = '\0';
  for (uint32_t i = 0; valid && i < c.depth && i < CRASH_LOG_BACKTRACE_DEPTH; i++) {
    k += snprintf(bt + k, sizeof(bt) - k, "%s0x%08x", i ? "," : "", (unsigned)c.backtrace[i]);
  }
  out("OK crash valid=%d bootsSince=%u core=%d exception=%d pc=0x%08x exccause=%u excvaddr=0x%08x bt=%s details=%s",
      valid, (unsigned)c.bootsSince, (int)c.core, (int)c.exception, (unsigned)c.pc, (unsigned)c.exccause,
      (unsigned)c.excvaddr, valid ? bt : "", valid ? c.details : "");
  /* the log lines: a PSRAM copy for this command only (internal RAM is short) */
  char *lines = heap_caps_malloc(CRASH_LOG_RING_LEN + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (lines == NULL) {
    lines = heap_caps_malloc(CRASH_LOG_RING_LEN + 1, MALLOC_CAP_8BIT);
  }
  if (lines != NULL) {
    crashLogGetLines(lines, CRASH_LOG_RING_LEN + 1);
    char *save = NULL;
    for (char *l = strtok_r(lines, "\n", &save); l; l = strtok_r(NULL, "\n", &save)) {
      out("L %s", l);
    }
    heap_caps_free(lines);
  }
  crashBoot_t hist[CRASH_LOG_BOOT_HISTORY];
  uint32_t total = 0;
  int nb = crashLogBoots(hist, CRASH_LOG_BOOT_HISTORY, &total);
  for (int i = 0; i < nb; i++) {
    out("B boot=-%d reset=%s(%d) raw=0x%02x,0x%02x prevUptimeS=%u", i,
        resetReasonName((esp_reset_reason_t)hist[i].resetReason), (int)hist[i].resetReason,
        hist[i].rawReason[0], hist[i].rawReason[1], (unsigned)hist[i].prevUptimeS);
  }
  out("END crash boots=%u", (unsigned)total);
}

/* The camera height observer's flight-summary fields (fsumCamVz_t), shared
 * by 'flights' and 'camvz'; tools/espfly_diag.py parses them by name. */
#define CVZ_FMT "cvf=%u cvFirstDs=%u cvG0=%d cvB3=%d cvB5=%d cvBtd=%d cvSb5=%u cvGain=%u cvHtd=%d cvVtd=%d " \
                "cvHmax=%d cvAir=%u cvCamAir=%u cvOk=%u cvGap=%u cvAcc=%u cvRej=%u cvAlarm=%u cvYaw=%u"
#define CVZ_ARGS(c) (unsigned)(c)->flags, (unsigned)(c)->firstOkDs, (int)(c)->g0Dmg, (int)(c)->bias3Cmg, \
                    (int)(c)->bias5Cmg, (int)(c)->biasTdCmg, (unsigned)(c)->sigB5Cmg, (unsigned)(c)->gainDpm, \
                    (int)(c)->hTdMm, (int)(c)->vTdMms, (int)(c)->hMaxMm, (unsigned)(c)->airCs, \
                    (unsigned)(c)->camAirCs, (unsigned)(c)->okCs, (unsigned)(c)->gapMaxCs, (unsigned)(c)->accepted, \
                    (unsigned)(c)->rejected, (unsigned)(c)->alarms, (unsigned)(c)->yawResets

static const char *camVzLiftName(uint8_t by)
{
  switch (by) {
    case CVS_LIFT_BY_THRUST: return "thrust";
    case CVS_LIFT_BY_ACC: return "acc";
    case CVS_LIFT_BY_HINT: return "hint";
    default: return "none";
  }
}

OUT_OF_LINE static void cmdCamVz(int argc, char **argv)
{
  if (argc >= 2) {
    if (strcmp(argv[1], "on") == 0) {
      camVzGlueSetPaused(false);   /* always allowed: it only resumes the observer */
      out("OK camvz on (not stored)");
    } else if (strcmp(argv[1], "off") == 0) {
      /* The pause freezes the controller's camera height input: never while
       * active, and only on the ground (a pause started in the air froze the
       * snapshot in CVS_AIR). An ARM for FLIGHT or BENCH-imu is refused while
       * paused (docs/autonomy-a2b.md section 7.3). */
      const char *why = NULL;
      if (!changeBegin()) {
        return;
      }
      if (!motorsIdle(&why)) {
        changeEnd();
        out("ERR busy: %s", why);
        return;
      }
      camVzGlueSetPaused(true);
      changeEnd();
      out("OK camvz off (not stored; an autonomy ARM in FLIGHT or BENCH imu is refused until 'camvz on')");
    } else if (strcmp(argv[1], "prof") == 0 && argc >= 3 && strcmp(argv[2], "reset") == 0) {
      camVzGlueProfileReset();
      out("OK camvz prof reset");
    } else if (strcmp(argv[1], "prof") == 0) {
      uint32_t mx[4], av[4];
      camVzGlueProfile(mx, av);
      /* The stabilizer's since-boot loop counters, for loop-rate bench tests
       * (deltas between two reads; kept out of 'status', which is near the
       * console's line limit). */
      stabilizerBenchTelemetry_t tel;
      const bool haveTel = stabilizerGetBenchTelemetry(&tel);
      out("OK camvz prof stab=%u.%u/%u block=%u.%u/%u frame=%u.%u/%u publish=%u.%u/%u (avg/max us) drops=%u "
          "upMs=%u loops=%u execSumUs=%llu s2oSumUs=%llu s2oN=%u s2o800=%u",
          (unsigned)(av[0] / 10), (unsigned)(av[0] % 10), (unsigned)mx[0], (unsigned)(av[1] / 10),
          (unsigned)(av[1] % 10), (unsigned)mx[1], (unsigned)(av[2] / 10), (unsigned)(av[2] % 10), (unsigned)mx[2],
          (unsigned)(av[3] / 10), (unsigned)(av[3] % 10), (unsigned)mx[3], (unsigned)camVzGlueRingDrops(),
          (unsigned)T2M(xTaskGetTickCount()), haveTel ? (unsigned)tel.loop_count : 0u,
          haveTel ? (unsigned long long)tel.exec_sum_us : 0ull,
          haveTel ? (unsigned long long)tel.sensor_to_output_sum_us : 0ull,
          haveTel ? (unsigned)tel.sensor_to_output_n : 0u, haveTel ? (unsigned)tel.sensor_to_output_over_800us : 0u);
    } else if (argc >= 3 && strcmp(argv[1], "batch") == 0 && atoi(argv[2]) >= 1 && atoi(argv[2]) <= 8) {
      if (!changeBegin()) {
        return;
      }
      camVzGlueSetBatch((uint32_t)atoi(argv[2]));
      changeEnd();
      out("OK camvz batch %d (not stored)", atoi(argv[2]));
    } else if (argc >= 3 && strcmp(argv[1], "air") == 0 &&
               (strcmp(argv[2], "off") == 0 || strcmp(argv[2], "on") == 0 || strcmp(argv[2], "full") == 0 ||
                strcmp(argv[2], "spin") == 0)) {
      /* Bench load test: the observer runs its in-flight path on a drone on
       * the bench (it never touches the motors); measure 'status' loops over
       * 20 s. 'spin' keeps it on through a props-off spin under a gcap cap. */
      const char *why = NULL;
      if (strcmp(argv[2], "off") == 0) {
        camVzGlueSetBenchAir(CVS_BENCH_OFF);   /* allowed while active: it only ends a bench mode */
        out("OK camvz air off");
      } else if (!changeBegin()) {
        /* 'air spin' would make the camera height estimate unusable mid-sortie */
      } else if (strcmp(argv[2], "spin") == 0) {
        const bool capped = stabilizerGetThrustCap() != 0;
        if (capped) {
          camVzGlueSetBenchAir(CVS_BENCH_SPIN);
        }
        changeEnd();
        if (!capped) {
          out("ERR camvz air spin needs a gcap bench cap (props off)");
        } else {
          out("OK camvz air spin (not stored; 'camvz air off' ends it)");
        }
      } else if (!motorsIdle(&why)) {
        changeEnd();
        out("ERR busy: %s", why);
      } else {
        camVzGlueSetBenchAir(strcmp(argv[2], "full") == 0 ? CVS_BENCH_FULL : CVS_BENCH_AIR);
        changeEnd();
        out("OK camvz air %s (not stored; ends at any thrust; frames used from 1.5 s on)", argv[2]);
      }
    } else {
      out("ERR camvz: usage camvz [on|off|prof [reset]] | camvz batch 1-8 | camvz air off|on|full|spin");
    }
    return;
  }
  camVzGlueStatus_t st;
  if (!camVzGlueGetStatus(&st)) {
    out("OK camvz on=0 (no observer: no memory, or the stabilizer has not run yet)");
    return;
  }
  const camVzObsOut_t *o = &st.out;
  const camVzObsCounters_t *n = &st.cnt;
  camVzGlueLate_t lt;
  camVzGlueLate(&lt);
  camVzGlueT2_t t2;
  camVzGlueT2(&t2);
  camVzGlueHb_t hb;
  camVzGlueHb(&hb);
  out("OK camvz on=1 mem=%s bytes=%u phase=%s sortie=%u lift=%s g0Mg=%.2f h=%.3f v=%.3f biasMg=%.3f sigH=%.3f "
      "sigV=%.3f sigBiasMg=%.3f gain=%.1f gainTracked=%d healthy=%d airborne=%d fault=%d sinceAcceptMs=%d "
      "innov=%.3f innovSig=%.3f lastFrame=%s frames=%u skipped=%u quality=%u exp=%u lat=%u noWindow=%u "
      "noGain=%u accepted=%u rejected=%u relocks=%u alarms=%u yawResets=%u reanchors=%u seqGaps=%u imuIgnored=%u "
      "stepUsMax=%u stepUsAvg=%.2f frameUsMax=%u frameUsAvg=%.1f frameCalls=%u stabStackFree=%d "
      "gyroBiasDps=%.2f,%.2f,%.2f bench=%u qlate=%.0f qlateS=%.0f qlateCfg=%s late=%s lateMs=%u lateGuards=%u "
      "biasHeldMg=%.3f t2k=%.2f t2r=%+.3f t2S=%.0f t2Cfg=%s t2=%s t2OnS=%.1f t2bR=%+.3f t2bRmin=%+.3f t2bRmax=%+.3f "
      "hb=%.1f hbCfg=%s hbOn=%d hbOnS=%.1f "
      CVZ_FMT,
      st.inPsram ? "psram" : "internal", (unsigned)st.bytes, camVzShadowPhaseName((camVzShadowPhase_t)st.phase),
      (unsigned)st.sortie, camVzLiftName(st.liftBy), (double)((st.g0 - 1.0f) * 1000.0f), (double)o->h,
      (double)o->v, (double)(o->biasG * 1000.0f), (double)o->sigmaH, (double)o->sigmaV,
      (double)(o->sigmaBiasG * 1000.0f), (double)o->gain, o->gainTracked, o->healthy, o->airborne, o->fault,
      o->sinceAcceptUs == UINT32_MAX ? -1 : (int)(o->sinceAcceptUs / 1000u), (double)o->lastInnov,
      (double)o->lastInnovSigma, camVzObsDecisionName((camVzObsDecision_t)st.lastDecision),
      (unsigned)n->frames, (unsigned)n->skipped, (unsigned)n->quality, (unsigned)n->expRejects,
      (unsigned)n->latRejects, (unsigned)n->noWindow, (unsigned)n->noGain, (unsigned)n->accepted,
      (unsigned)n->rejected, (unsigned)n->relocks, (unsigned)n->alarms, (unsigned)n->yawResets,
      (unsigned)n->reanchors, (unsigned)n->seqGaps, (unsigned)n->imuIgnored, (unsigned)st.stepMaxUs,
      st.stepSamples ? (double)st.stepSumUs / st.stepSamples : 0.0, (unsigned)st.frameMaxUs,
      st.frameCalls ? (double)st.frameSumUs / st.frameCalls : 0.0, (unsigned)st.frameCalls,
      st.stabStackFree == UINT32_MAX ? -1 : (int)st.stabStackFree, (double)st.gyroBiasDps[0],
      (double)st.gyroBiasDps[1], (double)st.gyroBiasDps[2], (unsigned)st.benchAir, (double)lt.qLateMult,
      (double)lt.qLateS, lt.cfgBad ? "bad" : "ok",
      lt.state == CAM_VZ_LATE_ON ? "on" : (lt.state == CAM_VZ_LATE_GUARD ? "guard" : "off"), (unsigned)lt.lateMs,
      (unsigned)lt.guards, (double)(o->biasHeldG * 1000.0f), (double)t2.kR, (double)t2.priorMgS, (double)t2.startS,
      t2.cfgBad ? "bad" : "ok", t2.kR <= 0.0f ? "off" : (!t2.t2.on ? "wait" : (t2.t2.drifting ? (t2.t2.guard ? "hold" : "on")
      : "frozen")), t2.t2.onUs / 1e6, (double)t2.t2.t2RateMgS, (double)t2.t2.minMgS, (double)t2.t2.maxMgS,
      (double)hb.mult, hb.cfgBad ? "bad" : "ok", hb.on, hb.onMs / 1e3, CVZ_ARGS(&st.stats));
}

OUT_OF_LINE static void cmdFlights(int argc, char **argv)
{
  if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
    flightRtcLogClear();
    out("OK flights cleared");
    return;
  }
  uint32_t total = 0, boots = 0;
  int n = flightRtcLogFlightCount(&total);
  crashBoot_t b;
  crashLogBoots(&b, 1, &boots);
  out("OK flights n=%d total=%u boot=%u", n, (unsigned)total, (unsigned)(boots & 0xFFFF));
  for (int i = 0; i < n; i++) {
    flightSummary_t fs;
    if (!flightRtcLogFlight(i, &fs)) {
      break;
    }
    const flightSummary_t *f = &fs;
    out("F boot=%u id=%u end=%u rst=%u thrCs=%u airCs=%u steadyCs=%u holdCs=%u coastCs=%u camCs=%u thrMax=%u "
        "vStart=%u vMin=%u tiltMax=%u gyroMax=%u tiltSq=%u gyroSq=%u m=%u,%u,%u,%u "
        "holdR=%d holdP=%d rI=%d pI=%d gDrift=%d,%d upS=%u joinDs=%u appDs=%u railMin=%u vFastMin=%u tx=%u rx=%u "
        "gapMaxUs=%u gaps2500=%u pwmKhz=%u soft=%u camsb=%u txlo=%u wifiq=%u sbDid=%u tlDid=%u cold=%u atrim=%d,%d "
        "imuT0=%.1f imuT1=%.1f camAwakeDs=%d gtcY=%d mki=%d,%d gtcRf=%u gtcR=%d,%d gtcRdT=%d gtcRland=%u " CVZ_FMT
        " cvqf=%u cvq=%u cvqS=%u cvqG=%u cvqDs=%u cvt2k=%u cvt2r=%d cvt2Ds=%u cvt2bR=%d,%d,%d atcRf=%u atcS=%d atcC=%d "
        "atcRdT=%d cool=%u%s%s",
        f->boot, f->flightId, f->end, f->resetReason, f->thrustCs, f->airCs, f->steadyCs, f->holdCs,
        f->coastCs, f->camCs, f->thrustMax, f->vbatStartMv, f->vbatMinMv, f->tiltMaxDd, f->gyroMaxDps,
        (unsigned)f->tiltSq, (unsigned)f->gyroSq, (unsigned)f->motorSum[0], (unsigned)f->motorSum[1],
        (unsigned)f->motorSum[2], (unsigned)f->motorSum[3], (int)f->holdRollSum, (int)f->holdPitchSum,
        (int)f->rollRateISum, (int)f->pitchRateISum, f->gyroDrift[0], f->gyroDrift[1],
        f->uptimeS, f->sinceJoinDs, f->sinceAppDs, f->railMinMv, f->vbatFastMinMv, f->txPackets, f->rxPackets,
        f->loopGapMaxUs, f->loopGaps2500, f->pwmKhz, (unsigned)(f->motorFlags & FSUM_MOTOR_SOFTSTART),
        (unsigned)!!(f->motorFlags & FSUM_MOTOR_CAMSB), (unsigned)!!(f->motorFlags & FSUM_MOTOR_TXLO),
        (unsigned)!!(f->motorFlags & FSUM_MOTOR_WIFIQ), (unsigned)!!(f->motorFlags & FSUM_MOTOR_CAMSB_DID),
        (unsigned)!!(f->motorFlags & FSUM_MOTOR_TXLO_DID), (unsigned)!!(f->motorFlags & FSUM_MOTOR_COLD),
        (int)f->atrimRoll, (int)f->atrimPitch, f->imuT0Hc == FSUM_TEMP_NONE ? NAN : f->imuT0Hc * 0.5,
        f->imuT1Hc == FSUM_TEMP_NONE ? NAN : f->imuT1Hc * 0.5, f->camAwakeDs == FSUM_AGE_NONE ? -1 : (int)f->camAwakeDs,
        (int)f->gtcYEndMdps, (int)f->mkiEndMdps[0], (int)f->mkiEndMdps[1], (unsigned)f->gtcResFlags,
        (int)f->gtcResMdps[0], (int)f->gtcResMdps[1], (int)f->gtcResDtCc, (unsigned)f->gtcResLandDs,
        CVZ_ARGS(&f->camVz), (unsigned)f->camVzLate.flags, (unsigned)f->camVzLate.qLate,
        (unsigned)f->camVzLate.qLateS, (unsigned)f->camVzLate.guards, (unsigned)f->camVzLate.lateDs,
        (unsigned)f->camVzLate.t2K, (int)f->camVzLate.t2PriorUgs, (unsigned)f->camVzLate.t2OnDs,
        (int)f->camVzLate.t2BrEnd, (int)f->camVzLate.t2BrMin, (int)f->camVzLate.t2BrMax, (unsigned)f->atcResFlags,
        (int)f->atcStepZCmg, (int)f->atcCorrZCmg, (int)f->atcResDtCc, (unsigned)!!(f->motorFlags & FSUM_MOTOR_COOL),
        f->end == FSUM_END_AUTO ? " auto=" : "",
        f->end == FSUM_END_AUTO ? autoReasonName((autoReason_t)f->autoReason) : "");
  }
  uint16_t trBoot = 0;
  int32_t trReason = 0;
  int nt = flightRtcLogResetTraceInfo(&trBoot, &trReason);
  for (int i = 0; i < nt; i++) {
    fsumCrumb_t cr;
    if (!flightRtcLogResetTraceGet(i, &cr)) {
      break;
    }
    const fsumCrumb_t *c = &cr;
    out("C i=%d thr=%u v=%u m=%u,%u,%u,%u r=%d p=%d g=%u f=%u la=%u rail=%u tx=%u rx=%u", i - nt, c->thrust,
        c->vbatMv, c->motor[0], c->motor[1], c->motor[2], c->motor[3], c->roll, c->pitch, c->gyroDps, c->flags,
        c->linkAge20ms, c->rail10mv == 0 ? 0u : (c->rail10mv == 255 ? 65535u : 2000u + 10u * c->rail10mv),
        c->txPk, c->rxPk);
  }
  uint16_t vmin[FLIGHT_RTC_LOG_VWIN];
  int nv = flightRtcLogResetVbat(vmin, FLIGHT_RTC_LOG_VWIN);
  if (nv > 0) {
    char vbuf[FLIGHT_RTC_LOG_VWIN * 6 + 1];
    int k = 0;
    vbuf[0] = '\0';
    for (int i = 0; i < nv; i++) {
      k += snprintf(vbuf + k, sizeof(vbuf) - k, "%s%u", i ? "," : "", vmin[i]);
    }
    out("V win_ms=10 mv=%s", vbuf);
    uint16_t rmin[FLIGHT_RTC_LOG_VWIN];
    int nr = flightRtcLogResetRail(rmin, FLIGHT_RTC_LOG_VWIN);
    k = 0;
    vbuf[0] = '\0';
    for (int i = 0; i < nr; i++) {
      k += snprintf(vbuf + k, sizeof(vbuf) - k, "%s%u", i ? "," : "", rmin[i]);
    }
    if (nr > 0) {
      out("R win_ms=10 mv=%s", vbuf);
    }
  }
  out("END flights trace=%d traceBoot=%u traceReset=%s(%d)", nt, trBoot,
      nt ? resetReasonName((esp_reset_reason_t)trReason) : "none", (int)trReason);
}

OUT_OF_LINE static void cmdLive(void)
{
  uint16_t vNow, vMin, rNow, rMin;
  flightRtcLogLive(&vNow, &vMin, &rNow, &rMin);
  flightRecord_t r;
  bool have = flightRecorderGetLatest(&r);
  uint32_t tx, rx, joinMs, appMs;
  wifiGetCounters(&tx, &rx, &joinMs, &appMs);
  const uint32_t nowMs = T2M(xTaskGetTickCount());
  out("OK live up_ms=%u thrust=%u raw=%u m=%d,%d,%d,%d vbat=%u vbatMin=%u rail=%u railMin=%u tx=%u rx=%u "
      "joinAgeMs=%d appAgeMs=%d gcap=%u cam=%s",
      (unsigned)nowMs, have ? (unsigned)r.thrust : 0u, have ? (unsigned)r.rawThrust : 0u,
      motorsGetRatio(0), motorsGetRatio(1), motorsGetRatio(2), motorsGetRatio(3),
      vNow, vMin, rNow, rMin, (unsigned)tx, (unsigned)rx,
      joinMs ? (int)(nowMs - joinMs) : -1, appMs ? (int)(nowMs - appMs) : -1,
      stabilizerGetThrustCap(), cameraFlowStateName());
}

OUT_OF_LINE static void cmdNet(void)
{
  wifiLinkStats_t w;
  wifiGetLinkStats(&w);
  uint32_t prx[CRTP_PORT_COUNT], ptx[CRTP_PORT_COUNT];
  crtpGetPortCounts(prx, ptx);
  char rxs[CRTP_PORT_COUNT * 11 + 1], txs[CRTP_PORT_COUNT * 11 + 1];
  int kr = 0, kt = 0;
  for (int i = 0; i < CRTP_PORT_COUNT; i++) {
    kr += snprintf(rxs + kr, sizeof(rxs) - kr, "%s%u", i ? "," : "", (unsigned)prx[i]);
    kt += snprintf(txs + kt, sizeof(txs) - kt, "%s%u", i ? "," : "", (unsigned)ptx[i]);
  }
  const uint32_t nowMs = T2M(xTaskGetTickCount());
  out("OK net up_ms=%u joins=%u leaves=%u joinAgeMs=%d appAgeMs=%d udpRx=%u udpTx=%u udpTxErr=%u "
      "cksumErr=%u qDrops=%u txDbm=%.2f bwMhz=%u crtpRx=%s crtpTx=%s",
      (unsigned)nowMs, (unsigned)w.staJoins, (unsigned)w.staLeaves,
      w.lastJoinMs ? (int)(nowMs - w.lastJoinMs) : -1, w.appFirstMs ? (int)(nowMs - w.appFirstMs) : -1,
      (unsigned)w.rxPackets, (unsigned)w.txPackets, (unsigned)w.txErrors, (unsigned)w.cksumErrors,
      (unsigned)w.queueDrops, w.txPowerQdbm / 4.0, wifiApBandwidthMhz(), rxs, txs);
}

#define TKOFF_USAGE "ERR tkoff: usage tkoff [camsb|camgnd|txlo|wifiq on|off] | tkoff camms|camcold|camgndms|camlead <ms> | " \
                    "tkoff camwake <s> | tkoff camclk 20|13"

/* Take-off load shedding: tkoff [camsb|txlo|wifiq on|off] | tkoff camms <ms>.
 * camsb, camms and txlo act at once; wifiq (no 11b rates, 300 TU beacons) at
 * the next boot. The camera's ground standby (docs/autonomy-a2b.md 17.1):
 * camgnd on|off, camgndms (the hold after a wake reason) and camlead (the
 * thermal lead) are stored and act at once; camwake <s> holds it awake for s
 * seconds (RAM only, any time; 0 ends the hold). */
OUT_OF_LINE static void cmdTakeoff(int argc, char **argv)
{
  if (argc >= 3 && strcmp(argv[1], "camclk") == 0) {
    /* the camera's input clock, at the next boot (docs/autonomy-a2b.md 17.2) */
    const bool slow = strcmp(argv[2], "13") == 0;
    const char *why;
    if (!slow && strcmp(argv[2], "20") != 0) {
      out("ERR tkoff: camclk 20|13 (MHz; 13 = 13.33 MHz, 14.8 fps; 20 = 22.2 fps)");
      return;
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR tkoff: %s", why);
      return;
    }
    const esp_err_t err = camClockStoreSave(slow);
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR tkoff: NVS save failed (%s); unchanged", esp_err_to_name(err));
      return;
    }
  } else if (argc >= 3 && strcmp(argv[1], "camwake") == 0) {
    char *e;
    const long sec = strtol(argv[2], &e, 10);
    if (*e != '\0' || sec < 0 || sec > 3600) {
      out("ERR tkoff: camwake <0..3600 s>");
      return;
    }
    if (sec == 0) {
      cameraFlowGroundRelease();
    } else {
      cameraFlowGroundWake((uint32_t)sec * 1000u);
    }
  } else if (argc >= 3 && (strcmp(argv[1], "camms") == 0 || strcmp(argv[1], "camcold") == 0 ||
                           strcmp(argv[1], "camgndms") == 0 || strcmp(argv[1], "camlead") == 0)) {
    const bool cold = strcmp(argv[1], "camcold") == 0;
    const bool gndMs = strcmp(argv[1], "camgndms") == 0;
    const bool lead = strcmp(argv[1], "camlead") == 0;
    const long lo = gndMs ? 5000 : (lead ? 0 : 200), hi = gndMs ? 65000 : (lead ? 60000 : 8000);
    char *e;
    long ms = strtol(argv[2], &e, 10);
    const char *why;
    if (*e != '\0' || ms < lo || ms > hi) {
      out("ERR tkoff: %s <%ld..%ld>", argv[1], lo, hi);
      return;
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR tkoff: %s", why);
      return;
    }
    esp_err_t err;
    if (gndMs) {
      err = camGroundMsStoreSave((uint16_t)ms);
      if (err == ESP_OK) {
        cameraFlowSetGroundHoldMs((uint32_t)ms);
      }
    } else if (lead) {
      err = camLeadMsStoreSave((uint16_t)ms);
      if (err == ESP_OK) {
        cameraFlowSetGroundLeadMs((uint32_t)ms);
      }
    } else {
      err = cold ? camStandbyColdMsStoreSave((uint16_t)ms) : camStandbyMsStoreSave((uint16_t)ms);
      if (err == ESP_OK) {
        cameraFlowSetStandbyMs((uint16_t)ms, cold);
      }
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR tkoff: NVS save failed (%s); unchanged", esp_err_to_name(err));
      return;
    }
  } else if (argc >= 3) {
    bool on;
    if (strcmp(argv[2], "on") == 0) {
      on = true;
    } else if (strcmp(argv[2], "off") == 0) {
      on = false;
    } else {
      out(TKOFF_USAGE);
      return;
    }
    const char *why;
    if (strcmp(argv[1], "camsb") != 0 && strcmp(argv[1], "camgnd") != 0 && strcmp(argv[1], "txlo") != 0 &&
        strcmp(argv[1], "wifiq") != 0) {
      out(TKOFF_USAGE);
      return;
    }
    if (!changeBegin()) {
      return;
    }
    if (!motorsIdle(&why)) {
      changeEnd();
      out("ERR tkoff: %s", why);
      return;
    }
    esp_err_t err;
    if (strcmp(argv[1], "camsb") == 0) {
      err = camStandbyStoreSave(on);
      if (err == ESP_OK) {
        cameraFlowSetStandbyEnabled(on);
      }
    } else if (strcmp(argv[1], "camgnd") == 0) {
      err = camGroundStoreSave(on);
      if (err == ESP_OK) {
        cameraFlowSetGroundStandby(on);
      }
    } else if (strcmp(argv[1], "txlo") == 0) {
      err = txLowStoreSave(on);
      if (err == ESP_OK) {
        takeoffAuxSetTxLowEnabled(on);
      }
    } else {
      err = wifiQuietStoreSave(on);
    }
    changeEnd();
    if (storeBusy(err)) {
      return;
    }
    if (err != ESP_OK) {
      out("ERR tkoff: NVS save failed (%s); unchanged", esp_err_to_name(err));
      return;
    }
  } else if (argc == 2) {
    out(TKOFF_USAGE);
    return;
  }
  bool wq = false, clkSlow = true;
  const bool clkNvs = !autonomyActive() && camClockStoreLoad(&clkSlow) == ESP_OK;
  if (!autonomyActive()) {   /* no NVS read during a sortie (it stops the flash cache) */
    wifiQuietStoreLoad(&wq);
  }
  cameraFlowClockStats_t clk;
  cameraFlowClockStats(&clk);
  uint32_t sbCount, sbFails, sbMs;
  cameraFlowStandbyStats(&sbCount, &sbFails, &sbMs);
  cameraFlowGroundStats_t g;
  cameraFlowGroundStats(&g);
  out("OK tkoff camsb=%d camms=%u camcold=%u txlo=%d wifiq=%d wifiqBoot=%d window=%d takeoffs=%u lastCold=%d cam=%s "
      "sbCount=%u sbFails=%u sbMs=%u txLowNow=%d camgnd=%d camgndms=%u camlead=%u asleep=%d gndCount=%u "
      "gndFails=%u gndMs=%u gndHoldLeftMs=%u gndAwakeMs=%d camclk=%s camClkNow=%.2f camClkFallback=%d",
      cameraFlowStandbyEnabled(), (unsigned)cameraFlowStandbyMs(false), (unsigned)cameraFlowStandbyMs(true),
      takeoffAuxTxLowEnabled(), wq, wifiQuiet(), stabilizerTakeoffWindow(), (unsigned)stabilizerTakeoffCount(),
      stabilizerTakeoffCold(), cameraFlowStateName(), (unsigned)sbCount, (unsigned)sbFails, (unsigned)sbMs,
      wifiTxLow(), cameraFlowGroundStandbyEnabled(), (unsigned)cameraFlowGroundHoldMs(),
      (unsigned)cameraFlowGroundLeadMs(), g.asleep, (unsigned)g.count, (unsigned)g.fails, (unsigned)g.asleepMs,
      (unsigned)g.holdLeftMs, g.awakeMs == UINT32_MAX ? -1 : (int)g.awakeMs,
      autonomyActive() ? "-" : (clkSlow ? (clkNvs ? "13" : "13(default)") : "20"), clk.xclkHz / 1e6, clk.fellBack);
}

/* The live pulses (hpoint, duty 0..255 per motor) show the placement: above
 * 50 % duty M1,M3 at 0 and M2,M4 at 255 - duty (aligned bits). Out of line,
 * so the command dispatcher's inlining and stack frame stay as they were. */
static void __attribute__((noinline)) pwmReport(void)
{
  uint32_t hp[NBR_OF_MOTORS], dt[NBR_OF_MOTORS], aligned = 0;
  if (motorsGetPulses(hp, dt, &aligned)) {
    out("OK pwm %u hpoint=%u,%u,%u,%u duty=%u,%u,%u,%u aligned=0x%x", (unsigned)motorsGetPwmFreq(),
        (unsigned)hp[0], (unsigned)hp[1], (unsigned)hp[2], (unsigned)hp[3],
        (unsigned)dt[0], (unsigned)dt[1], (unsigned)dt[2], (unsigned)dt[3], (unsigned)aligned);
  } else {
    out("OK pwm %u", (unsigned)motorsGetPwmFreq());
  }
}

OUT_OF_LINE static void cmdPwm(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    pwmReport();
    return;
  }
  uint32_t hz = MOTORS_PWM_FREQ_STOCK;
  if (strcmp(argv[1], "stock") != 0) {
    char *e;
    unsigned long v = strtoul(argv[1], &e, 10);
    if (*e != '\0' || v < MOTORS_PWM_FREQ_MIN || v > MOTORS_PWM_FREQ_MAX) {
      out("ERR pwm: usage pwm <%d..%d> | pwm stock", MOTORS_PWM_FREQ_MIN, MOTORS_PWM_FREQ_MAX);
      return;
    }
    hz = (uint32_t)v;
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR pwm: %s", why);
    return;
  }
  /* Live first (the driver may refuse), then stored; a store refused by the
   * interlock puts the previous frequency back. */
  const uint32_t prevHz = motorsGetPwmFreq();
  if (!motorsSetPwmFreq(hz)) {
    changeEnd();
    out("ERR pwm: driver refused %u Hz (motors not stopped?)", (unsigned)hz);
    return;
  }
  const esp_err_t err = motorPwmStoreSave(hz == MOTORS_PWM_FREQ_STOCK ? 0 : hz);
  if (err == ESP_ERR_INVALID_STATE) {
    motorsSetPwmFreq(prevHz);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR pwm: live %u Hz but NVS save failed (%s)", (unsigned)hz, esp_err_to_name(err));
    return;
  }
  out("OK pwm %u stored=nvs", (unsigned)motorsGetPwmFreq());
}

OUT_OF_LINE static void cmdSoft(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    out("OK soft %s", stabilizerSoftStartEnabled() ? "on" : "off");
    return;
  }
  bool on;
  if (strcmp(argv[1], "on") == 0) {
    on = true;
  } else if (strcmp(argv[1], "off") == 0) {
    on = false;
  } else {
    out("ERR soft: usage soft [on|off]");
    return;
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR soft: %s", why);
    return;
  }
  const esp_err_t err = softStartStoreSave(on);
  if (err == ESP_OK) {
    stabilizerSetSoftStart(on);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR soft: NVS save failed (%s); unchanged", esp_err_to_name(err));
    return;
  }
  out("OK soft %s stored=nvs", on ? "on" : "off");
}

OUT_OF_LINE static void cmdPlock(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    out("OK plock %s rlocked=%d", stabilizerRestartLockEnabled() ? "on" : "off", stabilizerRestartLocked());
    return;
  }
  bool on;
  if (strcmp(argv[1], "on") == 0) {
    on = true;
  } else if (strcmp(argv[1], "off") == 0) {
    on = false;
  } else {
    out("ERR plock: usage plock [on|off]");
    return;
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR plock: %s", why);
    return;
  }
  const esp_err_t err = restartLockStoreSave(on);
  if (err == ESP_OK) {
    stabilizerSetRestartLock(on);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR plock: NVS save failed (%s); unchanged", esp_err_to_name(err));
    return;
  }
  out("OK plock %s stored=nvs rlocked=%d", on ? "on" : "off", stabilizerRestartLocked());
}

OUT_OF_LINE static void cmdGcap(int argc, char **argv)
{
  const char *why;
  if (argc < 2) {
    out("OK gcap %u", stabilizerGetThrustCap());
    return;
  }
  uint32_t cap = 0;
  if (strcmp(argv[1], "off") != 0) {
    char *e;
    unsigned long v = strtoul(argv[1], &e, 10);
    if (*e != '\0' || v < 1000 || v > 60000) {
      out("ERR gcap: usage gcap <1000..60000> | gcap off");
      return;
    }
    cap = (uint32_t)v;
  }
  if (autonomyActive()) {
    /* docs/autonomy-a2b.md 7.3: no NVS write. A BENCH sortie may lower its
     * cap in RAM (the state machine follows); nothing else changes while
     * active. In FLIGHT the cap is 0, "no cap": a mid-air cap would sink. */
    autoStatusMsg_t m;
    autonomyGetStatus(&m);
    const uint16_t cur = stabilizerGetThrustCap();
    if (m.mode == AUTO_MODE_BENCH && cap > 0 && cur > 0 && cap < cur) {
      stabilizerSetThrustCap((uint16_t)cap);
      out("OK gcap %u stored=ram note=autonomy_active", (unsigned)cap);
    } else {
      out("ERR busy: autonomy active");
    }
    return;
  }
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR gcap: %s", why);
    return;
  }
  const esp_err_t err = thrustCapStoreSave((uint16_t)cap);
  if (err == ESP_OK) {
    stabilizerSetThrustCap((uint16_t)cap);
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR gcap: NVS save failed (%s); cap unchanged", esp_err_to_name(err));
    return;
  }
  out("OK gcap %u stored=nvs", (unsigned)cap);
}

/* The reset kinds `reboot` can take. A plain software reset (esp_restart) is a
 * core reset: the RTC domain, analog trims, VDD_SPI, flash and PSRAM keep
 * their state. The others reach further, to find one that clears what a
 * software reset taken on USB leaves behind (a power-on does). */
typedef enum { REBOOT_SW, REBOOT_SLEEP, REBOOT_RWDT, REBOOT_SWD } rebootKind_t;

static void rebootNow(rebootKind_t kind)
{
  switch (kind) {
    case REBOOT_SLEEP:
      /* Powers down the digital domain and VDD_SPI (flash, PSRAM); RTC slow
       * memory with the logs stays on. Wakes in 0.3 s (raw reason 0x05). */
      esp_sleep_enable_timer_wakeup(300000);
      esp_deep_sleep_start();
      break;
    case REBOOT_RWDT: {
      /* RTC watchdog, stage 0 resets the main system and the RTC module
       * (raw reason 0x10). ~20000 slow-clock ticks is ~0.15 s. */
      wdt_hal_context_t rwdt;
      wdt_hal_init(&rwdt, WDT_RWDT, 0, false);
      wdt_hal_write_protect_disable(&rwdt);
      wdt_hal_config_stage(&rwdt, WDT_STAGE0, 20000, WDT_STAGE_ACTION_RESET_RTC);
      wdt_hal_enable(&rwdt);
      wdt_hal_write_protect_enable(&rwdt);
      break;
    }
    case REBOOT_SWD:
      /* Super watchdog: stop its automatic feeding and let it expire
       * (raw reason 0x01 if it resets the whole chip, 0x12 if not). */
      REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, RTC_CNTL_SWD_WKEY_VALUE);
      REG_CLR_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN | RTC_CNTL_SWD_DISABLE);
      REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, 0);
      break;
    default:
      esp_restart();
  }
  /* The watchdogs fire within a few seconds; fall back to a software reset. */
  vTaskDelay(M2T(5000));
  esp_restart();
}

OUT_OF_LINE static void cmdReboot(int argc, char **argv)
{
  rebootKind_t kind = REBOOT_SW;
  if (argc >= 2) {
    if (strcmp(argv[1], "sleep") == 0) {
      kind = REBOOT_SLEEP;
    } else if (strcmp(argv[1], "rwdt") == 0) {
      kind = REBOOT_RWDT;
    } else if (strcmp(argv[1], "swd") == 0) {
      kind = REBOOT_SWD;
    } else {
      out("ERR reboot [sleep|rwdt|swd]");
      return;
    }
  }
  const char *why;
  if (autoBusy()) {
    return;
  }
  flightRecord_t r;
  /* Before the stabilizer runs (self-test failed, gyro not calibrated) nothing
   * drives the motors; then only require the outputs to read zero. */
  bool idle = flightRecorderGetLatest(&r) ? motorsIdle(&why) : motorsAllZero();
  if (!idle) {
    out("ERR reboot: %s", flightRecorderGetLatest(&r) ? why : "a motor output is nonzero");
    return;
  }
  /* Nothing may open a flight (and overwrite a saved pre-reset trace) or spin
   * a motor in the last 100 ms. */
  motorsSetInhibit(true);
  crtpCommanderRpytLockThrust();
  out("OK reboot%s%s", argc >= 2 ? " " : "", argc >= 2 ? argv[1] : "");
  vTaskDelay(M2T(100));
  rebootNow(kind);
}

OUT_OF_LINE static void cmdRegs(void)
{
  regSnapPrint(out);
  out("END regs");
}

/* Bench check of the GPIO9 rail meter: with the pad driven high it should read
 * near or at full scale on a healthy rail (65.535 = saturated); driven low it
 * should read ~0. VBATMON ignores the meter while the pad is low. */
OUT_OF_LINE static void cmdRail(void)
{
  const char *why;
  if (autoBusy()) {
    return;
  }
  if (!motorsIdle(&why)) {
    out("ERR rail: %s", why);
    return;
  }
  float hi = 0.0f, lo = 0.0f;
  int sat = 0;
  for (int i = 0; i < 8; i++) {
    float v = analogReadRailBench();
    if (v >= ANALOG_RAIL_SATURATED) {
      sat++;
      v = analogRailFullScale();
    }
    hi += v;
  }
  analogRailDrive(false);
  vTaskDelay(M2T(5));
  for (int i = 0; i < 8; i++) {
    float v = analogReadRailBench();
    lo += (v >= ANALOG_RAIL_SATURATED) ? analogRailFullScale() : v;
  }
  analogRailDrive(true);
  vTaskDelay(M2T(5));
  float again = analogReadRailBench();
  if (again >= ANALOG_RAIL_SATURATED) {
    again = analogRailFullScale();
  }
  hi /= 8;
  lo /= 8;
  const bool ok = lo < 0.3f && again > hi - 0.1f;
  out("OK rail high=%.3f saturated=%d/8 low=%.3f again=%.3f fullScale=%.3f battery=%.3f meter=%s drive=%s",
      (double)hi, sat, (double)lo, (double)again, (double)analogRailFullScale(),
      (double)pmMeasureExtBatteryVoltageFast(), DRONE_RAIL_METER_GPIO ? "gpio9" : "off", ok ? "ok" : "FAILED");
}

/* Parses exactly 32 hex digits into 16 bytes. */
static bool parseKeyHex(const char *hex, uint8_t key[AUTO_KEY_STORE_LEN])
{
  if (strlen(hex) != 2 * AUTO_KEY_STORE_LEN) {
    return false;
  }
  for (int i = 0; i < 2 * AUTO_KEY_STORE_LEN; i++) {
    const char c = hex[i];
    int v;
    if (c >= '0' && c <= '9') {
      v = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      v = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
      v = c - 'A' + 10;
    } else {
      return false;
    }
    key[i / 2] = (uint8_t)((i % 2) ? (key[i / 2] | v) : (v << 4));
  }
  return true;
}

/* Fingerprint of the key stored in NVS ("none" if there is none, "active"
 * while autonomy is active: an NVS read also stops the flash cache). */
static void storedKeyFp(char out[9])
{
  uint8_t key[AUTO_KEY_STORE_LEN];
  if (autonomyActive()) {
    strcpy(out, "active");
  } else if (autoKeyStoreLoad(key) == ESP_OK) {
    uint8_t fp[4];
    autoLinkKeyFingerprint(key, fp);
    snprintf(out, 9, "%02x%02x%02x%02x", fp[0], fp[1], fp[2], fp[3]);
  } else {
    strcpy(out, "none");
  }
  memset(key, 0, sizeof(key));
}

/* The key is never echoed: the reply shows its fingerprint only. */
OUT_OF_LINE static void cmdAutoKey(int argc, char **argv)
{
  const char *why;
  autoLinkStats_t st;
  autoLinkGetStats(&st);
  if (argc < 2) {
    char nvsFp[9], runFp[9] = "none";
    storedKeyFp(nvsFp);
    if (st.keyLoaded) {
      snprintf(runFp, sizeof(runFp), "%02x%02x%02x%02x", st.keyFp[0], st.keyFp[1], st.keyFp[2], st.keyFp[3]);
    }
    out("OK autokey stored=%s running=%s", nvsFp, runFp);
    return;
  }
  /* Saving writes flash, which stalls the CPU briefly: only while stopped.
   * Busy comes before any parsing, so any 'autokey <arg>' (the bench check
   * uses 'autokey x') answers busy while active and changes nothing. */
  if (autonomyActive()) {
    memset(argv[1], 0, strlen(argv[1]));   /* it may be a key: wipe it from the line buffer */
    out("ERR busy: autonomy active");
    return;
  }
  const bool clear = strcmp(argv[1], "clear") == 0;
  uint8_t key[AUTO_KEY_STORE_LEN];
  if (!clear) {
    const bool ok = parseKeyHex(argv[1], key);
    memset(argv[1], 0, strlen(argv[1]));   /* wipe the hex from the line buffer */
    if (!ok) {
      memset(key, 0, sizeof(key));
      out("ERR autokey: usage autokey <32 hex digits> | autokey clear");
      return;
    }
  }
  if (!motorsIdle(&why)) {
    memset(key, 0, sizeof(key));
    out("ERR autokey: %s", why);
    return;
  }
  esp_err_t err = clear ? autoKeyStoreErase() : autoKeyStoreSave(key);
  if (storeBusy(err)) {
    memset(key, 0, sizeof(key));
    return;
  }
  if (err != ESP_OK) {
    memset(key, 0, sizeof(key));
    out("ERR autokey: NVS %s failed (%s)", clear ? "erase" : "save", esp_err_to_name(err));
    return;
  }
  if (clear) {
    out("OK autokey cleared stored=none note=takes_effect_after_reboot");
    return;
  }
  uint8_t fp[4];
  autoLinkKeyFingerprint(key, fp);
  memset(key, 0, sizeof(key));
  out("OK autokey stored=nvs fp=%02x%02x%02x%02x note=takes_effect_after_reboot", fp[0], fp[1], fp[2], fp[3]);
}

static const char *autoModeName(uint8_t m)
{
  switch (m) {
    case AUTO_MODE_FLIGHT: return "FLIGHT";
    case AUTO_MODE_DRY: return "DRY";
    case AUTO_MODE_BENCH: return "BENCH";
    default: return "?";
  }
}

/* The ARM guard bitmap as names (auto_fsm.h AUTO_G_*, in bit order). */
static void autoGuardNames(uint32_t g, char *buf, size_t n)
{
  static const char *const names[] = {"LINK", "LOCKS", "STILL", "BATT", "CAMERA", "APP", "GCAP",
                                      "ENABLED", "VZPARAMS", "HEAP", "GESTURE", "KILL", "MODE"};
  size_t k = 0;
  buf[0] = '\0';
  for (unsigned i = 0; i < 32 && k + 1 < n; i++) {
    if (g & (1u << i)) {
      const char *nm = i < sizeof(names) / sizeof(names[0]) ? names[i] : "?";
      k += (size_t)snprintf(buf + k, n - k, "%s%s", k ? "|" : "", nm);
    }
  }
  if (k == 0) {
    snprintf(buf, n, "none");
  }
}

/* 'auto': the A1 link line, then the autonomy state and the A2b link counters. */
OUT_OF_LINE static void cmdAutoLink(void)
{
  autoLinkStats_t st;
  autoLinkGetStats(&st);
  char nvsFp[9];
  storedKeyFp(nvsFp);
  char last[32];
  if (st.anyAck) {
    snprintf(last, sizeof(last), "0x%02x:%s", st.lastAckType, autoLinkResultName(st.lastAckResult));
  } else {
    strcpy(last, "none");
  }
  char runFp[9] = "none";
  if (st.keyLoaded) {
    snprintf(runFp, sizeof(runFp), "%02x%02x%02x%02x", st.keyFp[0], st.keyFp[1], st.keyFp[2], st.keyFp[3]);
  }
  out("OK auto key=%d fp=%s nvsFp=%s port=%s session=0x%08x peer=%u.%u.%u.%u:%u hbAgeMs=%d "
      "rx=%u accepted=%u authFails=%u rateDrops=%u dups=%u oldSeq=%u badSession=%u tx=%u txErr=%u txNoMem=%u "
      "photos=%u photoBusy=%d photoId=%u last=%s stackFree=%u "
      "state=%s resetLocked=%d linkStack=%s kills=%u autoCmds=%u cmdTimeouts=%u cmdBusy=%u cmdTakenUnknown=%u "
      "autoStatusTx=%u autoCaptures=%u photosPreempted=%u photoHeldBusy=%u killRateDrops=%u",
      st.keyLoaded, runFp, nvsFp,
      st.portOpen ? "open" : (st.running ? "opening" : "off"), (unsigned)st.session,
      st.peerIp[0], st.peerIp[1], st.peerIp[2], st.peerIp[3], st.peerPort,
      st.hbAgeMs == UINT32_MAX ? -1 : (int)st.hbAgeMs,
      (unsigned)st.rxDatagrams, (unsigned)st.accepted, (unsigned)st.authFails, (unsigned)st.rateDrops,
      (unsigned)st.dups, (unsigned)st.oldSeq, (unsigned)st.badSession, (unsigned)st.txPackets,
      (unsigned)st.txErrors, (unsigned)st.txNoMem, (unsigned)st.photos, st.photoBusy, st.photoId, last,
      (unsigned)st.stackFree, autoStateName(autonomyState()), autonomyResetLocked(),
      st.stackInPsram ? "psram" : "internal", (unsigned)st.kills, (unsigned)st.autoCmds,
      (unsigned)st.cmdTimeouts, (unsigned)st.cmdBusy, (unsigned)st.cmdTakenUnknown, (unsigned)st.autoStatusTx,
      (unsigned)st.autoCaptures, (unsigned)st.photosPreempted, (unsigned)st.photoHeldBusy,
      (unsigned)st.killRateDrops);
}

OUT_OF_LINE static void cmdAutoStatus(void)
{
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  autoLinkStats_t ls;
  autoLinkGetStats(&ls);
  rpytLinkStats_t app;
  crtpCommanderRpytGetLinkStats(&app);
  char guards[112];
  autoGuardNames(m.failedGuards, guards, sizeof(guards));
  const uint32_t nowMs = T2M(xTaskGetTickCount());
  autoMvMargins_t mg;
  autoMotionGetMargins(&mg);
  out("OK auto status state=%s mode=%s reason=%s vz=%s vzReason=%s vzLand=%s guards=0x%x(%s) resetLocked=%d "
      "rtcValid=%d rst=%s prev=%s/%s/%s transitions=%u kill=%u session=0x%08x hbAgeMs=%d appAgeMs=%d "
      "camAwake=%d hold=%d camHeight=%d sim=%d waiting=%d waitLeftMs=%u airMs=%u stateMs=%u "
      "vbat=%.3f vmin=%.3f heap=%u heapMin=%u benchSrc=%s linkStack=%s mv=%s/%s why=%s end=%s mvSeq=%u "
      "prog=%d/%d/%d tgt=%d/%d/%d D=%u dFlags=0x%02x hTarget=%d yawOff=%.2f battLow=%d airLeftDs=%u mvSpeed=%u "
      "mvAcc=%ld mvSettle=%ld mvGuards=%u/%u/%u/%u/%u/%u/%u klow=%u lat=%u wfwd=%u mvView=%u holdRes=%u "
      "mvLatK=%u mvFfr=%d",
      autoStateName((autoState_t)m.state), autoModeName(m.mode), autoReasonName((autoReason_t)m.reason),
      vzPhaseName((vzPhase_t)m.vzPhase), vzReasonName((vzReason_t)m.vzReason),
      vzReasonName((vzReason_t)m.vzLandReason), (unsigned)m.failedGuards, guards, autonomyResetLocked(),
      rv.recordWasValid, resetReasonName((esp_reset_reason_t)rv.resetReason),
      autoStateName((autoState_t)rv.prevState), autoReasonName((autoReason_t)rv.prevReason),
      autoModeName(rv.prevMode), (unsigned)rv.transitions, (unsigned)m.killCount, (unsigned)ls.session,
      ls.hbAgeMs == UINT32_MAX ? -1 : (int)ls.hbAgeMs,
      app.packets ? (int)(nowMs - app.lastPacketMs) : -1, !!(m.flags & AUTO_STF_CAM_AWAKE),
      !!(m.flags & AUTO_STF_HOLD), !!(m.flags & AUTO_STF_CAM_HEIGHT), !!(m.flags & AUTO_STF_SIM),
      !!(m.flags & AUTO_STF_WAITING), (unsigned)m.waitLeftMs, (unsigned)m.airMs, (unsigned)m.stateMs,
      m.vbatMv / 1000.0, autonomySettingGet(AUTO_SET_VMIN) / 1000.0,
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
      autonomyBenchSource() == AUTO_BENCH_SRC_IMU ? "imu" : "sim", ls.stackInPsram ? "psram" : "internal",
      autoMvStateName((autoMvState_t)m.mvState), m.mvKind == AUTO_MVK_MOVE ? "MOVE" : m.mvKind == AUTO_MVK_YAW ? "YAW" : "-",
      autoMvWhyName((autoMvWhy_t)m.mvWhy), autoMvEndName((autoMvEnd_t)m.mvEnd), (unsigned)m.mvSeq, m.progRightCm,
      m.progFwdCm, m.progUpCm, m.tgtRightCm, m.tgtFwdCm, m.tgtUpCm, (unsigned)m.sceneDCm, (unsigned)m.mvFlags,
      m.hTargetCm, m.yawOffCdeg / 100.0, m.battLow, (unsigned)m.airLeftDs, (unsigned)m.mvSpeedPct,
      /* the fast steps' acceleration (cm/s^2) and settle floor (ms), NVS mvacc / mvsettle (2026-10-17): as mvSpeed, in
       * IDLE the next ARM's, in a sortie the latched ones (no settings writer runs outside IDLE) */
      (long)autonomySettingGet(AUTO_SET_MVACC), (long)autonomySettingGet(AUTO_SET_MVSETTLE),
      /* the guards in force (2026-10-18, auto_motion.h "guards off"; the latched margins, in a sortie the ones its ARM
       * took, in IDLE the last sortie's or the defaults): floor/plan/stop/progm/progr/latm/near cm and %, 0 = off; the
       * expansion scale x 100, the sideways cap x 100 (0 off), the expansion rate cap x 100 */
      (unsigned)mg.floorCm, (unsigned)mg.planCm, (unsigned)mg.stopCm, (unsigned)mg.progCm, (unsigned)mg.progPct,
      (unsigned)mg.latCm, (unsigned)mg.nearCm, (unsigned)mg.kLowPct, (unsigned)mg.latPct,
      (unsigned)autoMotionGetFwdRate(),
      /* the VIEW_FAST timer in force, ms (NVS mvview, 2026-10-22; the latched one, in IDLE the last ARM's or the
       * profile's) */
      (unsigned)autoMotionGetViewDeb(),
      /* the scene gate's residual limit in force, grey levels x 10 (NVS holdres, 2026-10-22; 0 off) */
      (unsigned)cameraFlowSceneResidual(), (unsigned)cameraFlowMoveLatGain(), (int)autoMotionGetFfRoll());
  /* hold and report (2026-10-15, docs/autonomy-a2b.md 22): AUTO_STATUS bytes 96-135 */
  out("OK auto report ev=%s/%u age=%ums n=%u prev=%s/%u,%s/%u holdBits=0x%02x camBits=0x%02x camOffMs=%u "
      "sinkInMs=%u contacts=%u peak=%.2fg/%udps force=%.2f/%.2fg yawAtContact=%.2f loopGaps=%u sinceCmdDs=%u airDs=%u "
      "khatPm=%u capQ=%u",
      autoEventName((autoEvent_t)m.evCode), (unsigned)m.evDetail, (unsigned)m.evAgeMs, (unsigned)m.evCount,
      autoEventName((autoEvent_t)m.ev1Code), (unsigned)m.ev1Detail, autoEventName((autoEvent_t)m.ev2Code),
      (unsigned)m.ev2Detail, (unsigned)m.holdBits, (unsigned)m.camBits, (unsigned)m.camOffMs, (unsigned)m.sinkInMs,
      (unsigned)m.contacts, m.peakAccCg / 100.0, (unsigned)m.peakRateDps, m.contactAxCg / 100.0, m.contactAyCg / 100.0,
      m.contactYawCdeg / 100.0, (unsigned)m.loopGaps, (unsigned)m.sinceCmdDs, (unsigned)m.airDs, (unsigned)m.khatPm,
      (unsigned)m.capQ);
}

/* Values from RAM (what the next ARM uses). The source is read from NVS in
 * IDLE only: an NVS read also stops the flash cache. */
OUT_OF_LINE static void cmdAutoGet(void)
{
  const bool active = autonomyActive();
  char line[AUTO_SET_COUNT * 28];
  size_t k = 0;
  for (int i = 0; i < AUTO_SET_COUNT && k + 1 < sizeof(line); i++) {
    const autoSettingInfo_t *s = autoSettingInfo((autoSettingId_t)i);
    const int32_t v = autonomySettingGet((autoSettingId_t)i);
    const char *src = "ram";
    if (!active) {
      int32_t nv = 0;
      esp_err_t err = autoSettingStoreLoad(s->name, (uint8_t)s->type, &nv);
      bool legacy = false;
      if (err == ESP_ERR_NVS_NOT_FOUND && s->legacyType != 0) {
        err = autoSettingStoreLoad(s->name, (uint8_t)s->legacyType, &nv);   /* autowait's u16 item (a2b 18.2) */
        legacy = err == ESP_OK;
      }
      src = (err == ESP_OK && nv == v) ? (legacy ? "legacy" : "nvs")
                                       : (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND ? "default" : "bad");
    }
    k += (size_t)snprintf(line + k, sizeof(line) - k, " %s=%ld(%s)", s->name, (long)v, src);
  }
  out("OK auto get%s", line);
}

/* The range of a refused value: "0 (off) or lo..hi" for a key whose 0 is off (autoSettingOnMin), else "lo..hi". */
OUT_OF_LINE static void autoSetRangeErr(const autoSettingInfo_t *s, autoSettingId_t id)
{
  const int32_t on = autoSettingOnMin(id);
  if (id == AUTO_SET_MVVIEW) {   /* its 0 is the profile's timer, not off (the timer only ever gets longer) */
    out("ERR auto set: %s must be 0 (the profile's %ld ms) or %ld..%ld %s", s->name, (long)on, (long)on,
        (long)s->max, s->unit);
  } else if (on > 0) {
    out("ERR auto set: %s must be 0 (off) or %ld..%ld %s", s->name, (long)on, (long)s->max, s->unit);
  } else {
    out("ERR auto set: %s must be %ld..%ld %s", s->name, (long)s->min, (long)s->max, s->unit);
  }
}

OUT_OF_LINE static void cmdAutoSet(int argc, char **argv)
{
  if (argc < 4) {
    out("ERR auto set: usage auto set <key> <value> (keys: autoon appreq autogest vmin autoair autowait autoaccel "
        "vzK vzBair vzcaplo photocol cvqlate cvqlates cvt2k cvt2r mvspeed mvfloor mvfstop mvprogm mvprogr mvlatm "
        "mvnear mvfix mvacc mvsettle cvhb mvklow mvlat mvwmax mvview vzest vzestms holdres)");
    return;
  }
  const int id = autoSettingFind(argv[2]);
  if (id < 0) {
    out("ERR auto set: unknown key '%s'", argv[2]);
    return;
  }
  const autoSettingInfo_t *s = autoSettingInfo((autoSettingId_t)id);
  char *e;
  const long v = strtol(argv[3], &e, 10);
  if (*e != '\0' || !autoSettingInRange((autoSettingId_t)id, (int32_t)v) || v != (long)(int32_t)v) {
    autoSetRangeErr(s, (autoSettingId_t)id);
    return;
  }
  if (autoBusy()) {
    return;
  }
  const char *why = "";
  if (!autonomySettingSet((autoSettingId_t)id, (int32_t)v, &why)) {
    if (strcmp(why, "range") == 0) {
      autoSetRangeErr(s, (autoSettingId_t)id);
    } else if (strcmp(why, "nvs") == 0) {
      out("ERR auto set: NVS write failed (%s unchanged)", s->name);
    } else {
      out("ERR %s", why);
    }
    return;
  }
  out("OK auto set %s=%ld stored=nvs", s->name, v);
}

OUT_OF_LINE static void cmdAutoReset(int argc, char **argv)
{
  const int id = argc >= 3 ? autoSettingFind(argv[2]) : -1;
  if (id < 0) {
    out("ERR auto reset: usage auto reset <key>");
    return;
  }
  if (autoBusy()) {
    return;
  }
  const autoSettingInfo_t *s = autoSettingInfo((autoSettingId_t)id);
  const char *why = "";
  if (!autonomySettingReset((autoSettingId_t)id, &why)) {
    if (strcmp(why, "nvs") == 0) {
      out("ERR auto reset: NVS erase failed (%s unchanged)", s->name);
    } else {
      out("ERR %s", why);
    }
    return;
  }
  out("OK auto reset %s=%ld(default) stored=erased", s->name, (long)s->def);
}

OUT_OF_LINE static void cmdAutoRtc(void)
{
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  out("OK auto rtc locked=%d valid=%d rst=%s(%d) prev=%s/%s/%s prevLocked=%u prevTransitions=%u transitions=%u",
      rv.locked, rv.recordWasValid, resetReasonName((esp_reset_reason_t)rv.resetReason), rv.resetReason,
      autoStateName((autoState_t)rv.prevState), autoReasonName((autoReason_t)rv.prevReason),
      autoModeName(rv.prevMode), (unsigned)rv.prevLocked, (unsigned)rv.prevTransitions,
      (unsigned)rv.transitions);
  for (unsigned i = 0; i < rv.n; i++) {
    out("T %u ms=%u from=%s to=%s reason=%s vz=%s", i, (unsigned)rv.trans[i].ms,
        autoStateName((autoState_t)rv.trans[i].from), autoStateName((autoState_t)rv.trans[i].to),
        autoReasonName((autoReason_t)rv.trans[i].reason), vzReasonName((vzReason_t)rv.trans[i].vzReason));
  }
  out("END auto rtc n=%u", (unsigned)rv.n);
}

OUT_OF_LINE static void cmdAutoProf(int argc, char **argv)
{
  if (argc >= 3 && strcmp(argv[2], "reset") == 0) {
    /* The toggles cannot change while autonomy is active (cmdBypass), so the counters the stabilizer clears at
     * its next IDLE step start under exactly this mask. */
    benchBypassSeen = benchBypassMask;
    autonomyProfileReset();
    out("OK auto prof reset (at the next IDLE step)");
    return;
  }
  autonomyProfile_t p;
  autonomyGetProfile(&p);
  const uint32_t lps10 = p.activeMs ? (uint32_t)((uint64_t)p.activeLoops * 10000u / p.activeMs) : 0;
  out("OK auto prof loops=%u full=%u fastMaxUs=%u idleAvgUs=%u.%u idleMaxUs=%u activeAvgUs=%u.%u activeMaxUs=%u "
      "activeLoops=%u activeMs=%u activeLoopsPerS=%u.%u cmdLatMaxMs=%u stabStackFree=%d flashRefused=%u "
      "armsBlocked=%u upMs=%u bypass=0x%02x bypassSeen=0x%02x shiftRefused=%u",
      (unsigned)p.loops, (unsigned)p.fullSteps, (unsigned)p.fastMaxUs, (unsigned)(p.idleAvgUs10 / 10),
      (unsigned)(p.idleAvgUs10 % 10), (unsigned)p.idleMaxUs, (unsigned)(p.activeAvgUs10 / 10),
      (unsigned)(p.activeAvgUs10 % 10), (unsigned)p.activeMaxUs, (unsigned)p.activeLoops, (unsigned)p.activeMs,
      (unsigned)(lps10 / 10), (unsigned)(lps10 % 10), (unsigned)p.cmdLatencyMaxMs,
      p.stabStackFreeMin == UINT32_MAX ? -1 : (int)p.stabStackFreeMin, (unsigned)p.flashRefused,
      (unsigned)p.armsBlocked, (unsigned)T2M(xTaskGetTickCount()), (unsigned)benchBypassMask,
      (unsigned)(benchBypassSeen | benchBypassMask), (unsigned)p.shiftRefused);
}

/* 'auto unlock' (user decision 2026-10-12, evening; docs/autonomy-a2b.md
 * section 6): clears the autonomy reset-lock as a battery power-on does.
 * Typing it on this console, which only the USB-Serial-JTAG port feeds
 * (pollRx; nothing on Wi-Fi, CRTP or the autonomy link reaches handleLine),
 * is the deliberate physical act after an unexpected reset. IDLE with the
 * motors stopped only. It changes no other lock: the restart lock (plock),
 * the motor driver's STOP latch after a KILL, the motor watchdog and the
 * emergency stop stay as they are, and the reply shows them; 'auto status'
 * then gives resetLocked=0 with rst= still the real reset reason. */
OUT_OF_LINE static void cmdAutoUnlock(void)
{
  if (autoBusy()) {
    return;
  }
  const char *why = "";
  if (!motorsIdle(&why)) {
    out("ERR auto unlock: %s", why);
    return;
  }
  bool was = false;
  if (!autonomyUsbUnlock(&was, &why)) {
    if (strcmp(why, "busy: autonomy active") == 0) {
      out("ERR %s", why);
    } else {
      out("ERR auto unlock: %s", why);
    }
    return;
  }
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  out("OK auto unlock resetLocked=%d was=%d rst=%s (other locks unchanged: plock=%s rlocked=%d motorsReady=%d "
      "estop=%d mguard=%s mguardTripped=%d)",
      autonomyResetLocked(), was, resetReasonName((esp_reset_reason_t)rv.resetReason),
      stabilizerRestartLockEnabled() ? "on" : "off", stabilizerRestartLocked(), motorsReadyNoLock(),
      stabilizerEmergencyStopActive(), motorGuardEnabled() ? "on" : "off", motorGuardTripped());
}

/* 'auto ...' (docs/autonomy-a2b.md section 7.2). USB only: nothing here is
 * reachable over Wi-Fi. */
OUT_OF_LINE static void cmdAuto(int argc, char **argv)
{
  const char *sub = argc >= 2 ? argv[1] : "";
  if (argc < 2) {
    cmdAutoLink();
  } else if (strcmp(sub, "status") == 0) {
    cmdAutoStatus();
  } else if (strcmp(sub, "get") == 0) {
    cmdAutoGet();
  } else if (strcmp(sub, "set") == 0) {
    cmdAutoSet(argc, argv);
  } else if (strcmp(sub, "reset") == 0) {
    cmdAutoReset(argc, argv);
  } else if (strcmp(sub, "on") == 0 || strcmp(sub, "off") == 0) {
    if (autoBusy()) {
      return;
    }
    const char *why = "";
    if (!autonomySettingSet(AUTO_SET_AUTOON, strcmp(sub, "on") == 0 ? 1 : 0, &why)) {
      if (strcmp(why, "nvs") == 0) {
        out("ERR auto %s: NVS write failed (autoon unchanged)", sub);
      } else {
        out("ERR %s", why);
      }
      return;
    }
    out("OK auto %s stored=nvs", sub);
  } else if (strcmp(sub, "rtc") == 0) {
    cmdAutoRtc();
  } else if (strcmp(sub, "bench") == 0 && argc >= 3 &&
             (strcmp(argv[2], "sim") == 0 || strcmp(argv[2], "imu") == 0)) {
    if (!changeBegin()) {
      return;
    }
    const bool imu = strcmp(argv[2], "imu") == 0;
    const bool ok = autonomySetBenchSource(imu ? AUTO_BENCH_SRC_IMU : AUTO_BENCH_SRC_SIM);
    changeEnd();
    if (!ok) {
      out("ERR busy: autonomy active");
      return;
    }
    out("OK auto bench %s (not stored)", argv[2]);
  } else if (strcmp(sub, "prof") == 0) {
    cmdAutoProf(argc, argv);
  } else if (strcmp(sub, "unlock") == 0 && argc == 2) {
    cmdAutoUnlock();
  } else {
    out("ERR auto: usage auto | auto status|get|rtc|prof [reset] | auto set <key> <value> | auto reset <key> | "
        "auto on|off | auto bench sim|imu | auto unlock");
  }
}

OUT_OF_LINE static void cmdMguardTest(int argc, char **argv)
{
  if (autoBusy()) {
    return;
  }
  /* A ground-test cap of at most 20000 keeps the drone on the table even if
   * the props are on; the props must be off anyway. */
  const unsigned cap = stabilizerGetThrustCap();
  if (cap < 1 || cap > MGUARD_TEST_GCAP) {
    out("ERR mguard test: bench only; set gcap <= %u first (props off)", MGUARD_TEST_GCAP);
    return;
  }
  const char *kind = (argc >= 3) ? argv[2] : "";
  if (strcmp(kind, "panic") == 0) {
    out("OK mguard test panic: abort() now");
    vTaskDelay(M2T(50));
    abort();
  }
  const bool hang = strcmp(kind, "hang") == 0;
  char *e = NULL;
  const unsigned long ms = (argc >= 4) ? strtoul(argv[3], &e, 10) : 0;
  if ((!hang && strcmp(kind, "imu") != 0) || argc < 4 || *e != '\0' || ms < 1 || ms > MGUARD_TEST_MAX_MS) {
    out("ERR mguard test: usage mguard test hang|imu <1..%u> | mguard test panic", MGUARD_TEST_MAX_MS);
    return;
  }
  if (motorGuardTripped() || !motorGuardEnabled()) {
    out("ERR mguard test: guard %s", motorGuardTripped() ? "already tripped (reboot)" : "off");
    return;
  }
  const bool motorsOn = motorsAnyDutyNonZero();
  if (hang) {
    stabilizerFaultHang((uint32_t)ms);
  } else {
    sensorsMpu6050FaultHoldData((uint32_t)ms);
  }
  out("OK mguard test %s %lu ms motorsOn=%d freezeMs=%u (trips if a motor runs and the stall reaches it)",
      kind, ms, motorsOn, (unsigned)(MOTOR_GUARD_FREEZE_US / 1000));
}

OUT_OF_LINE static void cmdMguard(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "status";
  if (strcmp(sub, "status") == 0) {
    motorGuardStats_t st;
    motorGuardGetStats(&st);
    out("OK mguard %s tripped=%d trips=%u tripMs=%u alive=%u maxStallUs=%u lateCalls=%u maxCallGapUs=%u "
        "callbacks=%u freezeMs=%u lateMs=%u periodMs=%u",
        st.enabled ? "on" : "off", st.tripped, (unsigned)st.trips, (unsigned)st.tripUptimeMs,
        (unsigned)st.aliveCount, (unsigned)st.maxStallUs, (unsigned)st.lateCalls, (unsigned)st.maxCallGapUs,
        (unsigned)st.callbacks, (unsigned)(MOTOR_GUARD_FREEZE_US / 1000), (unsigned)(MOTOR_GUARD_LATE_US / 1000),
        (unsigned)(MOTOR_GUARD_PERIOD_US / 1000));
    return;
  }
  if (strcmp(sub, "test") == 0) {
    cmdMguardTest(argc, argv);
    return;
  }
  bool on;
  if (strcmp(sub, "on") == 0) {
    on = true;
  } else if (strcmp(sub, "off") == 0) {
    on = false;
  } else {
    out("ERR mguard: usage mguard [status|on|off] | mguard test hang|imu <ms> | mguard test panic");
    return;
  }
  const char *why;
  if (!changeBegin()) {
    return;
  }
  if (!motorsIdle(&why)) {
    changeEnd();
    out("ERR mguard: %s", why);
    return;
  }
  const esp_err_t err = motorGuardStoreSave(on);
  if (err == ESP_OK) {
    motorGuardSetEnabled(on);   /* off never clears a trip: only a reboot does */
  }
  changeEnd();
  if (storeBusy(err)) {
    return;
  }
  if (err != ESP_OK) {
    out("ERR mguard: NVS save failed (%s); unchanged", esp_err_to_name(err));
    return;
  }
  out("OK mguard %s stored=nvs tripped=%d", on ? "on" : "off", motorGuardTripped());
}

OUT_OF_LINE static void cmdRec(int argc, char **argv)
{
  const char *sub = (argc >= 2) ? argv[1] : "info";

  if (strcmp(sub, "info") == 0) {
    out("OK rec n=%u cap=%u rate_hz=%d psramFree=%u fields=%s", (unsigned)flightRecorderCount(),
        (unsigned)flightRecorderCapacity(), FLIGHT_RECORDER_RATE_HZ,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), RECORD_FIELDS);
  } else if (strcmp(sub, "dump") == 0) {
    if (autoBusy()) {   /* the dump stops the recording */
      return;
    }
    flightRecord_t latest;
    if (flightRecorderGetLatest(&latest) && latest.thrust > 0) {
      out("ERR rec dump: land and set throttle to zero first");
      return;
    }
    flightRecorderFreeze(true);
    uint32_t n = flightRecorderCount();
    uint32_t droppedBefore = droppedLines;
    out("OK rec dump n=%u fields=%s", (unsigned)n, RECORD_FIELDS);
    for (uint32_t i = 0; i < n; i++) {
      flightRecord_t r;
      if (flightRecorderGet(i, &r)) {
        outRecord('R', i, &r);
      }
    }
    flightRecorderFreeze(false);
    out("END rec n=%u dropped=%u", (unsigned)n, (unsigned)(droppedLines - droppedBefore));
  } else if (strcmp(sub, "clear") == 0) {
    if (autoBusy()) {
      return;
    }
    flightRecorderClear();
    out("OK rec clear");
  } else {
    out("ERR rec: unknown subcommand '%s'", sub);
  }
}

static void handleLine(char *text)
{
  char *argv[6];
  int argc = 0;
  char *save = NULL;

  for (char *tok = strtok_r(text, " \t", &save); tok && argc < 6; tok = strtok_r(NULL, " \t", &save)) {
    argv[argc++] = tok;
  }
  if (argc == 0) {
    return;
  }

  if (strcmp(argv[0], "help") == 0) {
    cmdHelp();
  } else if (strcmp(argv[0], "status") == 0) {
    cmdStatus();
  } else if (strcmp(argv[0], "stream") == 0) {
    cmdStream(argc, argv);
  } else if (strcmp(argv[0], "measure") == 0) {
    cmdMeasure(argc, argv);
  } else if (strcmp(argv[0], "cal") == 0) {
    cmdCal(argc, argv);
  } else if (strcmp(argv[0], "cam") == 0) {
    cmdCam(argc, argv);
  } else if (strcmp(argv[0], "hold") == 0) {
    cmdHold(argc, argv);
  } else if (strcmp(argv[0], "holdk") == 0) {
    cmdHoldK(argc, argv);
  } else if (strcmp(argv[0], "facc") == 0) {
    cmdFlightAcc(argc, argv);
  } else if (strcmp(argv[0], "atrim") == 0) {
    cmdAttTrim(argc, argv);
  } else if (strcmp(argv[0], "gtc") == 0) {
    cmdGyroTc(argc, argv);
  } else if (strcmp(argv[0], "atc") == 0) {
    cmdAccTc(argc, argv);
  } else if (strcmp(argv[0], "cool") == 0) {
    cmdCool(argc, argv);
  } else if (strcmp(argv[0], "mki") == 0) {
    cmdMki(argc, argv);
  } else if (strcmp(argv[0], "flowfit") == 0) {
    cmdFlowFit(argc, argv);
  } else if (strcmp(argv[0], "bypass") == 0) {
    cmdBypass(argc, argv);
  } else if (strcmp(argv[0], "crash") == 0) {
    cmdCrash(argc, argv);
  } else if (strcmp(argv[0], "flights") == 0) {
    cmdFlights(argc, argv);
  } else if (strcmp(argv[0], "camvz") == 0) {
    cmdCamVz(argc, argv);
  } else if (strcmp(argv[0], "live") == 0) {
    cmdLive();
  } else if (strcmp(argv[0], "gcap") == 0) {
    cmdGcap(argc, argv);
  } else if (strcmp(argv[0], "reboot") == 0) {
    cmdReboot(argc, argv);
  } else if (strcmp(argv[0], "tkoff") == 0) {
    cmdTakeoff(argc, argv);
  } else if (strcmp(argv[0], "regs") == 0) {
    cmdRegs();
  } else if (strcmp(argv[0], "rail") == 0) {
    cmdRail();
  } else if (strcmp(argv[0], "net") == 0) {
    cmdNet();
  } else if (strcmp(argv[0], "pwm") == 0) {
    cmdPwm(argc, argv);
  } else if (strcmp(argv[0], "soft") == 0) {
    cmdSoft(argc, argv);
  } else if (strcmp(argv[0], "plock") == 0) {
    cmdPlock(argc, argv);
  } else if (strcmp(argv[0], "yawlock") == 0) {
    cmdYawLock(argc, argv);
  } else if (strcmp(argv[0], "trim") == 0) {
    cmdTrim(argc, argv);
  } else if (strcmp(argv[0], "rec") == 0) {
    cmdRec(argc, argv);
  } else if (strcmp(argv[0], "autokey") == 0) {
    cmdAutoKey(argc, argv);
  } else if (strcmp(argv[0], "auto") == 0) {
    cmdAuto(argc, argv);
  } else if (strcmp(argv[0], "mguard") == 0) {
    cmdMguard(argc, argv);
  } else {
    out("ERR unknown command '%s' (try help)", argv[0]);
  }
}

static void pollRx(void)
{
  uint8_t buf[64];

  while (usb_serial_jtag_ll_rxfifo_data_available()) {
    uint32_t n = usb_serial_jtag_ll_read_rxfifo(buf, sizeof(buf));
    for (uint32_t i = 0; i < n; i++) {
      char c = (char)buf[i];
      if (c == '\r' || c == '\n') {
        if (!lineOverflow && lineLen > 0) {
          line[lineLen] = '\0';
          handleLine(line);
        } else if (lineOverflow) {
          out("ERR line too long");
        }
        lineLen = 0;
        lineOverflow = false;
      } else if (lineLen < sizeof(line) - 1) {
        line[lineLen++] = c;
      } else {
        lineOverflow = true;
      }
    }
  }
}

/* Out of line, so its record (124 B) is not in the task's frame under every command. */
OUT_OF_LINE static void streamTick(void)
{
  uint32_t now = T2M(xTaskGetTickCount());
  if ((int32_t)(now - nextStreamMs) >= 0) {
    nextStreamMs = now + streamPeriodMs;
    flightRecord_t r;
    if (flightRecorderGetLatest(&r)) {
      outRecord('S', streamSeq++, &r);
    }
  }
}

static void usbDiagTask(void *param)
{
  uint32_t lastTickS = 0;
  while (1) {
    const uint32_t upS = T2M(xTaskGetTickCount()) / 1000;
    if (upS != lastTickS) {
      crashLogTick(upS);
      lastTickS = upS;
    }
    pollRx();

    if (streamPeriodMs > 0) {
      streamTick();
    }

    vTaskDelay(M2T(LOOP_PERIOD_MS));
  }
}

void usbDiagInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  outBuf = heap_caps_malloc(OUT_MAX_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (outBuf == NULL) {
    outBuf = heap_caps_malloc(OUT_MAX_LEN, MALLOC_CAP_8BIT);   /* no PSRAM: internal, as before */
  }
  STATIC_MEM_TASK_CREATE(usbDiagTask, usbDiagTask, USBDIAG_TASK_NAME, NULL, USBDIAG_TASK_PRI);
  isInit = true;
}
