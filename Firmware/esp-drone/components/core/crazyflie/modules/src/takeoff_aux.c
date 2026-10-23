/*
 * ESP-Drone Firmware
 *
 * takeoff_aux.c - Lighten the XIAO's load during a take-off spin-up (and run
 * the attitude-trim learner, att_trim_auto.c, in the same low-priority task)
 *
 * At lift-off on a part-charged battery the XIAO's 3.3 V rail sits a few
 * tenths of a volt above the brownout level, and short current spikes reset
 * the drone. Wi-Fi transmit bursts are such spikes; the phone is next to the
 * drone at take-off, so the TX power can drop to the minimum for those
 * seconds. (The camera powers itself down in camera_flow.c.) The same task
 * samples the internal heap for autonomy's ARM guard and stores a
 * vertical-controller gain learned in an autonomous flight (autonomyPoll()).
 */
#include "takeoff_aux.h"

#include "FreeRTOS.h"
#include "task.h"

#include "att_trim_auto.h"
#include "autonomy.h"
#include "imu_level_cal_store.h"
#include "stabilizer.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "TKOFF"
#include "debug_cf.h"

#define TAKEOFF_AUX_PERIOD_MS 10

static volatile bool txLowEnabled = true;

static void takeoffAuxTask(void *param)
{
  bool low = false;
  while (1) {
    const bool want = txLowEnabled && stabilizerTakeoffWindow();
    if (want != low) {
      esp_err_t err = wifiSetTxLow(want);
      if (err == ESP_OK) {
        low = want;
      } else {
        DEBUG_PRINTW("TX power %s failed (%s)", want ? "low" : "normal", esp_err_to_name(err));
        vTaskDelay(M2T(500));
      }
    }
    attTrimAutoPoll();
    autonomyPoll();   /* G9's heap reading; a learned vzK is stored here, on the ground (flash interlock) */
    vTaskDelay(M2T(TAKEOFF_AUX_PERIOD_MS));
  }
}

void takeoffAuxInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  bool on = true;
  if (txLowStoreLoad(&on) == ESP_OK) {
    txLowEnabled = on;
  }
  attTrimAutoInit();
  if (xTaskCreatePinnedToCore(takeoffAuxTask, "takeoff_aux", 3072, NULL, 2, NULL, 0) == pdPASS) {
    isInit = true;
  } else {
    DEBUG_PRINTE("takeoff_aux task not created: no take-off TX drop, trim learning or vzK save; "
                 "autonomy ARMs refused HEAP (G9)");
  }
}

void takeoffAuxSetTxLowEnabled(bool on)
{
  txLowEnabled = on;
}

bool takeoffAuxTxLowEnabled(void)
{
  return txLowEnabled;
}
