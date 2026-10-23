/*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs_flash.h"

#include "stm32_legacy.h"
#include "platform.h"
#include "system.h"
#include "motors.h"
#include "driver/gpio.h"
#include "stabilizer.h"
#include "reg_snap.h"
#include "esp_log.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"

static void report_manual_startup(void *unused)
{
    (void)unused;
    vTaskDelay(pdMS_TO_TICKS(10000));
    stabilizerBenchTelemetry_t telemetry = {0};
    const bool have_telemetry = stabilizerGetBenchTelemetry(&telemetry);
    ESP_LOGI("manual_flight", "ESP_FLY_MANUAL_ONLY_V1 ready=%d calibrated=%d stopped=%d faulted=%d motors=%u,%u,%u,%u",
             motorsFlightReady(), have_telemetry && telemetry.calibrated,
             motorsIsStopped(), motorsHasFault(),
             (unsigned)motorsGetRatio(0), (unsigned)motorsGetRatio(1),
             (unsigned)motorsGetRatio(2), (unsigned)motorsGetRatio(3));
    vTaskDelete(NULL);
}

void app_main()
{
    regSnapBoot();
    motorsBenchInit();
    // Keep the Sense SD socket deselected: its clock/data share motor/IMU pins.
    // Set the latch before enabling output, and never initialize an SD bus.
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_21, 1));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT));
    /*
    * Initialize the platform and Launch the system task
    * app_main will initialize and start everything
    */

    /* initialize nvs flash prepare for Wi-Fi */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    /*Initialize the platform.*/
    if (platformInit() == false) {
        while (1);//if  firmware is running on the wrong hardware, Halt
    }

    /*launch the system task */
    systemLaunch();
    ESP_LOGI("manual_flight", "ESP_FLY_MANUAL_ONLY_V1: phone-controlled stabilized flight");
    if (xTaskCreate(report_manual_startup, "manual_startup", 3072, NULL, 1, NULL) != pdPASS) {
        motorsEmergencyStop();
    }

}
