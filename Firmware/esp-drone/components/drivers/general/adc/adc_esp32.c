/**
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
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "esp_idf_version.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/rtc_io.h"
#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static esp_adc_cal_characteristics_t *adc_chars;
#ifdef CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_7; //GPIO35 if ADC1
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
static const adc_channel_t channel = ADC_CHANNEL_1;     // GPIO2 if ADC1
#endif

static const adc_bits_width_t width = ADC_WIDTH_MAX-1;
static const adc_atten_t atten = ADC_ATTEN_DB_11;   //11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   30          //Multisampling

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

float analogReadVoltage(uint32_t pin)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage / 1000.0;
}

#if DRONE_RAIL_METER_GPIO
#if !defined(CONFIG_IDF_TARGET_ESP32S3) || DRONE_RAIL_METER_GPIO != 9
#error "The rail meter is wired for GPIO9 = ADC1 channel 8 on the ESP32-S3"
#endif
static const adc1_channel_t railChannel = ADC1_CHANNEL_8;   /* GPIO9 */
static bool railReady;
static volatile bool railBench;   /* 'rail' bench check has the pad driven low */

/* The pad is driven high by its RTC output while ADC1 samples it, so the
 * reading follows the pad supply (the 3.3 V rail) up to the ~3.1 V full scale
 * of the 11 dB range. adc1_config_channel_atten() leaves the pad as an RTC IO
 * with its output disabled; adc1_get_raw() does not touch the pad again. */
static void railMeterInit(void)
{
    if (adc1_config_channel_atten(railChannel, atten) != ESP_OK ||
        rtc_gpio_set_drive_capability(DRONE_RAIL_METER_GPIO, GPIO_DRIVE_CAP_3) != ESP_OK ||
        rtc_gpio_set_level(DRONE_RAIL_METER_GPIO, 1) != ESP_OK ||
        rtc_gpio_set_direction(DRONE_RAIL_METER_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY) != ESP_OK) {
        DEBUG_PRINTW("rail meter: GPIO%d setup failed", DRONE_RAIL_METER_GPIO);
        return;
    }
    railReady = true;
}

static float railRead(void)
{
    int raw = adc1_get_raw(railChannel);
    if (raw < 0) {
        return 0.0f;
    }
    if (raw >= 4095) {
        return ANALOG_RAIL_SATURATED;   /* at or above this chip's full scale */
    }
    return esp_adc_cal_raw_to_voltage((uint32_t)raw, adc_chars) / 1000.0f;
}

/* XIAO 3.3 V rail in volts from one conversion; 0 if the meter is not set up
 * or the bench check has the pad driven low; ANALOG_RAIL_SATURATED at or above
 * full scale (per chip, about 2.85-3.2 V: see analogRailFullScale()). */
float analogReadRailOnce(void)
{
    if (!isInit || !railReady || railBench) {
        return 0.0f;
    }
    float v = railRead();
    return railBench ? 0.0f : v;   /* the bench check may have started meanwhile */
}

/* Same, for the bench check itself (ignores the bench flag). */
float analogReadRailBench(void)
{
    return (isInit && railReady) ? railRead() : 0.0f;
}

/* Highest voltage the meter can show on this chip (eFuse calibration). */
float analogRailFullScale(void)
{
    return isInit ? esp_adc_cal_raw_to_voltage(4094, adc_chars) / 1000.0f : 0.0f;
}

/* Bench check: drive the pad low or high again (the LED follows). Later
 * gpio_config() calls on other RTC-capable pins (motors, I2C, MPU interrupt)
 * gate the RTC IO MUX clock, so re-select the RTC function first; this keeps
 * the pad's output enable, level and drive. */
void analogRailDrive(bool high)
{
    if (!railReady) {
        return;
    }
    if (!high) {
        railBench = true;
    }
    rtc_gpio_init(DRONE_RAIL_METER_GPIO);
    rtc_gpio_set_level(DRONE_RAIL_METER_GPIO, high ? 1 : 0);
    if (high) {
        railBench = false;
    }
}
#else
float analogReadRailOnce(void)
{
    return 0.0f;
}

float analogReadRailBench(void)
{
    return 0.0f;
}

float analogRailFullScale(void)
{
    return 0.0f;
}

void analogRailDrive(bool high)
{
    (void)high;
}
#endif

/* One conversion, no averaging (~tens of us): for catching short supply dips. */
float analogReadVoltageOnce(uint32_t pin)
{
    (void)pin;
    if (!isInit || unit != ADC_UNIT_1) {
        return 0.0f;
    }
    int raw = adc1_get_raw((adc1_channel_t)channel);
    if (raw < 0) {
        return 0.0f;
    }
    return esp_adc_cal_raw_to_voltage((uint32_t)raw, adc_chars) / 1000.0f;
}

void adcInit(void)
{
    if (isInit) {
        return;
    }

    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    isInit = true;
#if DRONE_RAIL_METER_GPIO
    railMeterInit();
#endif
}

bool adcTest(void)
{
    return isInit;
}
