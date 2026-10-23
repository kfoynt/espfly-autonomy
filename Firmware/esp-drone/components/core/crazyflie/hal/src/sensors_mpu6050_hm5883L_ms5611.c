/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * Implements HAL for sensors MPU9250 and LPS25H
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "queue.h"
#include "projdefs.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"

#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"

#include "i2cdev.h"
// #include "lps25h.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
// #include "ak8963.h"
#include "zranger.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "flowdeck_v1v2.h"
#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"
#include "crtp_commander.h"
#include "imu_level_cal.h"
#include "imu_level_cal_store.h"
#include "gyro_tempco.h"
#include "autonomy.h"
#include "bench_bypass.h"

/**
 * Enable 250Hz digital LPF mode. However does not work with
 * multiple slave reading through MPU9250 (MAG and BARO), only single for some reason.
 */
//#define SENSORS_mpu6050_DLPF_256HZ

//#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define MAG_GAUSS_PER_LSB 666.7f

/**
 * Enable sensors on board 
 */
// #define SENSORS_ENABLE_MAG_HM5883L
// #define SENSORS_ENABLE_PRESSURE_MS5611
//#define SENSORS_ENABLE_RANGE_VL53L0X
#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901

#define SENSORS_GYRO_FS_CFG MPU6050_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6050_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6050_G_PER_LSB_16

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f             // Max degrees off

#define SENSORS_BIAS_SAMPLES 1000
#define SENSORS_ACC_SCALE_SAMPLES 200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// Buffer length for MPU9250 slave reads
#define GPIO_INTA_MPU6050_IO CONFIG_MPU_PIN_INT
#define SENSORS_MPU6050_BUFF_LEN 14
#define SENSORS_MAG_BUFF_LEN 8
#define SENSORS_BARO_BUFF_S_P_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_T_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_LEN (SENSORS_BARO_BUFF_S_P_LEN + SENSORS_BARO_BUFF_T_LEN)

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define ESP_INTR_FLAG_DEFAULT 0

#define PITCH_CALIB (CONFIG_PITCH_CALIB*1.0/100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB*1.0/100)

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;
/* Bench fault injection (USB 'mguard test imu'): this many IMU samples
 * (~1 per ms) are read but not handed to the stabilizer, as if the MPU-6050
 * had stopped interrupting. 0 = none. */
static volatile uint32_t faultHoldSamples;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined(GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
/* MPU6050 die temperature, raw TEMP_OUT (deg C = raw / 340 + 36.53): the
 * accelerometer zero-g level drifts with it (docs/autonomy-a2.md 4.11).
 * Exported for the inline sensorsMpu6050GetTempC(). */
volatile int16_t sensorsMpu6050ImuTempRaw;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
#if defined(SENSORS_ENABLE_RANGE_VL53L1X) && !DRONE_SENSE_CAMERA_PINS_RESERVED
static bool isVl53l1xPresent = false;
#endif
#if defined(SENSORS_ENABLE_RANGE_VL53L0X) && !DRONE_SENSE_CAMERA_PINS_RESERVED
static bool isVl53l0xPresent = false;
#endif
#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif
static bool isMpu6050TestPassed = false;
#ifdef SENSORS_ENABLE_MAG_HM5883L
static bool isHmc5883lTestPassed = false;
#endif
#ifdef SENSORS_ENABLE_PRESSURE_MS5611
static bool isMs5611TestPassed = false;
#endif

// Pre-calculated values for accelerometer alignment (Kconfig ROLL_CALIB/PITCH_CALIB)
static imuAlignment_t accAlignment;

// Accelerometer level offset in g, body frame after axis mapping (see imu_level_cal.h).
// Written by the USB diagnostics task, read by the sensors task.
static portMUX_TYPE levelCalMux = portMUX_INITIALIZER_UNLOCKED;
static float accLevelOffsetX = 0.0f;
static float accLevelOffsetY = 0.0f;
static esp_err_t levelCalLoadResult = ESP_ERR_NVS_NOT_FOUND;

// Extra accelerometer offset (g) applied only while flying: motor vibration
// shifts the MPU6050 reading in flight (+7..+20 mg on x in hover on
// 2026-10-10), which the bench level calibration cannot see. Ramped in/out
// over FLIGHT_ACC_RAMP_S so the estimator input never steps.
#define FLIGHT_ACC_MAX_G   0.05f
#define FLIGHT_ACC_RAMP_S  0.5f
static float flightAccOffsetX = 0.0f;
static float flightAccOffsetY = 0.0f;
volatile bool sensorsMpu6050InFlightNow;   /* sensorsMpu6050InFlight() */
static float flightAccRamp;          /* sensors task only, 0..1 */

// Gyro re-zero on the ground. The startup bias is measured once, but the
// MPU6050 gyro zero drifts as the board warms up: on 2026-10-11 the pitch gyro
// read +0.45..+0.61 deg/s before later take-offs (+0.06 before the first),
// which the estimator turns into a lean and a slow drift. Whenever the motors
// have been off for GYRO_REZERO_IDLE_MS (so the 1 s bias buffer holds no motor
// samples) and the gyro passes the same stillness test as at startup, the
// bias is re-measured.
#define GYRO_REZERO_IDLE_MS      2000
#define GYRO_REZERO_PERIOD_MS    1000
#define GYRO_REZERO_MAX_STEP_DPS 3.0f   /* a larger change is not drift: ignore */
static volatile TickType_t lastThrustTick;
static volatile TickType_t lastAirTick;   /* the latest thrust > 30000 (SetThrust): the compensation's air flags */
static TickType_t lastRezeroCheck;   /* sensors task only */
static Axis3f gyroBiasStartup;
static bool gyroBiasStartupSet;
volatile uint32_t sensorsMpu6050RezeroCountNow;   /* sensorsMpu6050GyroRezeroCount() */
static portMUX_TYPE gyroRezeroMux = portMUX_INITIALIZER_UNLOCKED;

// Gyro temperature compensation (gyro_tempco.h, docs/autonomy-a2b.md 19.1).
// The gyro zero moves with the die temperature (pitch +0.060 dps per C of
// cooling on 2026-10-14) and the IMU cools ~10 C in flight under the prop
// wash, far from the last ground re-zero. The model's correction, relative
// to the temperature when the bias in use was measured (the startup bias,
// then every accepted ground re-zero), is subtracted from the gyro before
// its low-pass filter. Config: the USB task under gyroTcMux; state, log and
// latch: the sensors task, under gyroTcMux for the readers.
//
// The loop-rate fix (docs/autonomy-a2b.md 19.11): the per-sample path is an
// add and three subtractions inline in the sensors task (no call, no lock, no
// divide); the filter and the model run once per GYRO_TC_BLOCK samples (20
// Hz, gyroTcBlockStep, flash, cold) on the block's mean temperature, and the
// thrust / airborne bookkeeping moved there from sensorsMpu6050SetThrust.
static portMUX_TYPE gyroTcMux = portMUX_INITIALIZER_UNLOCKED;
static gyroTcConfig_t gyroTcCfg = {
    .k = {GYRO_TC_KX_DEFAULT, GYRO_TC_KY_DEFAULT, GYRO_TC_KZ_DEFAULT},
    .kneeC = GYRO_TC_KNEE_DEFAULT,
    .clampDps = GYRO_TC_CLAMP_DEFAULT,
    .on = true,
};
static esp_err_t gyroTcLoadResult = ESP_ERR_NVS_NOT_FOUND;
static gyroTcState_t gyroTc;
// The correction subtracted from every sample until the next block or latch
// (+0.0f when off: x - (+0) is x bit for bit), dps. Written by the sensors
// task (under gyroTcMux, for the readers: status, the recorder's inline read)
// and read by it per sample without the lock; the bypass toggle zeroes it.
float sensorsMpu6050GyroTcCorrNow[3];
volatile uint32_t sensorsMpu6050GyroTcAirSeqNow; /* gyroTcLog.airSeq, for the flight summary's poll */
// The sensors task's own (no lock): the raw TEMP_OUT summed over the current
// block and its sample count.
static int32_t gyroTcRawSum;
static uint32_t gyroTcN;
static TickType_t gyroTcLatchTick;
// The re-zeros that carry news (after thrust, after a hold, a large step) and
// the first one after the latest airborne stretch, which the quiet re-zero
// once a second would otherwise push out of the ring ~10 s after landing,
// before USB is plugged in (gyro_tempco.h).
static gyroTcLog_t gyroTcLog;
// Since the latest latch, from lastThrustTick / lastAirTick moving between
// blocks (the sensors task, gyroTcBlockStep): thrust > 0, thrust above
// lift-off, and that stretch's first and latest airborne ticks (the first
// one to the block, <= 50 ms late; the latest exact: a latch comes >= 2 s
// after the last thrust). gyroTcAirLast is also the lastAirTick seen.
static bool gyroTcThrustSince;
static bool gyroTcAirSince;
static TickType_t gyroTcAirFirst, gyroTcAirLast;
static TickType_t gyroTcThrustSeen;   /* lastThrustTick at the latest block */
static volatile bool gyroTcHoldSince;   /* a 'gtc hold' started since the latest latch */
// Bench cooling test ('gtc hold <s>', RAM only): no ground re-zero until
// gyroTcHoldUntil; any thrust or an active autonomy ends it.
#define GYRO_TC_HOLD_MAX_MS 300000u
static volatile bool gyroTcHoldOn;
static volatile TickType_t gyroTcHoldUntil;

// Accelerometer temperature compensation (accel_tempco.h, docs/autonomy-a2b.md
// 36), the gyro's pattern on the body-frame accelerometer: the per-sample path
// adds the three raw samples to the block's sums and subtracts the cached
// correction (g); the model runs in gyroTcBlockStep on the same block (20 Hz).
// The reference is latched on request (ARM, a cooling spin, 'atc latch'):
// the request is a word any task writes, served at the next block. The check
// (the resting reading now minus the reference, at the first gyro re-zero
// after thrust) is logged beside the gyro's. Config: the USB task under
// gyroTcMux; state, window, log and latch: the sensors task, under gyroTcMux
// for the readers.
static accTcConfig_t accTcCfg = {
    .k = {ACC_TC_KX_DEFAULT, ACC_TC_KY_DEFAULT, ACC_TC_KZ_DEFAULT},
    .kneeC = ACC_TC_KNEE_DEFAULT,
    .clampMg = ACC_TC_CLAMP_DEFAULT,
    .on = ACC_TC_ON_DEFAULT,
};
static esp_err_t accTcLoadResult = ESP_ERR_NVS_NOT_FOUND;
static accTcState_t accTc;
float sensorsMpu6050AccTcCorrNow[3];   /* g; +0.0f when off: x - (+0) is x bit for bit */
volatile uint32_t sensorsMpu6050AccTcAirSeqNow;
static int32_t accTcRawSum[3];         /* the sensors task's own: the block's raw sums, mapped body frame */
static accTcWindow_t accTcWin;         /* the last 1 s of block means, mg, raw */
static float accTcRefMg[3];            /* the reference reading, mg (valid: accTcRefValid) */
static bool accTcRefValid;
static TickType_t accTcLatchTick;
static accTcLog_t accTcLog;
#define ACC_TC_REQ_LATCH 0x01u
#define ACC_TC_REQ_COOL  0x02u         /* ... and a ground cooling spin follows the latch */
static volatile uint8_t accTcLatchReq;  /* any task sets, the sensors task clears (one byte) */
static bool accTcCoolSince;            /* a cooling spin started since the latest gyro latch (the check's flag) */

// Raw-sample accumulator for stationary measurements, mapped body axes, raw LSB.
#define SENSORS_ACCUM_MAX_SAMPLES 60000
static portMUX_TYPE accumMux = portMUX_INITIALIZER_UNLOCKED;
static bool accumEnabled = false;
static uint32_t accumCount;
static int64_t accumAccSum[3];
static int64_t accumAccSumSq[3];
static int64_t accumGyroSum[3];
static int64_t accumGyroSumSq[3];

// This buffer needs to hold data from all sensors
static uint8_t buffer[SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
static void sensorsSetupSlaveRead(void);

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void gyroRezeroCheck(void);
static void gyroTcBlockStep(void);
static void accTcLatchNow(uint8_t req);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu6050Hmc5883lMs5611AreCalibrated()
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    //TODO:
    systemWaitStart();
    vTaskDelay(M2T(200));
    DEBUG_PRINTD("xTaskCreate sensorsTask IN");
    sensorsSetupSlaveRead(); //
    DEBUG_PRINTD("xTaskCreate sensorsTask SetupSlave done");

    while (1) {

        /* mpu6050 interrupt trigger: data is ready to be read */
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            sensorData.interruptTimestamp = imuIntTimestamp;

            /* sensors step 1-read data from I2C */
            uint8_t dataLen = (uint8_t)(SENSORS_MPU6050_BUFF_LEN +
                                        (isMagnetometerPresent ? SENSORS_MAG_BUFF_LEN : 0) +
                                        (isBarometerPresent ? SENSORS_BARO_BUFF_LEN : 0));
            i2cdevReadReg8(I2C0_DEV, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, dataLen, buffer);

            /* sensors step 2-process the respective data */
            processAccGyroMeasurements(&(buffer[0]));

            if (isMagnetometerPresent) {
                processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));
            }

            if (isBarometerPresent) {
                processBarometerMeasurements(&(buffer[isMagnetometerPresent ? SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6050_BUFF_LEN]));
            }

            /* sensors step 3- queue sensors data  on the output queues */
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            if (isMagnetometerPresent) {
                xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            }

            if (isBarometerPresent) {
                xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            }

            /* sensors step 4- Unlock stabilizer task (unless a bench fault withholds it) */
            if (faultHoldSamples > 0) {
                faultHoldSamples = faultHoldSamples - 1;
            } else {
                xSemaphoreGive(dataReady);
            }
#ifdef DEBUG_EP2
            DEBUG_PRINT_LOCAL("ax = %f,  ay = %f,  az = %f,  gx = %f,  gy = %f,  gz = %f , hx = %f , hy = %f, hz =%f \n", sensorData.acc.x, sensorData.acc.y, sensorData.acc.z, sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z, sensorData.mag.x, sensorData.mag.y, sensorData.mag.z);
#endif
        }
    }
}

void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

void sensorsMpu6050FaultHoldData(uint32_t ms)
{
    faultHoldSamples = ms;
}

void processBarometerMeasurements(const uint8_t *buffer)
{
    //TODO: replace it to MS5611
    DEBUG_PRINTW("processBarometerMeasurements NEED TODO");
//   static uint32_t rawPressure = 0;
//   static int16_t rawTemp = 0;

// Check if there is a new pressure update

// Check if there is a new temp update

//   sensorData.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
//   sensorData.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
//   sensorData.baro.asl = lps25hPressureToAltitude(&sensorData.baro.pressure);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
    //TODO: replace it to hmc5883l
    if (buffer[7] & (1 << HMC5883L_STATUS_READY_BIT)) {
        int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1]; //hmc5883 different from
        int16_t headingz = (((int16_t)buffer[4]) << 8) | buffer[3];
        int16_t headingy = (((int16_t)buffer[6]) << 8) | buffer[5];

        sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB; //to gauss
        sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
        DEBUG_PRINTI("hmc5883l DATA ready");
    } else {

        DEBUG_PRINTW("hmc5883l DATA not ready");
    }
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
    /*  Note the ordering to correct the rotated 90º IMU coordinate system */

    Axis3f accScaled;

#ifdef CONFIG_TARGET_ESPLANE_V1
    /* sensors step 2.1 read from buffer */
    /*
    accelRaw.x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
    */
    accelRaw.y = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.x = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.y = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.x = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
    /* sensors step 2.1 read from buffer */
    accelRaw.y = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.x = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.y = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.x = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#endif
    const int16_t tempRaw = (int16_t)((((uint16_t)buffer[6]) << 8) | buffer[7]);   /* TEMP_OUT_H/L, between accel and gyro */
    sensorsMpu6050ImuTempRaw = tempRaw;

#ifdef GYRO_BIAS_LIGHT_WEIGHT
    gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
    /* sensors step 2.2 Calculates the gyro bias first when the  variance is below threshold */
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif

    /*sensors step 2.3 Calculates the acc scale when platform is steady */
    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
        gyroRezeroCheck();
    }

    /* sensors step 2.4a the gyro zero's temperature drift since the bias in use was measured (gyro_tempco.h,
     * docs/autonomy-a2b.md 19.11): the model runs once per GYRO_TC_BLOCK samples on their mean temperature
     * (gyroTcBlockStep, called here so that the conversion and the subtraction below stay in registers); every
     * sample subtracts the cached correction (step 2.4b; +0 when off). The bisect toggle skips both (bench_bypass.h). */
    const bool tcRun = !benchBypassOn(BENCH_BYPASS_GTC);
    if (tcRun) {
        gyroTcRawSum += tempRaw;
        /* the accelerometer compensation's block sums (accel_tempco.h), mapped like step 2.6 below */
        accTcRawSum[0] -= accelRaw.x;
        accTcRawSum[1] += accelRaw.y;
        accTcRawSum[2] += accelRaw.z;
        if (++gyroTcN >= GYRO_TC_BLOCK) {
            gyroTcBlockStep();
        }
    }

    /* sensors step 2.4 convert  digtal value to physical angle (in registers: step 2.4b, then one store) */
#ifdef CONFIG_TARGET_ESPLANE_V1
    //sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    float gyroX = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#else
    float gyroX = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#endif

    float gyroY = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
    float gyroZ = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    /* sensors step 2.4b the cached temperature correction (step 2.4a) */
    if (tcRun) {
        gyroX -= sensorsMpu6050GyroTcCorrNow[0];
        gyroY -= sensorsMpu6050GyroTcCorrNow[1];
        gyroZ -= sensorsMpu6050GyroTcCorrNow[2];
    }
    sensorData.gyro.x = gyroX;
    sensorData.gyro.y = gyroY;
    sensorData.gyro.z = gyroZ;
    /* sensors step 2.5 low pass filter */
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

    if (accumEnabled) {
        /* Same axis mapping as below, before any bias, offset or scale. */
        const int32_t a[3] = {-accelRaw.x, accelRaw.y, accelRaw.z};
        const int32_t g[3] = {-gyroRaw.x, gyroRaw.y, gyroRaw.z};
        portENTER_CRITICAL(&accumMux);
        if (accumCount < SENSORS_ACCUM_MAX_SAMPLES) {
            for (int i = 0; i < 3; i++) {
                accumAccSum[i] += a[i];
                accumAccSumSq[i] += (int64_t)a[i] * a[i];
                accumGyroSum[i] += g[i];
                accumGyroSumSq[i] += (int64_t)g[i] * g[i];
            }
            accumCount++;
        }
        portEXIT_CRITICAL(&accumMux);
    }

    float levelOffsetX, levelOffsetY;
    const float rampStep = 1.0f / (FLIGHT_ACC_RAMP_S * 1000.0f);   /* 1 kHz samples */
    flightAccRamp = sensorsMpu6050InFlightNow ? fminf(flightAccRamp + rampStep, 1.0f) : fmaxf(flightAccRamp - rampStep, 0.0f);
    portENTER_CRITICAL(&levelCalMux);
    levelOffsetX = accLevelOffsetX + flightAccRamp * flightAccOffsetX;
    levelOffsetY = accLevelOffsetY + flightAccRamp * flightAccOffsetY;
    portEXIT_CRITICAL(&levelCalMux);

    /* sensors step 2.6 mapped axes in g, minus the level offset, then the startup scale */
#ifdef CONFIG_TARGET_ESPLANE_V1
    //accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.x = (-(accelRaw.x) * SENSORS_G_PER_LSB_CFG - levelOffsetX) / accScale;
#else
    accScaled.x = (-(accelRaw.x) * SENSORS_G_PER_LSB_CFG - levelOffsetX) / accScale;
#endif

    accScaled.y = ((accelRaw.y) * SENSORS_G_PER_LSB_CFG - levelOffsetY) / accScale;
    accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;
    /* sensors step 2.6a the accelerometer zero's temperature drift since its reference (accel_tempco.h,
     * docs/autonomy-a2b.md 36): the cached correction of step 2.4a's block (+0 when off) */
    if (tcRun) {
        accScaled.x -= sensorsMpu6050AccTcCorrNow[0];
        accScaled.y -= sensorsMpu6050AccTcCorrNow[1];
        accScaled.z -= sensorsMpu6050AccTcCorrNow[2];
    }

    /* sensors step 2.7 Compensate for a miss-aligned accelerometer. */
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
}
/*
 * Release an I2C bus held by the MPU6050. If the ESP32 resets in the middle
 * of a read (e.g. a brownout while the 1 kHz sensor task is reading), the
 * MPU6050 can keep driving SDA low and ignore the next boot's transfers.
 * Clocking SCL until SDA is released, then sending a STOP, frees it.
 */
static void sensorsI2cBusClear(int sdaPin, int sclPin)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << sdaPin) | (1ULL << sclPin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(sdaPin, 1);
    gpio_set_level(sclPin, 1);
    esp_rom_delay_us(10);
    int pulses = 0;
    while (pulses < 9 && gpio_get_level(sdaPin) == 0) {
        gpio_set_level(sclPin, 0);
        esp_rom_delay_us(5);
        gpio_set_level(sclPin, 1);
        esp_rom_delay_us(5);
        pulses++;
    }
    /* STOP: SDA low -> high while SCL is high */
    gpio_set_level(sclPin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(sdaPin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(sclPin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sdaPin, 1);
    esp_rom_delay_us(5);
    if (pulses > 0) {
        DEBUG_PRINTW("I2C0 bus was held low; released after %d clock pulses", pulses);
    }
}

static void sensorsDeviceInit(void)
{
    isMagnetometerPresent = false;
    isBarometerPresent = false;

    // Wait for sensors to startup
    while (xTaskGetTickCount() < 2000){
        vTaskDelay(M2T(50));
    };

    sensorsI2cBusClear(CONFIG_I2C0_PIN_SDA, CONFIG_I2C0_PIN_SCL);
    i2cdevInit(I2C0_DEV);
    mpu6050Init(I2C0_DEV);

    /* Retry instead of aborting on the first miss: a single failed probe
     * after a brownout used to put the drone in a reboot loop. */
    bool connected = false;
    for (int attempt = 1; attempt <= 10 && !connected; attempt++) {
        connected = mpu6050TestConnection();
        if (!connected) {
            DEBUG_PRINTW("MPU6050 not responding (attempt %d/10)", attempt);
            vTaskDelay(M2T(100));
        }
    }
    if (connected) {
        DEBUG_PRINTI("MPU6050 I2C connection [OK].\n");
    } else {
        DEBUG_PRINTE("MPU6050 I2C connection [FAIL].\n");
        // Please check your hardware !
        assert(0);
    }

    mpu6050Reset();
    vTaskDelay(M2T(50));
    // Activate mpu6050
    mpu6050SetSleepEnabled(false);
    // Delay until registers are reset
    vTaskDelay(M2T(100));
    // Set x-axis gyro as clock source
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    // Delay until clock is set and stable
    vTaskDelay(M2T(200));
    // Enable temp sensor
    mpu6050SetTempSensorEnabled(true);
    // Disable interrupts
    mpu6050SetIntEnabled(false);
    // Connect the MAG and BARO to the main I2C bus
    mpu6050SetI2CBypassEnabled(true);
    // Set gyro full scale range
    mpu6050SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
    // Set accelerometer full scale range
    mpu6050SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);

    // Set digital low-pass bandwidth for gyro and acc
    // board ESP32_S2_DRONE_V1_2 has more vibrations, bandwidth should be lower
#ifdef SENSORS_MPU6050_DLPF_256HZ
    // 256Hz digital low-pass filter only works with little vibrations
    // Set output rate (15): 8000 / (1 + 7) = 1000Hz
    mpu6050SetRate(7);
    // Set digital low-pass bandwidth
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#elif defined(CONFIG_TARGET_ESP32_S2_DRONE_V1_2)
    // To low DLPF bandwidth might cause instability and decrease agility
    // but it works well for handling vibrations and unbalanced propellers
    // Set output rate (1): 1000 / (1 + 0) = 1000Hz
    mpu6050SetRate(0);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
    // Init second order filer for accelerometer
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }
#else
    mpu6050SetRate(0);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
    // Init second order filer for accelerometer
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }
#endif

#ifdef SENSORS_ENABLE_MAG_HM5883L
    hmc5883lInit(I2C0_DEV);

    if (hmc5883lTestConnection() == true) {
        isMagnetometerPresent = true;
        hmc5883lSetMode(HMC5883L_MODE_CONTINUOUS); // 16bit 100Hz
        DEBUG_PRINTI("hmc5883l I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("hmc5883l I2C connection [FAIL].\n");
    }

#endif
#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    ms5611Init(I2C0_DEV);

    if (false) {
        isBarometerPresent = true;
        DEBUG_PRINTI("MS5611 I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("MS5611 I2C connection [FAIL].\n");
    }

#endif

#if DRONE_SENSE_CAMERA_PINS_RESERVED
    // The ESP-FLY + Sense bench has no range/flow deck. Range initialization
    // would install I2C1 and steal the camera SCCB bus even with deckInit off.
    DEBUG_PRINTI("Sense bench: absent range/flow initialization skipped; I2C1 reserved for camera.\n");
#else
#ifdef SENSORS_ENABLE_RANGE_VL53L1X
    zRanger2Init();

    if (zRanger2Test() == true) {
        isVl53l1xPresent = true;
        DEBUG_PRINTI("VL53L1X I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("VL53L1X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_RANGE_VL53L0X
    zRangerInit();

    if (zRangerTest() == true) {
        isVl53l0xPresent = true;
        DEBUG_PRINTI("VL53L0X I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("VL53L0X I2C connection [FAIL].\n");
    }

#endif

#ifdef SENSORS_ENABLE_FLOW_PMW3901
    flowdeck2Init();

    if (flowdeck2Test() == true) {
        isPmw3901Present = true;
        setCommandermode(POSHOLD_MODE);
        DEBUG_PRINTI("PMW3901 SPI connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
        DEBUG_PRINTW("PMW3901 SPI connection [FAIL].\n");
    }
#endif
#endif // DRONE_SENSE_CAMERA_PINS_RESERVED

    DEBUG_PRINTI("sensors init done");
    /*
    *get calib angle from NVS
    */
    // cosPitch = cosf(configblockGetCalibPitch() * (float)M_PI / 180);
    // sinPitch = sinf(configblockGetCalibPitch() * (float)M_PI / 180);
    // cosRoll = cosf(configblockGetCalibRoll() * (float)M_PI / 180);
    // sinRoll = sinf(configblockGetCalibRoll() * (float)M_PI / 180);
    imuAlignmentInit(&accAlignment, ROLL_CALIB, PITCH_CALIB);
    DEBUG_PRINTI("pitch_calib = %f,roll_calib = %f",PITCH_CALIB,ROLL_CALIB);

    /*
    * Accelerometer level offset from the "imucal" NVS namespace. A missing or
    * rejected record leaves the offsets at zero (original behavior).
    */
    imuLevelCalRecord_t levelCal;
    levelCalLoadResult = imuLevelCalStoreLoad(&levelCal);
    if (levelCalLoadResult == ESP_OK) {
        accLevelOffsetX = levelCal.accOffsetX;
        accLevelOffsetY = levelCal.accOffsetY;
        DEBUG_PRINTI("level cal loaded: x = %.4f g, y = %.4f g", accLevelOffsetX, accLevelOffsetY);
    } else if (levelCalLoadResult == ESP_ERR_NVS_NOT_FOUND) {
        DEBUG_PRINTI("level cal: none stored, offsets 0");
    } else {
        DEBUG_PRINTW("level cal: stored record rejected (%s), offsets 0", esp_err_to_name(levelCalLoadResult));
    }

    imuLevelCalRecord_t facc;
    if (flightAccStoreLoad(&facc) == ESP_OK &&
        fabsf(facc.accOffsetX) <= FLIGHT_ACC_MAX_G && fabsf(facc.accOffsetY) <= FLIGHT_ACC_MAX_G) {
        flightAccOffsetX = facc.accOffsetX;
        flightAccOffsetY = facc.accOffsetY;
        DEBUG_PRINTI("in-flight acc offset loaded: x = %.4f g, y = %.4f g", flightAccOffsetX, flightAccOffsetY);
    }

    gyroTcRecord_t gtc;
    gyroTcLoadResult = gyroTcStoreLoad(&gtc);
    if (gyroTcLoadResult == ESP_OK) {
        gyroTcConfig_t c = gyroTcCfg;
        gyroTcRecordApply(&gtc, &c);
        gyroTcCfg = c;   /* before the sensors task runs */
        DEBUG_PRINTI("gyro tempco loaded: on %d k %.3f %.3f %.3f knee %.1f", (int)c.on, (double)c.k[0],
                     (double)c.k[1], (double)c.k[2], (double)c.kneeC);
    }

    accTcRecord_t atc;
    accTcLoadResult = accTcStoreLoad(&atc);
    if (accTcLoadResult == ESP_OK) {
        accTcConfig_t c = accTcCfg;
        accTcRecordApply(&atc, &c);
        accTcCfg = c;   /* before the sensors task runs */
        DEBUG_PRINTI("accel tempco loaded: on %d k %.3f %.3f %.3f mg/C knee %.1f", (int)c.on, (double)c.k[0],
                     (double)c.k[1], (double)c.k[2], (double)c.kneeC);
    }
}

static void sensorsSetupSlaveRead(void)
{
    // Now begin to set up the slaves
#ifdef SENSORS_MPU6050_DLPF_256HZ
    // As noted in registersheet 4.4: "Data should be sampled at or above sample rate;
    // SMPLRT_DIV is only used for 1kHz internal sampling." Slowest update rate is then 500Hz.
    mpu6050SetSlave4MasterDelay(15); // read slaves at 500Hz = (8000Hz / (1 + 15))
#else
    mpu6050SetSlave4MasterDelay(9); // read slaves at 100Hz = (500Hz / (1 + 4))
#endif

    mpu6050SetI2CBypassEnabled(false);
    mpu6050SetWaitForExternalSensorEnabled(true);     // the slave data isn't so important for the state estimation
    mpu6050SetInterruptMode(0);                       // active high
    mpu6050SetInterruptDrive(0);                      // push pull
    mpu6050SetInterruptLatch(0);                      // latched until clear
    mpu6050SetInterruptLatchClear(1);                 // cleared on any register read
    mpu6050SetSlaveReadWriteTransitionEnabled(false); // Send a stop at the end of a slave read
    mpu6050SetMasterClockSpeed(13);                   // Set i2c speed to 400kHz

#ifdef SENSORS_ENABLE_MAG_HM5883L

    if (isMagnetometerPresent) {
        // Set registers for mpu6050 master to read from
        mpu6050SetSlaveAddress(0, 0x80 | HMC5883L_ADDRESS);        // set the magnetometer to Slave 0, enable read
        mpu6050SetSlaveRegister(0, HMC5883L_RA_MODE);       // read the magnetometer heading register
        mpu6050SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); // hmc5883l:model,x,z,y,status ak8963:read 8 bytes (ST1, x, y, z heading, ST2 (overflow check))
        mpu6050SetSlaveDelayEnabled(0, true);
        mpu6050SetSlaveEnabled(0, true);
        DEBUG_PRINTD("mpu6050SetSlaveAddress HMC5883L done \n");
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_MS5611

    if (isBarometerPresent) {
        // Configure the LPS25H as a slave and enable read
        // Setting up two reads works for LPS25H fifo avg filter as well as the
        // auto inc wraps back to LPS25H_PRESS_OUT_L after LPS25H_PRESS_OUT_H is read.
        mpu6050SetSlaveAddress(1, 0x80 | MS5611_ADDR_CSB_LOW);
        mpu6050SetSlaveRegister(1, MS5611_D1);
        mpu6050SetSlaveDataLength(1, MS5611_D1D2_SIZE);
        mpu6050SetSlaveDelayEnabled(1, true);
        mpu6050SetSlaveEnabled(1, true);

        mpu6050SetSlaveAddress(2, 0x80 | MS5611_ADDR_CSB_LOW); //temperature
        mpu6050SetSlaveRegister(2, MS5611_D2);
        mpu6050SetSlaveDataLength(2, MS5611_D1D2_SIZE);
        mpu6050SetSlaveDelayEnabled(2, true);
        mpu6050SetSlaveEnabled(2, true);
        DEBUG_PRINTD("mpu6050SetSlaveAddress MS5611 done \n");
    }

#endif

    // Enable sensors after configuration
    mpu6050SetI2CMasterModeEnabled(true);

    mpu6050SetIntDataReadyEnabled(true);

    DEBUG_PRINTD("sensorsSetupSlaveRead done \n");
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
  DEBUG_PRINTD("xTaskCreate sensorsTask \n");
}

static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp(); //This function returns the number of microseconds since esp_timer was initialized
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void sensorsInterruptInit(void)
{

    DEBUG_PRINTD("sensorsInterruptInit \n");
    gpio_config_t io_conf = {
        //interrupt of rising edge
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_POSEDGE,
#else
        .intr_type = GPIO_PIN_INTR_POSEDGE,
#endif
        //bit mask of the pins
        .pin_bit_mask = (1ULL << GPIO_INTA_MPU6050_IO),
        //set as input mode
        .mode = GPIO_MODE_INPUT,
        //disable pull-down mode
        .pull_down_en = 0,
        //enable pull-up mode
        .pull_up_en = 1,
    };
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    //install gpio isr service
    //portDISABLE_INTERRUPTS();
    gpio_set_intr_type(GPIO_INTA_MPU6050_IO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INTA_MPU6050_IO, sensors_inta_isr_handler, (void *)GPIO_INTA_MPU6050_IO);
    //portENABLE_INTERRUPTS();
    DEBUG_PRINTD("sensorsInterruptInit done \n");

    //   FSYNC "shall not be floating, must be set high or low by the MCU"

}

void sensorsMpu6050Hmc5883lMs5611Init(void)
{
    if (isInit) {
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit();
    sensorsTaskInit();
    isInit = true;
}

bool sensorsMpu6050Hmc5883lMs5611Test(void)
{

    bool testStatus = true;

    if (!isInit) {
        DEBUG_PRINTE("Error while initializing sensor task\r\n");
        testStatus = false;
    }

    // Try for 3 seconds so the quad has stabilized enough to pass the test
    for (int i = 0; i < 300; i++) {
        if (mpu6050SelfTest() == true) {
            isMpu6050TestPassed = true;
            break;
        } else {
            vTaskDelay(M2T(10));
        }
    }

    testStatus &= isMpu6050TestPassed;

#ifdef SENSORS_ENABLE_MAG_HM5883L
    testStatus &= isMagnetometerPresent;

    if (testStatus) {
        isHmc5883lTestPassed = hmc5883lSelfTest();
        testStatus &= isHmc5883lTestPassed;
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_MS5611
    testStatus &= isBarometerPresent;

    if (testStatus) {
        isMs5611TestPassed = ms5611SelfTest();

        testStatus &= isMs5611TestPassed;
    }

#endif

    return testStatus;

}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    static uint32_t gyroBiasSampleCount = 0;
    static bool gyroBiasNoBuffFound = false;
    static Axis3i64 gyroBiasSampleSum;
    static Axis3i64 gyroBiasSampleSumSquares;

    if (!gyroBiasNoBuffFound) {
        // If the gyro has not yet been calibrated:
        // Add the current sample to the running mean and variance
        gyroBiasSampleSum.x += gx;
        gyroBiasSampleSum.y += gy;
        gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
        gyroBiasSampleSumSquares.x += gx * gx;
        gyroBiasSampleSumSquares.y += gy * gy;
        gyroBiasSampleSumSquares.z += gz * gz;
#endif
        gyroBiasSampleCount += 1;

        // If we then have enough samples, calculate the mean and standard deviation
        if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES) {
            gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
            gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
            gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
            gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
            gyroBiasNoBuffFound = true;
        }
    }

    return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            //TODO:
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("isBiasValueFound!");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut)
{
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
    }

    meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
                (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }

    return foundBias;
}

/* Once per GYRO_TC_BLOCK samples (20 Hz; the sensors task): the filter and
 * the model on the block's mean temperature (gyroTcBlock, dt = the block),
 * published under the lock, and the thrust / airborne flags since the latest
 * latch from lastThrustTick / lastAirTick moving. Out of line and cold: the
 * 1 kHz path only adds, counts and subtracts (docs/autonomy-a2b.md 19.11). */
static void __attribute__((noinline, cold)) gyroTcBlockStep(void)
{
    const int32_t sum = gyroTcRawSum;
    const int n = (int)gyroTcN;
    const int32_t aSum[3] = {accTcRawSum[0], accTcRawSum[1], accTcRawSum[2]};
    gyroTcRawSum = 0;
    gyroTcN = 0;
    accTcRawSum[0] = accTcRawSum[1] = accTcRawSum[2] = 0;
    const TickType_t thrustTick = lastThrustTick, airTick = lastAirTick;
    float tc[3], ta[3];
    portENTER_CRITICAL(&gyroTcMux);
    if (benchBypassOn(BENCH_BYPASS_GTC)) {
        portEXIT_CRITICAL(&gyroTcMux);   /* bypassed since this sample began: publish nothing */
        return;
    }
    if (thrustTick != gyroTcThrustSeen) {
        gyroTcThrustSeen = thrustTick;
        gyroTcThrustSince = true;
    }
    if (airTick != gyroTcAirLast) {
        if (!gyroTcAirSince) {
            gyroTcAirFirst = airTick;
            gyroTcAirSince = true;
        }
        gyroTcAirLast = airTick;
    }
    gyroTcBlock(&gyroTc, &gyroTcCfg, sum, n, tc);
    for (int i = 0; i < 3; i++) {
        sensorsMpu6050GyroTcCorrNow[i] = tc[i];
    }
    /* the accelerometer: the block's raw means into the 1 s window, the model, a requested latch */
    accTcWindowPush(&accTcWin, aSum, n, SENSORS_G_PER_LSB_CFG * 1000.0f);
    accTcBlock(&accTc, &accTcCfg, sum, n, ta);
    const uint8_t req = accTcLatchReq;
    if (req) {
        accTcLatchReq = 0;
        accTcLatchNow(req);   /* recomputes the correction from the new reference */
    } else {
        for (int i = 0; i < 3; i++) {
            sensorsMpu6050AccTcCorrNow[i] = ta[i] * 0.001f;
        }
    }
    portEXIT_CRITICAL(&gyroTcMux);
}

/* The accelerometer's reference, now (sensors task, under gyroTcMux): the
 * window's 1 s mean if the drone rested that long (else the temperature
 * only), the model restarted from the temperature now. */
static void accTcLatchNow(uint8_t req)
{
    accTcRefValid = accTcWindowMean(&accTcWin, accTcRefMg);
    accTcLatch(&accTc);
    accTcLatchTick = xTaskGetTickCount();
    accTcCoolSince = (req & ACC_TC_REQ_COOL) != 0;
    float ta[3];
    accTcCorr(&accTcCfg, &accTc, ta);
    for (int i = 0; i < 3; i++) {
        sensorsMpu6050AccTcCorrNow[i] = ta[i] * 0.001f;
    }
}

/* The accelerometer's check at a gyro re-zero (sensors task, under
 * gyroTcMux): the resting reading now minus the reference against the model,
 * with the gyro's thrust flags (cleared by its latch right after) and the
 * cooling spin's; only entries after thrust or a spin are kept. */
static void accTcCheckAtRezero(TickType_t now, bool thrustSince, bool airSince, TickType_t airLast)
{
    float still[3];
    if (!accTc.refSet || !accTcRefValid || !accTcWindowMean(&accTcWin, still)) {
        accTcCoolSince = false;
        return;
    }
    float model[3];
    accTcModel(&accTcCfg, &accTc, model);
    const float step[3] = {still[0] - accTcRefMg[0], still[1] - accTcRefMg[1], still[2] - accTcRefMg[2]};
    const uint8_t flags = (thrustSince ? ACC_TC_RZ_FLIGHT : 0) | (accTcCfg.on ? ACC_TC_RZ_ON : 0) |
                          (airSince ? ACC_TC_RZ_AIR : 0) | (accTcCoolSince ? ACC_TC_RZ_COOL : 0);
    accTcRezero_t e;
    accTcRezeroFill(&e, accTc.tRef, accTc.tLp, step, model, flags);
    accTcLogRezero(&accTcLog, &e, T2M(now), T2M(airLast));
    sensorsMpu6050AccTcAirSeqNow = accTcLog.airSeq;
    accTcCoolSince = false;
}

/* The bias in use changed (sensors task): the compensation's reference is
 * the temperature now (the latest block's), and the correction restarts from
 * it at this sample. A re-zero (old, mean: raw LSB) is logged first: its
 * step (what a still gyro read with the old bias, body frame) against the
 * model's correction at that moment; their difference is the error the
 * compensation left since the previous latch. gyroTc and the thrust flags
 * are this task's own: the entry is built outside the lock (about once a
 * second), the lock only copies the config and publishes. */
static void gyroTcLatchBias(const Axis3f *old, const Axis3f *mean, TickType_t now)
{
    if (benchBypassOn(BENCH_BYPASS_GTC)) {
        return;   /* bypassed: no latch, no log (cleared, it restarts unseeded: sensorsMpu6050GyroTcBypass) */
    }
    gyroTcConfig_t cfg;
    portENTER_CRITICAL(&gyroTcMux);
    cfg = gyroTcCfg;
    portEXIT_CRITICAL(&gyroTcMux);
    sensorsGyroTcRezero_t e;
    const bool logIt = old != NULL && gyroTc.refSet;
    if (logIt) {
        float model[3];
        gyroTcModel(&cfg, &gyroTc, model);
        const float step[3] = {
            -(mean->x - old->x) * SENSORS_DEG_PER_LSB_CFG,
            (mean->y - old->y) * SENSORS_DEG_PER_LSB_CFG,
            (mean->z - old->z) * SENSORS_DEG_PER_LSB_CFG,
        };
        const uint8_t flags = (gyroTcThrustSince ? SENSORS_GTC_RZ_FLIGHT : 0) | (cfg.on ? SENSORS_GTC_RZ_ON : 0) |
                              (gyroTcAirSince ? SENSORS_GTC_RZ_AIR : 0) | (accTcCoolSince ? SENSORS_GTC_RZ_COOL : 0);
        gyroTcRezeroFill(&e, gyroTc.tRef, gyroTc.tLp, step, model, flags);
    }
    gyroTcState_t latched = gyroTc;
    gyroTcLatch(&latched);
    float tc[3];
    gyroTcCorr(&cfg, &latched, tc);
    portENTER_CRITICAL(&gyroTcMux);
    if (logIt) {
        if (gyroTcHoldSince) {
            e.flags |= SENSORS_GTC_RZ_HOLD;   /* read and cleared under the lock the hold command sets it under */
        }
        gyroTcLogRezero(&gyroTcLog, &e, T2M(now), T2M(gyroTcAirFirst), T2M(gyroTcAirLast));
        accTcCheckAtRezero(now, gyroTcThrustSince, gyroTcAirSince, gyroTcAirLast);
    } else if (!accTc.refSet) {
        accTcLatchNow(ACC_TC_REQ_LATCH);   /* the startup bias: the first reference */
    }
    gyroTc = latched;
    gyroTcLatchTick = now;
    gyroTcThrustSince = false;
    gyroTcAirSince = false;
    gyroTcHoldSince = false;
    for (int i = 0; i < 3; i++) {
        sensorsMpu6050GyroTcCorrNow[i] = tc[i];
    }
    sensorsMpu6050GyroTcAirSeqNow = gyroTcLog.airSeq;
    portEXIT_CRITICAL(&gyroTcMux);
}

static void gyroRezeroCheck(void)
{
    const TickType_t now = xTaskGetTickCount();
    if (!gyroBiasStartupSet) {
        gyroBiasStartup = gyroBiasRunning.bias;
        gyroBiasStartupSet = true;
        gyroTcLatchBias(NULL, NULL, now);
    }
    if (now - lastRezeroCheck < M2T(GYRO_REZERO_PERIOD_MS)) {
        return;
    }
    lastRezeroCheck = now;
    if (gyroTcHoldOn) {
        if ((int32_t)(now - gyroTcHoldUntil) < 0 && !autonomyActive()) {
            return;   /* the bench cooling test: keep the bias (and the reference) */
        }
        gyroTcHoldOn = false;
    }
    if (now - lastThrustTick < M2T(GYRO_REZERO_IDLE_MS) || !gyroBiasRunning.isBufferFilled) {
        return;
    }
    Axis3f var, mean;
    sensorsCalculateVarianceAndMean(&gyroBiasRunning, &var, &mean);
    if (var.x >= GYRO_VARIANCE_THRESHOLD_X || var.y >= GYRO_VARIANCE_THRESHOLD_Y ||
        var.z >= GYRO_VARIANCE_THRESHOLD_Z) {
        return;   /* moving, or being handled */
    }
    const float maxStep = GYRO_REZERO_MAX_STEP_DPS / SENSORS_DEG_PER_LSB_CFG;
    if (fabsf(mean.x - gyroBiasRunning.bias.x) > maxStep || fabsf(mean.y - gyroBiasRunning.bias.y) > maxStep ||
        fabsf(mean.z - gyroBiasRunning.bias.z) > maxStep) {
        return;
    }
    const Axis3f old = gyroBiasRunning.bias;
    portENTER_CRITICAL(&gyroRezeroMux);
    gyroBiasRunning.bias.x = mean.x;
    gyroBiasRunning.bias.y = mean.y;
    gyroBiasRunning.bias.z = mean.z;
    sensorsMpu6050RezeroCountNow++;
    portEXIT_CRITICAL(&gyroRezeroMux);
    gyroTcLatchBias(&old, &mean, now);
}

/* Every stabilizer loop (flash): 14bcfc6's two stores, plus the airborne tick
 * and the hold's end only when they apply. The compensation's thrust and
 * airborne flags are derived from lastThrustTick / lastAirTick by the sensors
 * task at 20 Hz (gyroTcBlockStep). */
void sensorsMpu6050SetThrust(float thrust)
{
    const bool air = thrust > 30000.0f;    /* above lift-off thrust (>= 30000 recorded with the camera fitted) */
    sensorsMpu6050InFlightNow = air;
    if (thrust > 0.0f) {
        const TickType_t now = xTaskGetTickCount();
        lastThrustTick = now;
        if (air) {
            lastAirTick = now;
        }
        if (gyroTcHoldOn) {
            gyroTcHoldOn = false;   /* any thrust ends a 'gtc hold' */
        }
    }
}

void sensorsMpu6050GyroTcBypass(bool on)
{
    if (on == benchBypassOn(BENCH_BYPASS_GTC)) {
        return;
    }
    portENTER_CRITICAL(&gyroTcMux);
    if (on) {
        benchBypassMask |= BENCH_BYPASS_GTC;   /* the USB task is the mask's only writer */
    } else {
        /* The sensors task has not touched these since the bit was set: restart unseeded, unlatched (the
         * correction 0 until the next accepted ground re-zero), with the thrust flags from now. */
        gyroTcInit(&gyroTc);
        gyroTcRawSum = 0;
        gyroTcN = 0;
        accTcInit(&accTc);
        accTcRawSum[0] = accTcRawSum[1] = accTcRawSum[2] = 0;
        accTcWindowInit(&accTcWin);
        accTcRefValid = false;
        gyroTcThrustSince = false;
        gyroTcAirSince = false;
        gyroTcThrustSeen = lastThrustTick;
        gyroTcAirLast = lastAirTick;
        benchBypassMask &= ~BENCH_BYPASS_GTC;
    }
    for (int i = 0; i < 3; i++) {
        sensorsMpu6050GyroTcCorrNow[i] = 0.0f;   /* nothing subtracted (and +0 when it comes back) */
        sensorsMpu6050AccTcCorrNow[i] = 0.0f;
    }
    portEXIT_CRITICAL(&gyroTcMux);
}

void sensorsMpu6050GetGyroTc(sensorsGyroTcStatus_t *st)
{
    const TickType_t now = xTaskGetTickCount();
    memset(st, 0, sizeof(*st));
    portENTER_CRITICAL(&gyroTcMux);
    st->cfg = gyroTcCfg;
    st->tLp = gyroTc.tLp;
    st->tRef = gyroTc.tRef;
    st->refSet = gyroTc.refSet;
    st->latchAgeMs = gyroTc.refSet ? T2M(now - gyroTcLatchTick) : 0;
    for (int i = 0; i < 3; i++) {
        st->corr[i] = sensorsMpu6050GyroTcCorrNow[i];
    }
    gyroTcModel(&gyroTcCfg, &gyroTc, st->model);
    st->logged = gyroTcLog.n;
    const uint32_t n = gyroTcLogCount(&gyroTcLog);
    for (uint32_t i = 0; i < n; i++) {
        st->last[i] = *gyroTcLogEntry(&gyroTcLog, i);
    }
    st->n = (uint8_t)n;
    st->airSeq = gyroTcLog.airSeq;
    if (gyroTcLog.airSeq > 0) {
        st->air = gyroTcLog.air;
        st->airAgeMs = T2M(now) - gyroTcLog.airAtMs;
        st->airLandMs = gyroTcLog.airLandMs;
        st->airSpanMs = gyroTcLog.airSpanMs;
    }
    st->holdOn = gyroTcHoldOn;
    st->holdLeftMs = gyroTcHoldOn && (int32_t)(gyroTcHoldUntil - now) > 0 ? T2M(gyroTcHoldUntil - now) : 0;
    st->thrustSinceLatch = gyroTcThrustSince;
    st->airSinceLatch = gyroTcAirSince;
    st->bypassed = benchBypassOn(BENCH_BYPASS_GTC);
    portEXIT_CRITICAL(&gyroTcMux);
    st->loadResult = gyroTcLoadResult;
    st->tempC = sensorsMpu6050GetTempC();
    st->rezeros = sensorsMpu6050RezeroCountNow;
}

uint32_t sensorsMpu6050GetGyroTcAir(uint32_t seenSeq, sensorsGyroTcRezero_t *e, float *dTeff, uint32_t *landMs)
{
    portENTER_CRITICAL(&gyroTcMux);
    const uint32_t seq = gyroTcLog.airSeq;
    if (seq != seenSeq && seq > 0) {
        *e = gyroTcLog.air;
        *dTeff = gyroTcEffDelta(&gyroTcCfg, gyroTcLog.air.tNowCc / 100.0f, gyroTcLog.air.tRefOldCc / 100.0f);
        *landMs = gyroTcLog.airLandMs;
    }
    portEXIT_CRITICAL(&gyroTcMux);
    return seq;
}

bool sensorsMpu6050SetGyroTcConfig(const gyroTcConfig_t *c, int loadResult)
{
    if (!gyroTcConfigValid(c)) {
        return false;
    }
    portENTER_CRITICAL(&gyroTcMux);
    gyroTcCfg = *c;
    portEXIT_CRITICAL(&gyroTcMux);
    gyroTcLoadResult = loadResult;
    return true;
}

void sensorsMpu6050AccTcLatch(bool coolFollows)
{
    accTcLatchReq = ACC_TC_REQ_LATCH | (coolFollows ? ACC_TC_REQ_COOL : 0);
}

void sensorsMpu6050GetAccTc(sensorsAccTcStatus_t *st)
{
    const TickType_t now = xTaskGetTickCount();
    memset(st, 0, sizeof(*st));
    portENTER_CRITICAL(&gyroTcMux);
    st->cfg = accTcCfg;
    st->tLp = accTc.tLp;
    st->tRef = accTc.tRef;
    st->refSet = accTc.refSet;
    st->refValid = accTcRefValid;
    for (int i = 0; i < 3; i++) {
        st->refMg[i] = accTcRefMg[i];
        st->corr[i] = sensorsMpu6050AccTcCorrNow[i] * 1000.0f;
    }
    st->stillValid = accTcWindowMean(&accTcWin, st->stillMg);
    st->latchAgeMs = accTc.refSet ? T2M(now - accTcLatchTick) : 0;
    accTcModel(&accTcCfg, &accTc, st->model);
    st->logged = accTcLog.n;
    const uint32_t n = accTcLogCount(&accTcLog);
    for (uint32_t i = 0; i < n; i++) {
        st->last[i] = *accTcLogEntry(&accTcLog, i);
    }
    st->n = (uint8_t)n;
    st->airSeq = accTcLog.airSeq;
    if (accTcLog.airSeq > 0) {
        st->air = accTcLog.air;
        st->airAgeMs = T2M(now) - accTcLog.airAtMs;
        st->airLandMs = accTcLog.airLandMs;
    }
    st->thrustSinceLatch = gyroTcThrustSince;
    st->airSinceLatch = gyroTcAirSince;
    st->coolSince = accTcCoolSince;
    st->latchPending = accTcLatchReq != 0;
    st->bypassed = benchBypassOn(BENCH_BYPASS_GTC);
    portEXIT_CRITICAL(&gyroTcMux);
    st->loadResult = accTcLoadResult;
    st->tempC = sensorsMpu6050GetTempC();
}

uint32_t sensorsMpu6050GetAccTcAir(uint32_t seenSeq, sensorsAccTcRezero_t *e, float *dTeff, uint32_t *landMs)
{
    portENTER_CRITICAL(&gyroTcMux);
    const uint32_t seq = accTcLog.airSeq;
    if (seq != seenSeq && seq > 0) {
        *e = accTcLog.air;
        *dTeff = accTcEffDelta(&accTcCfg, accTcLog.air.tNowCc / 100.0f, accTcLog.air.tRefCc / 100.0f);
        *landMs = accTcLog.airLandMs;
    }
    portEXIT_CRITICAL(&gyroTcMux);
    return seq;
}

bool sensorsMpu6050SetAccTcConfig(const accTcConfig_t *c, int loadResult)
{
    if (!accTcConfigValid(c)) {
        return false;
    }
    portENTER_CRITICAL(&gyroTcMux);
    accTcCfg = *c;
    float ta[3];
    accTcCorr(&accTcCfg, &accTc, ta);   /* 'atc off' stops subtracting at once; 'on' starts from the reference */
    for (int i = 0; i < 3; i++) {
        sensorsMpu6050AccTcCorrNow[i] = ta[i] * 0.001f;
    }
    portEXIT_CRITICAL(&gyroTcMux);
    accTcLoadResult = loadResult;
    return true;
}

bool sensorsMpu6050AccTcOn(void)
{
    return accTcCfg.on;
}

bool sensorsMpu6050GyroTcHold(uint32_t ms)
{
    if (ms > GYRO_TC_HOLD_MAX_MS) {
        return false;
    }
    gyroTcHoldUntil = xTaskGetTickCount() + M2T(ms);
    gyroTcHoldOn = ms > 0;
    if (ms > 0) {
        portENTER_CRITICAL(&gyroTcMux);
        gyroTcHoldSince = true;   /* the first re-zero after it is the cooling test's residual: logged */
        portEXIT_CRITICAL(&gyroTcMux);
    }
    return true;
}

void sensorsMpu6050GetGyroRezero(uint32_t *count, float drift[3])
{
    Axis3f b, b0;
    portENTER_CRITICAL(&gyroRezeroMux);
    *count = sensorsMpu6050RezeroCountNow;
    b = gyroBiasRunning.bias;
    b0 = gyroBiasStartup;
    portEXIT_CRITICAL(&gyroRezeroMux);
    if (!gyroBiasStartupSet) {
        drift[0] = drift[1] = drift[2] = 0.0f;
        return;
    }
    /* What a still gyro read with the startup bias, in the body frame (x is mirrored). */
    drift[0] = -(b.x - b0.x) * SENSORS_DEG_PER_LSB_CFG;
    drift[1] = (b.y - b0.y) * SENSORS_DEG_PER_LSB_CFG;
    drift[2] = (b.z - b0.z) * SENSORS_DEG_PER_LSB_CFG;
}

bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void)
{
    bool testStatus = false;
    Axis3i16 g;
    Axis3i16 a;
    Axis3f acc; // Accelerometer axis data in mG
    float pitch, roll;
    uint32_t startTick = xTaskGetTickCount();

    testStatus = mpu6050SelfTest();

    if (testStatus) {
        sensorsBiasObjInit(&gyroBiasRunning);

        while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT) {
            mpu6050GetMotion6(&a.y, &a.x, &a.z, &g.y, &g.x, &g.z);

            if (processGyroBias(g.x, g.y, g.z, &gyroBias)) {
                gyroBiasFound = true;
                DEBUG_PRINTI("Gyro variance test [OK]\n");
                break;
            }
        }

        if (gyroBiasFound) {
            acc.x = (a.x) * SENSORS_G_PER_LSB_CFG;
            acc.y = (a.y) * SENSORS_G_PER_LSB_CFG;
            acc.z = (a.z) * SENSORS_G_PER_LSB_CFG;

            // Calculate pitch and roll based on accelerometer. Board must be level
            pitch = tanf(-acc.x / (sqrtf(acc.y * acc.y + acc.z * acc.z))) * 180 / (float)M_PI;
            roll = tanf(acc.y / acc.z) * 180 / (float)M_PI;

            if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX)) {
                DEBUG_PRINTI("Acc level test [OK]\n");
                testStatus = true;
            } else {
                DEBUG_PRINTE("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
                testStatus = false;
            }
        } else {
            DEBUG_PRINTE("Gyro variance test [FAIL]\n");
            testStatus = false;
        }
    }

    return testStatus;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    imuAlignmentApply(&accAlignment, in->axis, out->axis);
}

void sensorsMpu6050AccumStart(void)
{
    portENTER_CRITICAL(&accumMux);
    accumCount = 0;
    for (int i = 0; i < 3; i++) {
        accumAccSum[i] = 0;
        accumAccSumSq[i] = 0;
        accumGyroSum[i] = 0;
        accumGyroSumSq[i] = 0;
    }
    accumEnabled = true;
    portEXIT_CRITICAL(&accumMux);
}

static void accumStats(int64_t sum, int64_t sumSq, uint32_t n, double lsbToUnit, double bias,
                       float *mean, float *std)
{
    double m = (double)sum / n;
    double var = (double)sumSq / n - m * m;
    *mean = (float)((m - bias) * lsbToUnit);
    *std = (float)(sqrt(var > 0.0 ? var : 0.0) * lsbToUnit);
}

bool sensorsMpu6050AccumStop(sensorsImuStats_t *out)
{
    int64_t accSum[3], accSumSq[3], gyroSum[3], gyroSumSq[3];
    uint32_t n;

    portENTER_CRITICAL(&accumMux);
    accumEnabled = false;
    n = accumCount;
    for (int i = 0; i < 3; i++) {
        accSum[i] = accumAccSum[i];
        accSumSq[i] = accumAccSumSq[i];
        gyroSum[i] = accumGyroSum[i];
        gyroSumSq[i] = accumGyroSumSq[i];
    }
    portEXIT_CRITICAL(&accumMux);

    out->count = n;
    if (n == 0) {
        return false;
    }

    /* Gyro bias is kept in native-order raw LSB; map it like the samples. */
    const double gyroBiasMapped[3] = {-gyroBias.x, gyroBias.y, gyroBias.z};
    for (int i = 0; i < 3; i++) {
        accumStats(accSum[i], accSumSq[i], n, SENSORS_G_PER_LSB_CFG, 0.0, &out->accMean[i], &out->accStd[i]);
        accumStats(gyroSum[i], gyroSumSq[i], n, SENSORS_DEG_PER_LSB_CFG, gyroBiasMapped[i],
                   &out->gyroMean[i], &out->gyroStd[i]);
        out->gyroBias[i] = (float)(gyroBiasMapped[i] * SENSORS_DEG_PER_LSB_CFG);
    }
    out->gyroBiasFound = gyroBiasFound;
    return true;
}

void sensorsMpu6050GetLevelOffset(float *x, float *y, int *loadResult)
{
    portENTER_CRITICAL(&levelCalMux);
    *x = accLevelOffsetX;
    *y = accLevelOffsetY;
    portEXIT_CRITICAL(&levelCalMux);
    *loadResult = levelCalLoadResult;
}

bool sensorsMpu6050SetLevelOffset(float x, float y)
{
    if (!imuLevelCalOffsetsValid(x, y)) {
        return false;
    }
    portENTER_CRITICAL(&levelCalMux);
    accLevelOffsetX = x;
    accLevelOffsetY = y;
    portEXIT_CRITICAL(&levelCalMux);
    return true;
}

float sensorsMpu6050GetAccScale(void)
{
    return accScale;
}

void sensorsMpu6050GetFlightAccOffset(float *x, float *y)
{
    portENTER_CRITICAL(&levelCalMux);
    *x = flightAccOffsetX;
    *y = flightAccOffsetY;
    portEXIT_CRITICAL(&levelCalMux);
}

bool sensorsMpu6050SetFlightAccOffset(float x, float y)
{
    if (!isfinite(x) || !isfinite(y) || fabsf(x) > FLIGHT_ACC_MAX_G || fabsf(y) > FLIGHT_ACC_MAX_G) {
        return false;
    }
    portENTER_CRITICAL(&levelCalMux);
    flightAccOffsetX = x;
    flightAccOffsetY = y;
    portEXIT_CRITICAL(&levelCalMux);
    return true;
}

/** set different low pass filters in different environment
 *
 *
 */
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode)
{
    switch (accMode)
    {
    case ACC_MODE_PROPTEST:
        mpu6050SetRate(7);
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        mpu6050SetRate(0);
#ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#else
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#endif
        break;
    }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

//TODO:
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6050, &isMpu6050TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, pmw3901, &isPmw3901Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_tests)
