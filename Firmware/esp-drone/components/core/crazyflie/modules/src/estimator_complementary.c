/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * estimator_complementary.c - a complementary estimator
 */



#include "FreeRTOS.h"
#include "queue.h"
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "imu_level_cal_store.h"
#include "bench_bypass.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement);

// Measurements of TOF from laser sensor
#define TOF_QUEUE_LENGTH (1)
static xQueueHandle tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));


void estimatorComplementaryInit(void)
{
  tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);

  sensfusion6Init();
  /* the in-flight bias learning gain (NVS mkiair, USB 'mki'; docs/autonomy-a2b.md 19.2) */
  float kiAir;
  if (mahonyKiAirStoreLoad(&kiAir) == ESP_OK) {
    sensfusion6SetKiAir(kiAir);
  }
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
  sensorsAcquire(sensorData, tick); // Read sensors at full rate (1000Hz)
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    /* In flight the filter learns the gyro bias faster; a new ground bias resets what it learned. Inline loads and
     * stores, no calls (docs/autonomy-a2b.md 19.11); the bisect toggle skips them (bench_bypass.h). */
    if (!benchBypassOn(BENCH_BYPASS_MKI)) {
      sensfusion6SetAirborne(sensorsMpu6050InFlight());
      sensfusion6NoteGyroBiasEpoch(sensorsMpu6050GyroRezeroCount());
    }
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    tofMeasurement_t tofMeasurement;

    latestTofMeasurement(&tofMeasurement);
    positionEstimate(state, sensorData, &tofMeasurement, POS_UPDATE_DT, tick);
  }
}

static bool latestTofMeasurement(tofMeasurement_t* tofMeasurement) {
  return xQueuePeek(tofDataQueue, tofMeasurement, 0) == pdTRUE;
}

static bool overwriteMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = false;//(SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueOverwriteFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueOverwrite(queue, measurement);
  }
  return (result==pdTRUE);
}


bool estimatorComplementaryEnqueueTOF(const tofMeasurement_t *tof)
{
  // A distance (distance) [m] to the ground along the z_B axis.
  return overwriteMeasurement(tofDataQueue, (void *)tof);
}
