/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
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
 *
 */
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"

#include "cf_math.h"
#include "param.h"
#include "stm32_legacy.h"
#include "static_mem.h"

static bool isInit;
const static setpoint_t nullSetpoint;
#if !DRONE_MANUAL_FLIGHT_CAMERA
static setpoint_t tempSetpoint;
#endif
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
#if !DRONE_MANUAL_FLIGHT_CAMERA
  crtpCommanderHighLevelInit();
#endif
  lastUpdate = xTaskGetTickCount();

  isInit = true;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
#if DRONE_MANUAL_FLIGHT_CAMERA
  // This build accepts only the manual RPYT path, never EXTRX/planner modes.
  if (setpoint == NULL || priority != COMMANDER_PRIORITY_CRTP ||
      !isfinite(setpoint->thrust) || !isfinite(setpoint->attitude.roll) ||
      !isfinite(setpoint->attitude.pitch) || !isfinite(setpoint->attitudeRate.yaw) ||
      setpoint->thrust < 0 || setpoint->thrust > 60000 ||
      fabsf(setpoint->attitude.roll) > 30 || fabsf(setpoint->attitude.pitch) > 30 ||
      fabsf(setpoint->attitudeRate.yaw) > 200 ||
      setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable ||
      setpoint->mode.z != modeDisable || setpoint->mode.quat != modeDisable ||
      setpoint->mode.roll != modeAbs || setpoint->mode.pitch != modeAbs ||
      setpoint->mode.yaw != modeVelocity) {
    return;
  }
#endif
  int currentPriority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
    // Send the high-level planner to idle so it will forget its current state
    // and start over if we switch from low-level to high-level in the future.
#if !DRONE_MANUAL_FLIGHT_CAMERA
    crtpCommanderHighLevelStop();
#endif
  }
}

void commanderNotifySetpointsStop(int remainValidMillisecs)
{
#if DRONE_MANUAL_FLIGHT_CAMERA
  // Generic meta commands cannot extend or hand off manual control.
  (void)remainValidMillisecs;
  return;
#else
  uint32_t currentTime = xTaskGetTickCount();
  int timeSetback = MIN(
    COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
    currentTime
  );
  xQueuePeek(setpointQueue, &tempSetpoint, 0);
  tempSetpoint.timestamp = currentTime - timeSetback;
  xQueueOverwrite(setpointQueue, &tempSetpoint);
  crtpCommanderHighLevelTellState(&lastState);
#endif
}

void commanderDropSetpoint(void)
{
  xQueueOverwrite(setpointQueue, &nullSetpoint);   /* timestamp 0: expired, thrust 0 */
  xQueueOverwrite(priorityQueue, &priorityDisable);
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
#if DRONE_MANUAL_FLIGHT_CAMERA
    memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    xQueueOverwrite(priorityQueue, &priorityDisable);
#else
    if (enableHighLevel) {
      crtpCommanderHighLevelGetSetpoint(setpoint, state);
    }
    if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    }
#endif
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    xQueueOverwrite(priorityQueue, &priorityDisable);
    // Leveling ...
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
    // Keep Z as it is
  }
  // This copying is not strictly necessary because stabilizer.c already keeps
  // a static state_t containing the most recent state estimate. However, it is
  // not accessible by the public interface.
  lastState = *state;
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;

#if DRONE_MANUAL_FLIGHT_CAMERA
  // Re-lock after a gap even if the IMU task has not yet polled the watchdog.
  setpoint_t queued;
  const BaseType_t setpointResult = xQueuePeek(setpointQueue, &queued, 0);
  ASSERT(setpointResult == pdTRUE);
  if ((uint32_t)(xTaskGetTickCount() - queued.timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    return COMMANDER_PRIORITY_DISABLE;
  }
#endif

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);

  return priority;
}

PARAM_GROUP_START(commander)
PARAM_ADD(PARAM_UINT8 | (DRONE_MANUAL_FLIGHT_CAMERA ? PARAM_RONLY : 0), enHighLevel, &enableHighLevel)
PARAM_GROUP_STOP(commander)
