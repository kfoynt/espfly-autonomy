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
 * Copyright (C) 2011-2013 Bitcraze AB
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
 * usec_time.c - microsecond-resolution timer and timestamps.
 */

#include "usec_time.h"
#include "esp_attr.h"
#include "esp_timer.h"
void initUsecTimer(void)
{

}

/* In IRAM (15 B, no literal: the call to esp_timer_get_time, itself in IRAM, relaxes to a call8; inside the
 * slack before the next 256 B heap step, docs/autonomy-a2b.md 19.11): the sensors' data-ready ISR (IRAM) calls
 * it every ms, the stabilizer three times a loop and the camera hold in the controller, so in flash it was one or
 * two instruction-cache lines missed every ms. */
IRAM_ATTR uint64_t usecTimestamp(void)
{
    return (uint64_t)esp_timer_get_time();
}


