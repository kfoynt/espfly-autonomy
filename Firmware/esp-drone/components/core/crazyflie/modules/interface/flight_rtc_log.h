/*
 * ESP-Drone Firmware
 *
 * flight_rtc_log.h - Flight summaries and a pre-reset trace in RTC memory
 *
 * Survives brownouts, panics and watchdog resets (not power-off). Keeps the
 * last FLIGHT_RTC_LOG_FLIGHTS flight summaries (flight_summary.h), a 25 Hz
 * trace of the last 4 s with thrust, the lowest battery voltage (and, in the
 * diagnostic build, the lowest XIAO 3.3 V rail reading) in each of the last
 * 32 x 10 ms windows (sampled at 1 kHz by a monitor task, so short dips show),
 * and a copy of these taken at boot whenever a reset interrupted a flight.
 */
#ifndef FLIGHT_RTC_LOG_H_
#define FLIGHT_RTC_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "flight_recorder.h"
#include "flight_summary.h"

#define FLIGHT_RTC_LOG_FLIGHTS 16
#define FLIGHT_RTC_LOG_CRUMBS  100   /* 4 s at 25 Hz */
#define FLIGHT_RTC_LOG_VWIN    32    /* 10 ms battery-minimum windows: 320 ms */

/* Call once at start-up, after crashLogInit(). */
void flightRtcLogInit(void);

/* Called with every flight-recorder sample (100 Hz). */
void flightRtcLogSample(const flightRecord_t *r);

/* Number of stored summaries (at most FLIGHT_RTC_LOG_FLIGHTS); *total counts
 * all flights since the log was created. */
int flightRtcLogFlightCount(uint32_t *total);
/* Summary 'index', 0 = oldest stored. False if out of range. */
bool flightRtcLogFlight(int index, flightSummary_t *out);

/* Trace saved when a reset interrupted a flight: number of samples (0 if
 * none) and the boot / reset reason it belongs to. */
int flightRtcLogResetTraceInfo(uint16_t *boot, int32_t *resetReason);
/* Trace sample 'index', 0 = oldest. False if out of range. */
bool flightRtcLogResetTraceGet(int index, fsumCrumb_t *out);
/* Battery minimum per 10 ms window before that reset, oldest first (mV); the
 * last entry is the window the reset cut short. Returns the count. */
int flightRtcLogResetVbat(uint16_t *out, int max);

/* Rail-meter minimum per 10 ms window before that reset, same windows as
 * flightRtcLogResetVbat (mV, 0 = no reading). Returns the count. */
int flightRtcLogResetRail(uint16_t *out, int max);

/* Lowest 1 kHz battery reading over the last 320 ms (mV, 0 if none yet). */
uint16_t flightRtcLogLiveVbatMin(void);
/* Latest 1 kHz battery and rail readings and their minima over the last
 * 320 ms (mV; rail 0 without the meter). */
void flightRtcLogLive(uint16_t *vbatNow, uint16_t *vbatMin, uint16_t *railNow, uint16_t *railMin);

void flightRtcLogClear(void);

#endif /* FLIGHT_RTC_LOG_H_ */
