/*
 * ESP-Drone Firmware
 *
 * att_trim_auto.h - Learn the attitude trim from the camera drift hold
 */
#ifndef ATT_TRIM_AUTO_H_
#define ATT_TRIM_AUTO_H_

#include <stdbool.h>
#include <stdint.h>

/* Loads the switch (NVS, default on). */
void attTrimAutoInit(void);

/* Called every ~10 ms from a low-priority task: follows the flight recorder,
 * learns at the end of each flight (att_trim_learn.h), applies the new trim
 * at once and saves it to NVS after a few seconds at rest on the ground. */
void attTrimAutoPoll(void);

/* The console changed the trim: liveOnly after 'atrim set' (not saved; learning
 * pauses until save or clear), false after 'atrim save' or 'atrim clear'.
 * Either way a pending learned save is dropped. */
void attTrimAutoNoteManual(bool liveOnly);
bool attTrimAutoManualLiveOnly(void);

void attTrimAutoSetEnabled(bool on);
bool attTrimAutoEnabled(void);

/* Flights learned from since boot, the last one's mean camera correction
 * (deg) and whether a learned trim still waits to be saved. */
void attTrimAutoStats(uint32_t *flights, float *lastMeanRoll, float *lastMeanPitch, bool *savePending);

#endif /* ATT_TRIM_AUTO_H_ */
