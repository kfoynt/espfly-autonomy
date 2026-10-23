/*
 * ESP-Drone Firmware
 *
 * takeoff_aux.h - Lighten the XIAO's load during a take-off spin-up
 */
#ifndef TAKEOFF_AUX_H_
#define TAKEOFF_AUX_H_

#include <stdbool.h>

/* Starts the task that lowers the Wi-Fi TX power during the stabilizer's
 * take-off window (switch persisted in NVS, default on). */
void takeoffAuxInit(void);
void takeoffAuxSetTxLowEnabled(bool on);
bool takeoffAuxTxLowEnabled(void);

#endif /* TAKEOFF_AUX_H_ */
