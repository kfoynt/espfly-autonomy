/*
 * ESP-Drone Firmware
 *
 * usb_diag.h - Line-based diagnostics console on the XIAO ESP32-S3 USB-C port
 *
 * Runs as a low-priority task. Reads commands from the USB-Serial-JTAG
 * endpoint and answers on the same endpoint; output is dropped (and
 * counted) when no host is reading, so it never blocks flight control.
 * See tools/espfly_diag.py and docs/imu-calibration.md for usage.
 */
#ifndef USB_DIAG_H_
#define USB_DIAG_H_

void usbDiagInit(void);

#endif /* USB_DIAG_H_ */
