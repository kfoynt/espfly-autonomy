/*
 * Snapshots of ESP32-S3 state that a software (core) reset keeps but a
 * power-on clears: RTC_CNTL, RTC IO, SENS, IO MUX, USB-Serial-JTAG, the
 * analog I2C master and the analog trim registers (regi2c). Used to find what
 * differs between a clean power-on boot and a boot after a software reset.
 */
#ifndef REG_SNAP_H_
#define REG_SNAP_H_

#include <stdint.h>

#define REG_SNAP_WORDS 512
#define REG_SNAP_ANA_BYTES 48

typedef struct {
  uint32_t takenUs;
  uint16_t words;
  uint32_t w[REG_SNAP_WORDS];
  uint8_t ana[REG_SNAP_ANA_BYTES];
} regSnap_t;

/* Call first thing in app_main: keeps a copy of the state the boot started with. */
void regSnapBoot(void);

/* Print the boot copy and a fresh copy with the given line printer
 * ("R <snap> <block> +<offset> <8 words>", "A <snap> <block> <bytes>", "E ..."). */
void regSnapPrint(void (*line)(const char *fmt, ...));

#endif /* REG_SNAP_H_ */
