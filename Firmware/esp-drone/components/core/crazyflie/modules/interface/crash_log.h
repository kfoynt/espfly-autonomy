/*
 * ESP-Drone Firmware
 *
 * crash_log.h - Post-mortem record that survives a panic / software reset
 *
 * Kept in RTC no-init memory, which is not cleared by a software reset, a
 * panic or a watchdog reset (it is invalid after power-on; a magic number
 * and CRC tell the two apart). Holds:
 *  - the last panic: exception, PC, EXCCAUSE/EXCVADDR, up to 12 backtrace PCs
 *    (map them with xtensa-esp32s3-elf-addr2line -e ESPDrone.elf) and the
 *    abort()/assert message if there was one;
 *  - the most recent ESP_LOG warning/error lines (ring buffer).
 *
 * The panic handler and esp_system_abort() are wrapped at link time
 * (-Wl,--wrap=...), so nothing else has to call into this module.
 */
#ifndef CRASH_LOG_H_
#define CRASH_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#define CRASH_LOG_BACKTRACE_DEPTH 12
#define CRASH_LOG_DETAILS_LEN     96
#define CRASH_LOG_RING_LEN        1024
#define CRASH_LOG_BOOT_HISTORY    8

typedef struct {
  uint32_t valid;                 /* a panic was captured and not yet cleared */
  uint32_t bootsSince;            /* boots since the panic */
  int32_t core;
  int32_t exception;              /* panic_exception_t */
  uint32_t pc;
  uint32_t exccause;
  uint32_t excvaddr;
  uint32_t backtrace[CRASH_LOG_BACKTRACE_DEPTH];
  uint32_t depth;
  char details[CRASH_LOG_DETAILS_LEN];   /* abort/assert message, may be empty */
} crashInfo_t;

typedef struct {
  int32_t resetReason;      /* esp_reset_reason_t of this boot */
  uint32_t prevUptimeS;     /* how long the previous boot ran before this reset */
  uint8_t rawReason[2];     /* ROM reset cause per core (soc_reset_reason_t). With this build's
                             * CONFIG_ESP_SYSTEM_BROWNOUT_INTR a brownout shows as 0x03 (software
                             * reset by the brownout ISR) + ESP_RST_BROWNOUT; 0x08 (MWDT1) = the
                             * hardware stage of the interrupt watchdog, CPU stalled (e.g. on
                             * flash/PSRAM); 0x01 with prevUptimeS > 0 = the supply fell to the
                             * power-on-reset level (or the bootloader's analog brownout fired),
                             * not a battery unplug. 0x0F does not occur in this configuration. */
  uint8_t pad[2];
} crashBoot_t;

/* Call once at start-up: validates the RTC record and hooks ESP_LOG. */
void crashLogInit(void);

/* Call about once per second with the current uptime (seconds). */
void crashLogTick(uint32_t uptimeS);

/* Boots recorded since the last power-on, newest first; returns the count. */
int crashLogBoots(crashBoot_t *out, int max, uint32_t *totalBoots);

bool crashLogGet(crashInfo_t *out);
void crashLogClear(void);

/* Append one line (at most CRASH_LOG_NOTE_MAX characters, no newline needed)
 * to the log ring without printing it: no formatting, no console, a short
 * spinlock. Any task; for code that must not block or use much stack (the
 * motor watchdog's esp_timer callback). */
#define CRASH_LOG_NOTE_MAX 110
void crashLogNote(const char *text);

/* Copy the log ring (oldest first, NUL-terminated) into buf. */
void crashLogGetLines(char *buf, int len);

#endif /* CRASH_LOG_H_ */
