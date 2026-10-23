/*
 * ESP-Drone Firmware
 *
 * motor_guard.h - Independent motor watchdog: cut the motors when the
 * stabilizer loop stops while they are driven (docs/autonomy-a2.md section 7)
 *
 * The LEDC hardware keeps the last motor duty when the stabilizer stops, so
 * a frozen loop (an MPU-6050 that browned out into sleep and gives no
 * data-ready interrupt, a hung task) keeps the motors at, say, climb thrust
 * with no attitude control, and every kill path serviced inside that loop is
 * dead. This guard runs outside it: an esp_timer every MOTOR_GUARD_PERIOD_US
 * (esp_timer task, priority 22, core 0) compares the stabilizer's alive
 * counter with its last value. If the counter has not moved for
 * MOTOR_GUARD_FREEZE_US of the guard's own on-time observation while any
 * motor duty is nonzero, it stops the motors through the driver's latched
 * STOP (motorsEmergencyStopNoWait(), never blocking on the driver mutex) and
 * runs the red restart-lock blink. Latched until reboot: a battery replug
 * or the reset button (power-on reset) always clears it; a software reboot
 * does too, but then the restart lock ("plock", stabilizer.c, default on)
 * holds thrust at 0 until a replug. Nothing about a trip is stored in NVS
 * (the crash-log ring line survives only resets that keep RTC memory).
 *
 * Freeze threshold 80 ms: in-flight loop gaps never exceeded 3.23 ms (57
 * logged flights); two one-off gaps of 53 and 55 ms were seen with the
 * motors off and no cause proven, so the threshold sits ~1.5x above them,
 * since a false cut in flight is a crash. The stabilizer's own worst
 * legitimate stalls are a console print spinning on a full USB-Serial-JTAG
 * FIFO (<= 50 ms, vfs_usb_serial_jtag.c) plus a failed IMU read (<= ~11 ms).
 * A stall of the whole system (flash erase, a long critical section, a
 * priority >= 23 task on core 0 such as cam_task printing) also stops this
 * timer; a callback gap > MOTOR_GUARD_LATE_US is not counted, so such stalls
 * cannot cut, but the stall observed before it is kept, so a guard that is
 * often late still trips on a real freeze. Panics and the interrupt watchdog
 * are covered separately: the crash-log panic wrappers call
 * motorGuardPanicPinsLow() first. A brownout resets the chip directly (pins
 * back to their reset state); nothing here runs for it.
 *
 * Active whenever any motor duty is nonzero, in manual flight too. The pure
 * decision (motorGuardCoreStep) is host-tested by
 * tools/tests/test_motor_guard.c; this header has no ESP-IDF dependency.
 */
#ifndef MOTOR_GUARD_H_
#define MOTOR_GUARD_H_

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_GUARD_PERIOD_US   5000    /* timer period */
#define MOTOR_GUARD_FREEZE_US   80000   /* no stabilizer progress this long with motors on: cut */
#define MOTOR_GUARD_LATE_US     15000   /* the guard's own callback gap above this: system-wide stall, that gap is not counted */

/* ---- pure decision (utils/src/motor_guard_core.c) ---------------------- */
typedef enum {
  MG_OK = 0,      /* progress seen, or motors off */
  MG_STALLING,    /* no progress yet, below the threshold (stallUs says how long) */
  MG_LATE,        /* first call, or this callback came late: its gap is not counted */
  MG_TRIP,        /* cut the motors now (returned once; then MG_TRIPPED) */
  MG_TRIPPED,     /* already tripped, latched */
  MG_DISABLED,    /* guard switched off */
} motorGuardVerdict_t;

typedef struct {
  uint64_t lastCallUs;    /* previous callback time, 0 = none yet */
  uint64_t windowStartUs; /* when the current no-progress window started */
  uint32_t lastAlive;     /* alive counter seen at windowStartUs */
  uint32_t stallUs;       /* no-progress time in this window, on-time gaps only */
  uint32_t maxStallUs;    /* longest no-progress time with motors on, since init */
  uint32_t lateCalls;     /* callbacks that came late */
  uint32_t maxCallGapUs;  /* longest gap between two callbacks */
  bool tripped;
} motorGuardCore_t;

void motorGuardCoreInit(motorGuardCore_t *c);
/* One timer callback at nowUs. alive is the stabilizer's counter, motorsOn
 * whether any motor duty is nonzero right now, enabled the user switch.
 * Rules (in order): tripped -> MG_TRIPPED; !enabled -> restart the window,
 * MG_DISABLED; first call (or a clock gone backwards) -> restart the window,
 * MG_LATE; alive changed or !motorsOn -> restart the window, MG_OK (MG_LATE
 * if the gap was late); gap = nowUs - lastCallUs > MOTOR_GUARD_LATE_US ->
 * MG_LATE, stallUs unchanged (the gap was not observed; what was observed
 * before it stays counted); else stallUs += gap and MG_TRIP once it reaches
 * MOTOR_GUARD_FREEZE_US, MG_STALLING before. */
motorGuardVerdict_t motorGuardCoreStep(motorGuardCore_t *c, uint64_t nowUs, uint32_t alive, bool motorsOn,
                                       bool enabled);

/* ---- drone side (modules/src/motor_guard.c) ---------------------------- */

/* Starts the timer. Call once from stabilizerInit() after
 * powerDistributionInit() (motors initialised), before the stabilizer task
 * starts. The switch is loaded from NVS ("mguard", default on) by the caller. */
void motorGuardInit(bool enabled);

/* Stabilizer task, once per loop, right after the motors were written
 * (powerStop() or powerDistribution()). Lock-free: one atomic increment. */
void motorGuardStabilizerAlive(void);

/* USB console 'mguard on|off' (persisted by the caller). Autonomy ARM
 * requires it on. Switching it off never clears a trip; only a reboot does. */
void motorGuardSetEnabled(bool on);
bool motorGuardEnabled(void);
bool motorGuardTripped(void);

typedef struct {
  bool enabled;
  bool tripped;
  uint32_t trips;          /* 0 or 1 (latched until reboot) */
  uint32_t tripUptimeMs;   /* when it tripped, 0 = never */
  uint32_t aliveCount;     /* stabilizer loops seen */
  uint32_t maxStallUs;     /* longest no-progress time with motors on */
  uint32_t lateCalls;      /* guard callbacks that came late (system-wide stalls) */
  uint32_t maxCallGapUs;
  uint32_t callbacks;
} motorGuardStats_t;

void motorGuardGetStats(motorGuardStats_t *out);

/* Drives the four motor pins (GPIO 7/4/3/1) low through direct GPIO
 * register writes and the ROM GPIO-matrix call: no locks, no flash, no
 * driver state. IRAM; called first thing in crash_log.c's
 * __wrap_esp_panic_handler and __wrap_esp_system_abort. */
void motorGuardPanicPinsLow(void);

#endif /* MOTOR_GUARD_H_ */
