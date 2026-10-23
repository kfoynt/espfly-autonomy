/*
 * ESP-Drone Firmware
 *
 * flight_summary.h - Per-flight statistics small enough to keep in RTC memory
 *
 * The RAM flight recorder is lost on every reset, and with the camera fitted
 * the drone browns out on a partly charged battery. These summaries (and a
 * short trace of the last seconds before a reset) live in RTC memory, which
 * survives a brownout, so each flight can still be judged afterwards: was the
 * camera hold engaged, how steady was the attitude, how hard did each motor
 * work, and how did the flight end. Pure code, unit-tested on the host.
 */
#ifndef FLIGHT_SUMMARY_H_
#define FLIGHT_SUMMARY_H_

#include <stdbool.h>
#include <stdint.h>

#define FSUM_AIRBORNE_THRUST  30000   /* same lift-off threshold as the camera hold */
#define FSUM_SETTLE_SAMPLES   100     /* skip 1 s of lift-off before the stability sums */
#define FSUM_LINK_LOST_MS     400     /* thrust gone with no packet for this long = link */

enum {
  FSUM_END_OPEN = 0,    /* still flying (or the reset interrupted it, see resetReason) */
  FSUM_END_ZERO = 1,    /* the app sent zero thrust (stick released) */
  FSUM_END_LINK = 2,    /* thrust dropped while the link was silent (lock / failsafe) */
  FSUM_END_RESET = 3,   /* the drone reset during the flight; resetReason says why */
  FSUM_END_AUTO = 4,    /* autonomy flew it and its thrust went to 0 (touchdown, cut, abort); autoReason says why */
};

/* One 100 Hz sample, in the flight recorder's units. */
typedef struct {
  uint16_t thrust;
  uint16_t vbatMv;
  int16_t roll, pitch;          /* 0.01 deg */
  int16_t gx, gy;               /* 0.1 deg/s */
  uint16_t motor[4];            /* 0..65535 */
  bool holdEngaged;             /* camera correction applied (engaged or coasting) */
  bool holdCoasting;            /* applied from the learned integral, view lost */
  bool flowOk;
  bool thrustLocked;
  int16_t holdRoll, holdPitch;  /* applied camera correction, 0.01 deg */
  int16_t rollRateI, pitchRateI;
  uint16_t linkAgeMs;
} fsumSample_t;

/* The camera height observer in shadow mode (cam_vz_shadow.h) over the
 * observer's sortie that started in this flight (all 0, flags 0, if none did:
 * no ground start, e.g. a hand launch). The camera wake is the first frame the
 * observer used (1.5 s or more after throttle-up, not before lift-off). Offsets
 * are against 1 g; the in-flight offset against the ground baseline is
 * bias - g0. Set by the caller (flight_rtc_log), not by fsumAdd(). */
#define FSUM_CVZ_NONE      INT16_MIN   /* int16 field not reached (no lift-off, landed before, no touchdown) */
typedef struct {
  uint8_t flags;          /* FSUM_CVZ_* */
  uint8_t firstOkDs;      /* camera wake to the first healthy step, 0.1 s (255: never, or 25.4 s or later) */
  int16_t g0Dmg;          /* ground baseline g0 - 1 g at lift-off, 0.1 mg */
  int16_t bias3Cmg;       /* observer offset at wake + 3 s, 0.01 mg (FSUM_CVZ_NONE: not airborne then) */
  int16_t bias5Cmg;       /* at wake + 5 s */
  int16_t biasTdCmg;      /* at touchdown (the latest airborne value until then) */
  uint16_t sigB5Cmg;      /* its sigma at wake + 5 s, 0.01 mg (0: not reached) */
  uint16_t gainDpm;       /* latest tracked camera gain G = f / D, 0.1 px/m (0: never tracked) */
  int16_t hTdMm;          /* height estimate at touchdown, mm: the drift (the drone is back on the floor) */
  int16_t vTdMms;         /* vertical speed estimate at touchdown, mm/s */
  int16_t hMaxMm;         /* highest airborne height estimate, mm */
  uint16_t airCs;         /* lift-off to touchdown (all hops), 10 ms */
  uint16_t camAirCs;      /* airborne after the camera wake, 10 ms */
  uint16_t okCs;          /* of which the observer was healthy */
  uint16_t gapMaxCs;      /* longest airborne stretch without an accepted camera update, after the first, 10 ms */
  uint16_t accepted;      /* camera updates accepted (saturating) */
  uint16_t rejected;      /* refused by the innovation gate or the CUSUM hold (saturating) */
  uint8_t alarms;         /* CUSUM alarms (saturating) */
  uint8_t yawResets;      /* gain-tracker restarts on an accumulated yaw (saturating) */
} fsumCamVz_t;

#define FSUM_CVZ_RAN       0x01   /* a sortie started (ground start) in this flight */
#define FSUM_CVZ_LIFT      0x02   /* lift-off detected */
#define FSUM_CVZ_LIFT_ACC  0x04   /* ... by the accelerometer (else by the thrust level or the external hint) */
#define FSUM_CVZ_TD        0x08   /* touchdown: the motors stopped after lift-off */
#define FSUM_CVZ_IMPACT    0x10   /* ... the touchdown values are from an impact before the stop */
#define FSUM_CVZ_GAIN      0x20   /* the gain tracker had a G */
#define FSUM_CVZ_PRIOR     0x40   /* the offset prior came from g0 (else the observer's vague prior) */
#define FSUM_CVZ_FAULT     0x80   /* the observer faulted (non-finite input or state) */

/* The camera height observer's late offset noise (cam_vz_obs.h "Late offset
 * noise", docs/autonomy-a2b.md 25), its offset drift rate T2 ("Offset
 * drift rate", 29) and its healthy offset speed-up ("Healthy offset
 * speed-up", 37: two flag bits, no field) over the same sortie as fsumCamVz_t
 * (all 0 if none started in this flight): which version flew. Set by the
 * caller. */
typedef struct {
  uint8_t flags;          /* FSUM_CVL_* */
  uint8_t qLate;          /* the multiplier in use (cam_vz_obs.h qLateMult; 1: off, the filter of before) */
  uint8_t qLateS;         /* its step, s after the sortie's first lift-off */
  uint8_t guards;         /* times a guard event or the ground stopped it (saturating) */
  uint16_t lateDs;        /* 0.1 s with the multiplier applied (saturating) */
  /* T2 (2026-10-16, appended): */
  uint8_t t2K;            /* its kR x 100 in use (cam_vz_obs.h t2kR; 0: off, the filter of before) */
  int8_t t2BrEnd;         /* bRate at the latest frame, 0.01 mg/s (0 before t2StartS) */
  int8_t t2BrMin, t2BrMax;/* its range since t2StartS, 0.01 mg/s */
  uint16_t t2OnDs;        /* 0.1 s with the drift applied (in the air, the camera accepting; saturating) */
  int16_t t2PriorUgs;     /* its prior in use, ug/s */
} fsumCvLate_t;

#define FSUM_CVL_RAN       0x01   /* a sortie started in this flight (the fields are set) */
#define FSUM_CVL_STEP      0x02   /* qLateS was reached in the air (qLate > 1) */
#define FSUM_CVL_CFG_BAD   0x04   /* the stored cvqlate / cvqlates was refused at boot: qLate 1 flew (cam_vz_glue.c) */
#define FSUM_CVL_T2_ON     0x08   /* T2's t2StartS was reached in the air (t2K > 0) */
#define FSUM_CVL_T2_CFG_BAD 0x10  /* the stored cvt2k / cvt2r was refused at boot, or came with cvqlate > 1: T2 off */
#define FSUM_CVL_HB_ON     0x20   /* the healthy offset speed-up was in use (hbMult > 0; cam_vz_obs.h, a2b 37) */
#define FSUM_CVL_HB_CFG_BAD 0x40  /* the stored cvhb was refused at boot (out of range, 1..9, unreadable, or with
                                   * cvqlate > 1 / cvt2k > 0): the speed-up off */

typedef struct {
  uint16_t boot;          /* crash-log boot counter, low 16 bits */
  uint8_t flightId;       /* the recorder's flight id in that boot */
  uint8_t end;            /* FSUM_END_* */
  uint8_t resetReason;    /* esp_reset_reason_t when end == FSUM_END_RESET */
  uint8_t autoReason;     /* FSUM_END_AUTO: autoReason_t of the state machine when the thrust went to 0 (the LAND or
                           * KILLED trigger, e.g. HOVER_TIMEOUT, APP_INPUT, TILT); 0 otherwise. Was 'pad' (always 0). */
  uint16_t coastCs;       /* samples with the hold coasting (part of holdCs) */
  int16_t gyroDrift[2];   /* gyro x/y zero drift since boot at take-off, 0.01 deg/s (set by the caller) */
  uint16_t thrustCs;      /* 10 ms samples with thrust > 0 */
  uint16_t airCs;         /* samples above FSUM_AIRBORNE_THRUST */
  uint16_t steadyCs;      /* airborne samples after the settle time (stability sums) */
  uint16_t holdCs;        /* samples with the camera correction applied */
  uint16_t camCs;         /* airborne samples with a fresh camera fit */
  uint16_t thrustMax;
  uint16_t vbatStartMv;
  uint16_t vbatMinMv;
  uint16_t tiltMaxDd;     /* max |roll| or |pitch| after settle, 0.1 deg */
  uint16_t gyroMaxDps;    /* max |gx| or |gy| after settle, deg/s */
  uint32_t tiltSq;        /* sum of roll^2 + pitch^2, (0.1 deg)^2, saturating */
  uint32_t gyroSq;        /* sum of gx^2 + gy^2, (deg/s)^2, saturating */
  uint32_t motorSum[4];   /* sum of motor/256 over airborne samples */
  int32_t holdRollSum;    /* sum of camera correction over engaged samples, 0.01 deg */
  int32_t holdPitchSum;
  int32_t rollRateISum;   /* sum over airborne samples, actuator units */
  int32_t pitchRateISum;
  /* Set by the caller (flight_rtc_log), not by fsumAdd(): */
  uint16_t uptimeS;       /* seconds since boot at the first thrust */
  uint16_t sinceJoinDs;   /* 0.1 s since the phone last joined the AP, at first thrust (FSUM_AGE_NONE: never) */
  uint16_t sinceAppDs;    /* 0.1 s since the app's first packet after that join (FSUM_AGE_NONE: none) */
  uint16_t railMinMv;     /* lowest 1 kHz XIAO 3.3 V rail reading during the flight (0: no meter) */
  uint16_t vbatFastMinMv; /* lowest 1 kHz battery reading during the flight */
  uint16_t txPackets;     /* UDP datagrams sent to the app during the flight */
  uint16_t rxPackets;     /* valid app datagrams received during the flight */
  uint16_t loopGapMaxUs;  /* longest stabilizer loop gap during the flight, us (65535 = longer) */
  uint16_t loopGaps2500;  /* stabilizer loop gaps > 2.5 ms during the flight */
  uint8_t pwmKhz;         /* motor PWM frequency at first thrust, kHz (0: unknown) */
  uint8_t motorFlags;     /* FSUM_MOTOR_* at first thrust */
  int16_t atrimRoll;      /* attitude trim flown with, 0.01 deg (set at first thrust) */
  int16_t atrimPitch;
  fsumCamVz_t camVz;      /* camera height observer, shadow mode (set by the caller) */
  /* The accelerometer's heat drift against the camera's ground standby (docs/autonomy-a2b.md 17.1), set by the
   * caller: */
  int8_t imuT0Hc;         /* IMU temperature at the first thrust, 0.5 C (FSUM_TEMP_NONE: not read) */
  int8_t imuT1Hc;         /* at the end of the flight (the latest airborne sample until then), 0.5 C */
  uint16_t camAwakeDs;    /* 0.1 s the camera had been awake since its latest ground wake, at the first thrust
                           * (0: asleep; FSUM_AGE_NONE: no ground standby this boot) */
  /* The gyro's thermal drift (docs/autonomy-a2b.md 19.1-19.3), set by the caller at the latest airborne sample: */
  int16_t gtcYEndMdps;    /* the gyro temperature compensation on y (gyro_tempco.h), mdps */
  int16_t mkiEndMdps[2];  /* the gyro bias the attitude filter learned in flight (sensfusion6.h), x / y, mdps */
  /* The first ground re-zero after this flight (the driver's airborne re-zero, gyro_tempco.h; ~2-3 s after the
   * motors stop, once the drone rests), set by the caller (fsumSetGtcResidual) when it comes; gtcResFlags 0 until
   * then. The post-flight check of the compensation (docs/autonomy-bench.md I5): */
  int16_t gtcResMdps[2];  /* r = step - corr, x / y: the still-gyro error the compensation left, mdps */
  int16_t gtcResDtCc;     /* its dTeff = min(T, knee) - min(Tref, knee), 0.01 C (negative when cooling) */
  uint8_t gtcResLandDs;   /* from the last airborne sample to that re-zero, 0.1 s (255: 25.5 s or later) */
  uint8_t gtcResFlags;    /* FSUM_GTCR_* */
  fsumCvLate_t camVzLate; /* the observer's late offset noise (2026-10-15, set by the caller with camVz) */
  /* The accelerometer compensation's check at the same post-flight re-zero (accel_tempco.h, docs/autonomy-a2b.md
   * 36.2), set by the caller (fsumSetAtcResidual); atcResFlags 0 until it comes: */
  int16_t atcStepZCmg;    /* the still z accelerometer after the flight minus its reference before it, 0.01 mg */
  int16_t atcCorrZCmg;    /* the model's correction at that moment (also when off), 0.01 mg; r = step - corr */
  int16_t atcResDtCc;     /* its dTeff, 0.01 C (negative when cooling) */
  uint8_t atcResFlags;    /* FSUM_GTCR_* (the same bits: set, on) */
  uint8_t atcPad;         /* 0 */
} flightSummary_t;

#define FSUM_TEMP_NONE INT8_MIN

#define FSUM_GTCR_SET 0x01   /* the re-zero came */
#define FSUM_GTCR_ON  0x02   /* the compensation was on */

#define FSUM_MOTOR_SOFTSTART 0x01
#define FSUM_MOTOR_CAMSB     0x02   /* camera standby armed (enabled, camera running, sensor supports it) */
#define FSUM_MOTOR_TXLO      0x04   /* low Wi-Fi TX power during the take-off window enabled */
#define FSUM_MOTOR_WIFIQ     0x08   /* Wi-Fi without 11b rates, 300 TU beacons (this boot) */
#define FSUM_MOTOR_CAMSB_DID 0x10   /* the camera was in standby during this flight */
#define FSUM_MOTOR_TXLO_DID  0x20   /* the Wi-Fi TX power was low during this flight */
#define FSUM_MOTOR_COLD      0x40   /* cold take-off (first after boot or a break) */
#define FSUM_MOTOR_COOL      0x80   /* a ground cooling spin, not a flight (ground_cool.h, docs/autonomy-a2b.md 36.3) */

#define FSUM_AGE_NONE 0xFFFF

/* Trace sample for the last seconds before a reset (25 Hz). */
typedef struct __attribute__((packed)) {
  uint16_t thrust;
  uint16_t vbatMv;
  uint8_t motor[4];       /* motor/256 */
  int8_t roll, pitch;     /* 0.5 deg */
  uint8_t gyroDps;        /* max(|gx|, |gy|), capped at 255 */
  uint8_t flags;          /* FSUM_CRUMB_* */
  uint8_t linkAge20ms;    /* link age in 20 ms units, capped at 255 */
  /* Set by the caller (flight_rtc_log), fsumCrumb() zeroes them: */
  uint8_t rail10mv;       /* lowest rail reading in the last 40 ms, 10 mV above 2.00 V (0: no reading,
                           * 1: <= 2.01 V, 255: >= 4.55 V incl. meter saturation) */
  uint8_t txPk;           /* UDP datagrams sent in the last 40 ms, capped at 255 */
  uint8_t rxPk;           /* app datagrams received in the last 40 ms, capped at 255 */
} fsumCrumb_t;

/* Rail encoding helpers for fsumCrumb_t.rail10mv. */
uint8_t fsumRailCode(uint16_t mv);

#define FSUM_CRUMB_HOLD    0x01
#define FSUM_CRUMB_FLOW    0x02
#define FSUM_CRUMB_LOCKED  0x04
#define FSUM_CRUMB_AIR     0x08
/* Set by the caller (flight_rtc_log): */
#define FSUM_CRUMB_CAMSB   0x10     /* camera in standby */
#define FSUM_CRUMB_TXLO    0x20     /* Wi-Fi TX power lowered */

void fsumStart(flightSummary_t *fs, uint16_t boot, uint8_t flightId, uint16_t vbatMv);
void fsumAdd(flightSummary_t *fs, const fsumSample_t *s);
/* End cause for a sample whose thrust is zero (the flight just ended). */
uint8_t fsumEndCause(const fsumSample_t *s);
void fsumCrumb(fsumCrumb_t *c, const fsumSample_t *s);
/* The first ground re-zero after the flight: r x / y (mdps), dTeff (0.01 C), the time from the last airborne
 * sample (ms), the compensation on; saturating. */
void fsumSetGtcResidual(flightSummary_t *fs, int32_t rxMdps, int32_t ryMdps, int32_t dTeffCc, uint32_t landMs,
                        bool on);
/* The accelerometer's check at that re-zero: the still z step and the model's correction (0.01 mg), dTeff
 * (0.01 C), the compensation on; saturating. */
void fsumSetAtcResidual(flightSummary_t *fs, int32_t stepZCmg, int32_t corrZCmg, int32_t dTeffCc, bool on);

#endif /* FLIGHT_SUMMARY_H_ */
