/*
 * ESP-Drone Firmware
 *
 * flight_recorder.h - RAM flight recorder and latest-sample snapshot
 *
 * The stabilizer calls flightRecorderSample() at 100 Hz. Each call only
 * packs one fixed-size record; formatting and USB output happen in the
 * low-priority USB diagnostics task.
 *
 * Records are kept while thrust is requested or autonomy is active (a DRY
 * sortie has no thrust) and for 1 s after. The ring keeps the most recent records (180 s in PSRAM, 120 s if
 * that would leave PSRAM under 1 MB, or 15 s in internal RAM if PSRAM is unavailable), is lost on power-off and
 * survives plugging in USB while the battery stays connected. Since 2026-10-14 (docs/autonomy-a2b.md 19.3) a
 * crash flight is no longer overwritten by the next flight's 60 s; the rule stays: dump after any PALM,
 * COLLISION or KILLED flight before the next take-off.
 */
#ifndef FLIGHT_RECORDER_H_
#define FLIGHT_RECORDER_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

#define FLIGHT_RECORDER_RATE_HZ           100
#define FLIGHT_RECORDER_SAMPLES_PSRAM     18000 /* 180 s at 100 Hz, 2.27 MB (126-byte records) */
#define FLIGHT_RECORDER_SAMPLES_PSRAM_LOW 12000 /* 120 s, if 180 s would leave less than ... */
#define FLIGHT_RECORDER_PSRAM_KEEP        (1024u * 1024u)   /* ... this much PSRAM free */
#define FLIGHT_RECORDER_SAMPLES_INTERNAL  1500  /* 15 s at 100 Hz, 189 KB */

#define FLIGHT_REC_FLAG_ARMED     0x01
#define FLIGHT_REC_FLAG_INHIBITED 0x02
#define FLIGHT_REC_FLAG_ANGLE     0x04  /* roll/pitch in angle (self-level) mode */

/* Units are chosen so int16 holds the useful range. */
typedef struct __attribute__((packed)) {
  uint32_t tickMs;      /* FreeRTOS tick, 1 ms */
  uint8_t flightId;     /* increments when recording starts (thrust or autonomy after >= 1 s of neither) */
  uint8_t flags;        /* FLIGHT_REC_FLAG_* */
  uint16_t thrust;      /* setpoint thrust, 0..65535 */
  int16_t rollDes;      /* setpoint attitude, 0.01 deg */
  int16_t pitchDes;
  int16_t roll;         /* estimated attitude, 0.01 deg */
  int16_t pitch;
  int16_t yaw;
  int16_t gyro[3];      /* 0.1 deg/s, filtered, body frame */
  int16_t acc[3];       /* 0.001 g, filtered, after level offset */
  uint16_t motor[4];    /* PWM ratio read back from the hardware, 0..65535 */
  int16_t rollI;        /* attitude PID integral output, 0.01 deg/s */
  int16_t pitchI;
  int16_t rollRateI;    /* rate PID integral output, actuator units */
  int16_t pitchRateI;
  int16_t yawRateDes;   /* setpoint yaw rate, 0.1 deg/s */
  uint16_t vbatMv;      /* battery voltage, mV */
  /* Latest camera-flow sample (camera_flow.h); repeats until a new frame. */
  uint8_t flowSeq;      /* frame counter 1..255, 0 = no sample yet */
  uint8_t flowOk;       /* 1 = fit ok and gyro interval available */
  uint8_t flowLum;      /* mean image brightness */
  uint8_t flowTexture;  /* texture measure, capped at 255 */
  uint8_t flowResidual; /* fit residual, 0.1 grey levels, capped at 255 */
  uint16_t flowDt;      /* frame interval, 0.1 ms */
  uint16_t flowAgeMs;   /* sample time minus frame capture time, ms */
  int16_t flowDx;       /* scene shift, 0.01 px at 160x120, +right */
  int16_t flowDy;       /* 0.01 px, +down */
  int16_t flowScale;    /* relative scale change, 1e-4 */
  int16_t flowRot;      /* image rotation, 1e-4 rad */
  int16_t flowGyro[3];  /* body rotation over the frame interval, 1e-4 rad */
  uint8_t holdFlags;    /* bit0 drift hold enabled, bit1 applied, bit2 coasting (view lost), bit3 the latest
                         * frame refused by the HOVER scene gate (camera_hold.h camHoldSceneOk) */
  int16_t holdRoll;     /* camera correction applied to the setpoint, 0.01 deg */
  int16_t holdPitch;
  uint16_t rawThrust;   /* thrust the app last sent, before the safety lock */
  uint16_t linkAgeMs;   /* time since the last valid setpoint packet, ms */
  uint8_t linkFlags;    /* bit0 thrust locked */
  /* Camera height observer in shadow mode (cam_vz_shadow.h camVzShadowRec_t). */
  int16_t cvH;          /* height estimate since lift-off, mm (0 on the ground) */
  int16_t cvV;          /* vertical speed estimate, mm/s, up */
  int16_t cvBias;       /* accelerometer world-up offset vs 1 g, 0.01 mg */
  uint16_t cvGain;      /* camera gain G = f / D the next update would use, 0.1 px/m (0: none) */
  uint8_t cvFlags;      /* bit0 airborne, bit1 healthy, bit2 gain tracked, bit3 fault,
                         * bits 4-7 the latest new frame's camVzObsDecision_t */
  uint8_t cvSigV;       /* sigma of the speed estimate, mm/s, capped at 255 */
  /* Autonomy (autonomy.h autoRecFields_t, A2b): 0 in IDLE outside a sortie. */
  uint8_t autoState;    /* autoState_t */
  uint8_t autoReason;   /* autoReason_t of the latest transition */
  uint8_t vzPhase;      /* vzPhase_t */
  int16_t vzHmm;        /* vertical controller height estimate, mm */
  int16_t vzVmms;       /* its vertical speed estimate, mm/s, up */
  uint16_t vzThrust;    /* its command, before the mode clamp (DRY: what the sim flies) */
  uint8_t autoFlags;    /* AUTO_STF_* (auto_proto.h) */
  /* Interactive hover (autonomy.h autoRecFields_t): the motion's references, 0 without one. */
  uint8_t mvState;      /* autoMvState_t | autoMvKind_t << 4 (auto_motion.h) */
  int16_t mvRefLat;     /* the hold's lateral reference, x / D, 1e-4 */
  int16_t mvRefFwd;     /* its forward reference, ln(D0 / D), 1e-4 */
  int16_t mvVRef;       /* the vertical speed reference, mm/s */
  int16_t mvYawOff;     /* the heading offset, 0.01 deg, + clockwise */
  /* The thermal drift and the exposure (docs/autonomy-a2b.md 19.3; 112 -> 124 bytes, 2026-10-14). */
  int16_t imuTemp;      /* IMU die temperature, 0.01 C */
  int16_t gtcCorrX;     /* the gyro temperature compensation subtracted (gyro_tempco.h), mdps, body x */
  int16_t gtcCorrY;     /* ... y */
  int16_t mkiBiasX;     /* the gyro bias the attitude filter has learned (sensfusion6.h), mdps, body x */
  int16_t mkiBiasY;     /* ... y */
  int8_t flowGain;      /* the flow fit's exposure gain, (a - 1) x 1000, +-127 (camera_flow.h gainE3) */
  uint8_t flowSat;      /* saturated % of the fit's pixels, bit 7: the frame was refused for its exposure step */
  /* The accelerometer temperature compensation (docs/autonomy-a2b.md 36; 124 -> 126 bytes, 2026-10-18). */
  int16_t atcCorrZ;     /* the correction subtracted from the body z accelerometer (accel_tempco.h), 0.01 mg */
} flightRecord_t;

void flightRecorderInit(void);

/* Called by the stabilizer task at FLIGHT_RECORDER_RATE_HZ. */
void flightRecorderSample(const setpoint_t *setpoint, const state_t *state,
                          const sensorData_t *sensors, uint32_t tick);

/* Latest sample (recorded or not). False until the stabilizer has run. */
bool flightRecorderGetLatest(flightRecord_t *out);

/* Ring access for the dump. Freeze stops recording (and waits for an
 * in-progress write) so flightRecorderGet() reads a stable ring. */
void flightRecorderFreeze(bool freeze);
uint32_t flightRecorderCount(void);
uint32_t flightRecorderCapacity(void);
bool flightRecorderGet(uint32_t index, flightRecord_t *out); /* 0 = oldest; only while frozen */
void flightRecorderClear(void);

#endif /* FLIGHT_RECORDER_H_ */
