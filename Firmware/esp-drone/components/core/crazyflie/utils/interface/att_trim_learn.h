/*
 * ESP-Drone Firmware
 *
 * att_trim_learn.h - Learn the attitude trim from the camera drift hold
 *
 * The gyro and accelerometer cannot see a lean of a few tenths of a degree
 * from true level, which makes the drone slide; the camera drift hold sees
 * the slide and leans against it (camera_hold.h). Over 14 flights on
 * 2026-10-11 its correction averaged +0.32 deg pitch (sd 0.22) and +0.14 deg
 * roll (sd 0.25). The attitude trim is added to the setpoint in the same way
 * (controller_pid.c), so moving the hold's steady correction into the trim
 * lets the drone fly straight while the camera is off (take-off standby, lost
 * texture) and leaves the hold only the flight-to-flight variation.
 *
 * Per flight, the samples with the hold applied on a fresh view (not
 * coasting), sticks centred, airborne and in steady hover (small tilt and
 * rotation) are averaged, but only once the hold has been applied with the
 * sticks centred, airborne and steady for ATT_LEARN_SETTLE_S without a break:
 * while it brakes velocity it did not cause (a stick release, the lift-off
 * kick, a bump) its mean equals trim error plus (v_start - v_end) / (g T),
 * which would be learned as trim. A coasting dropout (view lost briefly) does
 * not restart the settle time, it only skips samples. A flight with at least
 * ATT_LEARN_MIN_S of counted samples moves ATT_LEARN_GAIN of its mean
 * correction into the trim, at most ATT_LEARN_MAX_STEP_DEG per axis, within
 * +/-ATT_LEARN_LIMIT_DEG (a larger hand-set trim is only ever moved toward
 * zero); an axis whose flight mean exceeds ATT_LEARN_MAX_MEAN_DEG (a draft, a
 * one-off) is left alone. With gain 0.3 the trim settles within a handful of
 * flights and averages out the flight-to-flight scatter (to ~0.1 deg). Pure
 * code, host-tested by tools/tests/test_att_trim_learn.c.
 *
 * Guards (2026-10-13: the learner moved the trim from 0.26/0.78 to
 * 0.34/0.63 deg roll/pitch over two failed interactive-hover sorties, one
 * with a person drawing the hold and one landing HOVER_NO_CAM on a
 * heat-soaked IMU): samples during an interactive-hover motion (its
 * feedforward lean and references are in the hold's correction) restart the
 * settle like a stick input; a correction at the hold's limit
 * (ATT_LEARN_SAT_DEG, the hold chasing something it cannot hold) is
 * skipped; and an autonomy flight is learned only when it ended in a normal
 * LAND and the state machine reported nothing on its own in it
 * (attLearnEndOk). Manual flights are learned as before.
 */
#ifndef ATT_TRIM_LEARN_H_
#define ATT_TRIM_LEARN_H_

#include <stdbool.h>
#include <stdint.h>

#define ATT_LEARN_MIN_S         5.0f    /* counted samples per flight, seconds */
#define ATT_LEARN_SETTLE_S      3.0f    /* steady hold before samples count */
#define ATT_LEARN_GAIN          0.3f
#define ATT_LEARN_MAX_STEP_DEG  0.15f
#define ATT_LEARN_LIMIT_DEG     0.8f    /* ~2.5x the measured need; leaves the hold its integral range */
#define ATT_LEARN_MAX_MEAN_DEG  1.0f
#define ATT_LEARN_MAX_TILT_DEG  10.0f   /* steadier than this counts as hover */
#define ATT_LEARN_MAX_GYRO_DPS  60.0f
#define ATT_LEARN_SAT_DEG       2.4f    /* |correction| at or above this: the hold at its limit (2.5 deg) */

typedef struct {
  float sumRoll, sumPitch;   /* deg */
  uint32_t n;                /* counted samples */
  uint32_t settle;           /* consecutive steady samples with the hold applied */
  uint32_t settleSamples;    /* ATT_LEARN_SETTLE_S at the sample rate */
} attLearnFlight_t;

typedef struct {
  bool holdApplied;    /* hold output applied (engaged or coasting) */
  bool holdOnView;     /* ... on a fresh camera view (engaged, not coasting) */
  bool sticksCentred;  /* pilot roll and pitch exactly zero */
  bool airborne;
  float corrRollDeg, corrPitchDeg;   /* camera correction applied to the setpoint */
  float rollDeg, pitchDeg;           /* estimated attitude */
  float gyroDps;                     /* max(|gx|, |gy|) */
  bool motionActive;                 /* an interactive-hover motion runs (RUN, PAUSED, SETTLE) */
} attLearnSample_t;

void attLearnStart(attLearnFlight_t *f, uint32_t sampleRateHz);
void attLearnAdd(attLearnFlight_t *f, const attLearnSample_t *s);

/* Whether a flight may be learned from, by how it ended: true for a flight
 * without autonomy (manual) and for an autonomy flight whose thrust-0 record
 * (stateEnd, reasonEnd: autoState_t / autoReason_t) is a normal LAND
 * (LAND_CMD, SORTIE_DONE, HOVER_TIMEOUT, BATT_LOW) and whose sortie reported
 * no event (events: the state machine's evCount, autonomySortieEvents());
 * false for every other end (KILLED, the vertical controller's own LAND, the
 * camera rule's SINK, APP_INPUT, HB_LOST, the climb's LAND COLLISION, the
 * reasons the state machine produced until 2026-10-15: HOVER_NO_CAM,
 * NOT_STABLE, MOVE_*, AIR_TIME, CAPTURE_FAIL, ...). Since 2026-10-15
 * ("Claude decides") a contact, the camera height lost, NOT_STABLE, a stopped
 * motion or a controller event no longer ends the sortie: such a flight ends
 * LAND_CMD when Claude lands it, and its events exclude it (the hold's mean
 * there may hold a lean against a wall, the height estimate off ~0.3 m after
 * a contact; 2026-10-15 verification). */
bool attLearnEndOk(bool autonomySeen, uint8_t stateEnd, uint8_t reasonEnd, uint16_t events);

/* End of a flight sampled at sampleRateHz: if it qualifies, writes its mean
 * correction to meanRoll and meanPitch, updates trimRoll and trimPitch in
 * place and returns true. */
bool attLearnFinish(const attLearnFlight_t *f, uint32_t sampleRateHz, float *trimRoll, float *trimPitch,
                    float *meanRoll, float *meanPitch);

#endif /* ATT_TRIM_LEARN_H_ */
