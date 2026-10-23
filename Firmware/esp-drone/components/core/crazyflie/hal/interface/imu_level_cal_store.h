/*
 * ESP-Drone Firmware
 *
 * imu_level_cal_store.h - Persistent storage of the accelerometer level
 * calibration and the torque trim, each a separate record in their own NVS
 * namespace ("imucal"). Unrelated NVS data (e.g. Wi-Fi settings) is never
 * touched.
 *
 * These functions access flash and must not be called from the sensor or
 * stabilizer tasks.
 */
#ifndef IMU_LEVEL_CAL_STORE_H_
#define IMU_LEVEL_CAL_STORE_H_

#include "esp_err.h"
#include "nvs.h"
#include "imu_level_cal.h"
#include "torque_trim.h"
#include "gyro_tempco.h"
#include "accel_tempco.h"

/* ESP_OK and a validated record; ESP_ERR_NVS_NOT_FOUND if none is stored;
 * ESP_ERR_INVALID_CRC / ESP_ERR_INVALID_VERSION / ESP_ERR_INVALID_SIZE if a
 * stored record is rejected (it is then never applied). */
esp_err_t imuLevelCalStoreLoad(imuLevelCalRecord_t *rec);
esp_err_t torqueTrimStoreLoad(torqueTrimRecord_t *rec);

/* Writes, commits and reads back the record. Returns ESP_OK only if the
 * read-back record matches. */
esp_err_t imuLevelCalStoreSave(const imuLevelCalRecord_t *rec);
esp_err_t torqueTrimStoreSave(const torqueTrimRecord_t *rec);

/* Removes only that record. ESP_OK if it is gone. */
esp_err_t imuLevelCalStoreErase(void);
esp_err_t torqueTrimStoreErase(void);

/* In-flight accelerometer offset (same record format as the level
 * calibration, separate key "facc"). */
esp_err_t flightAccStoreLoad(imuLevelCalRecord_t *rec);
esp_err_t flightAccStoreSave(const imuLevelCalRecord_t *rec);
esp_err_t flightAccStoreErase(void);

/* Yaw-lock flag (a single byte, 0 or 1). Load returns ESP_ERR_NVS_NOT_FOUND
 * if it was never saved; Save writes, commits and reads back. */
esp_err_t yawLockStoreLoad(bool *on);
esp_err_t yawLockStoreSave(bool on);

/* Camera on/off (default off: the camera driver is not started at all). */
/* Ground-test thrust cap (diagnostic), 0 = off; save(0) erases the key. */
esp_err_t thrustCapStoreLoad(uint16_t *cap);
esp_err_t thrustCapStoreSave(uint16_t cap);

/* Motor PWM frequency in Hz (save(0) erases: stock) and the spin-up ramp switch. */
esp_err_t motorPwmStoreLoad(uint32_t *hz);
esp_err_t motorPwmStoreSave(uint32_t hz);
/* Restart lock switch (default on). */
esp_err_t restartLockStoreLoad(bool *on);
esp_err_t restartLockStoreSave(bool on);
esp_err_t softStartStoreLoad(bool *on);
esp_err_t softStartStoreSave(bool on);
/* Motor watchdog switch "mguard" (default on, motor_guard.h). */
esp_err_t motorGuardStoreLoad(bool *on);
esp_err_t motorGuardStoreSave(bool on);

/* Take-off load switches: camera standby and low Wi-Fi TX power during the
 * take-off window (default on), Wi-Fi without 11b rates and with 300 TU
 * beacons (default off, applied at boot). */
esp_err_t camStandbyStoreLoad(bool *on);
esp_err_t camStandbyStoreSave(bool on);
/* Camera standby length after a ground start, ms (warm, cold take-off). */
esp_err_t camStandbyMsStoreLoad(uint16_t *ms);
esp_err_t camStandbyMsStoreSave(uint16_t ms);
esp_err_t camStandbyColdMsStoreLoad(uint16_t *ms);
esp_err_t camStandbyColdMsStoreSave(uint16_t ms);
/* Camera ground standby (camera_flow.h, docs/autonomy-a2b.md 17.1): the switch
 * "camgnd" (default on), the hold after a wake reason "camgndms" (ms,
 * 5000..65000, default 45000) and the thermal lead "camlead" (ms, 0..60000,
 * default 0). */
esp_err_t camGroundStoreLoad(bool *on);
esp_err_t camGroundStoreSave(bool on);
esp_err_t camGroundMsStoreLoad(uint16_t *ms);
esp_err_t camGroundMsStoreSave(uint16_t ms);
esp_err_t camLeadMsStoreLoad(uint16_t *ms);
esp_err_t camLeadMsStoreSave(uint16_t ms);
/* The camera's input clock "camclk" (docs/autonomy-a2b.md 17.2): 1 (true) =
 * 13.33 MHz, 14.81 fps (the default when absent), 0 = 20 MHz, 22.21 fps; read
 * at boot. */
esp_err_t camClockStoreLoad(bool *slow);
esp_err_t camClockStoreSave(bool slow);
esp_err_t txLowStoreLoad(bool *on);
esp_err_t txLowStoreSave(bool on);
esp_err_t wifiQuietStoreLoad(bool *on);
esp_err_t wifiQuietStoreSave(bool on);

esp_err_t camEnableStoreLoad(bool *on);
esp_err_t camEnableStoreSave(bool on);

/* Camera drift-hold switch (single byte), same contract as the yaw lock. */
esp_err_t camHoldStoreLoad(bool *on);
esp_err_t camHoldStoreSave(bool on);

/* Attitude trim record (torque_trim.h), same contract as the others. */
esp_err_t attTrimStoreLoad(attTrimRecord_t *rec);
esp_err_t attTrimStoreSave(const attTrimRecord_t *rec);
esp_err_t attTrimStoreErase(void);
/* The flow fit's variant "flowfit" (optical_flow.h ofFitMode_t, one byte 0..2; camera_flow.h). */
esp_err_t flowFitStoreLoad(uint8_t *mode);
esp_err_t flowFitStoreSave(uint8_t mode);
/* Gyro temperature compensation record "gyrotc" (gyro_tempco.h), same contract as the attitude trim. */
esp_err_t gyroTcStoreLoad(gyroTcRecord_t *rec);
esp_err_t gyroTcStoreSave(const gyroTcRecord_t *rec);
esp_err_t gyroTcStoreErase(void);
/* Accelerometer temperature compensation record "acctc" (accel_tempco.h), the same contract. */
esp_err_t accTcStoreLoad(accTcRecord_t *rec);
esp_err_t accTcStoreSave(const accTcRecord_t *rec);
esp_err_t accTcStoreErase(void);
/* The attitude filter's in-flight bias learning gain "mkiair" (sensfusion6.h twoKiAir, a float, 0..0.2). Load:
 * ESP_ERR_INVALID_SIZE / ESP_ERR_INVALID_ARG for a blob of another size or a value out of range. */
esp_err_t mahonyKiAirStoreLoad(float *twoKiAir);
esp_err_t mahonyKiAirStoreSave(float twoKiAir);
esp_err_t mahonyKiAirStoreErase(void);
/* The camera hold's largest gain scale "holdkmax" (camera_hold.h, docs/autonomy-a2b.md 20; a float, 1.0..3.0;
 * absent: CAM_HOLD_GAIN_KMAX_DEFAULT). Load: ESP_ERR_INVALID_SIZE / ESP_ERR_INVALID_ARG for a blob of another size or a
 * value out of range. */
esp_err_t camHoldKmaxStoreLoad(float *kmax);
esp_err_t camHoldKmaxStoreSave(float kmax);
esp_err_t camHoldKmaxStoreErase(void);
/* Learning the attitude trim from the camera hold (default on). */
esp_err_t attTrimLearnStoreLoad(bool *on);
esp_err_t attTrimLearnStoreSave(bool on);

/* Autonomy-link key (docs/autonomy-link.md): 16 bytes, key "autokey". Load
 * returns ESP_ERR_NVS_NOT_FOUND if none is stored and ESP_ERR_INVALID_SIZE if
 * the stored blob is not 16 bytes; Save writes, commits and reads back. */
#define AUTO_KEY_STORE_LEN 16
esp_err_t autoKeyStoreLoad(uint8_t key[AUTO_KEY_STORE_LEN]);
esp_err_t autoKeyStoreSave(const uint8_t key[AUTO_KEY_STORE_LEN]);
esp_err_t autoKeyStoreErase(void);

/* Autonomy settings (docs/autonomy-a2.md 8.5; the key names, NVS types and
 * ranges are auto_glue_core.h's table, autoSettingId_t). One key each in
 * "imucal". nvsType is autoNvsType_t: 1 u8, 2 u16, 3 i16, 4 u32; the value
 * is converted to and from int32_t. Load: ESP_ERR_NVS_NOT_FOUND if absent,
 * also when stored with another type only (IDF 5.0.7 lookups are typed:
 * Storage::findItem turns the page's TYPE_MISMATCH into NOT_FOUND; the glue
 * then tries the key's legacyType, auto_glue_core.h). Save writes, commits
 * and reads back (ESP_FAIL on a mismatch); it adds an item of its type and
 * leaves an item of another type alone, so a key with a legacyType is erased
 * first. Erase removes every item of the key, whatever its type, in one
 * commit; an absent key is ESP_OK. Flash access: never from the stabilizer or sensor tasks, never
 * while autonomy is active (autonomy.c and usb_diag.c enforce it). */
esp_err_t autoSettingStoreLoad(const char *key, uint8_t nvsType, int32_t *value);
esp_err_t autoSettingStoreSave(const char *key, uint8_t nvsType, int32_t value);
esp_err_t autoSettingStoreErase(const char *key);

#endif /* IMU_LEVEL_CAL_STORE_H_ */
