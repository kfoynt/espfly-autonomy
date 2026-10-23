/*
 * ESP-Drone Firmware
 *
 * imu_level_cal_store.c - NVS storage for the accelerometer level
 * calibration and the torque trim
 *
 * Flash interlock (docs/autonomy-a2b.md section 7.3): every NVS write,
 * commit and erase in the firmware is in this file, and each write path runs
 * between autonomyFlashBegin() and autonomyFlashEnd(). A write or erase stops
 * both cores (cache disabled) for up to tens of ms, so while autonomy is
 * active, or an ARM is waiting in its mailbox, it is refused with
 * ESP_ERR_INVALID_STATE without touching flash; an ARM that arrives during a
 * write is refused by the state machine (G1). Read-backs run after the
 * bracket: they only read.
 */
#include <string.h>

#include "imu_level_cal_store.h"
#include "autonomy.h"

#define IMU_CAL_NVS_NAMESPACE "imucal"
#define IMU_CAL_NVS_KEY_LEVEL "level"
#define IMU_CAL_NVS_KEY_TRIM  "ttrim"
#define IMU_CAL_NVS_KEY_YAWLOCK "yawlock"
#define IMU_CAL_NVS_KEY_ATTTRIM "atrim"
#define IMU_CAL_NVS_KEY_CAMHOLD "camhold"
#define IMU_CAL_NVS_KEY_CAMON   "camon"
#define IMU_CAL_NVS_KEY_FLIGHTACC "facc"
#define IMU_CAL_NVS_KEY_GCAP    "gcap"
#define IMU_CAL_NVS_KEY_PWMHZ   "pwmhz"
#define IMU_CAL_NVS_KEY_SOFT    "soft"
#define IMU_CAL_NVS_KEY_PLOCK   "plock"
#define IMU_CAL_NVS_KEY_CAMSB   "camsb"
#define IMU_CAL_NVS_KEY_TXLO    "txlo"
#define IMU_CAL_NVS_KEY_WIFIQ   "wifiq"
#define IMU_CAL_NVS_KEY_CAMMS   "camms"
#define IMU_CAL_NVS_KEY_CAMCOLD "camcold"
#define IMU_CAL_NVS_KEY_CAMGND  "camgnd"
#define IMU_CAL_NVS_KEY_CAMGNDMS "camgndms"
#define IMU_CAL_NVS_KEY_CAMLEAD "camlead"
#define IMU_CAL_NVS_KEY_CAMCLK  "camclk"
#define IMU_CAL_NVS_KEY_ATLEARN "atlearn"
#define IMU_CAL_NVS_KEY_AUTOKEY "autokey"
#define IMU_CAL_NVS_KEY_MGUARD  "mguard"
#define IMU_CAL_NVS_KEY_GYROTC  "gyrotc"
#define IMU_CAL_NVS_KEY_ACCTC   "acctc"
#define IMU_CAL_NVS_KEY_MKIAIR  "mkiair"
#define IMU_CAL_NVS_KEY_FLOWFIT "flowfit"
#define IMU_CAL_NVS_KEY_HOLDKMAX "holdkmax"

/* Reads a fixed-size blob. ESP_ERR_NVS_NOT_FOUND if absent,
 * ESP_ERR_INVALID_SIZE if the stored size differs. */
static esp_err_t loadBlob(const char *key, void *out, size_t size)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    // The namespace does not exist until the first save.
    return err;
  }

  size_t len = size;
  err = nvs_get_blob(handle, key, out, &len);
  nvs_close(handle);

  if (err == ESP_ERR_NVS_INVALID_LENGTH) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (err == ESP_OK && len != size) {
    return ESP_ERR_INVALID_SIZE;
  }
  return err;
}

static esp_err_t saveBlob(const char *key, const void *data, size_t size)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(handle, key, data, size);
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  return err;
}

static esp_err_t eraseKey(const char *key)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    err = ESP_OK;   /* no namespace, no key */
  } else if (err == ESP_OK) {
    err = nvs_erase_key(handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      err = ESP_OK;
    }
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  return err;
}

esp_err_t imuLevelCalStoreLoad(imuLevelCalRecord_t *rec)
{
  imuLevelCalRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_LEVEL, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (tmp.size != sizeof(tmp)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (tmp.version != IMU_LEVEL_CAL_VERSION) {
    return ESP_ERR_INVALID_VERSION;
  }
  if (!imuLevelCalRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }

  *rec = tmp;
  return ESP_OK;
}

esp_err_t imuLevelCalStoreSave(const imuLevelCalRecord_t *rec)
{
  if (!imuLevelCalRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_LEVEL, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }

  imuLevelCalRecord_t readBack;
  err = imuLevelCalStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t imuLevelCalStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_LEVEL);
}

esp_err_t flightAccStoreLoad(imuLevelCalRecord_t *rec)
{
  imuLevelCalRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_FLIGHTACC, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (!imuLevelCalRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }
  *rec = tmp;
  return ESP_OK;
}

esp_err_t flightAccStoreSave(const imuLevelCalRecord_t *rec)
{
  if (!imuLevelCalRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_FLIGHTACC, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }
  imuLevelCalRecord_t readBack;
  err = flightAccStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t flightAccStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_FLIGHTACC);
}

esp_err_t torqueTrimStoreLoad(torqueTrimRecord_t *rec)
{
  torqueTrimRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_TRIM, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (tmp.size != sizeof(tmp)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (tmp.version != TORQUE_TRIM_VERSION) {
    return ESP_ERR_INVALID_VERSION;
  }
  if (!torqueTrimRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }

  *rec = tmp;
  return ESP_OK;
}

esp_err_t torqueTrimStoreSave(const torqueTrimRecord_t *rec)
{
  if (!torqueTrimRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_TRIM, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }

  torqueTrimRecord_t readBack;
  err = torqueTrimStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t torqueTrimStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_TRIM);
}

esp_err_t attTrimStoreLoad(attTrimRecord_t *rec)
{
  attTrimRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_ATTTRIM, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (tmp.size != sizeof(tmp)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (tmp.version != ATT_TRIM_VERSION) {
    return ESP_ERR_INVALID_VERSION;
  }
  if (!attTrimRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }
  *rec = tmp;
  return ESP_OK;
}

esp_err_t attTrimStoreSave(const attTrimRecord_t *rec)
{
  if (!attTrimRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_ATTTRIM, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }
  attTrimRecord_t readBack;
  err = attTrimStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t attTrimStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_ATTTRIM);
}

esp_err_t flowFitStoreLoad(uint8_t *mode)
{
  uint8_t v = 0;
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_FLOWFIT, &v, sizeof(v));
  if (err != ESP_OK) {
    return err;
  }
  if (v > 2) {
    return ESP_ERR_INVALID_ARG;
  }
  *mode = v;
  return ESP_OK;
}

esp_err_t flowFitStoreSave(uint8_t mode)
{
  if (mode > 2) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_FLOWFIT, &mode, sizeof(mode));
  if (err != ESP_OK) {
    return err;
  }
  uint8_t readBack = 0xFF;
  err = flowFitStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return readBack == mode ? ESP_OK : ESP_FAIL;
}

esp_err_t gyroTcStoreLoad(gyroTcRecord_t *rec)
{
  gyroTcRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_GYROTC, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (tmp.size != sizeof(tmp)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (tmp.version != GYRO_TC_VERSION) {
    return ESP_ERR_INVALID_VERSION;
  }
  if (!gyroTcRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }
  *rec = tmp;
  return ESP_OK;
}

esp_err_t gyroTcStoreSave(const gyroTcRecord_t *rec)
{
  if (!gyroTcRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_GYROTC, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }
  gyroTcRecord_t readBack;
  err = gyroTcStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t gyroTcStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_GYROTC);
}

esp_err_t accTcStoreLoad(accTcRecord_t *rec)
{
  accTcRecord_t tmp;
  memset(&tmp, 0, sizeof(tmp));
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_ACCTC, &tmp, sizeof(tmp));
  if (err != ESP_OK) {
    return err;
  }
  if (tmp.size != sizeof(tmp)) {
    return ESP_ERR_INVALID_SIZE;
  }
  if (tmp.version != ACC_TC_VERSION) {
    return ESP_ERR_INVALID_VERSION;
  }
  if (!accTcRecordValid(&tmp, sizeof(tmp))) {
    return ESP_ERR_INVALID_CRC;
  }
  *rec = tmp;
  return ESP_OK;
}

esp_err_t accTcStoreSave(const accTcRecord_t *rec)
{
  if (!accTcRecordValid(rec, sizeof(*rec))) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_ACCTC, rec, sizeof(*rec));
  if (err != ESP_OK) {
    return err;
  }
  accTcRecord_t readBack;
  err = accTcStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, rec, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t accTcStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_ACCTC);
}

esp_err_t mahonyKiAirStoreLoad(float *twoKiAir)
{
  float v = 0.0f;
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_MKIAIR, &v, sizeof(v));
  if (err != ESP_OK) {
    return err;
  }
  if (!(v >= 0.0f && v <= 0.2f)) {   /* NaN fails */
    return ESP_ERR_INVALID_ARG;
  }
  *twoKiAir = v;
  return ESP_OK;
}

esp_err_t mahonyKiAirStoreSave(float twoKiAir)
{
  if (!(twoKiAir >= 0.0f && twoKiAir <= 0.2f)) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_MKIAIR, &twoKiAir, sizeof(twoKiAir));
  if (err != ESP_OK) {
    return err;
  }
  float readBack = -1.0f;
  err = mahonyKiAirStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, &twoKiAir, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mahonyKiAirStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_MKIAIR);
}

esp_err_t camHoldKmaxStoreLoad(float *kmax)
{
  float v = 0.0f;
  esp_err_t err = loadBlob(IMU_CAL_NVS_KEY_HOLDKMAX, &v, sizeof(v));
  if (err != ESP_OK) {
    return err;
  }
  if (!(v >= 1.0f && v <= 3.0f)) {   /* NaN fails */
    return ESP_ERR_INVALID_ARG;
  }
  *kmax = v;
  return ESP_OK;
}

esp_err_t camHoldKmaxStoreSave(float kmax)
{
  if (!(kmax >= 1.0f && kmax <= 3.0f)) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_HOLDKMAX, &kmax, sizeof(kmax));
  if (err != ESP_OK) {
    return err;
  }
  float readBack = -1.0f;
  err = camHoldKmaxStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (memcmp(&readBack, &kmax, sizeof(readBack)) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t camHoldKmaxStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_HOLDKMAX);
}

static esp_err_t loadFlag(const char *key, bool *on)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  uint8_t v = 0;
  err = nvs_get_u8(handle, key, &v);
  nvs_close(handle);
  if (err != ESP_OK) {
    return err;
  }
  if (v > 1) {
    return ESP_ERR_INVALID_STATE;
  }
  *on = (v == 1);
  return ESP_OK;
}

static esp_err_t saveFlag(const char *key, bool on)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, key, on ? 1 : 0);
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  if (err != ESP_OK) {
    return err;
  }
  bool readBack = !on;
  err = loadFlag(key, &readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (readBack == on) ? ESP_OK : ESP_FAIL;
}

esp_err_t thrustCapStoreLoad(uint16_t *cap)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  uint16_t v = 0;
  err = nvs_get_u16(handle, IMU_CAL_NVS_KEY_GCAP, &v);
  nvs_close(handle);
  if (err != ESP_OK) {
    return err;
  }
  *cap = v;
  return ESP_OK;
}

esp_err_t thrustCapStoreSave(uint16_t cap)
{
  if (cap == 0) {
    esp_err_t err = eraseKey(IMU_CAL_NVS_KEY_GCAP);   /* bracketed itself */
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
  }
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_u16(handle, IMU_CAL_NVS_KEY_GCAP, cap);
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  if (err != ESP_OK) {
    return err;
  }
  uint16_t readBack = 0;
  err = thrustCapStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (readBack == cap) ? ESP_OK : ESP_FAIL;
}

esp_err_t motorPwmStoreLoad(uint32_t *hz)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  uint32_t v = 0;
  err = nvs_get_u32(handle, IMU_CAL_NVS_KEY_PWMHZ, &v);
  nvs_close(handle);
  if (err != ESP_OK) {
    return err;
  }
  *hz = v;
  return ESP_OK;
}

esp_err_t motorPwmStoreSave(uint32_t hz)
{
  if (hz == 0) {
    esp_err_t err = eraseKey(IMU_CAL_NVS_KEY_PWMHZ);   /* bracketed itself */
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
  }
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_u32(handle, IMU_CAL_NVS_KEY_PWMHZ, hz);
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  if (err != ESP_OK) {
    return err;
  }
  uint32_t readBack = 0;
  err = motorPwmStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (readBack == hz) ? ESP_OK : ESP_FAIL;
}

esp_err_t restartLockStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_PLOCK, on);
}

esp_err_t restartLockStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_PLOCK, on);
}

esp_err_t motorGuardStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_MGUARD, on);
}

esp_err_t motorGuardStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_MGUARD, on);
}

esp_err_t softStartStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_SOFT, on);
}

esp_err_t softStartStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_SOFT, on);
}

esp_err_t camStandbyStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_CAMSB, on);
}

esp_err_t camStandbyStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_CAMSB, on);
}

esp_err_t camStandbyMsStoreLoad(uint16_t *ms)
{
  return loadBlob(IMU_CAL_NVS_KEY_CAMMS, ms, sizeof(*ms));
}

esp_err_t camStandbyMsStoreSave(uint16_t ms)
{
  return saveBlob(IMU_CAL_NVS_KEY_CAMMS, &ms, sizeof(ms));
}

esp_err_t camStandbyColdMsStoreLoad(uint16_t *ms)
{
  return loadBlob(IMU_CAL_NVS_KEY_CAMCOLD, ms, sizeof(*ms));
}

esp_err_t camStandbyColdMsStoreSave(uint16_t ms)
{
  return saveBlob(IMU_CAL_NVS_KEY_CAMCOLD, &ms, sizeof(ms));
}

esp_err_t camGroundStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_CAMGND, on);
}

esp_err_t camGroundStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_CAMGND, on);
}

esp_err_t camGroundMsStoreLoad(uint16_t *ms)
{
  return loadBlob(IMU_CAL_NVS_KEY_CAMGNDMS, ms, sizeof(*ms));
}

esp_err_t camGroundMsStoreSave(uint16_t ms)
{
  return saveBlob(IMU_CAL_NVS_KEY_CAMGNDMS, &ms, sizeof(ms));
}

esp_err_t camLeadMsStoreLoad(uint16_t *ms)
{
  return loadBlob(IMU_CAL_NVS_KEY_CAMLEAD, ms, sizeof(*ms));
}

esp_err_t camLeadMsStoreSave(uint16_t ms)
{
  return saveBlob(IMU_CAL_NVS_KEY_CAMLEAD, &ms, sizeof(ms));
}

esp_err_t camClockStoreLoad(bool *slow)
{
  return loadFlag(IMU_CAL_NVS_KEY_CAMCLK, slow);
}

esp_err_t camClockStoreSave(bool slow)
{
  return saveFlag(IMU_CAL_NVS_KEY_CAMCLK, slow);
}

esp_err_t attTrimLearnStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_ATLEARN, on);
}

esp_err_t attTrimLearnStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_ATLEARN, on);
}

esp_err_t txLowStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_TXLO, on);
}

esp_err_t txLowStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_TXLO, on);
}

esp_err_t wifiQuietStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_WIFIQ, on);
}

esp_err_t wifiQuietStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_WIFIQ, on);
}

esp_err_t camEnableStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_CAMON, on);
}

esp_err_t camEnableStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_CAMON, on);
}

esp_err_t camHoldStoreLoad(bool *on)
{
  return loadFlag(IMU_CAL_NVS_KEY_CAMHOLD, on);
}

esp_err_t camHoldStoreSave(bool on)
{
  return saveFlag(IMU_CAL_NVS_KEY_CAMHOLD, on);
}

esp_err_t yawLockStoreLoad(bool *on)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  uint8_t v = 0;
  err = nvs_get_u8(handle, IMU_CAL_NVS_KEY_YAWLOCK, &v);
  nvs_close(handle);
  if (err != ESP_OK) {
    return err;
  }
  if (v > 1) {
    return ESP_ERR_INVALID_STATE;
  }
  *on = (v == 1);
  return ESP_OK;
}

esp_err_t yawLockStoreSave(bool on)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, IMU_CAL_NVS_KEY_YAWLOCK, on ? 1 : 0);
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  if (err != ESP_OK) {
    return err;
  }
  bool readBack = !on;
  err = yawLockStoreLoad(&readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (readBack == on) ? ESP_OK : ESP_FAIL;
}

esp_err_t autoKeyStoreLoad(uint8_t key[AUTO_KEY_STORE_LEN])
{
  return loadBlob(IMU_CAL_NVS_KEY_AUTOKEY, key, AUTO_KEY_STORE_LEN);
}

esp_err_t autoKeyStoreSave(const uint8_t key[AUTO_KEY_STORE_LEN])
{
  esp_err_t err = saveBlob(IMU_CAL_NVS_KEY_AUTOKEY, key, AUTO_KEY_STORE_LEN);
  if (err != ESP_OK) {
    return err;
  }
  uint8_t readBack[AUTO_KEY_STORE_LEN];
  err = autoKeyStoreLoad(readBack);
  if (err == ESP_OK && memcmp(readBack, key, AUTO_KEY_STORE_LEN) != 0) {
    err = ESP_FAIL;
  }
  memset(readBack, 0, sizeof(readBack));
  return err;
}

esp_err_t autoKeyStoreErase(void)
{
  return eraseKey(IMU_CAL_NVS_KEY_AUTOKEY);
}

/* Autonomy settings (auto_glue_core.h's table): one scalar key each. */
esp_err_t autoSettingStoreLoad(const char *key, uint8_t nvsType, int32_t *value)
{
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    return err;
  }
  switch (nvsType) {
    case 1: {
      uint8_t v = 0;
      err = nvs_get_u8(handle, key, &v);
      if (err == ESP_OK) {
        *value = v;
      }
      break;
    }
    case 2: {
      uint16_t v = 0;
      err = nvs_get_u16(handle, key, &v);
      if (err == ESP_OK) {
        *value = v;
      }
      break;
    }
    case 3: {
      int16_t v = 0;
      err = nvs_get_i16(handle, key, &v);
      if (err == ESP_OK) {
        *value = v;
      }
      break;
    }
    case 4: {
      uint32_t v = 0;
      err = nvs_get_u32(handle, key, &v);
      if (err == ESP_OK) {
        if (v > INT32_MAX) {
          err = ESP_ERR_INVALID_SIZE;
        } else {
          *value = (int32_t)v;
        }
      }
      break;
    }
    default:
      err = ESP_ERR_INVALID_ARG;
      break;
  }
  nvs_close(handle);
  return err;
}

esp_err_t autoSettingStoreSave(const char *key, uint8_t nvsType, int32_t value)
{
  const bool fits = (nvsType == 1 && value >= 0 && value <= UINT8_MAX) ||
                    (nvsType == 2 && value >= 0 && value <= UINT16_MAX) ||
                    (nvsType == 3 && value >= INT16_MIN && value <= INT16_MAX) || (nvsType == 4 && value >= 0);
  if (!fits) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    switch (nvsType) {
      case 1:
        err = nvs_set_u8(handle, key, (uint8_t)value);
        break;
      case 2:
        err = nvs_set_u16(handle, key, (uint16_t)value);
        break;
      case 3:
        err = nvs_set_i16(handle, key, (int16_t)value);
        break;
      default:
        err = nvs_set_u32(handle, key, (uint32_t)value);
        break;
    }
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  if (err != ESP_OK) {
    return err;
  }
  int32_t readBack = 0;
  err = autoSettingStoreLoad(key, nvsType, &readBack);
  if (err != ESP_OK) {
    return err;
  }
  return (readBack == value) ? ESP_OK : ESP_FAIL;
}

/* Every item of the key, whatever its type (nvs_erase_key erases one item of any type; a key whose type changed,
 * autowait u16 -> u32 on 2026-10-14, may hold one of each): until NOT_FOUND, at most 4 times, then one commit. An
 * absent key is ESP_OK. Bracketed in the flash interlock. */
esp_err_t autoSettingStoreErase(const char *key)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  nvs_handle_t handle;
  esp_err_t err = nvs_open(IMU_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    err = ESP_OK;   /* no namespace, no key */
  } else if (err == ESP_OK) {
    for (int i = 0; i < 4; i++) {
      err = nvs_erase_key(handle, key);
      if (err != ESP_OK) {
        break;
      }
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      err = ESP_OK;
    }
    if (err == ESP_OK) {
      err = nvs_commit(handle);
    }
    nvs_close(handle);
  }
  autonomyFlashEnd();
  return err;
}
