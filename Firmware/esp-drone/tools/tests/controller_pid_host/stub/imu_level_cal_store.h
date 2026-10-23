#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "torque_trim.h"
esp_err_t torqueTrimStoreLoad(torqueTrimRecord_t *rec);
esp_err_t attTrimStoreLoad(attTrimRecord_t *rec);
esp_err_t yawLockStoreLoad(bool *on);
