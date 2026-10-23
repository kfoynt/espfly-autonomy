#pragma once
#include <stdint.h>
#include "esp_err.h"
#define AUTO_KEY_STORE_LEN 16
esp_err_t autoKeyStoreLoad(uint8_t key[AUTO_KEY_STORE_LEN]);
