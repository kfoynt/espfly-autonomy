#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct __attribute__((packed)) { uint16_t thrust; int16_t roll, pitch, yaw; uint16_t vbatMv; } flightRecord_t;
bool flightRecorderGetLatest(flightRecord_t *out);
