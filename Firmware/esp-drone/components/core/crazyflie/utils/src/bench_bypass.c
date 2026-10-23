/*
 * ESP-Drone Firmware
 *
 * bench_bypass.c - RAM-only bisect toggles for the loop-rate bench (bench_bypass.h)
 */
#include "bench_bypass.h"

volatile uint32_t benchBypassMask;   /* .bss: 0 at every boot, never saved */
