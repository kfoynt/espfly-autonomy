#pragma once
#include <stdlib.h>
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define MALLOC_CAP_SPIRAM 4
/* The test defines it: records the caps, can fail every PSRAM request. */
void *heap_caps_malloc(size_t n, int caps);
static inline void heap_caps_free(void *p) { free(p); }
static inline size_t heap_caps_get_free_size(int caps) { (void)caps; return 27000; }
static inline size_t heap_caps_get_minimum_free_size(int caps) { (void)caps; return 21000; }
