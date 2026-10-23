/* Host stub (tools/tests/test_flight_recorder_host.c): the two motor reads flight_recorder.c makes; the real
 * motors.h is not host-clean. The test file defines them. */
#ifndef FREC_HOST_MOTORS_H_
#define FREC_HOST_MOTORS_H_
#include <stdbool.h>
#include <stdint.h>
#define NBR_OF_MOTORS 4
bool motorsIsInhibited(void);
int motorsGetRatio(uint32_t id);
#endif
