#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <tk/tkernel.h>

/* Hump detection for imu_task (Buddy 4).  Kept out of imu_task.c, away
 * from the I2C and kernel calls, so tests/imu_hump can run it on a PC. */

#define HUMP_SAMPLES	10		/* 100 ms detection window            */

/* Agreed hump_state values (2 bits, Tribunal Section 4). */
typedef enum { HUMP_NONE, HUMP_CLIMBING, HUMP_DESCENDING } HUMP_STATE;

/* One HUMP_SAMPLES window: mean tilt ax, ay in mg, distance driven in
 * mm (- reversing), and whether motion_task reported STEADY. */
IMPORT void hump_step(W ax, W ay, W dmm, BOOL steady);

IMPORT HUMP_STATE hump_get_state(void);
IMPORT const char *hump_state_name(HUMP_STATE s);
IMPORT W  hump_height_mm(void);		/* on the hump now; 0 if none */
IMPORT UW hump_get_count(void);		/* humps finished since boot  */
IMPORT UW hump_get_last_mm(void);	/* height of the last one     */
IMPORT UW hump_get_max_mm(void);	/* highest since boot (FR-12) */

IMPORT UW imu_isqrt(UW v);

#endif /* IMU_HUMP_H */
