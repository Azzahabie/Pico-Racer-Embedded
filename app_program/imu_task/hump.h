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

/* Drop a hump in progress, and the "on top" follow, without counting it:
 * a fault, or the car tilted past T (Statechart 6.4: fault discards the
 * unfinished hump). */
IMPORT void hump_abort(void);

/* Mission paused: drop a hump in progress like hump_abort(), but keep
 * the "on top" follow, since the car is still where it stopped. */
IMPORT void hump_pause(void);

/* A new mission (Ready -> Running, Statechart 6.4): drop any hump in
 * progress and start the count, last and highest (FR-12) from zero.
 * Paused -> Running keeps them. */
IMPORT void hump_mission_reset(void);

IMPORT HUMP_STATE hump_get_state(void);
IMPORT const char *hump_state_name(HUMP_STATE s);
IMPORT W  hump_height_mm(void);		/* on the hump now; 0 if none */
IMPORT W  hump_height_um(void);		/* the same in um, for logs   */
IMPORT UW hump_get_count(void);		/* humps finished since boot  */
IMPORT UW hump_get_last_mm(void);	/* height of the last one     */
IMPORT UW hump_get_max_mm(void);	/* highest since boot (FR-12) */

IMPORT UW imu_isqrt(UW v);

#endif /* IMU_HUMP_H */
