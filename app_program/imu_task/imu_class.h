#ifndef IMU_CLASS_H
#define IMU_CLASS_H

#include <tk/tkernel.h>

/* Motion event classifier for imu_task (Buddy 4, FR-27), run on every
 * 10 ms sample.  No I2C or kernel calls, so tests/imu_safety runs it on a
 * PC.  The class fits the 3-bit motion event field (Tribunal Version B);
 * 0-4 are the FR-27 classes in the MQTT_SCHEMA.md order, 5-6 are extra. */
typedef enum {
	MC_STATIONARY	= 0,
	MC_TURNING	= 1,
	MC_CLIMBING	= 2,
	MC_DESCENDING	= 3,
	MC_IMPACT	= 4,
	MC_ACCELERATING	= 5,		/* project brief's sixth event        */
	MC_MOVING	= 6		/* straight, steady: none of the above */
} MOTION_CLASS;

/* Thresholds: the Week 6 proposals, except where noted.  Not yet
 * validated on a driven car (assumption A8). */
#define MC_TURN_DPS	20		/* |turn| above this: turning         */
#define MC_STILL_DPS	5		/* |turn| below this to be stationary */
#define MC_STILL_N	10		/* no encoder mm for 100 ms: still.
					   Proposed 5 samples, but dist_mm is
					   1 mm per count: at 30 mm/s a mm
					   comes only every 33 ms            */
#define MC_IMPACT_MG	1500		/* A = 1.5 g on one sample            */
#define MC_IMPACT_HOLD	50		/* report impact for 500 ms           */
#define MC_MOVED_N	100		/* impact only within 1 s of moving:
					   carrying the car by hand hit 2.4 g */
#define MC_CONFIRM_N	3		/* a new class must hold 30 ms
					   (impact is reported at once)       */

IMPORT void mclass_reset(void);

/* One sample.  ok: the accel read was good; amag_mg: |a| in mg; dmm:
 * encoder mm since the last sample (- reversing); turn_dps: from
 * distance_state, CCW positive; accelerating: distance_state says
 * ACCELERATING; hump: hump_get_state() (0 none, 1 climbing,
 * 2 descending).  Returns TRUE when the reported class changes. */
IMPORT BOOL mclass_step(BOOL ok, UW amag_mg, W dmm, W turn_dps,
			BOOL accelerating, UINT hump);

IMPORT MOTION_CLASS mclass_get(void);
IMPORT const char *mclass_name(MOTION_CLASS c);

#endif /* IMU_CLASS_H */
