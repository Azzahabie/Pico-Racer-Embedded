#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <tk/tkernel.h>

/* Hump detection for imu_task (Buddy 4).  Kept out of imu_task.c, away
 * from the I2C and kernel calls, so tests/imu_hump can run it on a PC. */

#define HUMP_SAMPLES	10		/* 100 ms detection window            */

/* Build switches for a ramp longer than the practice ramps (19 Nov: the
 * final ramp is the same height, a different width).  Here, not in
 * hump.c, so tests/imu_hump reads the same values.  The defaults keep
 * the detector as it was on the car (2-3 Nov, 14/14 demo-hump passes).
 *
 * HUMP_ENTER_MG: tilt that starts a hump, held for 3 windows.  A climb
 * gentler than this is never seen.  At 50 a 10 mm ramp counts reliably
 * with its way up at most about 180 mm long (3.2 deg); at 40, about
 * 230 mm (2.5 deg) (sim).  40 is still 10 mg above the measured floor
 * spots (up to ~30 mg).  Build with -DHUMP_ENTER_MG=40 only if the real
 * ramp climbs at 2.5 to 3.2 deg, after a flat run with no false hump.
 *
 * HUMP_WAY_DOWN: 1 = once a hump closes while driving, downhill is the
 * rest of its way down until the next climb.  At 0, a hump that closes
 * part way down (a level top longer than the 120 mm level cap, or a
 * gentle climb seen late) takes the rest of the way down for a new
 * climb, and the ramp counts twice (sim).  The next hump still counts. */
#define HUMP_ENTER_MG_CAR 50		/* as driven on the car, 2-3 Nov      */
#ifndef HUMP_ENTER_MG
#define HUMP_ENTER_MG	HUMP_ENTER_MG_CAR /* ~3 deg: a hump starts, moving */
#endif
#ifndef HUMP_WAY_DOWN
#define HUMP_WAY_DOWN	0
#endif

/* Agreed hump_state values (2 bits, Tribunal Section 4). */
typedef enum { HUMP_NONE, HUMP_CLIMBING, HUMP_DESCENDING } HUMP_STATE;

/* One HUMP_SAMPLES window: mean tilt ax, ay in mg, distance driven in
 * mm (- reversing), and whether motion_task reported STEADY. */
IMPORT void hump_step(W ax, W ay, W dmm, BOOL steady);

/* Drop a hump in progress, and the "on top" follow, without counting it:
 * a fault, or the car tilted past T (Statechart 6.4: fault discards the
 * unfinished hump). */
IMPORT void hump_abort(void);

/* Mission paused / ended (not by a fault): a hump in progress that is
 * past its top is counted now; one still on the way up is dropped.  The
 * "on top" follow is kept, since the car is still where it stopped. */
IMPORT void hump_pause(void);
IMPORT void hump_mission_end(void);

/* A new mission (Ready -> Running, Statechart 6.4): drop any hump in
 * progress and start the count, last and highest (FR-12) from zero.
 * Paused -> Running keeps them. */
IMPORT void hump_mission_reset(void);

IMPORT HUMP_STATE hump_get_state(void);
IMPORT const char *hump_state_name(HUMP_STATE s);
IMPORT W  hump_height_mm(void);		/* on the hump now; 0 if none */
IMPORT W  hump_height_um(void);		/* the same in um, for logs   */
IMPORT UW hump_get_count(void);		/* humps counted this mission */
IMPORT UW hump_get_last_mm(void);	/* height of the last one     */
IMPORT UW hump_get_max_mm(void);	/* highest hump, this mission */

IMPORT UW imu_isqrt(UW v);
IMPORT UW imu_deg10(UW mg);		/* tilt in 0.1 deg from sin in mg */

#endif /* IMU_HUMP_H */
