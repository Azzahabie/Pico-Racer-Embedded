#ifndef IMU_SAFETY_H
#define IMU_SAFETY_H

#include <tk/tkernel.h>

/* Abnormal tilt and read-failure checks for imu_task (Buddy 4), run on
 * every 10 ms sample.  No I2C or kernel calls, so tests/imu_safety can
 * run them on a PC; imu_task acts on what safety_step() returns. */

#define SAFETY_TILT_MG	342		/* T = 20 deg (FR-28): sin 20 = 0.342.
					   Humps peak at ~10 deg (2026-10-27
					   ramp logs), so T >= steepest + 5  */
#define SAFETY_TILT_N	3		/* N samples in a row (FR-28)         */
#define SAFETY_G_LO_MG	700		/* |a| outside this: a jolt, not a    */
#define SAFETY_G_HI_MG	1300		/* tilt, so it does not count to N    */
#define SAFETY_BAD_N	5		/* Y = 50 ms with no good sample
					   (NFR-13)                           */
#define SAFETY_REINIT_N	2		/* re-set the chip up after this many */

/* Bits returned by safety_step(). */
#define SAFETY_EV_TILT		0x01U	/* tilt > T for N samples, just now   */
#define SAFETY_EV_NO_DATA	0x02U	/* no good sample for Y, just now     */
#define SAFETY_EV_REINIT	0x04U	/* reads failing: set the chip up again*/

IMPORT void safety_reset(void);

/* One sample.  ok: the read succeeded and was not all zeros (a chip that
 * reset reads 0, 0, 0).  ax, ay, az: acceleration in mg, offsets
 * removed, so a level car reads 0, 0, 1000.  Each event is returned
 * once, when it first happens; it is returned again only after a good
 * (or level) sample in between. */
IMPORT UINT safety_step(BOOL ok, W ax, W ay, W az);

IMPORT UW safety_tilt_mg(void);		/* tilt of the last good sample */
IMPORT BOOL safety_tilted(void);	/* over T for N samples, now    */
IMPORT BOOL safety_no_data(void);	/* no good sample for Y, now    */

#endif /* IMU_SAFETY_H */
