/*
 *	imu_safety.c
 *	Abnormal tilt (FR-28) and read-failure (NFR-13) checks for imu_task
 *	(Buddy 4), one call per 10 ms sample.
 *
 *	No I2C or kernel calls, so tests/imu_safety runs them on a PC.
 */

#include <tk/tkernel.h>

#include "hump.h"		/* imu_isqrt() */
#include "imu_safety.h"

LOCAL INT	tilt_n;			/* samples in a row over T           */
LOCAL INT	bad_n;			/* samples in a row with no data     */
LOCAL UW	tilt_mg;

EXPORT void safety_reset(void)
{
	tilt_n = 0;
	bad_n = 0;
	tilt_mg = 0;
}

EXPORT UW safety_tilt_mg(void)	{ return tilt_mg; }
EXPORT BOOL safety_tilted(void)	{ return tilt_n >= SAFETY_TILT_N; }
EXPORT BOOL safety_no_data(void)	{ return bad_n >= SAFETY_BAD_N; }

/* Tilt is the sideways part of gravity, |x, y|, so it works whichever
   way the car leans.  A jolt also pushes x and y, but it pulls |a| away
   from 1 g, so samples outside SAFETY_G_LO_MG..SAFETY_G_HI_MG neither
   count nor reset the run: a crash mid-tilt does not hide the tilt.
   Picked up level by hand reads ~1 g and no tilt, so it does not trip. */
EXPORT UINT safety_step(BOOL ok, W ax, W ay, W az)
{
	UINT	ev = 0U;
	UW	g;

	if(!ok) {
		bad_n++;
		if(bad_n == SAFETY_REINIT_N) ev |= SAFETY_EV_REINIT;
		if(bad_n == SAFETY_BAD_N) ev |= SAFETY_EV_NO_DATA;
		return ev;
	}
	bad_n = 0;

	tilt_mg = imu_isqrt((UW)(ax * ax + ay * ay));
	g = imu_isqrt((UW)(ax * ax + ay * ay + az * az));
	if(g < SAFETY_G_LO_MG || g > SAFETY_G_HI_MG) return ev;

	if(tilt_mg > SAFETY_TILT_MG) {
		tilt_n++;
		if(tilt_n == SAFETY_TILT_N) ev |= SAFETY_EV_TILT;
	} else {
		tilt_n = 0;
	}
	return ev;
}
