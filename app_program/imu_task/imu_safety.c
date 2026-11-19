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
LOCAL W		tilt_z;			/* z of the last good sample, mg     */

EXPORT void safety_reset(void)
{
	tilt_n = 0;
	bad_n = 0;
	tilt_mg = 0;
	tilt_z = 0;
}

EXPORT UW safety_tilt_mg(void)	{ return tilt_mg; }
EXPORT W safety_z_mg(void)	{ return tilt_z; }
EXPORT BOOL safety_tilted(void)	{ return tilt_n >= SAFETY_TILT_N; }
EXPORT INT safety_tilt_run(void)	{ return tilt_n; }
EXPORT BOOL safety_no_data(void)	{ return bad_n >= SAFETY_BAD_N; }

/* The stop message only.  It used SAFETY_FLIP_MG, the trip's 0 mg, so a
   car tipped onto its side (90 deg, |x, y| ~1 g) with z a little below 0
   printed "upside down" instead of its angle.  Below -500 mg (past 120
   deg) |x, y| is under 866 mg and falls to ~0 on its roof. */
EXPORT BOOL safety_upside_down(void)
{
	return tilt_z < SAFETY_UPSIDE_MSG_MG;
}

/* A chip that reset (brown-out, a loose wire) answers 0, 0, 0, and the
   5987 mg glitch (4 Nov 04:41:36, "raw mean x=5987 y=5987 z=5987") reads
   the same on all three axes.  Neither is a reading.  Counted as good,
   the glitch's ~10 g is outside the jolt band, so safety_step() ignored
   it and no stop could fire while it lasted; as a bad read it ends in
   the no-data fault after SAFETY_BAD_N.  A real reading with all three
   axes equal to the mg needs the car ~55 deg over, already past T. */
EXPORT BOOL safety_read_ok(W x, W y, W z)
{
	return !(x == y && y == z);
}

/* Tilt is the sideways part of gravity, |x, y|, so it works whichever
   way the car leans.  A jolt also pushes x and y, but it pulls |a| away
   from 1 g, so samples outside SAFETY_G_LO_MG..SAFETY_G_HI_MG neither
   count nor reset the run: a crash mid-tilt does not hide the tilt.
   Picked up level by hand reads ~1 g and no tilt, so it does not trip.
   A sideways knock under ~0.8 g stays inside the |a| band, so a sample
   only counts if z has dropped too, as gravity tipped away from z does;
   a knock on a level car leaves z near 1 g and restarts the run. */
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
	tilt_z = az;
	g = imu_isqrt((UW)(ax * ax + ay * ay + az * az));
	if(g < SAFETY_G_LO_MG || g > SAFETY_G_HI_MG) return ev;

	/* Past 90 deg (z below SAFETY_FLIP_MG) is tilted too: upside down
	   the sideways part is ~0 again (sin 180 = 0), so |x, y| alone read
	   a car resting on its roof as level. */
	if((tilt_mg > SAFETY_TILT_MG && az < SAFETY_Z_MAX_MG)
	   || az < SAFETY_FLIP_MG) {
		tilt_n++;
		if(tilt_n == SAFETY_TILT_N) ev |= SAFETY_EV_TILT;
	} else {
		tilt_n = 0;
	}
	return ev;
}
