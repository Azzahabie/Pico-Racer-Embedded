/*
 *	imu_class.c
 *	Motion event classifier for imu_task (Buddy 4, FR-27): one call per
 *	10 ms sample, reports a change of class once it has held.
 *
 *	No I2C or kernel calls, so tests/imu_safety runs it on a PC.
 */

#include <tk/tkernel.h>

#include "imu_class.h"

LOCAL const char * const mc_name[] = {
	"still", "turning", "climbing", "descending", "IMPACT", "accel",
	"moving"
};

LOCAL MOTION_CLASS	cur = MC_STATIONARY;	/* reported            */
LOCAL MOTION_CLASS	cand = MC_STATIONARY;	/* waiting to confirm  */
LOCAL INT		cand_n;
LOCAL INT		still_n;		/* samples with no mm  */
LOCAL INT		since_move = MC_MOVED_N; /* samples since a mm  */
LOCAL INT		impact_hold;

EXPORT void mclass_reset(void)
{
	cur = cand = MC_STATIONARY;
	cand_n = still_n = impact_hold = 0;
	since_move = MC_MOVED_N;
}

EXPORT MOTION_CLASS mclass_get(void)	{ return cur; }

EXPORT const char *mclass_name(MOTION_CLASS c)
{
	return ((UW)c <= MC_MOVING) ? mc_name[c] : "?";
}

/* The class this sample, before debouncing.  Order is priority: an
   impact or a hump outranks a turn, a turn outranks speeding up. */
LOCAL MOTION_CLASS raw_class(BOOL ok, UW amag_mg, W turn_dps,
			     BOOL accelerating, UINT hump)
{
	W	at = (turn_dps < 0) ? -turn_dps : turn_dps;

	if(ok && amag_mg > MC_IMPACT_MG && since_move < MC_MOVED_N) {
		impact_hold = MC_IMPACT_HOLD;
	}
	if(impact_hold > 0) {
		impact_hold--;
		return MC_IMPACT;
	}
	if(hump == 1) return MC_CLIMBING;
	if(hump == 2) return MC_DESCENDING;
	if(at > MC_TURN_DPS) return MC_TURNING;
	if(still_n >= MC_STILL_N) {
		/* Turning on the spot moves no mm: not still. */
		return (at < MC_STILL_DPS) ? MC_STATIONARY : cur;
	}
	if(accelerating) return MC_ACCELERATING;
	return MC_MOVING;
}

EXPORT BOOL mclass_step(BOOL ok, UW amag_mg, W dmm, W turn_dps,
			BOOL accelerating, UINT hump)
{
	MOTION_CLASS	raw;

	if(dmm != 0) {
		still_n = 0;
		since_move = 0;
	} else {
		if(still_n < MC_STILL_N) still_n++;
		if(since_move < MC_MOVED_N) since_move++;
	}

	raw = raw_class(ok, amag_mg, turn_dps, accelerating, hump);
	if(raw == cur) {
		cand_n = 0;
		return FALSE;
	}
	/* An impact is reported at once; anything else must hold. */
	if(raw == MC_IMPACT) {
		cur = raw;
		cand_n = 0;
		return TRUE;
	}
	if(raw == cand) {
		cand_n++;
	} else {
		cand = raw;
		cand_n = 1;
	}
	if(cand_n < MC_CONFIRM_N) return FALSE;
	cur = raw;
	cand_n = 0;
	return TRUE;
}
