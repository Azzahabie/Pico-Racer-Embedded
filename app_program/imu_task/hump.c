/*
 *	hump.c
 *	Hump detection for imu_task (Buddy 4): fed 100 ms means of the
 *	accelerometer's x and y and the distance driven, it reports CLIMBING /
 *	DESCENDING and counts humps with their height.
 *
 *	No I2C or kernel calls, only tm_printf, so tests/imu_hump runs it on
 *	a PC against the passes that went wrong on the car.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "hump.h"

/* Hump detection runs on 100 ms means of x and y.  Tilted by angle t, the
   car rises sin(t) mm per mm driven, and sin(t) is the tilt in mg / 1000,
   so tilt mg * distance mm = rise in um.  Checked against the course
   humps (9 mm, 23 mm): the 500 ms logs summed to 10 mm and 24 mm. */
#define HUMP_ENTER_MG	50		/* ~3 deg: a hump starts (while moving)*/
#define HUMP_EXIT_MG	35		/* below this the car counts as level;
					   floor spots differ by up to ~30   */
#define HUMP_GROUND_UM	2000		/* back within 2 mm of the start      */
#define HUMP_END_MM	40		/* level this far, once on the way
					   down: the hump is over            */
#define HUMP_TOP_MAX_MM	250		/* level this far, down or not: over.
					   For a far side that drops off too
					   fast to read as a slope; above the
					   practice ramp's level top (~100)  */
#define HUMP_MIN_MM	4		/* shorter climbs are not humps       */
#define HUMP_STILL_WIN	10		/* stopped 1 s, any tilt: hump over   */
#define HUMP_STILL_MM	1		/* per window: encoder jitter, still  */
#define HUMP_CONFIRM_WIN 3		/* tilt held 300 ms, one way: a hump  */
#define HUMP_FLIP_WIN	2		/* 200 ms the other way: up <-> down  */
#define HUMP_HAND_MG	60		/* tilt change between two stopped
					   windows: picked up by hand        */
#define BASE_DIV	8		/* level tilt tracking, per window    */
#define BASE_MAX_MG	40		/* level is only learned this close to
					   the calibration level, stopped     */

LOCAL const char * const hump_name[] = { "none", "climbing", "descending" };

LOCAL HUMP_STATE hump_state = HUMP_NONE;
LOCAL UW	hump_count;		/* humps finished since boot         */
LOCAL UW	hump_last_mm;		/* height of the last one            */
LOCAL UW	hump_max_mm;		/* highest since boot                */
LOCAL W		base[2];		/* level x, y: the floor's own tilt  */
LOCAL W		onset[2];		/* tilt as the hump began = uphill   */
LOCAL W		h_um, peak_um, trough_um; /* height now / highest / lowest, um */
LOCAL W		hump_len, level_mm;	/* distance on the hump / level      */
LOCAL UW	tilt_peak;
LOCAL INT	still_win;		/* windows stopped                   */
LOCAL INT	flip_win;		/* windows tilted against the state  */
LOCAL INT	pend_win;		/* windows of a hump not yet confirmed*/
LOCAL W		pend_sum[2], pend_h, pend_len;
LOCAL UW	pend_peak;
LOCAL BOOL	on_top;			/* a hump ended stopped on top, and
					   the car is not level again yet    */
LOCAL W		top_um;			/* height left up there, >= 0        */
LOCAL W		top_onset[2];		/* that hump's uphill                */
LOCAL W		top_level;		/* level distance since, mm          */
LOCAL BOOL	top_down;		/* has come down some since          */
LOCAL BOOL	top_still;		/* last window was stopped           */
LOCAL W		top_last[2];		/* its tilt                          */

EXPORT UW imu_isqrt(UW v)
{
	UW	r = 0, bit = 1UL << 30;

	while(bit > v) bit >>= 2;
	while(bit != 0) {
		if(v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
		else		 { r >>= 1; }
		bit >>= 2;
	}
	return r;
}

EXPORT HUMP_STATE hump_get_state(void)	{ return hump_state; }
EXPORT UW hump_get_count(void)		{ return hump_count; }
EXPORT UW hump_get_last_mm(void)	{ return hump_last_mm; }
EXPORT UW hump_get_max_mm(void)		{ return hump_max_mm; }

EXPORT const char *hump_state_name(HUMP_STATE s)
{
	return ((UW)s <= HUMP_DESCENDING) ? hump_name[s] : "?";
}

EXPORT W hump_height_mm(void)
{
	return (hump_state == HUMP_NONE) ? 0 : h_um / 1000;
}

/* End the hump in progress: count it if it rose high enough.  The height
   is the larger of the highest rise and the deepest drop, so it still
   comes out right if uphill was ever taken the wrong way round. */
LOCAL void hump_finish(void)
{
	W	top = (peak_um > -trough_um) ? peak_um : -trough_um;
	W	mm = (top + 500) / 1000;
	W	tenths = (top + 50) / 100;	/* 0.1 mm, for tuning logs */

	if(mm >= HUMP_MIN_MM) {
		hump_count++;
		hump_last_mm = (UW)mm;
		if(hump_last_mm > hump_max_mm) hump_max_mm = hump_last_mm;
		tm_printf((UB *)"[imu] ===== HUMP %u: %d.%d mm high =====  (up"
			       " %d.%d, down %d.%d, peak tilt %u mg, %d mm"
			       " long)\n",
			  hump_count, tenths / 10, tenths % 10,
			  (peak_um + 50) / 100 / 10, (peak_um + 50) / 100 % 10,
			  (-trough_um + 50) / 100 / 10,
			  (-trough_um + 50) / 100 % 10, tilt_peak, hump_len);
	} else {
		tm_printf((UB *)"[imu] hump ignored: only %d.%d mm high\n",
			  tenths / 10, tenths % 10);
	}
	hump_state = HUMP_NONE;
}

/* After a hump ended with the car stopped on top: follow the height left
   up there back down, so the way down is not taken for a new climb.
   Pushed up a ramp, stopped, then backed down, the backing down once
   counted as an 11 mm hump of its own.  Moving up on top counts too: a
   nudge forward on the slope once dropped the height, and the backing
   down after it became a hump again.  So the car is followed both ways
   until it is level again; top_um is only reported, it does not decide.
   The two ways disagree: a pivot on the way up under-reads the climb,
   and once a 2.8 mm climb ran out 20 mm into the way down, which then
   read as a new climb.  Returns TRUE if this window was on the hump;
   FALSE lets hump_step() go on as usual on level ground. */
LOCAL BOOL top_step(W dx, W dy, UW tilt, W dmm, W ad)
{
	W	dot = dx * top_onset[0] + dy * top_onset[1];	/* > 0: nose up */
	W	rise = ((dot >= 0) ? (W)tilt : -(W)tilt) * dmm;
	W	hx, hy;

	/* Picked up: the wheels stand still but the tilt swings.  Carried
	   off the top and put down at the start, the car never came down
	   as far as the encoders know, and the next climb was taken as
	   still being on top (log 14).  Stopped on a slope, the tilt holds. */
	if(ad == 0) {
		hx = dx - top_last[0];
		hy = dy - top_last[1];
		top_last[0] = dx; top_last[1] = dy;
		if(top_still && hx * hx + hy * hy > HUMP_HAND_MG * HUMP_HAND_MG) {
			on_top = FALSE;
			tm_printf((UB *)"[imu] hump: picked up, off the hump"
				       " (%d mm left)\n", top_um / 1000);
			return FALSE;
		}
		top_still = TRUE;
	} else {
		top_still = FALSE;
	}

	/* Once it has come down some, level for HUMP_END_MM or level and
	   stopped: off it.  Waiting for HUMP_END_MM alone once swallowed the
	   next climb, started from a stop at the foot.  Not before coming
	   down: stopped level on a round crest is still up, and on the
	   practice ramp's long level top a nudge on and back (40 mm level)
	   ended it there, so backing down the slope counted as HUMP 6. */
	if(tilt < HUMP_EXIT_MG) {
		top_level += ad;
		if((top_down && (top_level >= HUMP_END_MM || ad == 0))
		   || top_level >= HUMP_TOP_MAX_MM) {
			on_top = FALSE;
			tm_printf((UB *)"[imu] hump: on level ground again"
				       " (%d mm left)\n", top_um / 1000);
		}
		return FALSE;
	}
	top_level = 0;
	pend_win = 0;
	if(rise < 0) top_down = TRUE;
	top_um += rise;
	if(top_um < 0) top_um = 0;
	return TRUE;
}

/* One 100 ms step of hump detection.  ax, ay: mean tilt in mg; dmm:
   distance driven in the window (- reversing); steady: motion_task
   reports neither ACCELERATING nor DECELERATING.

   The mounting angle does not matter: tilt is measured from the level
   baseline as a magnitude, and a hump always starts with a climb, so the
   tilt then, flipped if reversing, is "nose up".  The car rises by
   +/- tilt * dmm: nose up and forward, or nose down and reversing, is
   up.  dmm keeps its sign, so backing down a slope lowers the car again
   (counting distance unsigned once added 9 mm per push up the same
   slope).  CLIMBING and DESCENDING follow which way the car is going.
   The height is the highest point of the running rise.

   A lurch reads as tilt too: one jolting window as the wheel struck the
   hump once set uphill backwards and lost a 23 mm hump.  So a hump only
   starts after HUMP_CONFIRM_WIN steady windows tilted the same way, and
   uphill is their sum; the state only flips after HUMP_FLIP_WIN.

   Level starts as the calibration level (0, 0) and only follows the
   floor while the car is stopped near it: following it while driving
   let a gentle ramp drag level up with it, so the climb never crossed
   HUMP_ENTER_MG and the way back down looked like a hump. */
EXPORT void hump_step(W ax, W ay, W dmm, BOOL steady)
{
	W	dx, dy, ad, dot, rise, fx, fy;
	UW	tilt;

	dx = ax - base[0];
	dy = ay - base[1];
	tilt = imu_isqrt((UW)(dx * dx + dy * dy));
	ad = (dmm < 0) ? -dmm : dmm;
	/* Tilt as if driving forward: reversing up a slope is nose down. */
	fx = (dmm < 0) ? -dx : dx;
	fy = (dmm < 0) ? -dy : dy;

	if(hump_state == HUMP_NONE) {
		if(on_top && top_step(dx, dy, tilt, dmm, ad)) return;
		if(tilt < HUMP_ENTER_MG) {
			pend_win = 0;
			if(ad == 0
			   && ax > -BASE_MAX_MG && ax < BASE_MAX_MG
			   && ay > -BASE_MAX_MG && ay < BASE_MAX_MG) {
				base[0] += (ax - base[0]) / BASE_DIV;
				base[1] += (ay - base[1]) / BASE_DIV;
			}
			return;
		}
		if(ad == 0) {		/* tilted by hand, not driven */
			pend_win = 0;
			return;
		}
		if(!steady) return;	/* lurching: neither count nor reset */

		/* Pending: restart if this window points the other way. */
		if(pend_win > 0
		   && fx * pend_sum[0] + fy * pend_sum[1] < 0) {
			pend_win = 0;
		}
		if(pend_win == 0) {
			pend_sum[0] = pend_sum[1] = 0;
			pend_h = pend_len = 0;
			pend_peak = 0;
		}
		pend_sum[0] += fx;
		pend_sum[1] += fy;
		pend_h += (W)tilt * ad;
		pend_len += ad;
		if(tilt > pend_peak) pend_peak = tilt;
		if(++pend_win < HUMP_CONFIRM_WIN) return;

		/* Confirmed: the pending windows were the start of the climb. */
		onset[0] = pend_sum[0]; onset[1] = pend_sum[1];
		h_um = peak_um = pend_h;
		trough_um = 0;
		hump_len = pend_len;
		tilt_peak = pend_peak;
		level_mm = 0;
		still_win = flip_win = pend_win = 0;
		hump_state = HUMP_CLIMBING;
		tm_printf((UB *)"[imu] hump: climbing (tilt %u mg, up %d mm)\n",
			  tilt, h_um / 1000);
		return;
	}

	hump_len += ad;
	dot = dx * onset[0] + dy * onset[1];	/* > 0: nose up */
	rise = (W)tilt * dmm;
	h_um += (dot >= 0) ? rise : -rise;
	if(h_um > peak_um) peak_um = h_um;
	if(h_um < trough_um) trough_um = h_um;
	if(tilt > tilt_peak) tilt_peak = tilt;

	/* Stopped: the pass is over, however the car is tilted.  It once
	   rested at ~50 mg after a pivot left a wheel on the hump's edge,
	   the hump stayed open, and the next pass was merged into it.  On
	   top of a hump this keeps the height so far, and top_step()
	   follows the way back down so it is not counted again. */
	still_win = (ad <= HUMP_STILL_MM) ? still_win + 1 : 0;
	if(still_win >= HUMP_STILL_WIN) {
		hump_finish();
		/* Any height above ground, not HUMP_MIN_MM: a 3.5 mm climb
		   rounds up to a counted 4 mm hump, and its way down once
		   slipped past a >= 4.0 mm test as HUMP 4. */
		if(h_um >= HUMP_GROUND_UM) {
			on_top = TRUE;
			top_um = h_um;
			top_onset[0] = onset[0]; top_onset[1] = onset[1];
			top_level = 0;
			top_down = FALSE;
			top_still = FALSE;
			tm_printf((UB *)"[imu] hump: stopped on top, %d mm up\n",
				  h_um / 1000);
		}
		return;
	}

	if(tilt >= HUMP_ENTER_MG) {
		level_mm = 0;
		if(dmm != 0 && steady) {
			HUMP_STATE s = ((dot >= 0) == (dmm > 0))
				       ? HUMP_CLIMBING : HUMP_DESCENDING;

			if(s == hump_state) {
				flip_win = 0;
			} else if(++flip_win >= HUMP_FLIP_WIN) {
				flip_win = 0;
				hump_state = s;
				tm_printf((UB *)"[imu] hump: %s (tilt %u mg,"
					       " up %d mm)\n", hump_name[s],
					  tilt, h_um / 1000);
			}
		}
		return;
	}
	flip_win = 0;

	/* Back down to where it started (driven over, or backed off). */
	if(h_um < HUMP_GROUND_UM && h_um > -HUMP_GROUND_UM
	   && (peak_um >= HUMP_MIN_MM * 1000
	       || -trough_um >= HUMP_MIN_MM * 1000)) {
		hump_finish();
		return;
	}
	/* Level: over after HUMP_END_MM, but only once on the way down.
	   Level on top is still up there: the practice ramp's top is longer
	   than HUMP_END_MM, the hump ended on it, and driving down the far
	   side counted as a second climb. */
	if(tilt < HUMP_EXIT_MG) {
		level_mm += ad;
		if((hump_state == HUMP_DESCENDING && level_mm >= HUMP_END_MM)
		   || level_mm >= HUMP_TOP_MAX_MM) {
			hump_finish();
		}
	}
}
