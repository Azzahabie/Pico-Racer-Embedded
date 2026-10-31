/*
 *	imu_events.c
 *	Plain-language console events for imu_task (Buddy 4): turn direction
 *	and angle, turn counts, picked up / put down, lean and nose.
 *
 *	No I2C or kernel calls, so tests/imu_safety runs it on a PC.
 */

#include <tk/tkernel.h>

#include "imu_events.h"

#define ABS(v)		(((v) < 0) ? -(v) : (v))
#define SIGN(v)		(((v) > 0) ? 1 : ((v) < 0) ? -1 : 0)

/* Turns. */
LOCAL BOOL	in_turn;
LOCAL W		turn_dir;
LOCAL W		acc_mdeg;		/* this turn so far, milli-degrees   */
LOCAL UW	acc_ms;
LOCAL W		last_deg;
LOCAL UW	last_ms;
LOCAL UW	n_left, n_right;
LOCAL W		head_mdeg;		/* since mission start               */

/* Lifted by hand. */
LOCAL BOOL	held;
LOCAL BOOL	have_prev;
LOCAL W		prev[3];		/* last window                       */
LOCAL W		rest[3];		/* last window resting on the floor  */
LOCAL INT	jump_n, quiet_n;

/* Lean (side to side) and nose (front to back). */
LOCAL W		lean, nose;		/* reported */
LOCAL W		lean_cand, nose_cand;
LOCAL INT	lean_n, nose_n;
LOCAL W		lean_mg, nose_mg;

EXPORT void ev_mission_reset(void)
{
	n_left = n_right = 0;
	head_mdeg = 0;
}

EXPORT void ev_reset(void)
{
	in_turn = FALSE;
	turn_dir = 0;
	acc_mdeg = 0;
	acc_ms = 0;
	last_deg = 0;
	last_ms = 0;
	ev_mission_reset();
	held = FALSE;
	have_prev = FALSE;
	jump_n = quiet_n = 0;
	lean = nose = lean_cand = nose_cand = 0;
	lean_n = nose_n = 0;
	lean_mg = nose_mg = 0;
}

/* The angle is summed from the moment the rate first looks like a turn,
   not from when the classifier confirms it 100 ms later, so a 90 deg
   turn reads 90, not 88.  The count goes by which way the turn ended up,
   so a wobble at the start cannot put a left turn in the right column. */
EXPORT UINT ev_turn_step(BOOL turning, W turn_dps, W sample_ms)
{
	UINT	ev = 0U;
	W	deg;

	head_mdeg += turn_dps * sample_ms;

	if(turning || ABS(turn_dps) > EV_TURN_PRE_DPS) {
		acc_mdeg += turn_dps * sample_ms;
		acc_ms += (UW)sample_ms;
	} else if(!in_turn) {
		acc_mdeg = 0;
		acc_ms = 0;
	}

	if(turning && !in_turn) {
		in_turn = TRUE;
		turn_dir = SIGN(acc_mdeg != 0 ? acc_mdeg : turn_dps);
		ev |= EV_TURN_START;
	} else if(!turning && in_turn) {
		in_turn = FALSE;
		deg = acc_mdeg / 1000;
		last_deg = deg;
		last_ms = acc_ms;
		turn_dir = SIGN(deg);
		if(ABS(deg) < EV_TURN_MIN_DEG) {
			ev |= EV_TURN_SMALL;
		} else {
			if(deg > 0) n_left++; else n_right++;
			ev |= EV_TURN_DONE;
		}
		acc_mdeg = 0;
		acc_ms = 0;
	}
	return ev;
}

/* One debounced three-way state (+1 / 0 / -1) with hysteresis. */
LOCAL BOOL dir_step(W mg, W *cur, W *cand, INT *n)
{
	W	raw;

	if(ABS(mg) > EV_LEAN_ON_MG) {
		raw = SIGN(mg);
	} else if(*cur != 0 && ABS(mg) > EV_LEAN_OFF_MG && SIGN(mg) == *cur) {
		raw = *cur;
	} else {
		raw = 0;
	}
	if(raw == *cur) {
		*n = 0;
		return FALSE;
	}
	if(raw == *cand) {
		(*n)++;
	} else {
		*cand = raw;
		*n = 1;
	}
	if(*n < EV_LEAN_WIN) return FALSE;
	*cur = raw;
	*n = 0;
	return TRUE;
}

/* Picked up: the wheels stand still but the 100 ms mean stays away from
   where the car rested (lifting, carrying, a hand's tremor).  Put down:
   the wheels roll, or it sits steady for a second.  Standing on the
   floor, the means of a still car agree within a few mg. */
EXPORT UINT ev_window_step(W mx, W my, W mz, W dmm)
{
	UINT	ev = 0U;
	W	a[3], d, jump = 0;
	INT	i;

	a[0] = mx; a[1] = my; a[2] = mz;

	lean_mg = IMU_LEFT_SIGN * a[IMU_LEFT_AXIS];
	nose_mg = IMU_NOSE_SIGN * a[IMU_NOSE_AXIS];

	/* On the floor: away from where it last rested.  In the hand: away
	   from the last window. */
	if(have_prev) {
		for(i = 0; i < 3; i++) {
			d = ABS(a[i] - (held ? prev[i] : rest[i]));
			if(d > jump) jump = d;
		}
	}
	for(i = 0; i < 3; i++) prev[i] = a[i];

	if(dmm != 0 || !have_prev) {
		jump_n = 0;
		quiet_n = 0;
		for(i = 0; i < 3; i++) rest[i] = a[i];
		if(held) {
			held = FALSE;
			ev |= EV_PUT_DOWN;
		}
	} else if(!held) {
		/* A knock springs back within a window, so it never holds
		   two in a row away from rest. */
		if(jump > EV_HAND_ON_MG) {
			jump_n++;
		} else {
			jump_n = 0;
			for(i = 0; i < 3; i++) rest[i] = a[i];
		}
		if(jump_n >= EV_HAND_ON_WIN) {
			held = TRUE;
			jump_n = 0;
			quiet_n = 0;
			ev |= EV_PICKED_UP;
		}
	} else {
		quiet_n = (jump < EV_HAND_OFF_MG) ? quiet_n + 1 : 0;
		if(quiet_n >= EV_HAND_OFF_WIN) {
			held = FALSE;
			quiet_n = 0;
			for(i = 0; i < 3; i++) rest[i] = a[i];
			ev |= EV_PUT_DOWN;
		}
	}
	have_prev = TRUE;

	/* In the hand the angles mean nothing to the car. */
	if(!held) {
		if(dir_step(lean_mg, &lean, &lean_cand, &lean_n)) ev |= EV_LEAN;
		if(dir_step(nose_mg, &nose, &nose_cand, &nose_n)) ev |= EV_NOSE;
	}
	return ev;
}

EXPORT W    ev_turn_dir(void)	 { return turn_dir; }
EXPORT W    ev_turn_deg(void)	 { return last_deg; }
EXPORT UW   ev_turn_ms(void)	 { return last_ms; }
EXPORT UW   ev_turns_left(void)	 { return n_left; }
EXPORT UW   ev_turns_right(void) { return n_right; }
EXPORT W    ev_heading_deg(void) { return head_mdeg / 1000; }
EXPORT BOOL ev_held(void)	 { return held; }
EXPORT W    ev_lean(void)	 { return lean; }
EXPORT W    ev_nose(void)	 { return nose; }
EXPORT W    ev_lean_mg(void)	 { return lean_mg; }
EXPORT W    ev_nose_mg(void)	 { return nose_mg; }

EXPORT const char *ev_side_name(W s)
{
	return (s > 0) ? "LEFT" : (s < 0) ? "RIGHT" : "";
}

EXPORT const char *ev_nose_name(W s)
{
	return (s > 0) ? "UP" : (s < 0) ? "DOWN" : "";
}
