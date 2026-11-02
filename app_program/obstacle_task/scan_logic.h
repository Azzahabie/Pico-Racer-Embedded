/*
 *	scan_logic.h
 *	Buddy 5 obstacle geometry and planning. PURE C: no RTOS, servo or GPIO
 *	calls, so it can be tested on a PC with recorded scans
 *	(tests/scan_logic_test.c, NFR-16).
 *
 *	Design (Statechart 5, Tribunal 1 interface row "scan_logic.c"):
 *	  in : arrays of (angle, distance), [D_obs], [C_pass], [D_rev],
 *	       beam correction
 *	  out: obstacle profile (edges, position, width W, closest point d_c,
 *	       left/right clearance) and the chosen action
 *	       (continue / bypass left / bypass right / reverse / blocked).
 *	  Also: the "N readings in a row" counters used by Monitoring,
 *	  Blocked and Sensor Lost, so those rules are testable on a PC.
 *
 *	Angle convention: negative = car's LEFT, positive = car's RIGHT, 0 = ahead.
 *	Units: degrees and millimetres, integers only (no FPU on the RP2040).
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	fixed-width integers (stdint.h). Tabs/brace placement follow the repo.
 */

#ifndef SCAN_LOGIC_H
#define SCAN_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

/* ---- grid: -45..+45 deg in 5 deg steps = 19 slots ---- */
#define SL_STEP_DEG		5
#define SL_LIMIT_DEG		45
#define SL_SLOTS		((2 * SL_LIMIT_DEG / SL_STEP_DEG) + 1)
#define SL_IDX(deg)		(((deg) + SL_LIMIT_DEG) / SL_STEP_DEG)
#define SL_DEG(idx)		(((idx) * SL_STEP_DEG) - SL_LIMIT_DEG)

/* ---- grid values ---- */
#define SL_NOT_MEASURED		0xFFFFU	/* this angle was not pinged     */
#define SL_FAR			0U	/* out of range / no echo: clear */

/* ---- thresholds: proposed values, set from A5 / NFR-08 test data ---- */
#define SL_D_STOP_MM		250	/* [D_stop] Monitoring stops below this (NFR-08) */
#define SL_D_OBS_MM		400	/* [D_obs]  a scan reading <= this is "obstacle" */
#define SL_D_REV_MM		100	/* [D_rev]  closer than this: too close to turn  */
/* Car geometry, measured 1 Nov 2026 (S7 attempt 1): 125 mm wheel to
 * wheel, axle to front 155 mm, axle to back 70 mm. */
#define SL_CAR_HALF_W_MM	63	/* half of 125 mm, rounded up                    */
#define SL_AXLE_FRONT_MM	155
#define SL_AXLE_BACK_MM		70
#define SL_MARGIN_MM		50	/* safety gap past the edge [TBD]                */

/* A pivot on the spot (the bypass legs, the line search) sweeps the front
 * corners through sqrt(155^2 + 63^2) = 167.3 -> 168 mm (calculated). S7 attempt 1
 * hit the box during pivots with C_pass = half width + margin (120 mm). */
#define SL_PIVOT_R_MM		168

/* [C_pass], Statechart 5: the clearance needed to pass. The bypass pivots
 * beside the obstacle, so it is the pivot sweep + the safety margin, not
 * half the width + margin (Integration Rules section 9, D12). The offset
 * sent to motion_task is the edge + C_pass, so the car's axle runs at
 * least C_pass from the obstacle's edge. */
#define SL_C_PASS_MM		(SL_PIVOT_R_MM + SL_MARGIN_MM)

/* Half-width of the car's path corridor: an obstacle wholly outside
 * +/-SL_CORRIDOR_MM is not in the way (FR-17 "continue"). The car drives
 * straight there, so half the width + the margin is enough. */
#define SL_CORRIDOR_MM		(SL_CAR_HALF_W_MM + SL_MARGIN_MM)

/* Longest sideways move motion_task will drive for a bypass. Sized for
 * the 200 mm box the team plans for (1 Nov): centred, its far edge is
 * 100 mm out, so the offset is 100 + C_pass = 318 mm; 400 leaves 82 mm
 * for a box placed off centre. Must fit the 9-bit command (511). */
#define SL_MAX_OFFSET_MM	400

/* Avoiding (Statechart 5): a re-plan needs SL_N_STOP readings below
 * D_stop, as in Monitoring, but only while the car moves along its
 * heading. While it is still or pivoting on the spot (the bypass turns
 * and the line search), the sensor sweeps across the obstacle it is
 * going round, so only a reading below D_rev, the too-close-to-turn
 * distance, re-plans (Integration Rules section 9, D13). "Not moving" =
 * distance_state changed by at most SL_PIVOT_MOVE_MM since the last
 * ping: a pivot averages to 0, a straight leg at the bypass speed moves
 * about 5 mm per 60 ms ping (calculated from S7 attempt 1, ~85 mm/s). */
#define SL_PIVOT_MOVE_MM	2

#define SL_REV_DIST_MM		150	/* reverse distance; the rescan uses D_obs + this */
#define SL_DEPTH_JUMP_MM	150	/* hits further apart than this in depth are two
					   different objects, not one */
#define SL_CLEAR_OPEN_MM	9999	/* clearance when no other object is seen */

/* Beam spread: the HC-SR04 cone makes edges read too far out, by about
 * d * k. Theory (+/-7.5 deg cone) gave k = 0.132; bench value 0.080
 * (29 Oct, NFR-20, boxes centred at ~20 cm, mode 3): 0.132 read a 200 mm
 * box 25-39 % narrow (1 of 3 within 30 %, and a narrow width shortens the
 * bypass offset); 0.080 passed both the 100 mm box (4/5) and the 200 mm
 * box (3/3). The 5-deg steps already land inside the true edge, which is
 * why the full theoretical correction is too much. */
#define SL_BEAM_CORR_PER_1000	80

/* ---- "N in a row" rules (Statechart 5) ---- */
#define SL_N_STOP		2	/* close readings to set STOP_SCAN           */
#define SL_N_STOP_HUMP		4	/* ... while hump_state is not NONE (8.3)    */
#define SL_N_CLEAR		3	/* clear readings to leave Blocked           */
#define SL_N_NO_ECHO		3	/* no-echo pings to enter Sensor Lost        */
#define SL_N_ECHO		3	/* valid echoes to leave Sensor Lost         */

typedef enum {
	SL_CONTINUE = 0,	/* false alarm, or obstacle not in the path */
	SL_BYPASS_LEFT,
	SL_BYPASS_RIGHT,
	SL_REVERSE,		/* too close to turn: back off, then rescan */
	SL_BLOCKED		/* no side is passable: stop                */
} sl_action_t;

/* Minimum-bit sizes: every distance and offset is within -4000..+9999 mm
 * (SL_CLEAR_OPEN_MM is the largest) and every angle within -45..+45, so
 * int16_t; the arithmetic is done in int32_t and stored. Sizes before
 * and after: docs/obstacle/week13_representation.md. */
_Static_assert(SL_CLEAR_OPEN_MM <= INT16_MAX, "profile fields must fit int16_t");

typedef struct {
	int16_t	d_close_mm;		/* closest reading d_c                    */
	int16_t	close_deg;		/* angle of d_c                           */
	int16_t	left_deg;		/* outermost hit on the left              */
	int16_t	right_deg;		/* outermost hit on the right             */
	int16_t	x_left_mm;		/* lateral position of left edge          */
	int16_t	x_right_mm;		/* lateral position of right edge         */
	int16_t	width_mm;		/* W, beam-corrected                      */
	int16_t	offset_left_mm;		/* sideways move to pass left             */
	int16_t	offset_right_mm;	/* sideways move to pass right            */
	int16_t	clear_left_mm;		/* free space beyond the left edge        */
	int16_t	clear_right_mm;		/* free space beyond the right edge       */
	bool	found;			/* any reading <= D_obs                   */
	bool	left_open;		/* free space found left of the edge      */
	bool	right_open;		/* free space found right of the edge     */
	bool	in_path;		/* overlaps the car's corridor +/-C_pass  */
} sl_profile_t;

/* ---- one forward ping, classified for the counters ---- */
typedef enum {
	SL_PING_CLEAR = 0,	/* valid echo at or beyond D_stop, or out of range */
	SL_PING_CLOSE,		/* valid echo below D_stop                         */
	SL_PING_NO_ECHO		/* no echo pulse at all: possible sensor fault     */
} sl_ping_t;

/* The four "in a row" counters packed into 9 bits (minimum-bit
 * requirement). Each saturates at its field maximum, which is at least
 * the largest threshold it is compared with, so ">= N" still works:
 *   bits 0-2 close   0-7  (SL_N_STOP_HUMP = 4)
 *   bits 3-4 clear   0-3  (SL_N_CLEAR = 3)
 *   bits 5-6 no_echo 0-3  (SL_N_NO_ECHO = 3)
 *   bits 7-8 echo    0-3  (SL_N_ECHO = 3)
 * Owned by one task; read it only through the sl_runs_* accessors. */
typedef struct {
	uint16_t	packed;
} sl_runs_t;

/* sin(deg) * 1000 for deg in -45..+45 (5 deg steps). */
int32_t		sl_sin1000(int32_t deg);

/* Build the profile from the grid. A reading counts as obstacle when it
 * is <= d_obs_mm (SL_D_OBS_MM, or SL_D_OBS_MM + SL_REV_DIST_MM right after
 * a reverse). Unmeasured slots between hits count as part of the same
 * obstacle; a depth jump > SL_DEPTH_JUMP_MM starts a different object. */
void		sl_profile(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
			   sl_profile_t *p_prof);

/* Where the fine scan starts (FR-15), from the coarse grid: the coarse
 * edges of the object in the car's path, not the outermost coarse hits.
 * With other objects either side, the outermost hits are those objects,
 * and fine-stepping outward from them never measures the edges of the
 * one in the way (its width then reads 0 and the bypass offset is too
 * short). Returns false if there is no hit. */
bool		sl_fine_starts(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
			       int32_t *p_left_deg, int32_t *p_right_deg);

/* Fine scan stepping outward: is this reading still the same object as
 * the previous hit (a hit, with no depth jump > SL_DEPTH_JUMP_MM)? */
bool		sl_same_object(uint16_t prev_mm, uint16_t dist_mm, int32_t d_obs_mm);

/* Choose the action from a profile (FR-17, NFR-07: larger clearance side). */
sl_action_t	sl_decide(const sl_profile_t *p_prof, bool reverse_tried);

/* The tried bypass side's lane was blocked (D14): the other side if this
 * profile says it is passable, else SL_BLOCKED. */
sl_action_t	sl_other_side(const sl_profile_t *p_prof, sl_action_t tried);

const char	*sl_action_name(sl_action_t action);

/* Counters for Monitoring / Blocked / Sensor Lost. sl_classify() uses
 * D_stop; sl_classify_at() a given stop distance (Avoiding). */
sl_ping_t	sl_classify(bool no_echo, uint32_t dist_mm);
sl_ping_t	sl_classify_at(bool no_echo, uint32_t dist_mm, int32_t stop_mm);

/* Avoiding: the stop distance for a ping, from how far the car moved
 * (distance_state, mm, signed) since the previous ping. */
int32_t		sl_avoid_stop_mm(int32_t moved_mm);

/* Bypass probe (D14): a side reading (grid value: SL_FAR, SL_NOT_MEASURED
 * or mm) as the AVOID_LOOK distance: 1 = no echo (unknown, counts as
 * still there), MOTION_AVOID_DIST_MAX (511) = nothing in range. */
#define SL_LOOK_UNKNOWN_MM	1U
#define SL_LOOK_FAR_MM		511U
uint16_t	sl_look_reply_mm(uint16_t grid_mm);
void		sl_runs_reset(sl_runs_t *p_runs);
void		sl_runs_add(sl_runs_t *p_runs, sl_ping_t ping);
uint8_t		sl_runs_close(const sl_runs_t *p_runs);
uint8_t		sl_runs_clear(const sl_runs_t *p_runs);
uint8_t		sl_runs_no_echo(const sl_runs_t *p_runs);
uint8_t		sl_runs_echo(const sl_runs_t *p_runs);

#endif /* SCAN_LOGIC_H */
