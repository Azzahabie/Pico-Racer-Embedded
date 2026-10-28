/*
 *	scan_logic.c
 *	Buddy 5 obstacle geometry and planning. Pure C, see scan_logic.h.
 *
 *	Worked example (bench, object ~19 cm ahead, coarse 15-deg data only):
 *	  hits at -30 (206 mm) ... +15 (193 mm), misses at -45 and +30
 *	  x_left  = 206 * sin(-30) = -103 mm, beam-corrected inward +27 -> -76
 *	  x_right = 193 * sin(+15) =  +49 mm, beam-corrected inward -25 -> +24
 *	  W = 24 - (-76) = 100 mm   (tests/scan_logic_test.c, Test 1)
 *	  Both sides open, nothing else seen -> both clearances "open";
 *	  offset L = 76 + 120 = 196, R = 24 + 120 = 144 -> BYPASS RIGHT.
 *	With only coarse data the true edges lie further out than the last hit,
 *	so W is underestimated; the fine scan closes that gap to 5 deg.
 */

#include <stdbool.h>
#include "scan_logic.h"

/* sin(deg) * 1000, deg = -45, -40, ..., +45 */
static const int16_t g_sin1000_tbl[SL_SLOTS] = {
	-707, -643, -574, -500, -423, -342, -259, -174, -87, 0,
	  87,  174,  259,  342,  423,  500,  574,  643,  707
};

static int32_t abs_val(int32_t value)
{
	return (value < 0) ? -value : value;
}

int32_t sl_sin1000(int32_t deg)
{
	if(deg < -SL_LIMIT_DEG) {
		deg = -SL_LIMIT_DEG;
	}
	if(deg > SL_LIMIT_DEG) {
		deg = SL_LIMIT_DEG;
	}
	/* round to the nearest 5 deg slot */
	if(deg >= 0) {
		deg = ((deg + 2) / SL_STEP_DEG) * SL_STEP_DEG;
	} else {
		deg = -(((-deg + 2) / SL_STEP_DEG) * SL_STEP_DEG);
	}
	return g_sin1000_tbl[SL_IDX(deg)];
}

static bool is_hit(uint16_t dist_mm, int32_t d_obs_mm)
{
	return (SL_NOT_MEASURED != dist_mm) && (SL_FAR != dist_mm) &&
	       ((int32_t)dist_mm <= d_obs_mm);
}

static bool is_miss(uint16_t dist_mm, int32_t d_obs_mm)
{
	return (SL_NOT_MEASURED != dist_mm) && !is_hit(dist_mm, d_obs_mm);
}

/* lateral position of a reading: x = d * sin(theta) */
static int32_t lateral_mm(int32_t dist_mm, int32_t idx)
{
	return (dist_mm * sl_sin1000(SL_DEG(idx))) / 1000;
}

static int32_t beam_corr_mm(int32_t dist_mm)
{
	return (dist_mm * SL_BEAM_CORR_PER_1000) / 1000;
}

/* Closest hit. Hits inside the car's corridor (|d sin theta| <= C_pass)
 * win over hits outside it, so an object off to the side cannot hide one
 * that is straight ahead. On a tie prefer the angle nearest to 0.
 * Returns -1 if there is no hit. */
static int32_t find_closest(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm)
{
	int32_t	idx;
	int32_t	best = -1;
	bool	best_in = false;
	bool	in_corr;
	bool	better;

	for(idx = 0; idx < SL_SLOTS; idx++) {
		if(is_hit(grid[idx], d_obs_mm)) {
			in_corr = (abs_val(lateral_mm((int32_t)grid[idx], idx)) <= SL_C_PASS_MM);
			if(-1 == best) {
				better = true;
			} else if(in_corr != best_in) {
				better = in_corr;
			} else if(grid[idx] != grid[best]) {
				better = (grid[idx] < grid[best]);
			} else {
				better = (abs_val(SL_DEG(idx)) < abs_val(SL_DEG(best)));
			}
			if(better) {
				best = idx;
				best_in = in_corr;
			}
		}
	}
	return best;
}

/* Grow the obstacle from close_idx in direction dir (-1 left, +1 right).
 * Stops at the first measured miss, or at a hit whose depth jumps by more
 * than SL_DEPTH_JUMP_MM (a different object). Unmeasured slots in between
 * count as part of the obstacle (the fine scan only measures near edges).
 * *p_edge = outermost hit of this obstacle; *p_stop = slot where growth
 * stopped (-1 or SL_SLOTS if it ran off the scan). */
static void grow(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
		 int32_t close_idx, int32_t dir, int32_t *p_edge, int32_t *p_stop)
{
	int32_t	idx = close_idx + dir;
	int32_t	prev_mm = (int32_t)grid[close_idx];
	bool	stopped = false;

	*p_edge = close_idx;
	while(!stopped && (idx >= 0) && (idx < SL_SLOTS)) {
		if(is_miss(grid[idx], d_obs_mm)) {
			stopped = true;
		} else if(is_hit(grid[idx], d_obs_mm)) {
			if(abs_val((int32_t)grid[idx] - prev_mm) > SL_DEPTH_JUMP_MM) {
				stopped = true;		/* another object */
			} else {
				*p_edge = idx;
				prev_mm = (int32_t)grid[idx];
				idx += dir;
			}
		} else {
			idx += dir;			/* not measured */
		}
	}
	*p_stop = idx;
}

/* Free space beyond an edge (FR-16): lateral gap from the edge to the
 * nearest other object seen on that side, or SL_CLEAR_OPEN_MM if none. */
static int32_t clearance(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
			 int32_t stop_idx, int32_t dir, int32_t x_edge_mm)
{
	int32_t	idx = stop_idx;
	int32_t	result = SL_CLEAR_OPEN_MM;
	int32_t	x_near;
	bool	seen = false;

	while(!seen && (idx >= 0) && (idx < SL_SLOTS)) {
		if(is_hit(grid[idx], d_obs_mm)) {
			seen = true;
			/* the neighbour's facing edge, beam-corrected towards itself */
			x_near = lateral_mm((int32_t)grid[idx], idx) +
				 (dir * beam_corr_mm((int32_t)grid[idx]));
			result = (x_near - x_edge_mm) * dir;
			if(result < 0) {
				result = 0;
			}
		} else {
			idx += dir;
		}
	}
	return result;
}

void sl_profile(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
		sl_profile_t *p_prof)
{
	int32_t close_idx;
	int32_t left_idx;
	int32_t right_idx;
	int32_t left_stop;
	int32_t right_stop;
	int32_t idx;

	p_prof->found = false;
	p_prof->d_close_mm = 0;
	p_prof->close_deg = 0;
	p_prof->left_deg = 0;
	p_prof->right_deg = 0;
	p_prof->x_left_mm = 0;
	p_prof->x_right_mm = 0;
	p_prof->width_mm = 0;
	p_prof->left_open = false;
	p_prof->right_open = false;
	p_prof->in_path = false;
	p_prof->offset_left_mm = 0;
	p_prof->offset_right_mm = 0;
	p_prof->clear_left_mm = 0;
	p_prof->clear_right_mm = 0;

	close_idx = find_closest(grid, d_obs_mm);
	if(-1 == close_idx) {
		return;				/* nothing within D_obs */
	}

	p_prof->found = true;
	p_prof->d_close_mm = (int32_t)grid[close_idx];
	p_prof->close_deg = SL_DEG(close_idx);

	grow(grid, d_obs_mm, close_idx, -1, &left_idx, &left_stop);
	grow(grid, d_obs_mm, close_idx, +1, &right_idx, &right_stop);
	p_prof->left_open  = (left_stop >= 0);		/* stopped before -45 */
	p_prof->right_open = (right_stop < SL_SLOTS);	/* stopped before +45 */

	p_prof->left_deg  = SL_DEG(left_idx);
	p_prof->right_deg = SL_DEG(right_idx);

	/* closest point d_c of THIS object (FR-16): the search above started
	 * from the closest in-corridor hit, so re-check the whole object */
	for(idx = left_idx; idx <= right_idx; idx++) {
		if(is_hit(grid[idx], d_obs_mm) &&
		   (((int32_t)grid[idx] < p_prof->d_close_mm) ||
		    (((int32_t)grid[idx] == p_prof->d_close_mm) &&
		     (abs_val(SL_DEG(idx)) < abs_val(p_prof->close_deg))))) {
			p_prof->d_close_mm = (int32_t)grid[idx];
			p_prof->close_deg = SL_DEG(idx);
		}
	}

	/* lateral edge positions x = d * sin(theta), then beam correction:
	 * the cone makes each edge look ~d*tan(7.5) too far out, so pull
	 * both edges inward */
	p_prof->x_left_mm  = lateral_mm((int32_t)grid[left_idx], left_idx) +
			     beam_corr_mm((int32_t)grid[left_idx]);
	p_prof->x_right_mm = lateral_mm((int32_t)grid[right_idx], right_idx) -
			     beam_corr_mm((int32_t)grid[right_idx]);

	p_prof->width_mm = p_prof->x_right_mm - p_prof->x_left_mm;
	if(p_prof->width_mm < 0) {
		p_prof->width_mm = 0;		/* tiny object: cone wider */
	}

	/* in the way if it overlaps the corridor -C_pass..+C_pass */
	p_prof->in_path = (p_prof->x_left_mm <= SL_C_PASS_MM) &&
			  (p_prof->x_right_mm >= -SL_C_PASS_MM);

	/* sideways move needed so the car's near side clears the edge:
	 * pass left  -> car centre must reach x_left  - C_pass
	 * pass right -> car centre must reach x_right + C_pass */
	p_prof->offset_left_mm  = -(p_prof->x_left_mm - SL_C_PASS_MM);
	p_prof->offset_right_mm =   p_prof->x_right_mm + SL_C_PASS_MM;
	if(p_prof->offset_left_mm < 0) {
		p_prof->offset_left_mm = 0;
	}
	if(p_prof->offset_right_mm < 0) {
		p_prof->offset_right_mm = 0;
	}

	/* free space beyond each edge; a side that runs off the scan has none */
	p_prof->clear_left_mm = p_prof->left_open ?
		clearance(grid, d_obs_mm, left_stop, -1, p_prof->x_left_mm) : 0;
	p_prof->clear_right_mm = p_prof->right_open ?
		clearance(grid, d_obs_mm, right_stop, +1, p_prof->x_right_mm) : 0;
}

bool sl_fine_starts(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
		    int32_t *p_left_deg, int32_t *p_right_deg)
{
	sl_profile_t prof;

	/* the same object rule sl_profile() uses, applied to the coarse grid */
	sl_profile(grid, d_obs_mm, &prof);
	if(prof.found) {
		*p_left_deg = prof.left_deg;
		*p_right_deg = prof.right_deg;
	}
	return prof.found;
}

bool sl_same_object(uint16_t prev_mm, uint16_t dist_mm, int32_t d_obs_mm)
{
	return is_hit(dist_mm, d_obs_mm) &&
	       (abs_val((int32_t)dist_mm - (int32_t)prev_mm) <= SL_DEPTH_JUMP_MM);
}

sl_action_t sl_decide(const sl_profile_t *p_prof, bool reverse_tried)
{
	bool		ok_left;
	bool		ok_right;
	sl_action_t	action;

	if(!p_prof->found || !p_prof->in_path) {
		action = SL_CONTINUE;			/* false alarm / not in the way */
	} else if(p_prof->d_close_mm < SL_D_REV_MM) {
		action = reverse_tried ? SL_BLOCKED : SL_REVERSE;
	} else {
		ok_left  = p_prof->left_open &&
			   (p_prof->clear_left_mm >= SL_C_PASS_MM) &&
			   (p_prof->offset_left_mm <= SL_MAX_OFFSET_MM);
		ok_right = p_prof->right_open &&
			   (p_prof->clear_right_mm >= SL_C_PASS_MM) &&
			   (p_prof->offset_right_mm <= SL_MAX_OFFSET_MM);

		if(ok_left && ok_right) {
			/* NFR-07: the side with the larger clearance; on a tie the
			 * shorter sideways move */
			if(p_prof->clear_left_mm != p_prof->clear_right_mm) {
				action = (p_prof->clear_left_mm > p_prof->clear_right_mm)
					? SL_BYPASS_LEFT : SL_BYPASS_RIGHT;
			} else {
				action = (p_prof->offset_left_mm <= p_prof->offset_right_mm)
					? SL_BYPASS_LEFT : SL_BYPASS_RIGHT;
			}
		} else if(ok_left) {
			action = SL_BYPASS_LEFT;
		} else if(ok_right) {
			action = SL_BYPASS_RIGHT;
		} else {
			action = SL_BLOCKED;
		}
	}
	return action;
}

const char *sl_action_name(sl_action_t action)
{
	const char *p_name;

	switch(action) {
	case SL_CONTINUE:	p_name = "CONTINUE";		break;
	case SL_BYPASS_LEFT:	p_name = "BYPASS LEFT";		break;
	case SL_BYPASS_RIGHT:	p_name = "BYPASS RIGHT";	break;
	case SL_REVERSE:	p_name = "REVERSE";		break;
	case SL_BLOCKED:	p_name = "BLOCKED";		break;
	default:		p_name = "?";			break;
	}
	return p_name;
}

/* ------------------------------------------------------------------ */
/* "N readings in a row" counters                                     */
/* ------------------------------------------------------------------ */
sl_ping_t sl_classify(bool no_echo, uint32_t dist_mm)
{
	sl_ping_t ping;

	if(no_echo) {
		ping = SL_PING_NO_ECHO;
	} else if((SL_FAR != dist_mm) && (dist_mm < (uint32_t)SL_D_STOP_MM)) {
		ping = SL_PING_CLOSE;
	} else {
		ping = SL_PING_CLEAR;		/* includes out of range */
	}
	return ping;
}

/* Field positions in sl_runs_t.packed (see scan_logic.h). */
#define RUN_CLOSE_SHIFT		0U
#define RUN_CLOSE_MAX		7U
#define RUN_CLEAR_SHIFT		3U
#define RUN_CLEAR_MAX		3U
#define RUN_NO_ECHO_SHIFT	5U
#define RUN_NO_ECHO_MAX		3U
#define RUN_ECHO_SHIFT		7U
#define RUN_ECHO_MAX		3U

_Static_assert((SL_N_STOP <= RUN_CLOSE_MAX) && (SL_N_STOP_HUMP <= RUN_CLOSE_MAX) &&
	       (SL_N_CLEAR <= RUN_CLEAR_MAX) && (SL_N_NO_ECHO <= RUN_NO_ECHO_MAX) &&
	       (SL_N_ECHO <= RUN_ECHO_MAX), "a run counter field is too narrow");

static uint8_t run_get(const sl_runs_t *p_runs, uint32_t shift, uint32_t max)
{
	return (uint8_t)((p_runs->packed >> shift) & max);
}

/* +1, saturating at the field maximum */
static void run_inc(sl_runs_t *p_runs, uint32_t shift, uint32_t max)
{
	uint32_t count = run_get(p_runs, shift, max);

	if(count < max) {
		count++;
	}
	p_runs->packed = (uint16_t)((p_runs->packed & ~(max << shift)) | (count << shift));
}

static void run_zero(sl_runs_t *p_runs, uint32_t shift, uint32_t max)
{
	p_runs->packed = (uint16_t)(p_runs->packed & ~(max << shift));
}

void sl_runs_reset(sl_runs_t *p_runs)
{
	p_runs->packed = 0U;
}

void sl_runs_add(sl_runs_t *p_runs, sl_ping_t ping)
{
	if(SL_PING_NO_ECHO == ping) {
		run_inc(p_runs, RUN_NO_ECHO_SHIFT, RUN_NO_ECHO_MAX);
		run_zero(p_runs, RUN_ECHO_SHIFT, RUN_ECHO_MAX);
		run_zero(p_runs, RUN_CLOSE_SHIFT, RUN_CLOSE_MAX);	/* a gap breaks "in a row" */
		run_zero(p_runs, RUN_CLEAR_SHIFT, RUN_CLEAR_MAX);
	} else {
		run_inc(p_runs, RUN_ECHO_SHIFT, RUN_ECHO_MAX);
		run_zero(p_runs, RUN_NO_ECHO_SHIFT, RUN_NO_ECHO_MAX);
		if(SL_PING_CLOSE == ping) {
			run_inc(p_runs, RUN_CLOSE_SHIFT, RUN_CLOSE_MAX);
			run_zero(p_runs, RUN_CLEAR_SHIFT, RUN_CLEAR_MAX);
		} else {
			run_inc(p_runs, RUN_CLEAR_SHIFT, RUN_CLEAR_MAX);
			run_zero(p_runs, RUN_CLOSE_SHIFT, RUN_CLOSE_MAX);
		}
	}
}

uint8_t sl_runs_close(const sl_runs_t *p_runs)
{
	return run_get(p_runs, RUN_CLOSE_SHIFT, RUN_CLOSE_MAX);
}

uint8_t sl_runs_clear(const sl_runs_t *p_runs)
{
	return run_get(p_runs, RUN_CLEAR_SHIFT, RUN_CLEAR_MAX);
}

uint8_t sl_runs_no_echo(const sl_runs_t *p_runs)
{
	return run_get(p_runs, RUN_NO_ECHO_SHIFT, RUN_NO_ECHO_MAX);
}

uint8_t sl_runs_echo(const sl_runs_t *p_runs)
{
	return run_get(p_runs, RUN_ECHO_SHIFT, RUN_ECHO_MAX);
}
