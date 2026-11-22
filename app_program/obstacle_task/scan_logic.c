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

/* the same, for the reported width only (SL_WIDTH_REPORT_CORR_PER_1000) */
static int32_t report_corr_mm(int32_t dist_mm)
{
	return (dist_mm * SL_WIDTH_REPORT_CORR_PER_1000) / 1000;
}

#if SL_WIDTH_EDGE_MID
/* 1000 x tan(deg), 0..60 deg */
static const int16_t g_tan1000_tbl[61] = {
	0, 17, 35, 52, 70, 87, 105, 123, 141, 158, 176, 194, 213, 231, 249,
	268, 287, 306, 325, 344, 364, 384, 404, 424, 445, 466, 488, 510, 532,
	554, 577, 601, 625, 649, 675, 700, 727, 754, 781, 810, 839, 869, 900,
	933, 966, 1000, 1036, 1072, 1111, 1150, 1192, 1235, 1280, 1327, 1376,
	1428, 1483, 1540, 1600, 1664, 1732
};

/* 1000 x cos(deg), deg = 0, 5 .. 45 */
static const int16_t g_cos1000_5[10] = {
	1000, 996, 985, 966, 940, 906, 866, 819, 766, 707
};

/* 1000 x tan(a2 / 2 deg): half degrees, the halves averaged */
static int32_t tan1000_half(int32_t a2)
{
	int32_t	mag = (a2 < 0) ? -a2 : a2;
	int32_t	lo;
	int32_t	t;

	if(mag > 120) {
		mag = 120;
	}
	lo = mag / 2;
	t = ((mag % 2) != 0)
	    ? (((int32_t)g_tan1000_tbl[lo] + (int32_t)g_tan1000_tbl[(lo < 60) ? (lo + 1) : 60]) / 2)
	    : (int32_t)g_tan1000_tbl[lo];
	return (a2 < 0) ? -t : t;
}

/* 1000 x cos(deg), 0..60 deg */
static const int16_t g_cos1000_1[61] = {
	1000, 1000, 999, 999, 998, 996, 995, 993, 990, 988, 985, 982, 978,
	974, 970, 966, 961, 956, 951, 946, 940, 934, 927, 921, 914, 906, 899,
	891, 883, 875, 866, 857, 848, 839, 829, 819, 809, 799, 788, 777, 766,
	755, 743, 731, 719, 707, 695, 682, 669, 656, 643, 629, 616, 602, 588,
	574, 559, 545, 530, 515, 500
};

int32_t sl_cos1000(int32_t deg)
{
	int32_t	a = (deg < 0) ? -deg : deg;

	return (int32_t)g_cos1000_1[(a > 60) ? 60 : a];
}

/* W from two face edges given as 2 x their angles (see SL_WIDTH_EDGE_MID). */
int32_t sl_width_from_edges(int32_t d_c, int32_t el2, int32_t er2)
{
	int32_t	w = (d_c * (tan1000_half(er2 - (2 * SL_BEAM_HALF_DEG)) -
			    tan1000_half(el2 + (2 * SL_BEAM_HALF_DEG)))) / 1000;

	return (w < 0) ? 0 : w;
}

/* The outermost hit of the face on one side, in degrees (the same walk as
 * face_edge_x2). */
int32_t sl_face_last_deg(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
			 int32_t close_deg, int32_t d_c, int32_t dir)
{
	int32_t	last = SL_IDX(close_deg);
	int32_t	idx = last + dir;

	while((idx >= 0) && (idx < SL_SLOTS)) {
		if(is_hit(grid[idx], d_obs_mm) &&
		   ((((int32_t)grid[idx] * sl_cos1000(SL_DEG(idx))) / 1000)
		    <= (d_c + SL_FACE_DEPTH_MM))) {
			last = idx;
		} else if(SL_NOT_MEASURED != grid[idx]) {
			break;
		} else {
			/* not measured */
		}
		idx += dir;
	}
	return SL_DEG(last);
}

/* One edge of the face, as 2 x its angle: out from close_idx over hits
 * within SL_FACE_DEPTH_MM of d_c (unmeasured slots skipped), then half-way
 * to the next measured slot (a miss or a deeper hit); half a step out if
 * the scan ends first. */
static int32_t face_edge_x2(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
			    int32_t close_idx, int32_t d_c, int32_t dir)
{
	int32_t	last = close_idx;
	int32_t	idx = close_idx + dir;

	while((idx >= 0) && (idx < SL_SLOTS)) {
		/* depth straight ahead (d cos theta): a flat face reads
		   longer toward its edges (car 23:01: 273 mm at 25 deg on a
		   242 mm face), which is not "behind" it */
		if(is_hit(grid[idx], d_obs_mm) &&
		   ((((int32_t)grid[idx] * (int32_t)g_cos1000_5[abs_val(SL_DEG(idx)) / SL_STEP_DEG]) / 1000)
		    <= (d_c + SL_FACE_DEPTH_MM))) {
			last = idx;
		} else if(SL_NOT_MEASURED != grid[idx]) {
			return SL_DEG(last) + SL_DEG(idx);
		} else {
			/* not measured: keep going */
		}
		idx += dir;
	}
	return (2 * SL_DEG(last)) + (dir * SL_STEP_DEG);
}
#endif

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
			in_corr = (abs_val(lateral_mm((int32_t)grid[idx], idx)) <= SL_CORRIDOR_MM);
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
	/* hits are <= d_obs_mm (a few hundred mm) and angles -45..+45, so
	 * every int16_t field below holds its value */
	p_prof->d_close_mm = (int16_t)grid[close_idx];
	p_prof->close_deg = (int16_t)SL_DEG(close_idx);

	grow(grid, d_obs_mm, close_idx, -1, &left_idx, &left_stop);
	grow(grid, d_obs_mm, close_idx, +1, &right_idx, &right_stop);
	p_prof->left_open  = (left_stop >= 0);		/* stopped before -45 */
	p_prof->right_open = (right_stop < SL_SLOTS);	/* stopped before +45 */

	p_prof->left_deg  = (int16_t)SL_DEG(left_idx);
	p_prof->right_deg = (int16_t)SL_DEG(right_idx);

	/* closest point d_c of THIS object (FR-16): the search above started
	 * from the closest in-corridor hit, so re-check the whole object */
	for(idx = left_idx; idx <= right_idx; idx++) {
		if(is_hit(grid[idx], d_obs_mm) &&
		   (((int32_t)grid[idx] < p_prof->d_close_mm) ||
		    (((int32_t)grid[idx] == p_prof->d_close_mm) &&
		     (abs_val(SL_DEG(idx)) < abs_val(p_prof->close_deg))))) {
			p_prof->d_close_mm = (int16_t)grid[idx];
			p_prof->close_deg = (int16_t)SL_DEG(idx);
		}
	}

	/* lateral edge positions x = d * sin(theta), then beam correction:
	 * the cone makes each edge look ~d*tan(7.5) too far out, so pull
	 * both edges inward */
	p_prof->x_left_mm  = (int16_t)(lateral_mm((int32_t)grid[left_idx], left_idx) +
				       beam_corr_mm((int32_t)grid[left_idx]));
	p_prof->x_right_mm = (int16_t)(lateral_mm((int32_t)grid[right_idx], right_idx) -
				       beam_corr_mm((int32_t)grid[right_idx]));

	/* W as reported (FR-16, NFR-20: the [scan] profile line and the E
	 * obstacle record): the same edges, pulled in by
	 * SL_WIDTH_REPORT_CORR_PER_1000 instead.  Nothing plans with W: the
	 * corridor, clearances and offsets below use x_left_mm / x_right_mm. */
	p_prof->width_mm = (int16_t)((lateral_mm((int32_t)grid[right_idx], right_idx) -
				      report_corr_mm((int32_t)grid[right_idx])) -
				     (lateral_mm((int32_t)grid[left_idx], left_idx) +
				      report_corr_mm((int32_t)grid[left_idx])));
#if SL_WIDTH_EDGE_MID
	{
		int32_t	d_c = (int32_t)p_prof->d_close_mm;
		int32_t	c_idx = SL_IDX((int32_t)p_prof->close_deg);
		int32_t	el2 = face_edge_x2(grid, d_obs_mm, c_idx, d_c, -1);
		int32_t	er2 = face_edge_x2(grid, d_obs_mm, c_idx, d_c, +1);

		p_prof->width_mm = (int16_t)((d_c *
			(tan1000_half(er2 - (2 * SL_BEAM_HALF_DEG)) -
			 tan1000_half(el2 + (2 * SL_BEAM_HALF_DEG)))) / 1000);
	}
#endif
	if(p_prof->width_mm < 0) {
		p_prof->width_mm = 0;		/* tiny object: cone wider */
	}

	/* in the way if it overlaps the corridor -SL_CORRIDOR_MM..+SL_CORRIDOR_MM */
	p_prof->in_path = (p_prof->x_left_mm <= SL_CORRIDOR_MM) &&
			  (p_prof->x_right_mm >= -SL_CORRIDOR_MM);

	/* sideways move needed so the car's near side clears the edge:
	 * pass left  -> car centre must reach x_left  - C_pass
	 * pass right -> car centre must reach x_right + C_pass */
#if SL_KNOWN_W_MM > 0
	{
		/* known obstacle: the real width around the scan's centre */
		int32_t	centre = ((int32_t)p_prof->x_left_mm + (int32_t)p_prof->x_right_mm) / 2;

		p_prof->offset_left_mm  = (int16_t)(-(centre - (SL_KNOWN_W_MM / 2)
						      - SL_KNOWN_CLEAR_MM));
		p_prof->offset_right_mm = (int16_t)(centre + (SL_KNOWN_W_MM / 2)
						    + SL_KNOWN_CLEAR_MM);
	}
#else
	p_prof->offset_left_mm  = (int16_t)(-(p_prof->x_left_mm - SL_C_PASS_MM));
	p_prof->offset_right_mm = (int16_t)(p_prof->x_right_mm + SL_C_PASS_MM);
#endif
	if(p_prof->offset_left_mm < 0) {
		p_prof->offset_left_mm = 0;
	}
	if(p_prof->offset_right_mm < 0) {
		p_prof->offset_right_mm = 0;
	}

	/* free space beyond each edge; a side that runs off the scan has none */
	p_prof->clear_left_mm = (int16_t)(p_prof->left_open ?
		clearance(grid, d_obs_mm, left_stop, -1, p_prof->x_left_mm) : 0);
	p_prof->clear_right_mm = (int16_t)(p_prof->right_open ?
		clearance(grid, d_obs_mm, right_stop, +1, p_prof->x_right_mm) : 0);
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

/* Free space a side needs beyond the measured edge. Straight past a known
 * box: the car's whole width past the gap (its side 70 mm off the face);
 * otherwise C_pass (pivots beside the obstacle). Measured from the
 * cone-widened edge, so it errs on the safe side. */
#if (SL_KNOWN_W_MM > 0) && SL_KNOWN_STRAIGHT
#define SL_SIDE_NEED_MM		(SL_KNOWN_CLEAR_MM + SL_CAR_HALF_W_MM)	/* 196 */
#else
#define SL_SIDE_NEED_MM		SL_C_PASS_MM
#endif

/* Can the car pass on this side (clearance and offset limit)? */
static bool side_ok(const sl_profile_t *p_prof, bool b_left)
{
	return b_left ? (p_prof->left_open &&
			 (p_prof->clear_left_mm >= SL_SIDE_NEED_MM) &&
			 (p_prof->offset_left_mm <= SL_MAX_OFFSET_MM))
		      : (p_prof->right_open &&
			 (p_prof->clear_right_mm >= SL_SIDE_NEED_MM) &&
			 (p_prof->offset_right_mm <= SL_MAX_OFFSET_MM));
}

sl_action_t sl_other_side(const sl_profile_t *p_prof, sl_action_t tried)
{
	sl_action_t other = SL_BLOCKED;

	if((SL_BYPASS_LEFT == tried) && side_ok(p_prof, false)) {
		other = SL_BYPASS_RIGHT;
	} else if((SL_BYPASS_RIGHT == tried) && side_ok(p_prof, true)) {
		other = SL_BYPASS_LEFT;
	} else {
		/* no other side to try */
	}
	return other;
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
		ok_left  = side_ok(p_prof, true);
		ok_right = side_ok(p_prof, false);

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
	return sl_classify_at(no_echo, dist_mm, SL_D_STOP_MM);
}

sl_ping_t sl_classify_at(bool no_echo, uint32_t dist_mm, int32_t stop_mm)
{
	sl_ping_t ping;

	if(no_echo) {
		ping = SL_PING_NO_ECHO;
	} else if((SL_FAR != dist_mm) && (dist_mm < (uint32_t)stop_mm)) {
		ping = SL_PING_CLOSE;
	} else {
		ping = SL_PING_CLEAR;		/* includes out of range */
	}
	return ping;
}

int32_t sl_avoid_stop_mm(int32_t moved_mm)
{
	return (abs_val(moved_mm) <= SL_PIVOT_MOVE_MM) ? SL_D_REV_MM : SL_D_STOP_MM;
}

uint16_t sl_look_reply_mm(uint16_t grid_mm)
{
	uint16_t reply;

	if(SL_NOT_MEASURED == grid_mm) {
		reply = SL_LOOK_UNKNOWN_MM;
	} else if((SL_FAR == grid_mm) || (grid_mm >= SL_LOOK_FAR_MM)) {
		reply = SL_LOOK_FAR_MM;
	} else {
		reply = grid_mm;		/* 1..510 mm */
	}
	return reply;
}

bool sl_lane_clear(uint16_t grid_mm, int32_t offset_mm)
{
	int32_t	need_mm = ((offset_mm * SL_SIDE_LEG_X1000) / 1000) +
			  SL_D_STOP_MM + SL_LANE_SLACK_MM;
	bool	clear;

	if(SL_FAR == grid_mm) {
		clear = true;			/* nothing in range */
	} else if(SL_NOT_MEASURED == grid_mm) {
		clear = false;			/* no echo: unknown */
	} else {
		clear = ((int32_t)grid_mm >= need_mm);
	}
	return clear;
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
