/*
 *  scan_sizes.c
 *  Buddy 5 (scan_task, nav_ctrl): Week 13 sizes, before and after the
 *  minimum-bit change.  Never part of the firmware or the host tests.
 *  tests/week13/scan_measure.sh compiles it with the car's ARM compiler
 *  (enum size and padding are the target's, not the PC's) and prints each
 *  object's size with nm -S.
 *
 *  "Before" types are copied from commit c3dea9d ("before requirement
 *  change for obstacle task"); "after" types come from the current
 *  headers, or are copied from obstacle_task.c where the type is private
 *  to that file (scan_ctx_t).  Keep the copies in step with the source.
 */

#include <stdint.h>
#include <stdbool.h>
#include "messages.h"
#include "scan_logic.h"

/* A message buffer stores a 4-byte header plus the message rounded up to
   4 bytes (kernel/tkernel/messagebuf.h, ROUNDSZ and HEADERSZ). */
#define MBF_SLOT(bytes) (4u + ((((bytes) + 3u) / 4u) * 4u))

/* ------------------------------------------------------------------ */
/* Before (c3dea9d)                                                    */
/* ------------------------------------------------------------------ */
#define B_SL_SLOTS	25		/* -60..+60 deg in 5 deg steps then */

typedef enum { B_ST_IDLE = 0, B_ST_SENSOR_LOST = 8 } b_scan_state_t;
typedef enum { B_SL_CONTINUE = 0, B_SL_BLOCKED = 4 } b_sl_action_t;

typedef struct {
	bool	found;
	int32_t	d_close_mm, close_deg, left_deg, right_deg;
	int32_t	x_left_mm, x_right_mm, width_mm;
	bool	left_open, right_open, in_path;
	int32_t	offset_left_mm, offset_right_mm, clear_left_mm, clear_right_mm;
} b_sl_profile_t;

typedef struct {
	uint8_t	close_run, clear_run, no_echo_run, echo_run;
} b_sl_runs_t;

typedef struct {
	b_scan_state_t	state;
	uint16_t	grid[B_SL_SLOTS];
	b_sl_profile_t	prof;
	b_sl_action_t	action;
	b_sl_runs_t	runs;
	int32_t		left_hit, right_hit;
	bool		reverse_tried, after_reverse, blocked_sent;
	bool		abort_scan, lost_in_scan;
	uint8_t		scan_no_echo;
	uint32_t	state_us, scan_us;
} b_scan_ctx_t;

typedef struct {		/* scan_task -> motion_task, avoid_cmd_mbf */
	uint8_t		type;
	uint8_t		side;
	uint16_t	dist_mm;
} b_avoid_cmd_t;

typedef struct {		/* scan_task -> tele_mbf: profile 'P' or fault 'F' */
	uint8_t		type;
	uint8_t		action;
	int16_t		d_close_mm, close_deg, width_mm;
	int16_t		x_left_mm, x_right_mm, clear_left_mm, clear_right_mm;
} b_scan_tele_t;

b_scan_ctx_t	before_scan_ctx;
b_sl_profile_t	before_sl_profile;
b_sl_runs_t	before_sl_runs;
b_avoid_cmd_t	before_avoid_cmd;
uint8_t		before_avoid_cmd_in_mbf[MBF_SLOT(sizeof(b_avoid_cmd_t))];
b_scan_tele_t	before_scan_tele;
uint8_t		before_scan_tele_in_mbf[MBF_SLOT(sizeof(b_scan_tele_t))];
uint16_t	before_scan_grid[B_SL_SLOTS];

/* ------------------------------------------------------------------ */
/* After (this tree)                                                   */
/* ------------------------------------------------------------------ */
typedef struct {		/* copy of obstacle_task.c's private scan_ctx_t */
	uint32_t	state_us;
	uint32_t	scan_us;
	uint32_t	no_echo_slots;
	sl_profile_t	prof;
	uint16_t	grid[SL_SLOTS];
	uint16_t	bits;
	sl_runs_t	runs;
	uint8_t		action;
	int8_t		left_hit;
	int8_t		right_hit;
	int8_t		look_deg;
} a_scan_ctx_t;

a_scan_ctx_t	after_scan_ctx;
sl_profile_t	after_sl_profile;
sl_runs_t	after_sl_runs;
uint16_t	after_avoid_cmd;			/* motion_avoid_cmd_pack() */
uint8_t		after_avoid_cmd_in_mbf[MBF_SLOT(sizeof(uint16_t))];
uint8_t		after_obstacle_body[COMM_W_INTAKE_SIZE_OBSTACLE];
uint8_t		after_obstacle_body_in_mbf[MBF_SLOT(COMM_W_INTAKE_SIZE_OBSTACLE)];
uint8_t		after_obstacle_record_e[COMM_W_E_SIZE_OBSTACLE];
uint16_t	after_scan_grid[SL_SLOTS];

/* end of file */
