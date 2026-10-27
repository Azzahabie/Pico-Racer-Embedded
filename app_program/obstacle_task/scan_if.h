/*
 *	scan_if.h
 *	scan_task's links to the rest of the system (Section 4 of the design).
 *
 *	  scan_flg       OWNED HERE. TICK (60 ms cyclic handler), ECHO (ECHO_ISR),
 *	                 REVERSE_DONE / LINE_FOUND / LINE_LOST (from motion_task).
 *	  motion_flg     Buddy 2. scan_task sets STOP_SCAN and CMD.
 *	  avoid_cmd_mbf  Buddy 2. scan_task -> motion_task, 4-byte avoid_cmd_t, room 2.
 *	  nav_flg        nav_ctrl_task. scan_task sets BLOCKED.
 *	  tele_mbfid     Buddy 1. scan_task sends profile / sensor-fault records,
 *	                 non-blocking; a full queue drops the record and counts it.
 *	  mission_state, hump_state: read through callbacks, so the owner's
 *	                 priority-inheritance mutex stays inside the owner's code.
 *
 *	FAIL-SAFE MODE: if either motion_flgid or avoid_mbfid is missing,
 *	STOP_SCAN can still be sent when possible, but avoid commands fail
 *	instead of pretending that motion or line reacquisition succeeded.
 *
 *	The bit values and structs marked PROPOSED belong in common/messages.h
 *	and move there through a pull request agreed with Buddies 1, 2 and 3.
 */

#ifndef SCAN_IF_H
#define SCAN_IF_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../common/messages.h"

/* ---- scan_flg bits (owned by scan_task) ---- */
#define SCAN_FLG_TICK		0x01U
#define SCAN_FLG_ECHO		0x02U
#define SCAN_FLG_REVERSE_DONE	0x04U	/* set by motion_task */
#define SCAN_FLG_LINE_FOUND	0x08U	/* set by motion_task, after a bypass */
#define SCAN_FLG_LINE_LOST	0x10U	/* set by motion_task, after a bypass */
#define SCAN_FLG_REPLIES	(SCAN_FLG_REVERSE_DONE | SCAN_FLG_LINE_FOUND | \
				 SCAN_FLG_LINE_LOST)

#define SCAN_TICK_MS		60	/* one HC-SR04 cycle */

/* ---- PROPOSED shared definitions (move to common/messages.h) ---- */
#define NAV_FLG_BLOCKED		0x0100U	/* path blocked / sensor lost > t_block */

typedef enum {
	AVOID_REVERSE = 1,
	AVOID_BYPASS  = 2
} avoid_type_t;

typedef enum {
	AVOID_SIDE_NONE  = 0,
	AVOID_SIDE_LEFT  = 1,
	AVOID_SIDE_RIGHT = 2
} avoid_side_t;

typedef struct {		/* 4 bytes, distances in mm (Section 4) */
	uint8_t		type;		/* avoid_type_t                     */
	uint8_t		side;		/* avoid_side_t (bypass only)       */
	uint16_t	dist_mm;	/* bypass: lateral offset; reverse: distance */
} avoid_cmd_t;

typedef enum {
	TELE_SCAN_PROFILE = 0x50,	/* 'P' */
	TELE_SCAN_FAULT   = 0x46	/* 'F' sensor lost */
} tele_scan_type_t;

typedef struct {		/* fixed-size record for tele_mbfid (FR-23) */
	uint8_t		type;		/* tele_scan_type_t                 */
	uint8_t		action;		/* sl_action_t                      */
	int16_t		d_close_mm;
	int16_t		close_deg;
	int16_t		width_mm;
	int16_t		x_left_mm;
	int16_t		x_right_mm;
	int16_t		clear_left_mm;
	int16_t		clear_right_mm;
} scan_tele_t;

/* ---- binding, filled in by app_main.c at integration ---- */
typedef struct {
	ID	motion_flgid;			/* 0 = standalone */
	ID	avoid_mbfid;
	ID	nav_flgid;
	ID	tele_mbfid;
	bool	(*p_mission_running)(void);	/* NULL = always Running */
	bool	(*p_hump_active)(void);		/* NULL = never on a hump */
} scan_if_bind_t;

/* Call BEFORE tk_sta_tsk(obstacle task). Optional. */
EXPORT void	scan_if_bind(const scan_if_bind_t *p_bind);

/* Called by scan_task once: creates scan_flg and the 60 ms tick. */
EXPORT ER	scan_if_init(void);
EXPORT ID	scan_if_flag(void);		/* scan_flg ID, for motion_task */

EXPORT UINT	scan_if_wait(UINT bits, TMO tmout);	/* returns bits seen */
EXPORT void	scan_if_clear(UINT bits);

EXPORT void	scan_if_stop_set(void);
EXPORT void	scan_if_stop_clear(void);
EXPORT bool	scan_if_send_cmd(const avoid_cmd_t *p_cmd);	/* false = buffer full */
EXPORT void	scan_if_set_blocked(void);
EXPORT void	scan_if_send_tele(const scan_tele_t *p_rec);

EXPORT bool	scan_if_mission_running(void);
EXPORT bool	scan_if_hump_active(void);
EXPORT bool	scan_if_standalone(void);	/* missing motion or avoid binding */
EXPORT UW	scan_if_tele_drops(void);

#endif /* SCAN_IF_H */
