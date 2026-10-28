/*
 *	scan_if.h
 *	scan_task's links to the rest of the system (Section 4 of the design).
 *
 *	  scan_flg       OWNED HERE. TICK (60 ms cyclic handler), ECHO (ECHO_ISR),
 *	                 REVERSE_DONE / LINE_FOUND / LINE_LOST (from motion_task).
 *	  motion_flg     Buddy 2. scan_task sets STOP_SCAN and CMD.
 *	  avoid_cmd_mbf  Buddy 2. scan_task -> motion_task, packed 16-bit command, room 2.
 *	  nav_flg        nav_ctrl_task. scan_task sets BLOCKED.
 *	  tele_mbf       Buddy 1. scan_task sends the packed obstacle body
 *	                 (comm_w_pack_obstacle, kind 4) through comms_tele_send(),
 *	                 non-blocking; a full queue drops it and comms counts it.
 *	  fault_flg      Buddy 5 (nav_ctrl). scan_task sets / clears only
 *	                 FAULT_SCAN_SENSOR_LOST; the S record carries it.
 *	  mission_state, hump_state: read through callbacks bound in app_main.c,
 *	                 so scan_task never includes the owner's layout.
 *	                 Both are one-writer aligned 32-bit words (nav_ctrl_task's
 *	                 mission_state; Buddy 4's packed hump_state word,
 *	                 imu_word_read()), so neither read takes a mutex.
 *
 *	FAIL-SAFE MODE: if either motion_flgid or avoid_mbfid is missing,
 *	STOP_SCAN can still be sent when possible, but avoid commands fail
 *	instead of pretending that motion or line reacquisition succeeded.
 *
 *	The packed avoid-command layout is shared in common/messages.h. The
 *	planner command and telemetry profile remain scan_task interfaces.
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

/* ---- Buddy 5 planning interface ---- */
#define NAV_FLG_BLOCKED		0x0100U	/* path blocked / sensor lost > t_block */

typedef enum {
	AVOID_REVERSE = MOTION_AVOID_OP_REVERSE,
	AVOID_BYPASS  = MOTION_AVOID_OP_BYPASS
} avoid_type_t;

typedef enum {
	AVOID_SIDE_NONE  = MOTION_AVOID_SIDE_NONE,
	AVOID_SIDE_LEFT  = MOTION_AVOID_SIDE_LEFT,
	AVOID_SIDE_RIGHT = MOTION_AVOID_SIDE_RIGHT
} avoid_side_t;

typedef struct {		/* scan planner form; packed before MBF send */
	uint8_t		type;		/* avoid_type_t                     */
	uint8_t		side;		/* avoid_side_t (bypass only)       */
	uint16_t	dist_mm;	/* bypass: lateral offset; reverse: distance */
} avoid_cmd_t;

/* Obstacle event body "bypass" field (MQTT_SCHEMA.md, kind 4) */
#define SCAN_TELE_BYPASS_NONE	0U
#define SCAN_TELE_BYPASS_LEFT	1U
#define SCAN_TELE_BYPASS_RIGHT	2U

/* ---- binding, filled in by app_main.c at integration ---- */
typedef struct {
	ID	motion_flgid;			/* 0 = standalone */
	ID	avoid_mbfid;
	ID	nav_flgid;
	bool	(*p_mission_running)(void);	/* NULL = always Running */
	bool	(*p_hump_active)(void);		/* hump_state != NONE; NULL = never */
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
EXPORT bool	scan_if_send_cmd(const avoid_cmd_t *p_cmd);	/* false = unavailable/full/invalid */
EXPORT void	scan_if_set_blocked(void);
/* Obstacle profile -> telemetry (FR-16, FR-23): width in mm (sent in cm,
 * clamped to 8 bits) and the bypass side, SCAN_TELE_BYPASS_*. */
EXPORT void	scan_if_send_profile(uint32_t width_mm, uint32_t bypass);
/* Sensor Lost entered (true) or left (false): FAULT_SCAN_SENSOR_LOST. */
EXPORT void	scan_if_sensor_lost(bool lost);

EXPORT bool	scan_if_mission_running(void);
EXPORT bool	scan_if_hump_active(void);
EXPORT bool	scan_if_standalone(void);	/* missing motion or avoid binding */

#endif /* SCAN_IF_H */
