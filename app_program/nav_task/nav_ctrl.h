/*
 *	nav_ctrl.h
 *	nav_ctrl_task: owns the mission state (design Section 5, System-Level
 *	Statechart). Priority 15, pinned to core 0. Owner: Buddy 5.
 *
 *	mission_state is written ONLY here, as one aligned 32-bit store, so a
 *	reader on either core sees a whole value without a mutex (config.h
 *	allows only 4 mutexes; see Buddy 4's hump_state note).
 *
 *	The nav_flg bits and mission command opcodes are PROPOSED and move to
 *	common/messages.h through the team PR.
 */

#ifndef NAV_CTRL_H
#define NAV_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

typedef enum {
	MS_INITIALISING = 0,
	MS_CALIBRATING,
	MS_READY,
	MS_RUNNING,
	MS_PAUSED,
	MS_FINISHED,
	MS_FAULT
} mission_state_t;

/* ---- nav_flg bits (PROPOSED) ---- */
#define NAV_FLG_BTN		0x0001U	/* BTN_ISR: GP20 or GP21 pressed           */
#define NAV_FLG_JUNCTION	0x0002U	/* ir_sense_task                            */
#define NAV_FLG_CAL_OK_IR	0x0004U	/* ir_sense_task calibration passed         */
#define NAV_FLG_CAL_OK_IMU	0x0008U	/* imu_task calibration passed              */
#define NAV_FLG_CAL_FAIL	0x0010U	/* either calibration (or IMU init) failed  */
#define NAV_FLG_IMU_FAULT	0x0020U	/* imu_task                                 */
#define NAV_FLG_MOTION_FAULT	0x0040U	/* motion_task: stall / encoder fault       */
#define NAV_FLG_MOVE_DONE	0x0080U	/* motion_task: junction command finished   */
/*      NAV_FLG_BLOCKED		0x0100U    scan_task, defined in scan_if.h          */
#define NAV_FLG_CMD_REJECTED	0x0200U	/* motion_task: junction command refused    */
#define NAV_FLG_LINE_FOUND	0x0400U	/* motion_task: line reacquired             */
#define NAV_FLG_LINE_LOST	0x0800U	/* motion_task: line search failed          */
#define NAV_FLG_MISSION_CMD	0x1000U	/* comms_task placed a mission command      */
#define NAV_FLG_CONN		0x2000U	/* cyw43_utk_task connection change         */
#define NAV_FLG_MOTION_STOPPED	0x4000U	/* motion_task: motion_stop() done          */

/* ---- mission commands from comms_task (mission_cmd_mbf, 1 byte) ---- */
typedef enum {
	MCMD_START  = 1,	/* Ready  -> Running  */
	MCMD_PAUSE  = 2,	/* Running -> Paused  */
	MCMD_RESUME = 3,	/* Paused -> Running  */
	MCMD_RESET  = 4,	/* Finished -> Ready, Fault -> Calibrating */
	MCMD_FINISH = 5		/* operator stop: Running/Paused -> Finished */
} mission_cmd_t;

/* ---- optional links, filled in by app_main.c as teammates' tasks appear */
typedef struct {
	ID	ir_flgid;		/* ir_sense_task's flag, for CAL_REQ (0 = none) */
	ID	imu_flgid;		/* imu_task's flag, for CAL_REQ (0 = none)      */
	UINT	cal_req_bit;		/* CAL_REQ bit on those flags [agree with B3/B4] */
	ID	mission_cmd_mbfid;	/* comms_task -> nav (0 = none)                 */
	ID	nav_cmd_mbfid;		/* nav -> motion_task, for FAULT_CLEAR (0 = none) */
} nav_bind_t;

/* Call in usermain() BEFORE creating any task that uses nav_flg. */
EXPORT ER		nav_ctrl_init(void);
EXPORT void		nav_bind(const nav_bind_t *p_bind);	/* optional, before start */
EXPORT ID		nav_flag(void);

/* Readers (any task, any core). */
EXPORT mission_state_t	nav_mission_state(void);
EXPORT bool		nav_mission_running(void);
EXPORT const char	*nav_state_name(mission_state_t state);

EXPORT void		nav_ctrl_task(INT stacd, void *exinf);

#endif /* NAV_CTRL_H */
