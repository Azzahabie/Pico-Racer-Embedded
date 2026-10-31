#ifndef IMU_EVENTS_H
#define IMU_EVENTS_H

#include <tk/tkernel.h>

/* Plain-language console events for imu_task (Buddy 4): which way a turn
 * went and how far, how many turns, picked up / put down, and which way
 * the car leans.  No I2C or kernel calls, so tests/imu_safety runs it on
 * a PC; imu_task prints what the step functions return. */

/* Board mounting: which accel axis (0 x, 1 y) reads positive when the car
 * leans to its LEFT, and which when the nose points UP.  Unchecked guess:
 * tilt the car left by hand once; if the console says RIGHT, flip
 * IMU_LEFT_SIGN (likewise IMU_NOSE_SIGN for nose up). */
#ifndef IMU_LEFT_AXIS
#define IMU_LEFT_AXIS	1
#endif
#ifndef IMU_LEFT_SIGN
#define IMU_LEFT_SIGN	1
#endif
#ifndef IMU_NOSE_AXIS
#define IMU_NOSE_AXIS	0
#endif
#ifndef IMU_NOSE_SIGN
#define IMU_NOSE_SIGN	1
#endif

/* Thresholds: first guesses, not yet tuned on the car. */
#define EV_TURN_MIN_DEG	15		/* smaller turns are wiggles, uncounted */
#define EV_TURN_PRE_DPS	12		/* angle summed from this rate on (the
					   classifier's turn-off level), so
					   straight-line jitter adds nothing */
#define EV_LEAN_ON_MG	87		/* 5 deg (sin 5 = 0.087): leaning      */
#define EV_LEAN_OFF_MG	52		/* 3 deg: level again (hysteresis)     */
#define EV_LEAN_WIN	2		/* held 200 ms before it is reported   */
#define EV_HAND_ON_MG	30		/* 100 ms mean jumps this much, wheels
					   still: lifted.  Floor spots differ
					   by ~30 mg, but that needs driving */
#define EV_HAND_ON_WIN	2		/* ... for 200 ms (one knock: 1 window)*/
#define EV_HAND_OFF_MG	8		/* steadier than this ...              */
#define EV_HAND_OFF_WIN	10		/* ... for 1 s: put down               */

/* Events, as bits. */
#define EV_TURN_START	0x01U		/* ev_turn_dir() says which way        */
#define EV_TURN_DONE	0x02U		/* ev_turn_deg(): the whole turn       */
#define EV_TURN_SMALL	0x04U		/* ended under EV_TURN_MIN_DEG         */
#define EV_PICKED_UP	0x08U
#define EV_PUT_DOWN	0x10U
#define EV_LEAN		0x20U		/* ev_lean() changed                   */
#define EV_NOSE		0x40U		/* ev_nose() changed                   */

IMPORT void ev_reset(void);		/* boot */
IMPORT void ev_mission_reset(void);	/* new mission: counts and heading */

/* Every 10 ms sample.  turning: the motion class is MC_TURNING;
 * turn_dps: from distance_state, CCW (left) positive. */
IMPORT UINT ev_turn_step(BOOL turning, W turn_dps, W sample_ms);

/* Every 100 ms window: mean x, y, z in mg (offsets removed, level reads
 * 0, 0, 1000) and the encoder mm driven in the window. */
IMPORT UINT ev_window_step(W mx, W my, W mz, W dmm);

IMPORT W    ev_turn_dir(void);		/* +1 left, -1 right, 0 none */
IMPORT W    ev_turn_deg(void);		/* last turn, signed, left + */
IMPORT UW   ev_turn_ms(void);		/* how long it took          */
IMPORT UW   ev_turns_left(void);
IMPORT UW   ev_turns_right(void);
IMPORT W    ev_heading_deg(void);	/* since mission start, left + */
IMPORT BOOL ev_held(void);		/* picked up, not put down    */
IMPORT W    ev_lean(void);		/* +1 left, -1 right, 0 level */
IMPORT W    ev_nose(void);		/* +1 up, -1 down, 0 level    */
IMPORT W    ev_lean_mg(void);		/* signed, left + */
IMPORT W    ev_nose_mg(void);		/* signed, up +   */

/* "LEFT" / "RIGHT" / "" for a sign, and "UP" / "DOWN" / "". */
IMPORT const char *ev_side_name(W s);
IMPORT const char *ev_nose_name(W s);

#endif /* IMU_EVENTS_H */
