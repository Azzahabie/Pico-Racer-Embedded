/*
 *  motion_sm.h
 *  Statechart 6.2 of the design: motion_task's states and the rules for
 *  moving between them (Buddy 2).
 *
 *  Pure logic, no kernel calls and no register access, so tests/motion_sm/
 *  checks every transition on a PC.  motor_task.c gathers the inputs once
 *  per tick (mission state, STOP bits, commands, faults, progress of the
 *  current manoeuvre), calls motion_sm_step(), and then does what the
 *  output asks: start the new state's behaviour and send the replies.
 *
 *  States (9, so 4 bits):
 *    Idle          mission not Running; motors braked
 *    Active        mission Running: Line Following, Turning, Bypass,
 *                  Line Search, Line Lost (with shallow history H)
 *    Stopped       a STOP bit is set: Braked, Reversing
 *    Motion Fault  stall or encoder fault (FR-30); left only on
 *                  FAULT_CLEAR(MOTION)
 *
 *  Each step makes at most one transition, checked in this order:
 *  Motion Fault, a new fault, the mission leaving Running, a STOP bit,
 *  then the current state's own rules.  The brake itself is written by
 *  motor_task.c before this runs (NFR-18).
 *
 *  Decisions where the design is silent (docs/motion/README.md):
 *    - a turn dropped by a reverse or bypass resumes as Line Following;
 *    - REVERSE_DONE is sent on every exit from Reversing (diagram note),
 *      and when a reverse is refused because STOP_IMU is set (REVIEW D12).
 */

#ifndef MOTION_SM_H
#define MOTION_SM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTION_ST_IDLE = 0,
    MOTION_ST_LINE_FOLLOW,
    MOTION_ST_TURNING,
    MOTION_ST_BYPASS,
    MOTION_ST_LINE_SEARCH,
    MOTION_ST_LINE_LOST,
    MOTION_ST_BRAKED,
    MOTION_ST_REVERSING,
    MOTION_ST_FAULT,
    MOTION_ST_COUNT
} motion_state_t;

/* History value meaning "no history" (4-bit field). */
#define MOTION_ST_NONE (0xFu)

/* Avoidance command taken this tick (from scan_task). */
typedef enum
{
    MOTION_SM_AVOID_NONE = 0,
    MOTION_SM_AVOID_REVERSE,
    MOTION_SM_AVOID_BYPASS
} motion_sm_avoid_t;

/* Navigation command taken this tick (from nav_ctrl_task). */
typedef enum
{
    MOTION_SM_NAV_NONE = 0,
    MOTION_SM_NAV_JUNCTION,    /* left, right, straight or U-turn */
    MOTION_SM_NAV_CLEAR_IMU,   /* FAULT_CLEAR(IMU): clear STOP_IMU */
    MOTION_SM_NAV_CLEAR_MOTION /* FAULT_CLEAR(MOTION)              */
} motion_sm_nav_t;

/* Everything one step looks at.  b_done: the turn, bypass or reverse
   reached its target.  b_line_missing: the line was missing for [t_lost]
   (2 x t_lost on a hump).  b_line_found: the line was seen for [N_line]
   samples in a row. */
typedef struct
{
    bool              b_running;      /* mission is Running             */
    bool              b_stop_scan;    /* STOP_SCAN is set               */
    bool              b_stop_imu;     /* STOP_IMU is set                */
    motion_sm_avoid_t avoid;          /* avoidance command, or NONE     */
    motion_sm_nav_t   nav;            /* navigation command, or NONE    */
    bool              b_fault;        /* stall or encoder fault (FR-30) */
    bool              b_done;         /* manoeuvre at its target        */
    bool              b_line_seen;    /* line under the sensors now     */
    bool              b_line_found;   /* found after a search           */
    bool              b_line_missing; /* missing for [t_lost]           */
    bool              b_search_over;  /* [L_search] travelled           */
} motion_sm_in_t;

/* Replies to send after a step; motor_task.c maps them to the nav_flg and
   scan_flg bits.  TO_SCAN: LINE_FOUND or LINE_LOST also goes to scan_task,
   because the search followed a bypass. */
#define MOTION_SM_REP_MOVE_DONE    (1u << 0) /* nav                        */
#define MOTION_SM_REP_CMD_REJECTED (1u << 1) /* nav                        */
#define MOTION_SM_REP_MOTION_FAULT (1u << 2) /* nav                        */
#define MOTION_SM_REP_LINE_FOUND   (1u << 3) /* nav (and scan, see TO_SCAN) */
#define MOTION_SM_REP_LINE_LOST    (1u << 4) /* nav (and scan, see TO_SCAN) */
#define MOTION_SM_REP_REVERSE_DONE (1u << 5) /* scan                       */
#define MOTION_SM_REP_TO_SCAN      (1u << 6) /* see above                  */

typedef struct
{
    uint32_t replies;          /* MOTION_SM_REP_* bits                */
    bool     b_changed;        /* a new state was entered this step   */
    bool     b_resumed;        /* ... and it came from history (H)    */
    bool     b_clear_stop_imu; /* FAULT_CLEAR(IMU): clear STOP_IMU    */
} motion_sm_out_t;

/* The whole statechart in two bytes (minimum-bit data):
     packed  bits 0-3 state, bits 4-7 history (MOTION_ST_NONE = none)
     flags   MOTION_SM_F_* below */
typedef struct
{
    uint8_t packed;
    uint8_t flags;
} motion_sm_t;

#define MOTION_SM_F_TURN_OPEN    (1u << 0) /* a turn is owed one reply      */
#define MOTION_SM_F_BYPASS_WAIT  (1u << 1) /* bypass waits for STOP to clear */
#define MOTION_SM_F_AFTER_BYPASS (1u << 2) /* this search follows a bypass  */

/* Idle, no history, no flags. */
void motion_sm_init(motion_sm_t *p_sm);

/* One step: at most one transition, and the replies it owes. */
void motion_sm_step(motion_sm_t *p_sm, motion_sm_in_t const *p_in,
                    motion_sm_out_t *p_out);

motion_state_t motion_sm_state(motion_sm_t const *p_sm);

/* The remembered Active state, or MOTION_ST_NONE. */
uint32_t motion_sm_history(motion_sm_t const *p_sm);

/* States in which the wheels may be driven (the stall check applies). */
bool motion_sm_driving(motion_state_t state);

/* Short name for the console, "?" for a value out of range. */
char const *motion_sm_name(uint32_t state);

#endif /* MOTION_SM_H */
