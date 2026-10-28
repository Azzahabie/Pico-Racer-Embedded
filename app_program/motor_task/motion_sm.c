/*
 *  motion_sm.c
 *  Statechart 6.2 for motion_task (Buddy 2).  See motion_sm.h.
 *
 *  Sources: design 6.2 (week 6, text and diagram), Section 4 (order of
 *  work in a wake), and the rules agreed with Buddy 5 on 22 Oct
 *  (PostTri1 Version C row 15): an interrupted turn resumes after a false
 *  alarm and then sends MOVE_DONE; if a reverse or bypass command arrives,
 *  or the mission leaves Running, it is dropped with exactly one
 *  CMD_REJECTED.
 */

#include "motion_sm.h"

#define SM_STATE_MASK    (0x0Fu)
#define SM_HISTORY_SHIFT (4u)

_Static_assert((uint32_t)MOTION_ST_COUNT <= SM_STATE_MASK,
               "states fit in 4 bits with one value left for 'none'");

/* ------------------------------------------------------------------ *
 *  Packed state and history
 * ------------------------------------------------------------------ */

motion_state_t
motion_sm_state (motion_sm_t const *p_sm)
{
    return (motion_state_t)((uint32_t)p_sm->packed & SM_STATE_MASK);
}

uint32_t
motion_sm_history (motion_sm_t const *p_sm)
{
    return ((uint32_t)p_sm->packed >> SM_HISTORY_SHIFT) & SM_STATE_MASK;
}

static void
sm_set (motion_sm_t *p_sm, motion_state_t state, uint32_t history)
{
    p_sm->packed =
        (uint8_t)((((uint32_t)history & SM_STATE_MASK) << SM_HISTORY_SHIFT)
                  | ((uint32_t)state & SM_STATE_MASK));
}

static bool
sm_flag (motion_sm_t const *p_sm, uint32_t flag)
{
    return 0u != ((uint32_t)p_sm->flags & flag);
}

static void
sm_flag_set (motion_sm_t *p_sm, uint32_t flag, bool b_on)
{
    if (b_on)
    {
        p_sm->flags = (uint8_t)((uint32_t)p_sm->flags | flag);
    }
    else
    {
        p_sm->flags = (uint8_t)((uint32_t)p_sm->flags & ~flag);
    }
}

void
motion_sm_init (motion_sm_t *p_sm)
{
    sm_set(p_sm, MOTION_ST_IDLE, MOTION_ST_NONE);
    p_sm->flags = 0u;
}

/* ------------------------------------------------------------------ *
 *  State groups
 * ------------------------------------------------------------------ */

/* The Active composite state (mission Running, no STOP bit). */
static bool
sm_active (motion_state_t state)
{
    return (MOTION_ST_LINE_FOLLOW == state) || (MOTION_ST_TURNING == state)
           || (MOTION_ST_BYPASS == state) || (MOTION_ST_LINE_SEARCH == state)
           || (MOTION_ST_LINE_LOST == state);
}

bool
motion_sm_driving (motion_state_t state)
{
    return (MOTION_ST_LINE_FOLLOW == state) || (MOTION_ST_TURNING == state)
           || (MOTION_ST_BYPASS == state) || (MOTION_ST_LINE_SEARCH == state)
           || (MOTION_ST_REVERSING == state);
}

char const *
motion_sm_name (uint32_t state)
{
    static char const *const names[MOTION_ST_COUNT] = {
        "IDLE",      "LINE_FOLLOW", "TURNING",   "BYPASS", "LINE_SEARCH",
        "LINE_LOST", "BRAKED",      "REVERSING", "FAULT"
    };
    char const *p_name = "?";

    if (state < (uint32_t)MOTION_ST_COUNT)
    {
        p_name = names[state];
    }

    return p_name;
}

/* ------------------------------------------------------------------ *
 *  Step
 * ------------------------------------------------------------------ */

/* A turn that will not finish owes exactly one CMD_REJECTED. */
static uint32_t
sm_drop_turn (motion_sm_t *p_sm)
{
    uint32_t rep = 0u;

    if (sm_flag(p_sm, MOTION_SM_F_TURN_OPEN))
    {
        rep = MOTION_SM_REP_CMD_REJECTED;
        sm_flag_set(p_sm, MOTION_SM_F_TURN_OPEN, false);
        if ((uint32_t)MOTION_ST_TURNING == motion_sm_history(p_sm))
        {
            /* Nothing to resume: back to Line Following afterwards. */
            sm_set(p_sm, motion_sm_state(p_sm), MOTION_ST_LINE_FOLLOW);
        }
    }

    return rep;
}

/* Braked: carry out or queue an avoidance command, and leave once every
   STOP bit is clear (T6, T8, T9, T10). */
static motion_state_t
sm_braked (motion_sm_t *p_sm, motion_sm_in_t const *p_in, uint32_t *p_rep,
           bool *p_resumed)
{
    motion_state_t next = MOTION_ST_BRAKED;
    uint32_t       history;

    if (MOTION_SM_AVOID_REVERSE == p_in->avoid)
    {
        if (!p_in->b_stop_imu)
        {
            *p_rep |= sm_drop_turn(p_sm);
            next = MOTION_ST_REVERSING;
        }
        else
        {
            /* Refused while STOP_IMU is set (REVIEW D12): it still ends any
               interrupted turn and tells scan_task the reverse is over. */
            *p_rep |= sm_drop_turn(p_sm) | MOTION_SM_REP_REVERSE_DONE;
        }
    }
    else if (MOTION_SM_AVOID_BYPASS == p_in->avoid)
    {
        *p_rep |= sm_drop_turn(p_sm);
        sm_flag_set(p_sm, MOTION_SM_F_BYPASS_WAIT, true);
    }
    else
    {
        /* No avoidance command. */
    }

    if ((MOTION_ST_BRAKED == next) && !p_in->b_stop_scan && !p_in->b_stop_imu)
    {
        if (sm_flag(p_sm, MOTION_SM_F_BYPASS_WAIT))
        {
            sm_flag_set(p_sm, MOTION_SM_F_BYPASS_WAIT, false);
            sm_set(p_sm, MOTION_ST_BRAKED, MOTION_ST_NONE);
            next = MOTION_ST_BYPASS;
        }
        else
        {
            history = motion_sm_history(p_sm); /* after any dropped turn */
            if (history < (uint32_t)MOTION_ST_COUNT)
            {
                next       = (motion_state_t)history;
                *p_resumed = true;
            }
            else
            {
                next = MOTION_ST_LINE_FOLLOW; /* no history: default */
            }
        }
    }

    return next;
}

/* The current state's own rules, once the higher-priority checks have
   not fired. */
static motion_state_t
sm_state_rules (motion_sm_t *p_sm, motion_sm_in_t const *p_in, uint32_t *p_rep,
                bool *p_resumed, bool *p_nav_used)
{
    motion_state_t state = motion_sm_state(p_sm);
    motion_state_t next  = state;

    switch (state)
    {
        case MOTION_ST_IDLE: /* T1, T2 */
            next = (p_in->b_stop_scan || p_in->b_stop_imu)
                       ? MOTION_ST_BRAKED
                       : MOTION_ST_LINE_FOLLOW;
            break;

        case MOTION_ST_BRAKED:
            next = sm_braked(p_sm, p_in, p_rep, p_resumed);
            break;

        case MOTION_ST_REVERSING: /* T7 */
            if (p_in->b_done || p_in->b_stop_imu)
            {
                *p_rep |= MOTION_SM_REP_REVERSE_DONE;
                next = MOTION_ST_BRAKED;
            }
            break;

        case MOTION_ST_LINE_FOLLOW:
            if (MOTION_SM_NAV_JUNCTION == p_in->nav) /* T11 */
            {
                sm_flag_set(p_sm, MOTION_SM_F_TURN_OPEN, true);
                *p_nav_used = true;
                next        = MOTION_ST_TURNING;
            }
            else if (p_in->b_line_missing) /* T14 */
            {
                sm_flag_set(p_sm, MOTION_SM_F_AFTER_BYPASS, false);
                next = MOTION_ST_LINE_SEARCH;
            }
            else
            {
                /* Keep following. */
            }
            break;

        case MOTION_ST_TURNING: /* T12, T13 */
            if (p_in->b_done)
            {
                *p_rep |= MOTION_SM_REP_MOVE_DONE;
                sm_flag_set(p_sm, MOTION_SM_F_TURN_OPEN, false);
                sm_flag_set(p_sm, MOTION_SM_F_AFTER_BYPASS, false);
                next = p_in->b_line_seen ? MOTION_ST_LINE_FOLLOW
                                         : MOTION_ST_LINE_SEARCH;
            }
            break;

        case MOTION_ST_BYPASS: /* T16 */
            if (p_in->b_done)
            {
                sm_flag_set(p_sm, MOTION_SM_F_AFTER_BYPASS, true);
                next = MOTION_ST_LINE_SEARCH;
            }
            break;

        case MOTION_ST_LINE_SEARCH: /* T15, T17 */
            if (p_in->b_line_found || p_in->b_search_over)
            {
                *p_rep |= p_in->b_line_found ? MOTION_SM_REP_LINE_FOUND
                                             : MOTION_SM_REP_LINE_LOST;
                if (sm_flag(p_sm, MOTION_SM_F_AFTER_BYPASS))
                {
                    *p_rep |= MOTION_SM_REP_TO_SCAN;
                }
                sm_flag_set(p_sm, MOTION_SM_F_AFTER_BYPASS, false);
                next = p_in->b_line_found ? MOTION_ST_LINE_FOLLOW
                                          : MOTION_ST_LINE_LOST;
            }
            break;

        default:
            /* Line Lost waits for nav_ctrl_task; Motion Fault is handled
               before this. */
            break;
    }

    return next;
}

void
motion_sm_step (motion_sm_t *p_sm, motion_sm_in_t const *p_in,
                motion_sm_out_t *p_out)
{
    motion_state_t state      = motion_sm_state(p_sm);
    motion_state_t next       = state;
    uint32_t       rep        = 0u;
    bool           b_resumed  = false;
    bool           b_nav_used = false;
    bool           b_stopped  = p_in->b_stop_scan || p_in->b_stop_imu;

    if (MOTION_SM_NAV_CLEAR_IMU == p_in->nav) /* any state (diagram note) */
    {
        b_nav_used = true;
    }

    if (MOTION_ST_FAULT == state) /* T20 */
    {
        if (MOTION_SM_NAV_CLEAR_MOTION == p_in->nav)
        {
            b_nav_used = true;
            next       = MOTION_ST_IDLE;
        }
    }
    else if (p_in->b_fault && motion_sm_driving(state)) /* T18, T19 */
    {
        rep |= sm_drop_turn(p_sm);
        if (MOTION_ST_REVERSING == state)
        {
            rep |= MOTION_SM_REP_REVERSE_DONE;
        }
        rep |= MOTION_SM_REP_MOTION_FAULT;
        next = MOTION_ST_FAULT;
    }
    else if (!p_in->b_running) /* T3, T4 */
    {
        if (MOTION_ST_IDLE != state)
        {
            rep |= sm_drop_turn(p_sm);
            if (MOTION_ST_REVERSING == state)
            {
                rep |= MOTION_SM_REP_REVERSE_DONE;
            }
            next = MOTION_ST_IDLE;
        }
    }
    else if (sm_active(state) && b_stopped) /* T5: remember where it was */
    {
        sm_set(p_sm, state, (uint32_t)state);
        next = MOTION_ST_BRAKED;
    }
    else
    {
        next = sm_state_rules(p_sm, p_in, &rep, &b_resumed, &b_nav_used);
    }

    /* A junction command not taken is refused (only Line Following takes
       one). */
    if ((MOTION_SM_NAV_JUNCTION == p_in->nav) && !b_nav_used)
    {
        rep |= MOTION_SM_REP_CMD_REJECTED;
    }

    if (next != state)
    {
        if ((MOTION_ST_IDLE == next) || (MOTION_ST_FAULT == next))
        {
            /* Entering Idle clears the history (diagram note). */
            sm_set(p_sm, next, MOTION_ST_NONE);
            p_sm->flags = 0u;
        }
        else if (b_resumed)
        {
            sm_set(p_sm, next, MOTION_ST_NONE);
        }
        else
        {
            sm_set(p_sm, next, motion_sm_history(p_sm));
        }
    }

    p_out->replies          = rep;
    p_out->b_changed        = (next != state);
    p_out->b_resumed        = b_resumed;
    p_out->b_clear_stop_imu = (MOTION_SM_NAV_CLEAR_IMU == p_in->nav);
}
