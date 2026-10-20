/*
 *  motion_lib.c
 *  Pure motion arithmetic for motion_task (Buddy 2).  See motion_lib.h.
 */

#include "motion_lib.h"

/* ------------------------------------------------------------------ *
 *  Quadrature decoding
 * ------------------------------------------------------------------ */

#define QUAD_STATE_MASK  (0x3u)
#define QUAD_A_SHIFT     (1u)
#define QUAD_INDEX_SHIFT (2u)
#define QUAD_TABLE_SIZE  (16u)

/*
 * Index = (previous state << 2) | new state, where state = (A << 1) | B.
 * Forward is the sequence 00 -> 10 -> 11 -> 01 -> 00 (A leads B).
 * 00 <-> 11 and 01 <-> 10 change both channels at once: a missed edge.
 */
static const uint8_t g_quad_table[QUAD_TABLE_SIZE] = {
    /* prev 00 */ MOTION_QUAD_NONE,
    MOTION_QUAD_REV,
    MOTION_QUAD_FWD,
    MOTION_QUAD_INVALID,
    /* prev 01 */ MOTION_QUAD_FWD,
    MOTION_QUAD_NONE,
    MOTION_QUAD_INVALID,
    MOTION_QUAD_REV,
    /* prev 10 */ MOTION_QUAD_REV,
    MOTION_QUAD_INVALID,
    MOTION_QUAD_NONE,
    MOTION_QUAD_FWD,
    /* prev 11 */ MOTION_QUAD_INVALID,
    MOTION_QUAD_FWD,
    MOTION_QUAD_REV,
    MOTION_QUAD_NONE
};

uint8_t
motion_quad_ab (uint32_t level_a, uint32_t level_b, bool b_swap)
{
    uint8_t bit_a = (level_a != 0u) ? 1u : 0u;
    uint8_t bit_b = (level_b != 0u) ? 1u : 0u;
    uint8_t ab_state;

    if (b_swap)
    {
        ab_state = (uint8_t)((uint8_t)(bit_b << QUAD_A_SHIFT) | bit_a);
    }
    else
    {
        ab_state = (uint8_t)((uint8_t)(bit_a << QUAD_A_SHIFT) | bit_b);
    }

    return ab_state;
}

void
motion_quad_init (motion_quad_t *p_quad, uint8_t ab_state)
{
    p_quad->count   = 0u;
    p_quad->invalid = 0u;
    p_quad->state   = (uint8_t)(ab_state & QUAD_STATE_MASK);
}

motion_quad_result_t
motion_quad_step (motion_quad_t *p_quad, uint8_t ab_state)
{
    uint8_t              new_state = (uint8_t)(ab_state & QUAD_STATE_MASK);
    uint8_t              index;
    motion_quad_result_t result;

    index = (uint8_t)((uint8_t)(p_quad->state << QUAD_INDEX_SHIFT) | new_state);
    result = (motion_quad_result_t)g_quad_table[index];

    switch (result)
    {
        case MOTION_QUAD_FWD:
            p_quad->count = (uint16_t)(p_quad->count + 1u);
            break;

        case MOTION_QUAD_REV:
            p_quad->count = (uint16_t)(p_quad->count - 1u);
            break;

        case MOTION_QUAD_INVALID:
            if (p_quad->invalid != UINT16_MAX)
            {
                p_quad->invalid++;
            }
            break;

        default:
            /* MOTION_QUAD_NONE: nothing to do. */
            break;
    }

    p_quad->state = new_state;

    return result;
}

/* ------------------------------------------------------------------ *
 *  Packed encoder counts
 * ------------------------------------------------------------------ */

#define COUNT_HALF_RANGE (0x8000u)
#define COUNT_FULL_RANGE (0x10000L)

uint32_t
motion_pack_counts (uint16_t left, uint16_t right)
{
    return ((uint32_t)right << MOTION_COUNT_R_SHIFT) | (uint32_t)left;
}

uint16_t
motion_unpack_left (uint32_t packed)
{
    return (uint16_t)(packed & MOTION_COUNT_MASK);
}

uint16_t
motion_unpack_right (uint32_t packed)
{
    return (uint16_t)((packed >> MOTION_COUNT_R_SHIFT) & MOTION_COUNT_MASK);
}

int32_t
motion_count_delta (uint16_t now, uint16_t prev)
{
    /* Unsigned subtraction is defined modulo 2^16 once truncated. */
    uint16_t diff = (uint16_t)(now - prev);
    int32_t  delta;

    if (diff >= COUNT_HALF_RANGE)
    {
        delta = (int32_t)diff - (int32_t)COUNT_FULL_RANGE;
    }
    else
    {
        delta = (int32_t)diff;
    }

    return delta;
}

/* ------------------------------------------------------------------ *
 *  Stall detector
 * ------------------------------------------------------------------ */

void
motion_stall_init (motion_stall_t *p_stall, uint16_t window_ticks,
                   int32_t min_counts)
{
    p_stall->window_ticks = window_ticks;
    p_stall->min_counts   = min_counts;
    motion_stall_reset(p_stall);
}

void
motion_stall_reset (motion_stall_t *p_stall)
{
    p_stall->idle_ticks = 0u;
    p_stall->progress   = 0;
}

bool
motion_stall_update (motion_stall_t *p_stall, bool b_driven, int32_t progress)
{
    bool b_stalled = false;

    if (!b_driven)
    {
        motion_stall_reset(p_stall);
    }
    else
    {
        if (progress > 0)
        {
            p_stall->progress += progress;
        }

        if (p_stall->progress >= p_stall->min_counts)
        {
            /* Enough movement: start a fresh window. */
            motion_stall_reset(p_stall);
        }
        else
        {
            if (p_stall->idle_ticks < UINT16_MAX)
            {
                p_stall->idle_ticks++;
            }
            if (p_stall->idle_ticks >= p_stall->window_ticks)
            {
                b_stalled = true;
            }
        }
    }

    return b_stalled;
}

/* ------------------------------------------------------------------ *
 *  PID
 * ------------------------------------------------------------------ */

void
motion_pid_init (motion_pid_t *p_pid, int32_t kp_q8, int32_t ki_q8,
                 int32_t kd_q8, int32_t out_limit)
{
    p_pid->kp_q8 =
        motion_clamp(kp_q8, -MOTION_PID_GAIN_MAX, MOTION_PID_GAIN_MAX);
    p_pid->ki_q8 =
        motion_clamp(ki_q8, -MOTION_PID_GAIN_MAX, MOTION_PID_GAIN_MAX);
    p_pid->kd_q8 =
        motion_clamp(kd_q8, -MOTION_PID_GAIN_MAX, MOTION_PID_GAIN_MAX);
    p_pid->out_limit = motion_clamp(out_limit, 0, MOTION_PID_OUT_MAX);
    motion_pid_reset(p_pid);
}

void
motion_pid_reset (motion_pid_t *p_pid)
{
    p_pid->integ_q8    = 0;
    p_pid->prev_err    = 0;
    p_pid->b_have_prev = false;
}

int32_t
motion_pid_step (motion_pid_t *p_pid, int32_t setpoint, int32_t measured)
{
    int32_t limit_q8 = p_pid->out_limit * MOTION_PID_Q8_ONE;
    int32_t err;
    int32_t p_term;
    int32_t d_term = 0;
    int32_t integ_new;
    int32_t out;

    /* Clamp both inputs before subtracting so the subtraction itself
       cannot overflow, then clamp the error. */
    setpoint = motion_clamp(setpoint, -MOTION_PID_ERR_MAX, MOTION_PID_ERR_MAX);
    measured = motion_clamp(measured, -MOTION_PID_ERR_MAX, MOTION_PID_ERR_MAX);
    err      = motion_clamp(setpoint - measured, -MOTION_PID_ERR_MAX,
                            MOTION_PID_ERR_MAX);

    p_term = p_pid->kp_q8 * err;

    if (p_pid->b_have_prev)
    {
        d_term = p_pid->kd_q8 * (err - p_pid->prev_err);
    }

    integ_new = motion_clamp(p_pid->integ_q8 + (p_pid->ki_q8 * err), -limit_q8,
                             limit_q8);

    /* Division, not a shift: right-shifting a negative value is
       implementation-defined (Barr-C 5.3b). */
    out = (p_term + integ_new + d_term) / MOTION_PID_Q8_ONE;

    if (out > p_pid->out_limit)
    {
        out = p_pid->out_limit;
        if (err < 0)
        {
            p_pid->integ_q8 = integ_new; /* helps unwind */
        }
    }
    else if (out < -p_pid->out_limit)
    {
        out = -p_pid->out_limit;
        if (err > 0)
        {
            p_pid->integ_q8 = integ_new; /* helps unwind */
        }
    }
    else
    {
        p_pid->integ_q8 = integ_new;
    }

    p_pid->prev_err    = err;
    p_pid->b_have_prev = true;

    return out;
}

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

int32_t
motion_clamp (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* In range. */
    }

    return result;
}

int32_t
motion_abs (int32_t value)
{
    int32_t result;

    if (INT32_MIN == value)
    {
        result = INT32_MAX;
    }
    else if (value < 0)
    {
        result = -value;
    }
    else
    {
        result = value;
    }

    return result;
}
