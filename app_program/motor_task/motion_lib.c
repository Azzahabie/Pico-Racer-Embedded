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

#define STALL_DELTA_MAX ((int32_t)0x8000)   /* one tick, as from _delta() */
#define STALL_NET_MAX   ((int32_t)0x100000) /* keeps the window sum bounded */

void
motion_stall_init (motion_stall_t *p_stall, uint16_t window_ticks,
                   int32_t min_counts, uint16_t enc_edges)
{
    p_stall->window_ticks = window_ticks;
    p_stall->min_counts =
        (min_counts < MOTION_STALL_MIN_COUNTS_LOW) ? MOTION_STALL_MIN_COUNTS_LOW
                                                   : min_counts;
    p_stall->enc_edges = (0u == enc_edges) ? 1u : enc_edges;
    p_stall->dir       = 0;
    motion_stall_reset(p_stall);
}

void
motion_stall_reset (motion_stall_t *p_stall)
{
    p_stall->idle_ticks = 0u;
    p_stall->edges_a    = 0u;
    p_stall->edges_b    = 0u;
    p_stall->net        = 0;
}

/* Saturating add for the per-window edge counts. */
static uint16_t
stall_add_edges (uint16_t total, uint32_t edges)
{
    uint32_t sum = UINT16_MAX;

    if (edges < UINT16_MAX)
    {
        sum = (uint32_t)total + edges; /* both below 2^16: cannot wrap */
    }

    return (sum > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)sum;
}

/* Which fault a window without enough progress means (see the header). */
static motion_stall_result_t
stall_classify (motion_stall_t const *p_stall)
{
    motion_stall_result_t result = MOTION_STALL_MOTOR;
    uint16_t              need   = p_stall->enc_edges;

    if (((p_stall->edges_a >= need) && (0u == p_stall->edges_b))
        || ((p_stall->edges_b >= need) && (0u == p_stall->edges_a)))
    {
        result = MOTION_STALL_ENCODER;
    }

    return result;
}

motion_stall_result_t
motion_stall_update (motion_stall_t *p_stall, bool b_driven, int32_t dir,
                     int32_t delta, uint32_t edges_a, uint32_t edges_b)
{
    motion_stall_result_t result = MOTION_STALL_NONE;
    int32_t               way;
    int32_t               progress;

    if (!b_driven)
    {
        motion_stall_reset(p_stall);
    }
    else
    {
        way = (dir > 0) ? 1 : ((dir < 0) ? -1 : 0);
        if (way != p_stall->dir)
        {
            /* A new commanded direction starts a fresh window. */
            motion_stall_reset(p_stall);
            p_stall->dir = way;
        }

        p_stall->net = motion_clamp(
            p_stall->net + motion_clamp(delta, -STALL_DELTA_MAX, STALL_DELTA_MAX),
            -STALL_NET_MAX, STALL_NET_MAX);
        /* Each phase counts only since the other phase last moved. */
        p_stall->edges_a = (0u != edges_b)
                               ? 0u
                               : stall_add_edges(p_stall->edges_a, edges_a);
        p_stall->edges_b = (0u != edges_a)
                               ? 0u
                               : stall_add_edges(p_stall->edges_b, edges_b);

        if (0 == way)
        {
            progress = motion_abs(p_stall->net);
        }
        else if (way > 0)
        {
            progress = p_stall->net;
        }
        else
        {
            progress = -p_stall->net;
        }

        if (progress >= p_stall->min_counts)
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
                result = stall_classify(p_stall);
            }
        }
    }

    return result;
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
