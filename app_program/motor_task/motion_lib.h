/*
 *  motion_lib.h
 *  Pure motion arithmetic for motion_task (Buddy 2).
 *
 *  No kernel calls and no register access, so every function here also
 *  builds and runs on a PC: tests/motion_lib/ checks them with host gcc.
 *
 *  Contents
 *    - quadrature decoding (one 2-bit state per wheel)
 *    - encoder counts packed as two 16-bit halves of one 32-bit word,
 *      with a wrap-safe delta
 *    - stall detector (FR-30)
 *    - integer PI(D) speed controller with anti-windup
 *
 *  Only unsigned types are used with bitwise operators (Barr-C 5.3b).
 */

#ifndef MOTION_LIB_H
#define MOTION_LIB_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ *
 *  Quadrature decoding
 * ------------------------------------------------------------------ */

/* Result of one decoder step. */
typedef enum
{
    MOTION_QUAD_NONE = 0, /* no change on either channel          */
    MOTION_QUAD_FWD,      /* one step forward                     */
    MOTION_QUAD_REV,      /* one step backward                    */
    MOTION_QUAD_INVALID   /* both channels changed: edge missed   */
} motion_quad_result_t;

/* Decoder state for one wheel. */
typedef struct
{
    uint16_t count;   /* position, wraps modulo 65536         */
    uint16_t invalid; /* illegal transitions, saturates       */
    uint8_t  state;   /* last (A << 1) | B, 0..3              */
} motion_quad_t;

/* Build the 2-bit state from the two channel levels (0 or non-zero).
   b_swap swaps A and B, which reverses the counting direction. */
uint8_t motion_quad_ab(uint32_t level_a, uint32_t level_b, bool b_swap);

/* Start decoding from the current pin state. */
void motion_quad_init(motion_quad_t *p_quad, uint8_t ab_state);

/* Feed the current pin state; updates count/invalid and returns what
   happened.  On MOTION_QUAD_INVALID the count is left alone and the
   decoder re-synchronises to the new state. */
motion_quad_result_t motion_quad_step(motion_quad_t *p_quad, uint8_t ab_state);

/* ------------------------------------------------------------------ *
 *  Packed encoder counts
 *
 *  Both wheel counts live in one aligned 32-bit word, left in the low
 *  half and right in the high half.  The single writer (the GPIO ISR)
 *  publishes both with one store and any reader takes both with one
 *  load, so a reader can never see the left count from one edge and the
 *  right count from another.  No bit-fields: layout is fixed by the
 *  masks and shifts below (Barr-C 5.5).
 * ------------------------------------------------------------------ */

#define MOTION_COUNT_MASK    (0xFFFFu)
#define MOTION_COUNT_R_SHIFT (16u)

uint32_t motion_pack_counts(uint16_t left, uint16_t right);
uint16_t motion_unpack_left(uint32_t packed);
uint16_t motion_unpack_right(uint32_t packed);

/* Signed change from prev to now, correct across the 65535 -> 0 wrap as
   long as the wheel moved fewer than 32768 counts in between (at 10 ms
   per read that is far beyond any speed this car can reach). */
int32_t motion_count_delta(uint16_t now, uint16_t prev);

/* ------------------------------------------------------------------ *
 *  Stall detector (FR-30)
 *
 *  A driven wheel must make at least min_counts of progress within
 *  window_ticks, otherwise it is stalled (blocked wheel or dead
 *  encoder).  Progress restarts the window.
 * ------------------------------------------------------------------ */

typedef struct
{
    uint16_t window_ticks; /* [t_stall] in ticks                   */
    uint16_t idle_ticks;   /* ticks since the last progress        */
    int32_t  min_counts;   /* [N_min]                              */
    int32_t  progress;     /* counts gathered in this window       */
} motion_stall_t;

void motion_stall_init(motion_stall_t *p_stall, uint16_t window_ticks,
                       int32_t min_counts);
void motion_stall_reset(motion_stall_t *p_stall);

/* Call once per tick.  b_driven: the wheel is commanded above the dead
   band.  progress: counts moved this tick that count as progress (the
   caller decides: absolute value, or only the commanded direction).
   Returns true when the wheel is stalled. */
bool motion_stall_update(motion_stall_t *p_stall, bool b_driven,
                         int32_t progress);

/* ------------------------------------------------------------------ *
 *  Integer PID speed controller
 *
 *  Gains are Q8 fixed point (256 = 1.0).  Units are the caller's: in
 *  motion_task the setpoint and measurement are counts per tick and the
 *  output is PWM duty in per-mille.
 *
 *  Ranges that keep every intermediate inside int32_t:
 *    |gain| <= MOTION_PID_GAIN_MAX, |error| clamped to MOTION_PID_ERR_MAX,
 *    out_limit <= MOTION_PID_OUT_MAX.
 *
 *  Anti-windup: the integral is clamped to the output range, and it is
 *  not allowed to grow while the output is saturated in the direction
 *  the error is pushing (conditional integration).
 * ------------------------------------------------------------------ */

#define MOTION_PID_Q8_ONE   (256)
#define MOTION_PID_GAIN_MAX (32767)
#define MOTION_PID_ERR_MAX  (4096)
#define MOTION_PID_OUT_MAX  (10000)

typedef struct
{
    int32_t kp_q8;
    int32_t ki_q8;
    int32_t kd_q8;
    int32_t out_limit; /* |output| never exceeds this          */
    int32_t integ_q8;  /* integral term, Q8 output units       */
    int32_t prev_err;
    bool    b_have_prev; /* false until the first step           */
} motion_pid_t;

/* Gains and limit are clamped into the safe ranges above. */
void    motion_pid_init(motion_pid_t *p_pid, int32_t kp_q8, int32_t ki_q8,
                        int32_t kd_q8, int32_t out_limit);
void    motion_pid_reset(motion_pid_t *p_pid);
int32_t motion_pid_step(motion_pid_t *p_pid, int32_t setpoint,
                        int32_t measured);

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

int32_t motion_clamp(int32_t value, int32_t low, int32_t high);
int32_t motion_abs(int32_t value);

#endif /* MOTION_LIB_H */
