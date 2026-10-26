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
 *    - distance, turn rate and acceleration for distance_state
 *
 *  Only unsigned types are used with bitwise operators (Barr-C 5.3b).
 */

#ifndef MOTION_LIB_H
#define MOTION_LIB_H

#include <stdint.h>
#include <stdbool.h>

#include "motion_cfg.h"
#include "motion_ds.h"

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
 *  A driven wheel must make at least min_counts of NET progress within
 *  window_ticks, otherwise it has stalled.  Net means the signed sum of
 *  the counts since the window started.  An encoder with one dead phase
 *  steps +1, -1, +1 ... and never gets anywhere, so it is caught; summing
 *  only the forward steps would miss it.  Progress restarts the window,
 *  and so does a change of commanded direction.
 *
 *  The phase edges tell the two FR-30 faults apart.  Each phase's edges
 *  are counted only since the other phase last moved, so edges from
 *  before a wire came off (and the one edge its pull-up makes) do not
 *  hide the fault:
 *    one phase toggled at least enc_edges times, the other stayed still
 *                                  -> encoder fault (one wire dead)
 *    anything else                 -> motor stall (blocked wheel, a wheel
 *                                     turning the wrong way, or an encoder
 *                                     with no power: without current
 *                                     sensing these look the same)
 *  Known limit: a blocked wheel whose encoder rests on a phase edge and
 *  chatters on that one phase also reads as an encoder fault.  enc_edges
 *  is set from the bench test to stay above that chatter.
 * ------------------------------------------------------------------ */

#define MOTION_STALL_MIN_COUNTS_LOW (2) /* a dead phase can leave +1 */

typedef enum
{
    MOTION_STALL_NONE = 0,
    MOTION_STALL_MOTOR,  /* motor-stall fault bit                  */
    MOTION_STALL_ENCODER /* encoder-fault bit                      */
} motion_stall_result_t;

typedef struct
{
    uint16_t window_ticks; /* [t_stall] in ticks                   */
    uint16_t idle_ticks;   /* ticks since the window started       */
    uint16_t edges_a;      /* A edges since B last moved, saturates */
    uint16_t edges_b;      /* B edges since A last moved, saturates */
    uint16_t enc_edges;    /* live-phase edges for an encoder fault */
    int32_t  min_counts;   /* [N_min], never below the _LOW value  */
    int32_t  net;          /* signed counts since the window started */
    int32_t  dir;          /* +1 / -1 commanded, 0 = either way    */
} motion_stall_t;

/* enc_edges: edges one phase must make, with the other still, before a
   stall is called an encoder fault (at least 1). */
void motion_stall_init(motion_stall_t *p_stall, uint16_t window_ticks,
                       int32_t min_counts, uint16_t enc_edges);
void motion_stall_reset(motion_stall_t *p_stall);

/* Call once per tick.
   b_driven: the wheel is commanded above its dead band.
   dir:      > 0 or < 0: only movement in that direction is progress
             (closed loop); 0: movement either way is progress (open loop,
             while the wheel's polarity is not known yet).
   delta:    signed counts moved this tick.
   edges_a, edges_b: edges seen on each encoder phase this tick.
   Returns MOTION_STALL_NONE, or which fault has been detected. */
motion_stall_result_t motion_stall_update(motion_stall_t *p_stall,
                                          bool b_driven, int32_t dir,
                                          int32_t delta, uint32_t edges_a,
                                          uint32_t edges_b);

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
 *  distance_state arithmetic  (the word itself: motion_ds.h)
 *
 *  Uses the car geometry and the window in motion_cfg.h.  Each result is
 *  worked out again from whole counts and rounded to the nearest unit,
 *  so rounding never builds up.  Counts are + forward on both wheels.
 *
 *  A wheel moves pi x D / CPR mm per count (D = wheel diameter, CPR =
 *  counts per wheel turn), so:
 *    distance  = (pos_l + pos_r) / 2 x pi x D / CPR            mm
 *    turn rate = (s_r - s_l) / B over the window, in deg/s     B = wheelbase
 *                (pi cancels: counts x D x 180 / (CPR x B x window s))
 *    accel     = change in the average wheel speed between the newest
 *                window and the one before it, in mm/s^2
 *  pi is taken as 355/113 (7 significant figures).
 * ------------------------------------------------------------------ */

/* Ticks of history kept: the newest window and the one before it. */
#define MOTION_DS_HIST (2u * MOTION_DS_WINDOW_TICKS)

typedef struct
{
    int16_t fwd[MOTION_DS_HIST];  /* d_l + d_r in each tick, counts */
    int16_t turn[MOTION_DS_HIST]; /* d_r - d_l in each tick, counts */
    uint8_t next;                 /* slot the next tick goes in     */
} motion_ds_win_t;

/* All history to zero: the car was standing still. */
void motion_ds_win_reset(motion_ds_win_t *p_win);

/* Add one tick's wheel movement in counts (each clamped to int16). */
void motion_ds_win_add(motion_ds_win_t *p_win, int32_t d_l, int32_t d_r);

/* Turn rate over the newest window, deg/s, counter-clockwise positive. */
int32_t motion_ds_win_turn_dps(motion_ds_win_t const *p_win);

/* Forward acceleration between the two windows, mm/s^2. */
int32_t motion_ds_win_accel_mmps2(motion_ds_win_t const *p_win);

/* STEADY, ACCEL or DECEL for an acceleration (MOTION_DS_ACCEL_MMPS2). */
motion_ds_state_t motion_ds_accel_state(int32_t accel_mmps2);

/* Net distance in mm from the two wheels' positions since start-up. */
int32_t motion_ds_dist_mm(int32_t pos_l, int32_t pos_r);

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

int32_t motion_clamp(int32_t value, int32_t low, int32_t high);
int32_t motion_abs(int32_t value);

#endif /* MOTION_LIB_H */
