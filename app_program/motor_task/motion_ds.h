/*
 *  motion_ds.h
 *  distance_state: the word motion_task (Buddy 2) publishes every 10 ms
 *  tick for imu_task (Buddy 4).  "ds" below means distance_state.
 *
 *  This is the only file a reader needs.  Read the word with
 *  motion_ds_read() and take the fields out with the helpers below,
 *  never with your own shifts.
 *
 *  Layout, agreed with Buddy 4 (PostTri1 Version C row 17, 22-23 Oct):
 *
 *    bits  0-14  dist_mm   car's net distance in mm, modulo 32768.  It
 *                          falls when the car reverses.  Only the
 *                          difference between two reads means anything:
 *                          use motion_ds_delta_mm().
 *    bits 15-16  state     STEADY, ACCELERATING or DECELERATING (below).
 *    bits 17-19  counter   +1 on every publish, modulo 8.  The same value
 *                          twice means motion_task has not published
 *                          since your last read (stopped, or a lost tick).
 *    bits 20-31  turn      turn rate in deg/s, counter-clockwise (left)
 *                          positive, -2048..+2047, stored + 2048.
 *
 *  Distance is the average of the two wheels, so a turn on the spot adds
 *  nothing.  Turn rate and acceleration are averaged over
 *  MOTION_DS_WINDOW_TICKS ticks (motion_cfg.h).
 *
 *  One writer (motion_task, core 1) builds the whole word and publishes
 *  it with one aligned 32-bit store; the reader takes it with one load, so
 *  all four fields always come from the same tick.  No mutex: PostTri1
 *  row 2 allows that for one reader on the writer's core, which is
 *  imu_task on core 1.  Another reader, or one on core 0, needs the team
 *  to agree first (row 2 says such data keeps a mutex).
 *
 *  PROPOSED shared definition: the layout moves to common/messages.h
 *  through the team pull request.  No bit-fields (Barr-C 5.5), and only
 *  unsigned values go through the bitwise operators (Barr-C 5.3b), which
 *  is why the turn rate is stored with an offset.
 */

#ifndef MOTION_DS_H
#define MOTION_DS_H

#include <stdint.h>

/* ------------------------------------------------------------------ *
 *  Layout
 * ------------------------------------------------------------------ */

#define MOTION_DS_MM_SHIFT    (0u)
#define MOTION_DS_MM_BITS     (15u)
#define MOTION_DS_STATE_SHIFT (15u)
#define MOTION_DS_STATE_BITS  (2u)
#define MOTION_DS_CTR_SHIFT   (17u)
#define MOTION_DS_CTR_BITS    (3u)
#define MOTION_DS_TURN_SHIFT  (20u)
#define MOTION_DS_TURN_BITS   (12u)

#define MOTION_DS_MM_MASK    ((1u << MOTION_DS_MM_BITS) - 1u)    /* 0x7FFF */
#define MOTION_DS_STATE_MASK ((1u << MOTION_DS_STATE_BITS) - 1u) /* 0x3    */
#define MOTION_DS_CTR_MASK   ((1u << MOTION_DS_CTR_BITS) - 1u)   /* 0x7    */
#define MOTION_DS_TURN_MASK  ((1u << MOTION_DS_TURN_BITS) - 1u)  /* 0xFFF  */

#define MOTION_DS_MM_HALF     (1u << (MOTION_DS_MM_BITS - 1u)) /* 16384 */
#define MOTION_DS_TURN_OFFSET (2048)
#define MOTION_DS_TURN_MIN    (-2048)
#define MOTION_DS_TURN_MAX    (2047)

/* Each field starts where the one before it ends, and together they fill
   exactly 32 bits. */
_Static_assert(MOTION_DS_STATE_SHIFT
                   == (MOTION_DS_MM_SHIFT + MOTION_DS_MM_BITS),
               "state follows dist_mm");
_Static_assert(MOTION_DS_CTR_SHIFT
                   == (MOTION_DS_STATE_SHIFT + MOTION_DS_STATE_BITS),
               "counter follows state");
_Static_assert(MOTION_DS_TURN_SHIFT
                   == (MOTION_DS_CTR_SHIFT + MOTION_DS_CTR_BITS),
               "turn follows counter");
_Static_assert((MOTION_DS_TURN_SHIFT + MOTION_DS_TURN_BITS) == 32u,
               "the fields fill one 32-bit word");
_Static_assert((MOTION_DS_TURN_MAX + MOTION_DS_TURN_OFFSET)
                   == (int32_t)MOTION_DS_TURN_MASK,
               "turn rate offset matches its width");

/* Forward acceleration along the car, from the change in the average
   wheel speed (motion_cfg.h has the threshold).  While reversing and
   speeding up the car accelerates backward, so that reads DECELERATING.
   3 is never sent. */
typedef enum
{
    MOTION_DS_STEADY = 0,
    MOTION_DS_ACCEL  = 1,
    MOTION_DS_DECEL  = 2
} motion_ds_state_t;

/* The word before the first publish: distance 0, STEADY, counter 0,
   turn rate 0.  Not a starting point for differences: seed a reader's
   previous word from motion_ds_read(). */
#define MOTION_DS_WORD_ZERO                                                    \
    ((uint32_t)MOTION_DS_TURN_OFFSET << MOTION_DS_TURN_SHIFT)

/* ------------------------------------------------------------------ *
 *  Reading (imu_task, core 1: see the top of this file)
 * ------------------------------------------------------------------ */

/* The latest word, in one load.  Never blocks in the committed build
   (the MOTION_PACKED 0 measurement build waits briefly for a mutex). */
uint32_t motion_ds_read(void);

/* dist_mm field, 0..32767. */
static inline uint32_t
motion_ds_mm (uint32_t word)
{
    return (word >> MOTION_DS_MM_SHIFT) & MOTION_DS_MM_MASK;
}

/* Distance driven between two reads, in mm: + forward, - reversing.
   Both words must come from motion_ds_read().  Correct across the
   32767 -> 0 wrap while the car moves less than 16384 mm between the
   reads (at 10 ms per read, far beyond any speed this car can reach). */
static inline int32_t
motion_ds_delta_mm (uint32_t now, uint32_t then)
{
    uint32_t diff =
        (motion_ds_mm(now) - motion_ds_mm(then)) & MOTION_DS_MM_MASK;
    int32_t delta = (int32_t)diff;

    if (diff >= MOTION_DS_MM_HALF)
    {
        delta -= (int32_t)(MOTION_DS_MM_MASK + 1u);
    }

    return delta;
}

static inline motion_ds_state_t
motion_ds_state (uint32_t word)
{
    return (motion_ds_state_t)((word >> MOTION_DS_STATE_SHIFT)
                               & MOTION_DS_STATE_MASK);
}

/* Update counter, 0..7. */
static inline uint32_t
motion_ds_counter (uint32_t word)
{
    return (word >> MOTION_DS_CTR_SHIFT) & MOTION_DS_CTR_MASK;
}

/* Publishes between two reads, modulo 8: 0 = nothing new (repeated
   read), 1 = the next one, more = reads were skipped. */
static inline uint32_t
motion_ds_updates (uint32_t now, uint32_t then)
{
    return (motion_ds_counter(now) - motion_ds_counter(then))
           & MOTION_DS_CTR_MASK;
}

/* Turn rate in deg/s, counter-clockwise (left) positive. */
static inline int32_t
motion_ds_turn_dps (uint32_t word)
{
    return (int32_t)((word >> MOTION_DS_TURN_SHIFT) & MOTION_DS_TURN_MASK)
           - MOTION_DS_TURN_OFFSET;
}

/* ------------------------------------------------------------------ *
 *  Building the word (motion_task only)
 * ------------------------------------------------------------------ */

/* dist_mm is the net distance and is stored modulo 32768 (a negative
   value wraps the same way); counter keeps its low 3 bits; turn_dps is
   clamped to -2048..+2047. */
static inline uint32_t
motion_ds_pack (int32_t dist_mm, motion_ds_state_t state, uint32_t counter,
                int32_t turn_dps)
{
    int32_t turn = turn_dps;

    if (turn < MOTION_DS_TURN_MIN)
    {
        turn = MOTION_DS_TURN_MIN;
    }
    else if (turn > MOTION_DS_TURN_MAX)
    {
        turn = MOTION_DS_TURN_MAX;
    }
    else
    {
        /* In range. */
    }

    return (((uint32_t)dist_mm & MOTION_DS_MM_MASK) << MOTION_DS_MM_SHIFT)
           | (((uint32_t)state & MOTION_DS_STATE_MASK) << MOTION_DS_STATE_SHIFT)
           | ((counter & MOTION_DS_CTR_MASK) << MOTION_DS_CTR_SHIFT)
           | ((uint32_t)(turn + MOTION_DS_TURN_OFFSET) << MOTION_DS_TURN_SHIFT);
}

#endif /* MOTION_DS_H */
