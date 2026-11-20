/*
 *  Region B of the Line & Barcode statechart:
 *  WAIT_QUIET -> ARMED -> CAPTURING -> (VALID | REJECT) -> COOLDOWN.
 *  Pure C (host-tested in tests/line).
 *
 *  Fed one barcode-sensor sample at a time (1 kHz) with the distance the
 *  robot moved since the previous sample, so every bar / space width is in
 *  micrometres of travel, independent of speed.
 *
 *  Cores: ir_sense_task (core 1) calls bc_sample() every 1 ms, and
 *  bc_resume(), bc_set_gate(), bc_junction(), bc_abort().  When
 *  bc_sample() or bc_junction() returns BC_DONE the capture is in
 *  BC_ST_DECODING and belongs to barcode_task (core 0) until that task has
 *  run bc_finish() on it and replied: in BC_ST_DECODING bc_sample() writes
 *  only the swing level and the cooldown travel, which bc_finish() never
 *  reads.
 */

#ifndef BC_ANALOG_CAPTURE_H
#define BC_ANALOG_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include "bc_analog_cfg.h"
#include "bc_analog_code39.h"
#include "bc_analog_partial.h"

typedef enum
{
    BC_NONE   = 0,
    BC_VALID  = 1,              /* p_out->ch holds the character        */
    BC_REJECT = 2,              /* p_out->reason holds the code39_result_t */
    BC_DONE   = 3               /* hand the capture to bc_finish()   */
} bc_result_t;

typedef struct
{
    char     ch;
    uint8_t  reason;            /* code39_result_t */
    bool     b_reversed;
    uint16_t elements;
    uint32_t length_um;
    uint16_t peak_norm;         /* darkest bar seen (black-level learning) */
    uint8_t  pass;              /* which reading decoded: 0 swing, 1 mid, 2 low, 3 adaptive swing,
                                   4 best match, 5 exact without an end speck,
                                   6 best match without an end speck,
                                   7 partial-read decoder */
    bc_part_t part;             /* the partial-read decoder's verdict
                                   (reason BC_PART_NOT_RUN if it did not run) */
} bc_out_t;

typedef struct
{
    uint8_t  state;             /* bc_state_t */
    bool     b_black;           /* hysteresis level */
    uint32_t run_um;            /* current run      */
    uint32_t white_um;          /* WAIT_QUIET: white so far */
    uint32_t total_um;
    uint32_t max_elem_um;
    uint32_t cooldown_um;
    uint16_t n;
    uint16_t peak;
    uint16_t ext;               /* extreme of the current element (BC_DELTA) */
    uint32_t widths[BC_MAX_ELEMENTS + 1u];
    uint16_t prof[BC_PROF_BINS];    /* darkest norm per BC_PROF_BIN_UM   */
    uint16_t prof_n;
    uint32_t bin_um;                /* travel into the current bin       */
    /* 7 Nov: white-paper levels over the last BC_QUIET_UM before a code
       (lightest, darkest: their difference is the noise a bar must beat) */
    uint16_t q_lo;
    uint16_t q_hi;
    uint16_t q_wlo;                 /* the window being measured            */
    uint16_t q_whi;
    uint16_t q3_lo[BC_QUIET_WINDOWS];   /* the last windows, newest first */
    uint16_t q3_hi[BC_QUIET_WINDOWS];
    uint8_t  q_n;                   /* windows held                   */
    /* 7 Nov: darkness record of the last BC_PRE_BINS bins while armed */
    uint16_t pre[BC_PRE_BINS];
    uint16_t pre_n;                 /* bins held (up to BC_PRE_BINS)        */
    uint16_t pre_i;                 /* next slot                            */
    uint16_t pre_cur;               /* darkest in the bin being filled      */
    uint32_t pre_um;                /* travel into it                       */
    uint8_t  read_id;               /* +1 at every capture start            */
    uint8_t  early_reason;          /* TOO_MANY / TOO_LONG seen by bc_sample() */
    bool     b_cut;                 /* ended by a junction: judge only      */
    bool     gate_open;             /* the scan gate: a capture may start   */
    uint16_t ignored;               /* captures the shut gate did not start */
    uint16_t run_prof;              /* prof_n where the current run began   */
    uint32_t max_before_um;         /* widest element before the last one   */
    bool     b_run_on;              /* the run-on last bar tried this run   */
} bc_capture_t;

void        bc_init(bc_capture_t * p_bc);

/* From a darkness profile (bins of BC_PROF_BIN_UM), the elements against
   threshold thr (bars at or above thr + hyst, spaces below thr - hyst);
   runs under BC_PROF_MIN_BINS merge into their neighbours.  Returns the
   element count (widths in um), the trailing white run excluded. */
uint16_t    bc_prof_elements(uint16_t const * p_prof, uint16_t n_bins, uint16_t thr, uint16_t hyst,
                             uint32_t * p_widths, uint16_t max);
/* As bc_prof_elements() but by swing: a bar starts delta above the
   lightest point of the space before it and ends delta below its darkest
   point (BC_ADAPT_*, 9 Nov). */
uint16_t    bc_prof_swing(uint16_t const * p_prof, uint16_t n_bins, uint16_t delta,
                          uint32_t * p_widths, uint16_t max);
/* The widths of one reading of the capture just ended (for the log):
   reading 0 swing, 1 mid, 2 low, 3 / 4 / 5 adaptive swing at
   BC_ADAPT_PCT / PCT2 / PCT3 (0 if not tried); returns the count. */
#define BC_READINGS                 6u
uint16_t    bc_reading(bc_capture_t const * p_bc, uint8_t reading, uint32_t * p_widths);
/* Light / dark levels of a profile (percentiles BC_PROF_WHITE_PCT / DARK_PCT). */
void        bc_prof_levels(uint16_t const * p_prof, uint16_t n_bins, uint16_t * p_white, uint16_t * p_dark);

/* norm: 0..1000 (1000 = black).  step_um: distance since the last sample
   (absolute value).  Returns BC_VALID once per code that ends at its stop
   character (BC_END_ON_STOP: p_out filled, capture in COOLDOWN; the
   swing level is still black when its last bar ran on into the next
   black), or
   BC_DONE when the trailing white ended a code or the capture gave up on
   it (too long, too many elements): the capture is then in
   BC_ST_DECODING for bc_finish().  Core 1. */
bc_result_t bc_sample(bc_capture_t * p_bc, uint16_t norm, uint32_t step_um, bc_out_t * p_out);

/* The end-of-code decode, on a capture in
   BC_ST_DECODING; changes nothing in it.  Returns BC_VALID (p_out->ch),
   BC_REJECT (p_out->reason) or BC_NONE (not a barcode: a crossing line or
   a speck).  Then the partial-read decoder (pass 7, p_out->part).
   Core 0: it can take milliseconds (the best match, the partial). */
bc_result_t bc_finish(bc_capture_t const * p_bc, bc_out_t * p_out);

/* After bc_finish() (its result r): BC_ST_DECODING -> ARMED for BC_NONE
   (the white that ended it counts as the next quiet zone), else COOLDOWN,
   the travel since the end already counted.  A capture cut by a junction
   always goes to COOLDOWN.  Core 1. */
void        bc_resume(bc_capture_t * p_bc, bc_result_t r);

/* The scan gate for the samples that follow (open after bc_init()).
   Shut: a bar that would start a capture starts none, adds one to
   ignored, and the capture waits for a quiet zone again. */
void        bc_set_gate(bc_capture_t * p_bc, bool b_open);

/* A junction fired while the sensor is on its crossbar.  A capture with
   no element yet (the crossbar started it) is dropped: BC_NONE, back to
   WAIT_QUIET.  A capture with elements before the crossbar is cut there
   (the crossbar and the gap before it left out): BC_DONE, judge only
   (b_cut).  Anything else: BC_NONE, nothing changed. */
bc_result_t bc_junction(bc_capture_t * p_bc);

/* Mission change: a capture in progress is dropped (back to WAIT_QUIET);
   returns true if there was one.  BC_ST_DECODING is left alone (core 0
   holds it; bc_resume() follows). */
bool        bc_abort(bc_capture_t * p_bc);

/* Bars of a capture are arriving: capturing, past its first bar, and the
   current bar is not too long to be one (BARCODE_NOT_BAR_RATIO_X10 /
   _EARLY_X10 times the widest element before the space in front of it,
   line_config.h).  The junction side never fires on such a bar. */
bool        bc_in_read(bc_capture_t const * p_bc);

/* A capture is open or being decoded (the steering hold, the console). */
bool        bc_busy(bc_capture_t const * p_bc);

#endif /* BC_ANALOG_CAPTURE_H */
