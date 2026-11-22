/*
 *  motor_task.c
 *  motion_task (Buddy 2), bench bring-up version.  See motor_task.h.
 *
 *  Two tasks:
 *
 *    motion_task  core 1, priority 11.  Woken every 10 ms by a cyclic
 *                 handler through an event flag.  Reads both encoder
 *                 counts together, runs the bench test and the stall
 *                 check, writes the motors, publishes distance_state for
 *                 imu_task (motion_ds.h), then feeds the hardware
 *                 watchdog.  Never prints and never waits on anything
 *                 except its event flag.
 *
 *    motion_report  core 0, low priority.  Receives fixed-size records
 *                 from motion_task through a message buffer and prints
 *                 them.  tm_printf masks interrupts on the calling core
 *                 for the whole line in the USB-CDC build, which on core 1
 *                 would delay the encoder interrupt and lose counts.
 *
 *  Bench test (MOTION_BENCH = 1 only, wheels off the ground): press the
 *  Robo Pico button on MOTION_PIN_BUTTON to start, press again to abort.
 *  With MOTION_BENCH = 0 (the default) the button code is not built, GP20
 *  is never touched and the motors stay braked.  Either way, turning a
 *  wheel by hand prints its count.  Steps and how to read the output:
 *  docs/motion/README.md.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "../usb_console_compat.h"
#include "../common/gpio_irq.h"
#include "../common/messages.h"
#include "../comms_task/comms_task.h"
#include "../nav_task/nav_ctrl.h"
#include "../obstacle_task/scan_if.h"
#include "../line_task/line_task.h"
#include "../line_task/line_state.h"
#include "../line_task/barcode_action.h"
#include "../imu_task/imu_task.h"       /* hump_state word (D1) */
#include "../imu_task/imu_events.h"     /* ev_heading_deg: bypass heading */
#include "../common/line_config.h"      /* LINE_LOST_MS */
#include "motion_cfg.h"
#include "motion_lib.h"
#include "motion_ds.h"
#include "motor.h"
#include "encoder.h"
#include "motion_wdt.h"
#include "motion_sm.h"
#include "motor_task.h"

/* A lost tick is braked before the watchdog resets the chip (W8). */
_Static_assert(((uint32_t)MOTION_TICK_TIMEOUT_MS > MOTION_TICK_MS)
                   && ((uint32_t)MOTION_TICK_TIMEOUT_MS < MOTION_WDT_MS),
               "MOTION_TICK_TIMEOUT_MS must lie between a tick and the"
               " watchdog timeout");

/* The motion_flg bits (common/messages.h) are four different bits. */
_Static_assert((MOTION_EVT_TICK | MOTION_EVT_CMD | MOTION_EVT_STOP_SCAN
                | MOTION_EVT_STOP_IMU)
                   == 0x0Fu,
               "motion_flg bits overlap");

/* ------------------------------------------------------------------ *
 *  Constants
 * ------------------------------------------------------------------ */

#define TIMER_RAWL (TIMER_BASE + 0x28u) /* 1 MHz, no side effects */

#define REPORT_TASK_PRI   (24)   /* below every design task */
#define REPORT_TASK_STACK (4096) /* SMP kernel path, see app_main */
#define REPORT_QUEUE_LEN  (8u)   /* records in the mbf      */
#define REPORT_LINE_LEN   (192)
#define MBF_HEADER_BYTES  (4u) /* per message, messagebuf.h */

#define INIT_FAIL_POLL_MS  (100) /* set-up failed: mission check */

#define CONSOLE_WAIT_POLLS (200) /* x 100 ms = 20 s max     */
#define CONSOLE_WAIT_MS    (100)
#define CONSOLE_ENUMERATED (2u)

#define TIMING_REPORT_S (5u)
#define TICKS_PER_S     (MS_PER_S / (INT)MOTION_TICK_MS)
#define TIMING_REPORT_TICKS                                                    \
    ((TIMING_REPORT_S * (uint32_t)MS_PER_S) / MOTION_TICK_MS)
#define MS_PER_S              (1000)
#define US_PER_MS             (1000u)
#define CROSSTALK_RATIO       (4) /* other wheel > 1/4 of driven */
#define MOTION_PRC_SINGLE     (1) /* processor number, 1 core    */
#define CROSSTALK_MIN_COUNTS  (20)

/* ------------------------------------------------------------------ *
 *  Bench sequence
 * ------------------------------------------------------------------ */

typedef struct
{
    char const *p_name;
    int16_t     duty_l;
    int16_t     duty_r;
    uint16_t    ticks;
} bench_step_t;

#define D_TEST (MOTION_TEST_DUTY)

static bench_step_t const g_steps[] = {
    { "L fwd", D_TEST, 0, MOTION_TEST_STEP_TICKS },
    { "brake", 0, 0, MOTION_TEST_GAP_TICKS },
    { "L rev", -D_TEST, 0, MOTION_TEST_STEP_TICKS },
    { "brake", 0, 0, MOTION_TEST_GAP_TICKS },
    { "R fwd", 0, D_TEST, MOTION_TEST_STEP_TICKS },
    { "brake", 0, 0, MOTION_TEST_GAP_TICKS },
    { "R rev", 0, -D_TEST, MOTION_TEST_STEP_TICKS },
    { "brake", 0, 0, MOTION_TEST_GAP_TICKS },
    { "both fwd", D_TEST, D_TEST, MOTION_TEST_STEP_TICKS },
    { "brake", 0, 0, MOTION_TEST_GAP_TICKS }
};

#define STEP_COUNT (sizeof(g_steps) / sizeof(g_steps[0]))

typedef enum
{
    PHASE_IDLE = 0,
    PHASE_STEPS,
    PHASE_RAMP,
    PHASE_PID,
    PHASE_STOPTEST /* MOTION_TEST_STOP only */
} motion_phase_t;

/* STOP sources, for the request stamps and the stop test. */
#define STOP_SRC_SCAN (0u)
#define STOP_SRC_IMU  (1u)
#define STOP_SOURCES  (2u)

/* ------------------------------------------------------------------ *
 *  Report records (motion_task -> motion_report)
 * ------------------------------------------------------------------ */

typedef enum
{
    REP_STATUS = 0, /* val = position, rate = counts/s        */
    REP_STEP,       /* val = moved in the step, rate = c/s    */
    REP_EVENT,      /* code = EVT_*, val/rate = arguments     */
    REP_TIMING,     /* timing of the last 5 s; rate_l = longest
                       gap between watchdog feeds, us; rate_r
                       = E motion sends refused so far        */
    REP_DIST        /* distance_state: code = state, val_l =
                       net mm, val_r = turn deg/s, rate_l =
                       accel mm/s^2, rate_r = counter         */
} report_kind_t;

typedef enum
{
    EVT_READY = 0, /* val_l = init ercd, val_r = processor    */
    EVT_START,     /* val_l = PID target in counts/s (or 0)   */
    EVT_DONE,
    EVT_ABORT,    /* button                                  */
    EVT_FAULT,    /* faults byte says which                  */
    EVT_RAMP,     /* val = first duty that moved, 0 = none   */
    EVT_BOOT,     /* val_l = motion_boot_t, val_r = watchdog
                     test result in us or -1, rate_l = watchdog
                     timeout in ms, 0 = not armed            */
    EVT_STOP,     /* val_l = STOP bit newly seen, val_r = us
                     from the request to the brake, or -1 if
                     not stamped, rate_l = 1 if this stop
                     wrote the brake, 0 if already braked   */
    EVT_GO,       /* every STOP bit clear; val_l = ms since the first */
    EVT_STOPTEST, /* stop test result for one source: val_l
                     = STOP bit, val_r = stops, rate_l = max
                     us, rate_r = mean us, duty_l = min us  */
    EVT_STATE,    /* statechart: val_l = new state, val_r =
                     old state, rate_l = replies sent       */
    EVT_CMD,      /* a command taken from a buffer: val_l =
                     CMD_SRC_*, val_r = the raw 2 bytes,
                     rate_l = CMD_RES_*; for a JUNCTION's
                     outcome rate_r = the state it met and
                     the state it went to (CMD_WENT_SHIFT) */
    EVT_BYP,      /* bypass phase: val_l = phase, val_r =
                     sideways mm toward the side, rate_l =
                     heading - line deg (x side), rate_r =
                     1 IMU heading in use, 0 encoders only  */
    EVT_SM_FAULT  /* statechart entered Motion Fault: faults
                     byte; duty_l/_r = the duty the stall check
                     judged; val_l/_r = net counts in each
                     wheel's stall window, rate_l/_r = its
                     length in ticks                        */
} report_event_t;

/* EVT_CMD fields */
#define CMD_SRC_NAV       (0)
#define CMD_SRC_AVOID     (1)
#define CMD_RES_TAKEN     (0) /* passed to the statechart          */
#define CMD_RES_RECEIVED  (1) /* JUNCTION read; its outcome follows */
#define CMD_RES_INVALID   (2) /* unknown or malformed: dropped      */
#define CMD_RES_ACCEPTED  (3) /* JUNCTION: a new turn started       */
#define CMD_RES_REFUSED   (4) /* JUNCTION: refused, CMD_REJECTED    */
/* A JUNCTION's outcome, rate_r: bits 0-7 the state the JUNCTION met, bits
   8-15 the state the statechart went to in that tick. */
#define CMD_STATE_MASK    (0xFFu)
#define CMD_WENT_SHIFT    (8u)

typedef struct
{
    uint8_t  kind;
    uint8_t  code;
    uint8_t  phase;
    uint8_t  faults;
    uint32_t t_ms;
    int32_t  val_l;
    int32_t  val_r;
    int32_t  rate_l;
    int32_t  rate_r;
    int16_t  duty_l;
    int16_t  duty_r;
    uint16_t inv_l;
    uint16_t inv_r;
    uint32_t per_min_us;
    uint32_t per_max_us;
    uint32_t work_max_us;
    uint32_t irq_count;
    uint32_t isr_max_us;
} motion_report_t;

/* ------------------------------------------------------------------ *
 *  motion_task state (touched only by motion_task)
 * ------------------------------------------------------------------ */

typedef struct
{
    /* encoders */
    uint16_t prev_l;
    uint16_t prev_r;
    int32_t  pos_l;
    int32_t  pos_r;        /* in left-wheel units (see tick_encoders) */
    int32_t  raw_pos_r;    /* right counts as the encoder gives them   */
    int32_t  d_l;
    int32_t  d_r;
    uint32_t prev_edges_l; /* packed per-phase edge counts */
    uint32_t prev_edges_r;
    uint32_t e_la;         /* phase edges in the last tick  */
    uint32_t e_lb;
    uint32_t e_ra;
    uint32_t e_rb;

    /* bench test */
    motion_phase_t phase;
    uint32_t       step;
    uint32_t       step_ticks;
    int32_t        mark_l; /* position at step / level start */
    int32_t        mark_r;
    int32_t        ramp_duty;
    int32_t        ramp_found_l;
    int32_t        ramp_found_r;
    int32_t        cmd_l;
    int32_t        cmd_r;
    motion_pid_t   pid_l;
    motion_pid_t   pid_r;
    motion_stall_t stall_l;
    motion_stall_t stall_r;
    uint8_t        faults;

#if MOTION_BENCH
    /* button */
    bool     b_btn_raw;
    bool     b_btn_down;
    uint32_t btn_same;
#endif

    /* timing */
    uint32_t ticks;
    uint32_t last_us;
    uint32_t per_min_us;
    uint32_t per_max_us;
    uint32_t work_max_us;
    uint32_t lost_ticks;
    uint32_t irq_mark;
    uint32_t wdt_fed_us;     /* when the watchdog was last fed  */
    uint32_t wdt_gap_max_us; /* longest gap between two feeds   */

    /* stop path (NFR-18) */
    uint32_t stop_bits;                   /* STOP bits at the last check */
    uint32_t stop_t_us;                   /* when the brake was written  */
    uint32_t stop_req_used[STOP_SOURCES]; /* last request stamp used */
#if MOTION_BENCH && MOTION_TEST_STOP
    uint32_t st_drive; /* ticks driving since the last stop */
    uint32_t st_n[STOP_SOURCES];
    uint32_t st_min_us[STOP_SOURCES];
    uint32_t st_max_us[STOP_SOURCES];
    uint32_t st_sum_us[STOP_SOURCES];
#endif

    /* distance_state (motion_ds.h): the last values published */
    motion_ds_win_t   ds_win;
    motion_ds_state_t ds_state;
    int32_t           ds_mm;      /* net distance, mm               */
    int32_t           ds_turn;    /* deg/s, counter-clockwise +     */
    int32_t           ds_accel;   /* mm/s^2, for the status line    */
    uint32_t          ds_counter; /* publishes so far (word: 3 bits) */

    /* statechart 6.2 (normal image only) */
    motion_sm_t sm;
    int32_t     man_start_l; /* positions when the manoeuvre began */
    int32_t     man_start_r;
    int32_t     man_mm;    /* distance it was asked to drive, mm */
    uint8_t     avoid_cmd; /* motion_sm_avoid_t waiting for a tick */
    uint16_t    avoid_mm;  /* its distance (step 1.7 fills both)  */
    uint8_t     avoid_side; /* MOTION_AVOID_SIDE_* of a BYPASS     */
    uint32_t    tele_dropped; /* E motion sends tele_mbf refused   */
    uint32_t    tele_ticks;   /* ticks since the last E motion try   */
    int32_t     tele_pos_l;   /* positions at the last E motion sent */
    int32_t     tele_pos_r;
    uint8_t     tele_refused; /* E motion sends refused in a row     */
    uint32_t    mission_counts; /* |change of pos_l + pos_r|, mission */
    int32_t     odo_prev;     /* pos_l + pos_r at the last tick      */
    uint8_t     mission_prev; /* mission_state_t at the last tick    */

    /* line_state (line_state.h), read once per tick */
    uint32_t line_word;     /* the word this tick                  */
    uint32_t line_ctr;      /* its counter at the last change      */
    uint32_t line_same;     /* ticks the counter has not moved     */
    uint32_t line_seen_run; /* ticks in a row the line was seen    */
    bool     b_on_hump;     /* hump_state != NONE this tick (M3)    */
    uint32_t dark_sat_ticks; /* ticks with line dark time saturated */
    int32_t  line_dir;      /* +1 line last left, -1 right, 0 none */
    int32_t  lf_rem;        /* steering remainder, x 1/MOTION_LF_SCALE
                               counts per tick, carried to the next */
    int32_t  lf_corr;       /* applied correction, same units      */
    int32_t  lf_hist[MOTION_LF_DAMP_TICKS]; /* recent errors, ring */
    uint32_t lf_hist_i;     /* the oldest entry                    */
    uint32_t lf_hard_run;   /* ticks |error| >= MOTION_LF_HARD_ERR */

    /* line search: +1 pivots left (counter-clockwise), -1 right */
    int32_t search_dir;
    uint8_t search_phase;   /* 0 toward the line, 1 the other way  */
    bool    b_search_over;  /* both sweeps done, no line            */

    /* turning: the junction's action and how far the turn has got */
    uint8_t turn_action;    /* barcode_action_t of the turn now     */
    uint8_t turn_req;       /* ... of the last JUNCTION received     */
    uint8_t turn_phase;     /* TURN_PH_*                            */
    int32_t turn_start_l;   /* positions at the phase's start, kept */
    int32_t turn_start_r;   /* while Braked so a resume carries on  */

    /* bypass: side, offset and leg, kept like the turn's */
    int32_t bypass_side;    /* +1 left (counter-clockwise), -1 right */
    int32_t bypass_off_mm;  /* lateral offset from scan_task         */
    uint8_t bypass_phase;   /* BYP_PH_*                              */
    int32_t bypass_start_l; /* positions at the phase's start        */
    int32_t bypass_start_r;
    int32_t bypass_pass_mm; /* driven alongside so far               */
    bool    b_bypass_seen;  /* a probe has seen the obstacle         */
    bool    b_look_present; /* the last probe's answer               */

    /* IMU heading reference (MOTION_HD_*, motion_cfg.h) */
    int64_t  hd_line_udeg;  /* line direction: average heading, udeg */
    int32_t  hd_prev_deg;   /* heading last tick                     */
    int32_t  hd_travel;     /* Line Following travel in the average,
                               2 x counts (capped at MOTION_HD_MIN_MM) */
    bool     b_hd_line;     /* an average has started                */
    bool     b_hd_ok;       /* this tick: heading usable             */
    int32_t  hd_err_deg;    /* this tick: heading - line, + = left   */
    int32_t  hd_lat_acc;    /* sideways: sum of counts x sin1000     */
    int32_t  hd_lon_acc;    /* along the line: sum of counts x cos1000 */
    int32_t  byp_lat_target;/* sidestep end, mm toward the side      */
    int32_t  byp_pass_more; /* known box: extra mm past, way in blocked */
    bool     b_arc;         /* this bypass runs the arc (MOTION_BYPASS_ARC) */
    int32_t  arc_r;         /* circle radius about the box centre, mm */
    int32_t  arc_cx;        /* box centre: sideways toward the side, mm */
    int32_t  arc_cy;        /* box centre: along the line, mm         */
    int32_t  arc_deg;       /* tangent angle theta off the line, deg  */
    int32_t  arc_xt;        /* tangent point: sideways, mm            */
    int32_t  arc_yt;        /* tangent point: along the line, mm      */
    int32_t  arc_leg;       /* straight leg to the tangent point, mm  */
    bool     b_smooth;      /* this bypass runs the smooth curve      */
    bool     b_sm_out;      /* smooth: has been out (x > Y / 2)       */
    int32_t  sm_y;          /* smooth: offset alongside the box, mm   */
    int32_t  sm_x0;         /* smooth: axle off the line at the start */
    int32_t  sm_rem;        /* smooth: steering remainder, x16        */
    int32_t  sm_s_in;       /* smooth: where the way back in starts   */
    bool     b_sm_watch;    /* smooth: the side watch is on           */
    bool     b_sm_seen;     /* smooth: it has seen the box            */
    bool     b_sm_new;      /* smooth: a new side reading             */
    uint8_t  sm_clear_n;    /* smooth: clear readings in a row        */
    uint16_t sm_watch_mm;   /* smooth: the last side reading          */
    int32_t  sm_seen_s;     /* smooth: along the line when first seen */
    int32_t  sm_fb_s_in;    /* smooth: way back in if the side is never seen */
    int32_t  sm_y_plan;     /* smooth: the planned pass (widening cap)  */
    int32_t  sm_far_seen;   /* smooth: furthest box echo along the line */
    int32_t  sm_edge_max;   /* smooth: box side's furthest echo out     */
    int32_t  sm_need_prev;  /* smooth: last echo's wanted pass, 0 none  */
    int32_t  hd_snap_trav;  /* travel off the average past the snap   */
    bool     b_lf_edge;     /* the line was last seen by one sensor  */
    int32_t  lf_edge_err;   /* the error it read then                */
    uint16_t lf_curve_ticks; /* curve speed: ticks left               */
    uint16_t lf_edge_run;   /* ticks in a row at one sensor's edge   */
    int32_t  lf_gap2c;      /* white so far: d_l + d_r summed         */
    int32_t  lf_gap_h0;     /* heading when the white began          */
    uint16_t lf_single_ticks; /* ticks in a row with one sensor on    */
    /* MOTION_LF_STRADDLE: sensors either side of the line (st_*) */
    int32_t  st_mem;        /* learnt curve, 1e6 / radius mm, + = left */
    int32_t  st_steer;      /* curvature steered now                   */
    int32_t  st_kick;       /* self-tuned nudge for a touch            */
    int8_t   st_side;       /* -1 left black, +1 right black, 0 none   */
    int8_t   st_last_touch; /* side of the last touch                  */
    uint32_t st_touch_um;   /* driven since the last touch began       */
    uint32_t st_black_um;   /* driven with this side black             */
    uint32_t st_bb_um;      /* driven with both black                  */
    uint32_t st_nt_um;      /* driven with neither black (lost after)  */
    int32_t  st_rem_l;      /* mm/s -> counts per tick remainders      */
    uint32_t st_end_um;     /* driven since the last touch ended       */
    int8_t   st_end_side;   /* side of the touch that ended last       */
    int8_t   st_out;        /* line gone out past this side's sensor   */
    bool     b_st_trav;     /* this touch began just after the other side's */
    bool     b_st_bc;       /* holding the heading over a barcode      */
    bool     b_st_outer;    /* the touch that ended last was the curve's outside */
    int32_t  turn_h_target; /* MOTION_TURN_IMU: heading to pivot to    */
    bool     b_turn_lead;   /* the pivot's leading sensor has met the new line */
    uint32_t sm_touch_um;   /* bypass: driven since this touch began     */
    bool     b_sm_cross;    /* bypass: this touch is a crossing line     */
    int32_t  st_dd16;       /* wheels d_r - d_l, averaged, x16         */
    int32_t  st_ds16;       /* wheels d_r + d_l, averaged, x16         */
    uint32_t st_out_um;     /* driven while pivoting back to the line  */
    bool     b_st_recov;    /* the touch now ended a recovery pivot     */
    bool     b_st_after_recov; /* white since a recovery's touch: lost soon */
    bool     b_post_bypass; /* a bypass ran: check the line once back   */
    int32_t  st_bc_h0;      /* heading held, deg left +                */
    int32_t  st_rem_r;
    uint8_t  ra_phase;      /* realign after a search: RA_*          */
    bool     b_ra_valid;    /* ra_h_pre holds the heading before loss */
    bool     b_ra_pending;  /* just found the line after a search     */
    bool     b_gap_start;   /* this white began as a gap, not an edge  */
    int32_t  ra_h_pre;      /* heading when the line went white       */
    uint32_t ra_ticks;
    int32_t  ra_l0;
    int32_t  ra_r0;
    int32_t  lf_corr_avg;   /* correction, averaged (~160 ms)         */
    uint8_t  rt_phase;      /* barcode re-read: RT_*                  */
    bool     b_rt_used;     /* re-read done for this barcode          */
    bool     b_rt_bar;      /* barcode bit last tick                  */
    bool     b_rt_seen;     /* slow pass: the read has started        */
    uint32_t rt_req_seen;   /* barcode_retry_req() last seen          */
    uint32_t rt_ticks;      /* ticks in this phase                    */
    int32_t  rt_back_mm;    /* how far to back up                     */
    int32_t  rt_read_l;     /* wheel counts where the read began      */
    int32_t  rt_read_r;
    int32_t  rt_l0;         /* wheel counts at this phase's start     */
    int32_t  rt_r0;
    int32_t  lf_rate16;     /* turn rate, d_r - d_l smoothed, x16    */
    bool     b_sm_settle;   /* smooth: lined up, braked a moment      */
    uint8_t look_state;     /* LOOK_*: the probe's request to scan   */
    uint16_t look_mm;       /* scan_task's AVOID_LOOK reading        */
    uint32_t look_ticks;    /* ticks waited for it                   */

    /* reporting */
    int32_t  rep_l;
    int32_t  rep_r;
    uint32_t rep_ticks;
    uint32_t rep_dropped;
} motion_ctx_t;

/* Every wheel speed target goes through here: capped at V_hump while
   imu_task reports a hump (M3, motion_cfg.h). */
static int32_t
pid_capped (motion_ctx_t const *p_ctx, motion_pid_t *p_pid, int32_t target,
            int32_t measured)
{
    if (p_ctx->b_on_hump)
    {
        target = motion_clamp(target, -MOTION_HUMP_MAX_CPT,
                              MOTION_HUMP_MAX_CPT);
    }
    return motion_pid_step(p_pid, target, measured);
}

static motion_ctx_t g_ctx;

/* ------------------------------------------------------------------ *
 *  distance_state, shared with imu_task (motion_ds.h)
 * ------------------------------------------------------------------ */

#if MOTION_PACKED
/* One aligned word: motion_task is the only writer and stores it whole
   once per tick; a reader takes it with one load. */
static volatile uint32_t g_ds_word = MOTION_DS_WORD_ZERO;
#else
/* Week 6 form, kept only to measure the Week 13 "before": a struct
   guarded by a priority-inheritance mutex, held only to copy. */
typedef struct
{
    int32_t dist_mm;
    int16_t turn_dps;
    uint8_t state;
    uint8_t counter;
} ds_week6_t;

static ds_week6_t g_ds;
static ID         g_ds_mtxid;

static T_CMTX const g_cmtx_ds = {
    .mtxatr = TA_INHERIT,
};
#endif

/* The statechart state for ir_sense_task's scan gate (motor_task.h,
   motion_state_read()): motion_task is the only writer and stores it
   whole once per tick, after the statechart step; a reader takes it with
   one load.  Never written in bench images. */
static volatile uint32_t g_state_word = (uint32_t)MOTION_ST_IDLE;

/* ------------------------------------------------------------------ *
 *  Kernel objects
 * ------------------------------------------------------------------ */

static ID g_flgid;
static ID g_mbfid;
static ID g_nav_mbfid;   /* nav_cmd_mbf:   nav_ctrl_task -> motion */
static ID g_avoid_mbfid; /* avoid_cmd_mbf: scan_task -> motion     */

/* What tk_cre_cyc gave for the tick.  Set by motor_task_start() before
   motion_task starts; motion_task arms the watchdog only if it is E_OK. */
static ER g_tick_ercd = E_NOEXS;

/* NFR-18 request times, one word per STOP source.  Each is written only
   by its own sender (motion_stop_stamp) and read only by motion_task. */
static volatile uint32_t g_stop_req_us[STOP_SOURCES];

#if MOTION_BENCH && MOTION_TEST_STOP
/* Stop test: motion_task -> stand-in, "both wheels are turning". */
static volatile bool gb_st_ready;
#endif

static UW g_report_buf[(REPORT_QUEUE_LEN
                        * (sizeof(motion_report_t) + MBF_HEADER_BYTES))
                       / sizeof(UW)];

/* Both task stacks are ours (TA_USERBUF), not taken from the kernel heap,
   so the linker map and nm show all of motion's RAM (Week 13 evidence 2).
   8-byte aligned, as the ARM calling standard needs for a stack. */
static _Alignas(8) UW g_motion_stack[MOTION_TASK_STACK / sizeof(UW)];
static _Alignas(8) UW g_report_stack[REPORT_TASK_STACK / sizeof(UW)];
/* With TA_USERBUF the kernel no longer rounds the size, so the stack top
   stays 8-byte aligned only if each size is a multiple of 8. */
_Static_assert((0 == (MOTION_TASK_STACK % 8)) && (0 == (REPORT_TASK_STACK % 8)),
               "task stack sizes must be multiples of 8");

static void motion_cyclic_isr(void *exinf);
static void motion_report_task(INT stacd, void *exinf);

static T_CFLG const g_cflg = {
    .flgatr  = TA_TFIFO | TA_WSGL,
    .iflgptn = 0u,
};

static T_CMBF const g_cmbf = {
    .mbfatr = TA_TFIFO | TA_USERBUF,
    .bufsz  = sizeof(g_report_buf),
    .maxmsz = sizeof(motion_report_t),
    .bufptr = g_report_buf,
};

/* The two command buffers (design Section 4), one writer each.  Both
   messages are 2 bytes, which the kernel stores in one word after its
   header. */
#define CMD_MSG_BYTES  (2u)
#define CMD_SLOT_BYTES (MBF_HEADER_BYTES + 4u)
#define NAV_CMD_ROOM   (4u) /* nav_cmd_mbf:   JUNCTION, FAULT_CLEAR */
#define AVOID_CMD_ROOM (2u) /* avoid_cmd_mbf: REVERSE, BYPASS       */

static UW g_nav_buf[(NAV_CMD_ROOM * CMD_SLOT_BYTES) / sizeof(UW)];
static UW g_avoid_buf[(AVOID_CMD_ROOM * CMD_SLOT_BYTES) / sizeof(UW)];

static T_CMBF const g_cmbf_nav = {
    .mbfatr = TA_TFIFO | TA_USERBUF,
    .bufsz  = sizeof(g_nav_buf),
    .maxmsz = CMD_MSG_BYTES,
    .bufptr = g_nav_buf,
};

static T_CMBF const g_cmbf_avoid = {
    .mbfatr = TA_TFIFO | TA_USERBUF,
    .bufsz  = sizeof(g_avoid_buf),
    .maxmsz = CMD_MSG_BYTES,
    .bufptr = g_avoid_buf,
};

static T_CCYC const g_ccyc = {
    .cycatr = TA_HLNG | TA_STA,
    .cychdr = (FP)motion_cyclic_isr,
    .cyctim = MOTION_TICK_MS,
    .cycphs = MOTION_TICK_MS,
};

static T_CTSK const g_ctsk_motion = {
    .itskpri = MOTION_TASK_PRI,
    .stksz   = sizeof(g_motion_stack),
    .bufptr  = g_motion_stack,
    .task    = (FP)motor_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC2, /* core 1 */
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

static T_CTSK const g_ctsk_report = {
    .itskpri = REPORT_TASK_PRI,
    .stksz   = sizeof(g_report_stack),
    .bufptr  = g_report_stack,
    .task    = (FP)motion_report_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1, /* core 0 */
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

#if MOTION_BENCH && MOTION_TEST_STOP
static void stop_standin_task(INT stacd, void *exinf);

static _Alignas(8) UW g_standin_stack[REPORT_TASK_STACK / sizeof(UW)];

/* Same core as scan_task and imu_task, at imu_task's priority. */
static T_CTSK const g_ctsk_standin = {
    .itskpri = MOTION_TEST_STOP_PRI,
    .stksz   = sizeof(g_standin_stack),
    .bufptr  = g_standin_stack,
    .task    = (FP)stop_standin_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC2, /* core 1 */
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};
#endif

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

static uint32_t
now_us (void)
{
    return in_w(TIMER_RAWL);
}

/* Bench images return 0, so scan_task stays print-only: the GP20 press
   that starts a bench test also starts nav's mission, and scan_task's own
   STOP_SCAN (anything within 25 cm, or no sensor) would end the test.
   The stop test's stand-in uses g_flgid directly. */
ID
motor_task_flag (void)
{
    ID flgid = g_flgid;

#if MOTION_BENCH
    flgid = 0;
#endif

    return flgid;
}

/* The command buffers, for nav_bind() and scan_if_bind().  0 in bench
   images, for the same reason as motor_task_flag(). */
ID
motor_task_nav_cmd_mbf (void)
{
    ID mbfid = g_nav_mbfid;

#if MOTION_BENCH
    mbfid = 0;
#endif

    return mbfid;
}

ID
motor_task_avoid_mbf (void)
{
    ID mbfid = g_avoid_mbfid;

#if MOTION_BENCH
    mbfid = 0;
#endif

    return mbfid;
}

void
motion_stop_stamp (UINT stop_bit)
{
    uint32_t t_now = now_us();

    if (0u != (stop_bit & MOTION_EVT_STOP_SCAN))
    {
        g_stop_req_us[STOP_SRC_SCAN] = t_now;
    }
    if (0u != (stop_bit & MOTION_EVT_STOP_IMU))
    {
        g_stop_req_us[STOP_SRC_IMU] = t_now;
    }
}

/* Counts moved over n ticks -> counts per second. */
static int32_t
rate_cps (int32_t moved, uint32_t n_ticks)
{
    int32_t rate = 0;

    if (n_ticks > 0u)
    {
        rate = (moved * MS_PER_S) / (int32_t)(n_ticks * MOTION_TICK_MS);
    }

    return rate;
}

static void
report_fill (motion_ctx_t const *p_ctx, motion_report_t *p_rep,
             report_kind_t kind, uint8_t code)
{
    uint32_t inv = encoder_invalid_packed();

    p_rep->kind        = (uint8_t)kind;
    p_rep->code        = code;
    p_rep->phase       = (uint8_t)p_ctx->phase;
    p_rep->faults      = p_ctx->faults;
    p_rep->t_ms        = p_ctx->ticks * MOTION_TICK_MS;
    p_rep->val_l       = 0;
    p_rep->val_r       = 0;
    p_rep->rate_l      = 0;
    p_rep->rate_r      = 0;
    p_rep->duty_l      = (int16_t)motor_get_duty(MOTOR_LEFT);
    p_rep->duty_r      = (int16_t)motor_get_duty(MOTOR_RIGHT);
    p_rep->inv_l       = motion_unpack_left(inv);
    p_rep->inv_r       = motion_unpack_right(inv);
    p_rep->per_min_us  = p_ctx->per_min_us;
    p_rep->per_max_us  = p_ctx->per_max_us;
    p_rep->work_max_us = p_ctx->work_max_us;
    p_rep->irq_count   = encoder_irq_count();
    p_rep->isr_max_us  = 0u;
}

/* Never blocks: a full queue drops the record and counts it. */
static void
report_send (motion_ctx_t *p_ctx, motion_report_t const *p_rep)
{
    if (tk_snd_mbf(g_mbfid, (void *)p_rep, (INT)sizeof(*p_rep), TMO_POL)
        != E_OK)
    {
        p_ctx->rep_dropped++;
    }
}

static void
report_event (motion_ctx_t *p_ctx, report_event_t evt, int32_t val_l,
              int32_t val_r)
{
    motion_report_t rep;

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)evt);
    rep.val_l = val_l;
    rep.val_r = val_r;
    report_send(p_ctx, &rep);
}

/* ------------------------------------------------------------------ *
 *  Button (polled, debounced, active low).  Bench images only.
 * ------------------------------------------------------------------ */

#if MOTION_BENCH

#define BUTTON_DEBOUNCE_TICKS (3u) /* 30 ms                   */
#define BUTTON_SETTLE_MS      (2)  /* new pull-up lifts the pin */

static bool
button_raw (void)
{
    return 0u == (in_w(GPIO_IN) & (1u << MOTION_PIN_BUTTON));
}

static void
button_init (motion_ctx_t *p_ctx)
{
    (void)gpio_set_pin(MOTION_PIN_BUTTON, GPIO_MODE_IN);
    clr_w(GPIO(MOTION_PIN_BUTTON), GPIO_PDE);
    set_w(GPIO(MOTION_PIN_BUTTON), GPIO_PUE | GPIO_SHEMITT);
    (void)tk_dly_tsk(BUTTON_SETTLE_MS);

    /* A button already held at boot must not start a test. */
    p_ctx->b_btn_raw  = button_raw();
    p_ctx->b_btn_down = p_ctx->b_btn_raw;
    p_ctx->btn_same   = BUTTON_DEBOUNCE_TICKS;
}

/* True once per press, after BUTTON_DEBOUNCE_TICKS stable ticks. */
static bool
button_pressed (motion_ctx_t *p_ctx)
{
    bool b_raw   = button_raw();
    bool b_press = false;

    if (b_raw != p_ctx->b_btn_raw)
    {
        p_ctx->b_btn_raw = b_raw;
        p_ctx->btn_same  = 0u;
    }
    else if (p_ctx->btn_same < BUTTON_DEBOUNCE_TICKS)
    {
        p_ctx->btn_same++;
    }
    else
    {
        /* Stable. */
    }

    if ((BUTTON_DEBOUNCE_TICKS == p_ctx->btn_same)
        && (b_raw != p_ctx->b_btn_down))
    {
        p_ctx->b_btn_down = b_raw;
        b_press           = b_raw;
    }

    return b_press;
}

#endif /* MOTION_BENCH */

/* ------------------------------------------------------------------ *
 *  Bench test
 * ------------------------------------------------------------------ */

static void
bench_mark (motion_ctx_t *p_ctx)
{
    p_ctx->mark_l     = p_ctx->pos_l;
    p_ctx->mark_r     = p_ctx->pos_r;
    p_ctx->step_ticks = 0u;
    motion_stall_reset(&p_ctx->stall_l);
    motion_stall_reset(&p_ctx->stall_r);
}

/* End the test: brake, go idle, and report what each wheel did in the
   current step (the record keeps the duties that were being driven). */
static void
bench_stop (motion_ctx_t *p_ctx, report_event_t evt)
{
    motion_report_t rep;

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)evt);
    rep.val_l = p_ctx->pos_l - p_ctx->mark_l;
    rep.val_r = p_ctx->pos_r - p_ctx->mark_r;

    motor_brake_all();
    p_ctx->phase = PHASE_IDLE;
    p_ctx->cmd_l = 0;
    p_ctx->cmd_r = 0;
#if MOTION_BENCH && MOTION_TEST_STOP
    gb_st_ready = false;
#endif

    report_send(p_ctx, &rep);
}

#if MOTION_BENCH
/* The only way out of PHASE_IDLE, so without it the motors stay braked. */
static void
bench_start (motion_ctx_t *p_ctx)
{
    p_ctx->faults = 0u;
    p_ctx->step   = 0u;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
    bench_mark(p_ctx);

#if MOTION_BENCH && MOTION_TEST_STOP
    {
        uint32_t src;

        for (src = 0u; src < STOP_SOURCES; src++)
        {
            p_ctx->st_n[src]      = 0u;
            p_ctx->st_min_us[src] = UINT32_MAX;
            p_ctx->st_max_us[src] = 0u;
            p_ctx->st_sum_us[src] = 0u;
        }
        p_ctx->st_drive = 0u;
    }
    p_ctx->phase = PHASE_STOPTEST;
    report_event(p_ctx, EVT_START, 0, 1);
#elif MOTION_TEST_PID
    p_ctx->phase = PHASE_PID;
    report_event(p_ctx, EVT_START, rate_cps(MOTION_PID_TARGET, 1u), 0);
#else
    p_ctx->phase = PHASE_STEPS;
    report_event(p_ctx, EVT_START, 0, 0);
#endif
}
#endif /* MOTION_BENCH */

static void
bench_steps (motion_ctx_t *p_ctx)
{
    bench_step_t const *p_step = &g_steps[p_ctx->step];
    motion_report_t     rep;
    int32_t             moved_l;
    int32_t             moved_r;

    p_ctx->cmd_l = p_step->duty_l;
    p_ctx->cmd_r = p_step->duty_r;
    p_ctx->step_ticks++;

#if MOTION_BENCH && MOTION_TEST_WDT
    /* Watchdog test: hang in step 0 ("L fwd") with the left wheel
       turning.  The tick never completes, so it is never fed again. */
    if ((0u == p_ctx->step) && (MOTION_TEST_WDT_TICKS == p_ctx->step_ticks))
    {
        motion_wdt_hang(p_ctx->wdt_fed_us); /* never returns */
    }
#endif

    if (p_ctx->step_ticks >= p_step->ticks)
    {
        if ((p_step->duty_l != 0) || (p_step->duty_r != 0))
        {
            moved_l = p_ctx->pos_l - p_ctx->mark_l;
            moved_r = p_ctx->pos_r - p_ctx->mark_r;
            report_fill(p_ctx, &rep, REP_STEP, (uint8_t)p_ctx->step);
            rep.val_l  = moved_l;
            rep.val_r  = moved_r;
            rep.rate_l = rate_cps(moved_l, p_step->ticks);
            rep.rate_r = rate_cps(moved_r, p_step->ticks);
            rep.duty_l = p_step->duty_l;
            rep.duty_r = p_step->duty_r;
            report_send(p_ctx, &rep);
        }

        p_ctx->step++;
        bench_mark(p_ctx);

        if (p_ctx->step >= STEP_COUNT)
        {
            p_ctx->phase        = PHASE_RAMP;
            p_ctx->ramp_duty    = MOTION_RAMP_STEP_DUTY;
            p_ctx->ramp_found_l = 0;
            p_ctx->ramp_found_r = 0;
        }
    }
}

/* Dead band: raise both duties in small steps and note the first level
   at which each wheel really turns.  No stall check here: not moving at
   a low duty is exactly what this phase is measuring. */
static void
bench_ramp (motion_ctx_t *p_ctx)
{
    p_ctx->cmd_l = p_ctx->ramp_duty;
    p_ctx->cmd_r = p_ctx->ramp_duty;
    p_ctx->step_ticks++;

    if (p_ctx->step_ticks >= MOTION_RAMP_STEP_TICKS)
    {
        if ((0 == p_ctx->ramp_found_l)
            && (motion_abs(p_ctx->pos_l - p_ctx->mark_l)
                >= MOTION_RAMP_MOVE_COUNTS))
        {
            p_ctx->ramp_found_l = p_ctx->ramp_duty;
        }
        if ((0 == p_ctx->ramp_found_r)
            && (motion_abs(p_ctx->pos_r - p_ctx->mark_r)
                >= MOTION_RAMP_MOVE_COUNTS))
        {
            p_ctx->ramp_found_r = p_ctx->ramp_duty;
        }
        bench_mark(p_ctx);

        if (((p_ctx->ramp_found_l != 0) && (p_ctx->ramp_found_r != 0))
            || (p_ctx->ramp_duty >= MOTION_RAMP_MAX_DUTY))
        {
            report_event(p_ctx, EVT_RAMP, p_ctx->ramp_found_l,
                         p_ctx->ramp_found_r);
            bench_stop(p_ctx, EVT_DONE);
        }
        else
        {
            p_ctx->ramp_duty += MOTION_RAMP_STEP_DUTY;
        }
    }
}

static void
bench_pid (motion_ctx_t *p_ctx)
{
    p_ctx->cmd_l =
        pid_capped(p_ctx, &p_ctx->pid_l, MOTION_PID_TARGET, p_ctx->d_l);
    p_ctx->cmd_r =
        pid_capped(p_ctx, &p_ctx->pid_r, MOTION_PID_TARGET, p_ctx->d_r);
    p_ctx->step_ticks++;

    if (p_ctx->step_ticks >= MOTION_PID_TICKS)
    {
        bench_stop(p_ctx, EVT_DONE);
    }
}

#if MOTION_BENCH && MOTION_TEST_STOP
/* Stop test (NFR-18): both wheels at MOTION_TEST_DUTY whenever no STOP
   bit is set.  After MOTION_TEST_STOP_DRIVE ticks of driving it tells
   the stand-in it may set the next STOP; stop_check() times each one. */
static void
bench_stoptest (motion_ctx_t *p_ctx)
{
    motion_report_t rep;
    uint32_t        src;
    bool            b_done = true;

    if (0u == p_ctx->stop_bits)
    {
        p_ctx->cmd_l = MOTION_TEST_DUTY;
        p_ctx->cmd_r = MOTION_TEST_DUTY;
        if (p_ctx->st_drive < MOTION_TEST_STOP_DRIVE)
        {
            p_ctx->st_drive++;
        }
    }
    else
    {
        p_ctx->st_drive = 0u;
    }

    for (src = 0u; src < STOP_SOURCES; src++)
    {
        if (p_ctx->st_n[src] < MOTION_TEST_STOP_TRIALS)
        {
            b_done = false;
        }
    }

    if (b_done && (0u == p_ctx->stop_bits))
    {
        for (src = 0u; src < STOP_SOURCES; src++)
        {
            report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_STOPTEST);
            rep.val_l  = (int32_t)((STOP_SRC_SCAN == src) ? MOTION_EVT_STOP_SCAN
                                                          : MOTION_EVT_STOP_IMU);
            rep.val_r  = (int32_t)p_ctx->st_n[src];
            rep.rate_l = (int32_t)p_ctx->st_max_us[src];
            rep.rate_r = (int32_t)(p_ctx->st_sum_us[src] / p_ctx->st_n[src]);
            rep.duty_l = (int16_t)motion_clamp((int32_t)p_ctx->st_min_us[src],
                                               0, INT16_MAX);
            report_send(p_ctx, &rep);
        }
        bench_stop(p_ctx, EVT_DONE);
    }
    else
    {
        gb_st_ready = (p_ctx->st_drive >= MOTION_TEST_STOP_DRIVE);
    }
}
#endif

/* FR-30.  Judged on the duty that was applied during the tick just
   measured.  Open loop counts net movement either way (polarity is not
   known yet); closed loop counts only net movement in the commanded
   direction, so a wrong INVERT flag also ends as a stall instead of a
   runaway.  Net movement (not summed steps) also catches an encoder with
   one dead wire, whose count only steps back and forth. */
static motion_stall_result_t
stall_wheel (motion_stall_t *p_stall, motor_side_t side, int32_t delta,
             uint32_t edges_a, uint32_t edges_b, bool b_directional)
{
    int32_t duty     = motor_get_duty(side);
    bool    b_driven = (motion_abs(duty) >= MOTION_DEADBAND_DUTY);
    int32_t dir      = 0;

    if (b_directional)
    {
        dir = (duty >= 0) ? 1 : -1;
    }

    return motion_stall_update(p_stall, b_driven, dir, delta, edges_a,
                               edges_b);
}

/* Fault bit for a stall result: motor stall or encoder fault. */
static uint8_t
stall_fault_bit (motion_stall_result_t result, uint8_t motor_bit,
                 uint8_t encoder_bit)
{
    uint8_t bit = 0u;

    if (MOTION_STALL_MOTOR == result)
    {
        bit = motor_bit;
    }
    else if (MOTION_STALL_ENCODER == result)
    {
        bit = encoder_bit;
    }
    else
    {
        /* No stall. */
    }

    return bit;
}

static void
stall_check (motion_ctx_t *p_ctx)
{
    bool                  b_sm_drive = false;
    bool                  b_dir;
    motion_stall_result_t result;

#if !MOTION_BENCH
    /* Every state that may drive is checked (design FR-30). */
    b_sm_drive = motion_sm_driving(motion_sm_state(&p_ctx->sm));
#endif
    b_dir = (PHASE_PID == p_ctx->phase) || b_sm_drive; /* closed loop */

    if ((PHASE_STEPS == p_ctx->phase) || (PHASE_PID == p_ctx->phase)
        || b_sm_drive)
    {
        if (encoder_left_enabled())
        {
            result = stall_wheel(&p_ctx->stall_l, MOTOR_LEFT, p_ctx->d_l,
                                 p_ctx->e_la, p_ctx->e_lb, b_dir);
            p_ctx->faults |= stall_fault_bit(result,
                                             (uint8_t)MOTION_FAULT_STALL_L,
                                             (uint8_t)MOTION_FAULT_ENC_L);
        }
        result = stall_wheel(&p_ctx->stall_r, MOTOR_RIGHT, p_ctx->d_r,
                             p_ctx->e_ra, p_ctx->e_rb, b_dir);
        p_ctx->faults |= stall_fault_bit(result, (uint8_t)MOTION_FAULT_STALL_R,
                                         (uint8_t)MOTION_FAULT_ENC_R);
        if ((p_ctx->faults != 0u) && (PHASE_IDLE != p_ctx->phase))
        {
            bench_stop(p_ctx, EVT_FAULT); /* the statechart handles its own */
        }
    }
}

/* ------------------------------------------------------------------ *
 *  The 10 ms tick
 * ------------------------------------------------------------------ */

static void
tick_timing (motion_ctx_t *p_ctx, uint32_t t_now)
{
    uint32_t period;

    if (p_ctx->ticks > 0u)
    {
        period = t_now - p_ctx->last_us; /* modulo 2^32 */
        if (period < p_ctx->per_min_us)
        {
            p_ctx->per_min_us = period;
        }
        if (period > p_ctx->per_max_us)
        {
            p_ctx->per_max_us = period;
        }
    }
    p_ctx->last_us = t_now;
    p_ctx->ticks++;
}

/* Edges on one 16-bit half since the last read (wrapping). */
static uint32_t
edges_since (uint16_t now, uint16_t prev)
{
    return (uint32_t)(uint16_t)(now - prev);
}

/* A right-encoder total in left-wheel units: raw x MOTION_COUNTS_PER_REV /
   MOTION_COUNTS_PER_REV_R, rounded to the nearest unit (half away from
   zero, the same both ways, so forward and back stay symmetric; truncating
   would lag the left wheel by up to 1.6 units and read as a turn).
   64-bit, so a total of +-2^31 counts cannot overflow. */
static int32_t
right_to_left_units (int32_t raw)
{
#if MOTION_COUNTS_PER_REV_R == MOTION_COUNTS_PER_REV
    return raw;
#else
    int64_t num  = (int64_t)raw * MOTION_COUNTS_PER_REV;
    int64_t half = MOTION_COUNTS_PER_REV_R / 2;

    return (int32_t)(((num >= 0) ? (num + half) : (num - half))
                     / MOTION_COUNTS_PER_REV_R);
#endif
}

static void
tick_encoders (motion_ctx_t *p_ctx)
{
    uint16_t now_l;
    uint16_t now_r;
    uint32_t edges_l;
    uint32_t edges_r;

    encoder_read_counts(&now_l, &now_r); /* both from the same moment */
    edges_l = encoder_read_edges_left();
    edges_r = encoder_read_edges_right();

    p_ctx->d_l    = motion_count_delta(now_l, p_ctx->prev_l);
    p_ctx->prev_l = now_l;
    p_ctx->pos_l += p_ctx->d_l;

    /* The right encoder gives fewer counts per wheel turn than the left
       (MOTION_COUNTS_PER_REV_R, motion_cfg.h).  Scale its running total
       to left-wheel units, then take the delta from the scaled total, so
       rounding never builds up: everything after this line (speed PID,
       distance_state, pivots, bypass) sees one unit for both wheels. */
    p_ctx->raw_pos_r += motion_count_delta(now_r, p_ctx->prev_r);
    p_ctx->prev_r     = now_r;
    {
        int32_t scaled = right_to_left_units(p_ctx->raw_pos_r);

        p_ctx->d_r   = scaled - p_ctx->pos_r;
        p_ctx->pos_r = scaled;
    }

    p_ctx->e_la = edges_since(motion_unpack_left(edges_l),
                              motion_unpack_left(p_ctx->prev_edges_l));
    p_ctx->e_lb = edges_since(motion_unpack_right(edges_l),
                              motion_unpack_right(p_ctx->prev_edges_l));
    p_ctx->e_ra = edges_since(motion_unpack_left(edges_r),
                              motion_unpack_left(p_ctx->prev_edges_r));
    p_ctx->e_rb = edges_since(motion_unpack_right(edges_r),
                              motion_unpack_right(p_ctx->prev_edges_r));
    p_ctx->prev_edges_l = edges_l;
    p_ctx->prev_edges_r = edges_r;
}

/* Build this tick's distance_state from the counts and publish it.  The
   distance comes from the total counts since start-up, so a skipped or
   lost tick loses nothing: the next publish includes it. */
static void
tick_distance (motion_ctx_t *p_ctx)
{
    motion_ds_win_add(&p_ctx->ds_win, p_ctx->d_l, p_ctx->d_r);
    p_ctx->ds_mm    = motion_ds_dist_mm(p_ctx->pos_l, p_ctx->pos_r);
    p_ctx->ds_turn  = motion_ds_win_turn_dps(&p_ctx->ds_win);
    p_ctx->ds_accel = motion_ds_win_accel_mmps2(&p_ctx->ds_win);
    p_ctx->ds_state = motion_ds_accel_state(p_ctx->ds_accel);

#if MOTION_PACKED
    p_ctx->ds_counter++;
    g_ds_word = motion_ds_pack(p_ctx->ds_mm, p_ctx->ds_state, p_ctx->ds_counter,
                               p_ctx->ds_turn);
#else
    /* Never wait: if a reader holds the mutex, skip this tick. */
    if (E_OK == tk_loc_mtx(g_ds_mtxid, TMO_POL))
    {
        p_ctx->ds_counter++;
        g_ds.dist_mm = p_ctx->ds_mm;
        g_ds.turn_dps =
            (int16_t)motion_clamp(p_ctx->ds_turn, INT16_MIN, INT16_MAX);
        g_ds.state   = (uint8_t)p_ctx->ds_state;
        g_ds.counter = (uint8_t)(p_ctx->ds_counter & MOTION_DS_CTR_MASK);
        (void)tk_unl_mtx(g_ds_mtxid);
    }
#endif
}

uint32_t
motion_ds_read (void)
{
#if MOTION_PACKED
    return g_ds_word; /* one aligned load: all four fields from one tick */
#else
    ds_week6_t copy = { 0, 0, 0u, 0u };

    /* The reader waits, as in Week 6: the writer holds the mutex only to
       copy and never waits itself, so the wait is a few microseconds. */
    if (E_OK == tk_loc_mtx(g_ds_mtxid, TMO_FEVR))
    {
        copy = g_ds;
        (void)tk_unl_mtx(g_ds_mtxid);
    }

    return motion_ds_pack(copy.dist_mm, (motion_ds_state_t)copy.state,
                          copy.counter, copy.turn_dps);
#endif
}

uint32_t
motion_state_read (void)
{
    return g_state_word; /* one aligned load */
}

static void
tick_report (motion_ctx_t *p_ctx)
{
    motion_report_t rep;
    uint32_t        irq_now;
    bool            b_moved;

    p_ctx->rep_ticks++;
    b_moved = (p_ctx->pos_l != p_ctx->rep_l) || (p_ctx->pos_r != p_ctx->rep_r);

    /* Status every 200 ms, but only while a wheel is moving (driven, or
       turned by hand), plus always during the closed-loop test. */
    if ((p_ctx->rep_ticks >= MOTION_REPORT_TICKS)
        && (b_moved || (PHASE_PID == p_ctx->phase)))
    {
        report_fill(p_ctx, &rep, REP_STATUS, 0u);
        rep.val_l  = p_ctx->pos_l;
        rep.val_r  = p_ctx->pos_r;
        rep.rate_l = rate_cps(p_ctx->pos_l - p_ctx->rep_l, p_ctx->rep_ticks);
        rep.rate_r = rate_cps(p_ctx->pos_r - p_ctx->rep_r, p_ctx->rep_ticks);
        report_send(p_ctx, &rep);

        report_fill(p_ctx, &rep, REP_DIST, (uint8_t)p_ctx->ds_state);
        rep.val_l  = p_ctx->ds_mm;
        rep.val_r  = p_ctx->ds_turn;
        rep.rate_l = p_ctx->ds_accel;
        rep.rate_r = (int32_t)(p_ctx->ds_counter & MOTION_DS_CTR_MASK);
        report_send(p_ctx, &rep);
    }
    if (p_ctx->rep_ticks >= MOTION_REPORT_TICKS)
    {
        p_ctx->rep_l     = p_ctx->pos_l;
        p_ctx->rep_r     = p_ctx->pos_r;
        p_ctx->rep_ticks = 0u;
    }

    /* Timing evidence every 5 s, then restart the min/max window. */
    if (0u == (p_ctx->ticks % TIMING_REPORT_TICKS))
    {
        irq_now = encoder_irq_count();
        report_fill(p_ctx, &rep, REP_TIMING, 0u);
        rep.val_l      = (int32_t)p_ctx->lost_ticks;
        rep.val_r      = (int32_t)p_ctx->rep_dropped;
        rep.irq_count  = irq_now - p_ctx->irq_mark;
        rep.isr_max_us = encoder_take_irq_max_us();
        rep.rate_l     = (int32_t)p_ctx->wdt_gap_max_us;
        rep.rate_r     = (int32_t)p_ctx->tele_dropped;
        report_send(p_ctx, &rep);

        p_ctx->irq_mark       = irq_now;
        p_ctx->per_min_us     = UINT32_MAX;
        p_ctx->per_max_us     = 0u;
        p_ctx->work_max_us    = 0u;
        p_ctx->wdt_gap_max_us = 0u;
    }
}

/* The last thing in every completed tick.  A tick that hangs, or never
   comes, leaves the watchdog unfed, and the chip resets MOTION_WDT_MS
   after this feed. */
static void
tick_feed_watchdog (motion_ctx_t *p_ctx, uint32_t t_now)
{
    uint32_t gap = t_now - p_ctx->wdt_fed_us; /* modulo 2^32 */

    if (gap > p_ctx->wdt_gap_max_us)
    {
        p_ctx->wdt_gap_max_us = gap;
    }
    p_ctx->wdt_fed_us = t_now;
    motion_wdt_feed();
}

/* ------------------------------------------------------------------ *
 *  Stop path (NFR-18)
 * ------------------------------------------------------------------ */

/* Report one newly seen STOP bit, with the time from its request to the
   brake if the sender stamped it (motion_stop_stamp). */
static void
stop_report (motion_ctx_t *p_ctx, uint32_t src, uint32_t t_brake,
             bool b_braked_now)
{
    motion_report_t rep;
    uint32_t        req     = g_stop_req_us[src];
    int32_t         lat_us  = -1;
    uint32_t        lat_raw = 0u;

    if (req != p_ctx->stop_req_used[src]) /* a new stamp */
    {
        p_ctx->stop_req_used[src] = req;
        lat_raw                   = t_brake - req; /* modulo 2^32 */
        lat_us                    = (int32_t)lat_raw;
    }

#if MOTION_BENCH && MOTION_TEST_STOP
    if ((PHASE_STOPTEST == p_ctx->phase) && (lat_us >= 0) && b_braked_now)
    {
        p_ctx->st_n[src]++;
        p_ctx->st_sum_us[src] += lat_raw;
        if (lat_raw < p_ctx->st_min_us[src])
        {
            p_ctx->st_min_us[src] = lat_raw;
        }
        if (lat_raw > p_ctx->st_max_us[src])
        {
            p_ctx->st_max_us[src] = lat_raw;
        }
    }
#endif

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_STOP);
    rep.val_l  = (int32_t)((STOP_SRC_SCAN == src) ? MOTION_EVT_STOP_SCAN
                                                  : MOTION_EVT_STOP_IMU);
    rep.val_r  = lat_us;
    rep.rate_l = b_braked_now ? 1 : 0;
    report_send(p_ctx, &rep);
}

/* The first thing in every wake (design Section 4: STOP bits first, and
   the brake is the first write).  ptn is the whole flag pattern the wait
   returned, STOP bits included even when they were not waited on. */
static void
stop_check (motion_ctx_t *p_ctx, UINT ptn)
{
    uint32_t stops    = (uint32_t)ptn & MOTION_EVT_STOPS;
    uint32_t new_bits = stops & ~p_ctx->stop_bits;
    bool     b_first  = (0u == p_ctx->stop_bits) && (0u != stops);
    uint32_t t_brake;

    /* A reverse drives while STOP_SCAN stays set, so a new STOP bit then
       must brake too. */
    if ((0u != new_bits)
        && (MOTION_ST_REVERSING == motion_sm_state(&p_ctx->sm)))
    {
        b_first = true;
    }

    if (b_first)
    {
        motor_brake_all();
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }
    t_brake = now_us();
    if (b_first)
    {
        p_ctx->stop_t_us = t_brake;
    }
#if MOTION_BENCH && MOTION_TEST_STOP
    if (0u != stops)
    {
        gb_st_ready = false; /* no new stop until it has driven again */
    }
#endif

    if (0u != new_bits)
    {
        if (0u != (new_bits & MOTION_EVT_STOP_SCAN))
        {
            stop_report(p_ctx, STOP_SRC_SCAN, t_brake, b_first);
        }
        if (0u != (new_bits & MOTION_EVT_STOP_IMU))
        {
            stop_report(p_ctx, STOP_SRC_IMU, t_brake, b_first);
        }

        /* A bench step, ramp or PID test ends on a STOP (reported with
           the phase it ended, so the line can say so). */
        if ((PHASE_IDLE != p_ctx->phase) && (PHASE_STOPTEST != p_ctx->phase))
        {
            p_ctx->phase = PHASE_IDLE;
        }
    }
    else if ((0u == stops) && (0u != p_ctx->stop_bits))
    {
        report_event(p_ctx, EVT_GO,
                     (int32_t)((t_brake - p_ctx->stop_t_us) / US_PER_MS), 0);
    }
    else
    {
        /* No change. */
    }

    /* Each stamp is looked at once, in the first wake after it: it times
       a stop only if its bit is newly set in that wake.  So a stamp made
       for a bit that was already set is never charged to a later stop. */
    p_ctx->stop_req_used[STOP_SRC_SCAN] = g_stop_req_us[STOP_SRC_SCAN];
    p_ctx->stop_req_used[STOP_SRC_IMU]  = g_stop_req_us[STOP_SRC_IMU];

    p_ctx->stop_bits = stops;
}

/* ------------------------------------------------------------------ *
 *  Statechart 6.2 (motion_sm.c), normal image only
 * ------------------------------------------------------------------ */

#if !MOTION_BENCH
/* Set the reply bits a step asked for on nav_flg and scan_flg.  scan_flg
   exists only once scan_task has started, so its ID is fetched here. */
static void
sm_send_replies (uint32_t replies)
{
    ID   nav_id    = nav_flag();
    ID   scan_id   = scan_if_flag();
    UINT nav_bits  = 0u;
    UINT scan_bits = 0u;

    if (0u != (replies & MOTION_SM_REP_MOVE_DONE))
    {
        nav_bits |= NAV_FLG_MOVE_DONE;
    }
    if (0u != (replies & MOTION_SM_REP_CMD_REJECTED))
    {
        nav_bits |= NAV_FLG_CMD_REJECTED;
    }
    if (0u != (replies & MOTION_SM_REP_MOTION_FAULT))
    {
        nav_bits |= NAV_FLG_MOTION_FAULT;
    }
    if (0u != (replies & MOTION_SM_REP_LINE_FOUND))
    {
        nav_bits |= NAV_FLG_LINE_FOUND;
        if (0u != (replies & MOTION_SM_REP_TO_SCAN))
        {
            scan_bits |= SCAN_FLG_LINE_FOUND;
        }
    }
    if (0u != (replies & MOTION_SM_REP_LINE_LOST))
    {
        nav_bits |= NAV_FLG_LINE_LOST;
        if (0u != (replies & MOTION_SM_REP_TO_SCAN))
        {
            scan_bits |= SCAN_FLG_LINE_LOST;
        }
    }
    if (0u != (replies & MOTION_SM_REP_REVERSE_DONE))
    {
        scan_bits |= SCAN_FLG_REVERSE_DONE;
    }

    if ((0u != nav_bits) && (nav_id > 0))
    {
        (void)tk_set_flg(nav_id, nav_bits);
    }
    if ((0u != scan_bits) && (scan_id > 0))
    {
        (void)tk_set_flg(scan_id, scan_bits);
    }
}

/* Turning phases: straight to the crossing (or across it), then the
   pivot, then done. */
#define TURN_PH_FORWARD (0u)
#define TURN_PH_PIVOT   (1u)
#define TURN_PH_DONE    (2u)

/* Bypass legs (motion_cfg.h): pivot out, out, pivot parallel, alongside,
   pivot in, back in, pivot parallel.  The pivot direction of each leg, x
   the side; 0 = a straight leg. */
/* Bypass phases (motion_cfg.h, D14).  Each measures from its own start. */
#define BYP_PH_OUT       (0u)  /* pivot SIDE_DEG toward the side      */
#define BYP_PH_LANE_LOOK (1u)  /* braked: scan_task looks along it    */
#define BYP_PH_LANE_BACK (2u)  /* blocked: pivot back to the heading  */
#define BYP_PH_SWITCH    (3u)  /* braked: scan_task picks the other   */
#define BYP_PH_SIDESTEP  (4u)  /* offset / sin SIDE_DEG               */
#define BYP_PH_PARALLEL  (5u)  /* pivot SIDE_DEG back to the heading  */
#define BYP_PH_STEP      (6u)  /* alongside, one step                 */
#define BYP_PH_PROBE_IN  (7u)  /* pivot 45 toward the obstacle        */
#define BYP_PH_LOOK      (8u)  /* braked: scan_task looks at its side */
#define BYP_PH_PROBE_OUT (9u)  /* pivot 45 back                       */
#define BYP_PH_CLEAR     (10u) /* past the far face                   */
#define BYP_PH_TURN_IN   (11u) /* pivot SIDE_DEG toward the line      */
#define BYP_PH_STEP_IN   (12u) /* until GP26/GP27 see the line        */
#define BYP_PH_ONTO      (13u) /* axle onto the line                  */
#define BYP_PH_ALONG     (14u) /* pivot SIDE_DEG onto the heading     */
#define BYP_PH_IN_LOOK   (15u) /* known box: braked, look the way in  */
#define BYP_PH_IN_BACK   (16u) /* way in not clear: pivot back along  */
#define BYP_PH_ARC_LEG   (17u) /* arc: straight out to the circle     */
#define BYP_PH_ARC       (18u) /* arc: round the box on the circle    */
#define BYP_PH_SMOOTH    (19u) /* smooth: one curve round, to the line */
#define BYP_PH_DONE      (20u)

/* Pivot direction of each phase, x the side (+1 = toward the bypass
   side), and its angle; 0 = straight or braked (the looks, SWITCH, DONE).
   BYP_PH_ARC is neither: arc_drive() sets the wheels. */
static int8_t const g_byp_turn[BYP_PH_DONE] = {
    1, 0, -1, 0, 0, -1, 0, -1, 0, 1, 0, -1, 0, 0, 1, 0, 1, 0, 0, 0
};
static uint8_t const g_byp_deg[BYP_PH_DONE] = {
    MOTION_BYPASS_SIDE_DEG, 0, MOTION_BYPASS_SIDE_DEG, 0, 0,
    MOTION_BYPASS_SIDE_DEG, 0, 45, 0, 45, 0, MOTION_BYPASS_SIDE_DEG, 0, 0,
    MOTION_BYPASS_SIDE_DEG, 0, MOTION_BYPASS_SIDE_DEG, 0, 0, 0
};

/* A bypass pivot's encoder target: scaled for the overshoot measured at
   the bypass pivot speed (S7 attempt 3). */
#define BYP_PIVOT_COUNTS(deg)                                                  \
    ((PIVOT_COUNTS(deg) * MOTION_BYPASS_PIVOT_PCT) / 100)

/* The sidestep leg and the lane it needs clear ahead (capped at the
   largest LOOK reading, 511 = nothing in range). */
#define BYP_LEG_MM(off) (((off) * MOTION_BYPASS_SIDE_LEG_X1000) / 1000)

#define LOOK_NONE     (0u)
#define LOOK_ASKED    (1u)
#define LOOK_ANSWERED (2u)

static int32_t hd_lat_mm(motion_ctx_t const *p_ctx); /* IMU heading, below */
static int32_t hd_lon_mm(motion_ctx_t const *p_ctx);
static void    byp_report(motion_ctx_t *p_ctx);
static void    arc_plan(motion_ctx_t *p_ctx);
static void    sm_plan(motion_ctx_t *p_ctx);
extern uint32_t scan_profile_word(void); /* obstacle_task.c */

/* Has the manoeuvre of this state reached its target? */
static bool
man_done (motion_ctx_t const *p_ctx, motion_state_t state)
{
    bool    b_done = false;
    int32_t moved_mm;

    if (MOTION_ST_REVERSING == state)
    {
        moved_mm = motion_ds_dist_mm(p_ctx->pos_l - p_ctx->man_start_l,
                                     p_ctx->pos_r - p_ctx->man_start_r);
        b_done   = ((-moved_mm) >= p_ctx->man_mm);
    }
    else if (MOTION_ST_TURNING == state)
    {
        b_done = (TURN_PH_DONE == p_ctx->turn_phase);
    }
    else if (MOTION_ST_BYPASS == state)
    {
        b_done = (BYP_PH_DONE == p_ctx->bypass_phase);
    }
    else
    {
        /* Not a manoeuvre with a target. */
    }

    return b_done;
}

/* Line Following steering state back to rest.  The error history starts
   at the error now, so entering with the car already off centre gives no
   damper kick. */
static void
lf_reset (motion_ctx_t *p_ctx)
{
    int32_t  err = line_state_error(p_ctx->line_word);
    uint32_t i;

    p_ctx->lf_curve_ticks = 0u;
    p_ctx->lf_rate16      = 0;
    /* straddle: the curve memory starts straight, the nudge stays tuned
       (it depends on the sensors, not the track) */
    p_ctx->st_mem        = 0;
    p_ctx->st_steer      = 0;
    p_ctx->st_side       = 0;
    p_ctx->st_last_touch = 0;
    p_ctx->st_black_um   = 0u;
    p_ctx->st_bb_um      = 0u;
    p_ctx->st_rem_l      = 0;
    p_ctx->st_rem_r      = 0;
    p_ctx->st_end_um     = 100000000u;
    p_ctx->st_end_side   = 0;
    p_ctx->st_out        = 0;
    p_ctx->b_st_trav     = false;
    p_ctx->b_st_bc       = false;
    p_ctx->b_st_outer    = false;
    p_ctx->st_out_um     = 0u;
    p_ctx->b_st_recov    = false;
    p_ctx->b_st_after_recov = false;
    if (0 == p_ctx->st_kick)
    {
        p_ctx->st_kick = MOTION_ST_KICK_CURV;
    }
    p_ctx->lf_edge_run    = 0u;
    p_ctx->lf_gap2c       = 0;
    p_ctx->lf_corr_avg    = 0;

    p_ctx->lf_rem      = 0;
    p_ctx->lf_corr     = 0;
    p_ctx->lf_hist_i   = 0u;
    p_ctx->lf_hard_run = 0u;
    for (i = 0u; i < MOTION_LF_DAMP_TICKS; i++)
    {
        p_ctx->lf_hist[i] = err;
    }
}

/* MOTION_LF_LOST_DIR_MS (L4 experiment, motion_cfg.h): the line has
   been dark long enough for the full last-side error.  0, the default:
   from the first 00. */
static bool
lf_dark_long (uint32_t word)
{
#if MOTION_LF_LOST_DIR_MS > 0
    return line_state_dark_ms(word) >= (uint32_t)MOTION_LF_LOST_DIR_MS;
#else
    (void)word;
    return true;
#endif
}

/* The error Line Following steers by.  Off the line (rl 00, both sensors
   white) the word's error is only the difference between two white
   readings, so the car ran straight off the outside of a curve (S4
   attempt 7: about -30 for the 1.5 s until LOST).  Instead it keeps
   turning, at full error, to the side the line was last seen on (with
   MOTION_LF_LOST_DIR_MS above 0, only once the 00 has lasted that long).
   Not during a barcode hold, which also reads rl 00 by design. */
/* TEST IMAGE (20 Nov): 0 = the barcode bit only slows the car
   (MOTION_LF_BASE_READ); steering stays live over the barcode. */
#ifndef MOTION_LF_HOLD_ON_BARCODE
#define MOTION_LF_HOLD_ON_BARCODE (1)
#endif

static int32_t
lf_error (const motion_ctx_t *p_ctx)
{
    uint32_t word = p_ctx->line_word;

    /* MOTION_LF_EDGE_LOST: slipped off the line's edge (last seen by one
       sensor): back toward it at once.  Lost from both sensors on it (a
       white gap between track tiles) waits MOTION_LF_LOST_DIR_MS as before.
       Waiting 300 ms after an edge too let the car run on, turn back hard
       and overshoot: a weave across the line (car, 20 Nov 21:46). */
    /* (21 Nov, car 22:09: the full error at once swung the car 80 deg
       off the end of the S-curve, past a tile gap.)  Off an edge it now
       holds the error it last read at the edge, so it keeps turning as it
       was, and only after MOTION_LF_LOST_DIR_MS takes the full error. */
    if ((LINE_RL_CENTRED == line_state_rl(word))
        && !(MOTION_LF_HOLD_ON_BARCODE && line_state_barcode(word))
        && (0 != p_ctx->line_dir))
    {
        if (lf_dark_long(word))
        {
            return p_ctx->line_dir * LINE_ST_ERR_MAX;
        }
        if (MOTION_LF_EDGE_LOST && p_ctx->b_lf_edge)
        {
            return p_ctx->lf_edge_err;
        }
    }
    return line_state_error(word);
}

static int32_t
lf_clamp (int32_t v, int32_t lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

/* One tick of the Line Following correction (motion_cfg.h), x
   MOTION_LF_SCALE counts per tick, + = turn left.  During a barcode hold
   the error is 0 by design (line_state.h), so the history is reset rather
   than read as a fast return to centre. */
/* MOTION_LF_GAP_MM in d_l + d_r units (2 x counts), pi as 355/113 */
#define LF_GAP_2C ((int32_t)((2 * (int64_t)MOTION_LF_GAP_MM * MOTION_COUNTS_PER_REV * 113) \
                             / ((int64_t)355 * MOTION_WHEEL_DIA_MM)))

static int32_t
lf_correction (motion_ctx_t *p_ctx, int32_t err, bool b_hold)
{
    int32_t target;
    int32_t cap;
    int32_t de;

    if (b_hold)
    {
        int32_t corr = p_ctx->lf_corr;

        lf_reset(p_ctx);
        p_ctx->lf_corr = corr; /* still eases out below */
        target         = 0;
        cap            = MOTION_LF_SOFT_MAX;
    }
#if MOTION_LF_GAP_MM > 0
    else if ((p_ctx->lf_gap2c > 0) && (p_ctx->lf_gap2c < LF_GAP_2C)
             && !(MOTION_LF_EDGE_LOST && p_ctx->b_lf_edge))
    {
        /* (22 Nov, car 01:06: in a right curve the line slipped off the
           right sensor; holding the averaged steering for 60 mm let the
           curve run away.  Off an edge the EDGE_LOST hold in lf_error now
           turns toward it; only a white met with both sensors on the line
           (a gap) keeps the steering here.) */
        /* Both white for less than MOTION_LF_GAP_MM (21 Nov, user: the
           white seams between track panels): keep steering as the car was,
           on average, just before (straight on a straight, on round a
           curve), no turn toward the last side yet.  Holding the heading
           instead stopped the car turning mid-curve (car 21:11), the line
           curved away and it pivoted after the 30 mm. */
        target = p_ctx->lf_corr_avg;
        cap    = MOTION_LF_SOFT_MAX;
    }
#endif
    else
    {
        de = err - p_ctx->lf_hist[p_ctx->lf_hist_i];
        p_ctx->lf_hist[p_ctx->lf_hist_i] = err;
        p_ctx->lf_hist_i = (p_ctx->lf_hist_i + 1u) % MOTION_LF_DAMP_TICKS;

        if ((err >= MOTION_LF_HARD_ERR) || (err <= -MOTION_LF_HARD_ERR))
        {
            if (p_ctx->lf_hard_run < MOTION_LF_HARD_TICKS)
            {
                p_ctx->lf_hard_run++;
            }
        }
        else
        {
            p_ctx->lf_hard_run = 0u;
        }

        if (p_ctx->lf_hard_run >= MOTION_LF_HARD_TICKS)
        {
            /* last resort: held far off for too long */
            target = (err > 0) ? MOTION_LF_HARD_MAX : -MOTION_LF_HARD_MAX;
            cap    = MOTION_LF_HARD_MAX;
        }
        else
        {
            target = (err * MOTION_LF_GAIN) + (de * MOTION_LF_DAMP);
            cap    = MOTION_LF_SOFT_MAX;
        }
    }

#if MOTION_LF_RATE_DAMP > 0
    /* Turn-rate damping (21 Nov, car 20:29: after the bypass the car swung
       across the line twice a second for 16 s, off it most of the time,
       ~70 deg/s through the line).  The wheels' count difference is the
       turn rate; pushing back against it stops a fast swing and hardly
       touches slow steering (a 2-count difference costs 2 x K). */
    p_ctx->lf_rate16 += (((p_ctx->d_r - p_ctx->d_l) * 16) - p_ctx->lf_rate16) / 4;
    target -= (MOTION_LF_RATE_DAMP * p_ctx->lf_rate16) / 16;
#endif
    target = lf_clamp(target, cap);
    p_ctx->lf_corr += lf_clamp(target - p_ctx->lf_corr, MOTION_LF_SLEW);
#if MOTION_LF_GAP_MM > 0
    if (0 == p_ctx->lf_gap2c)
    {
        p_ctx->lf_corr_avg += (p_ctx->lf_corr - p_ctx->lf_corr_avg) / 16;
    }
#endif
    return p_ctx->lf_corr;
}

/* A state was just entered: set up its manoeuvre. */
static void
man_start (motion_ctx_t *p_ctx, motion_state_t state, bool b_resumed)
{
    p_ctx->rt_phase = 0u; /* any state change ends a barcode re-read */
    p_ctx->ra_phase = 0u;
    p_ctx->b_ra_pending = false;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
    motion_stall_reset(&p_ctx->stall_l);
    motion_stall_reset(&p_ctx->stall_r);

    p_ctx->man_start_l = p_ctx->pos_l;
    p_ctx->man_start_r = p_ctx->pos_r;

    if (MOTION_ST_REVERSING == state)
    {
        p_ctx->man_mm = (int32_t)p_ctx->avoid_mm;
    }
    else if ((MOTION_ST_BYPASS == state) && !b_resumed)
    {
        /* A new BYPASS starts over; a resumed one carries on (as Turning). */
        p_ctx->bypass_side    = (MOTION_AVOID_SIDE_RIGHT == p_ctx->avoid_side)
                                    ? -1 : 1;
        p_ctx->bypass_off_mm  = (int32_t)p_ctx->avoid_mm;
        p_ctx->bypass_phase   = BYP_PH_OUT;
        p_ctx->bypass_start_l = p_ctx->pos_l;
        p_ctx->bypass_start_r = p_ctx->pos_r;
        p_ctx->bypass_pass_mm = 0;
        p_ctx->b_bypass_seen  = false;
        p_ctx->b_look_present = false;
        p_ctx->look_state     = LOOK_NONE;
        p_ctx->look_ticks     = 0u;
        /* The offset is from where the car is now (a re-plan may start
           off the line): the sidestep ends that far beyond it. */
        p_ctx->byp_lat_target = (hd_lat_mm(p_ctx) * p_ctx->bypass_side)
                                + p_ctx->bypass_off_mm;
        p_ctx->byp_pass_more  = 0;
        arc_plan(p_ctx);
        sm_plan(p_ctx);
        byp_report(p_ctx);
    }
    else if ((MOTION_ST_TURNING == state) && !b_resumed)
    {
        /* A resumed turn carries on from its own start positions: the
           wheels did not move while Braked (a reverse drops the turn). */
        p_ctx->turn_phase   = TURN_PH_FORWARD;
        p_ctx->turn_start_l = p_ctx->pos_l;
        p_ctx->turn_start_r = p_ctx->pos_r;
    }
    else if (MOTION_ST_LINE_FOLLOW == state)
    {
        lf_reset(p_ctx);
    }
    else if (MOTION_ST_LINE_SEARCH == state)
    {
        p_ctx->search_dir    = (p_ctx->line_dir < 0) ? -1 : 1;
        p_ctx->search_phase  = 0u;
        p_ctx->b_search_over = false;
    }
    else
    {
        /* Nothing else to set up. */
    }
}

/* Read line_state (one load) and work out the statechart's line inputs.
   The word is stale when its counter has not moved for
   LINE_ST_STALE_TICKS ticks: ir_sense_task has stopped publishing. */
static void
line_inputs (motion_ctx_t *p_ctx, motion_sm_in_t *p_in)
{
    uint32_t word = line_state_word();

    if (line_state_counter(word) == p_ctx->line_ctr)
    {
        if (p_ctx->line_same < LINE_ST_STALE_TICKS)
        {
            p_ctx->line_same++;
        }
    }
    else
    {
        p_ctx->line_ctr  = line_state_counter(word);
        p_ctx->line_same = 0u;
    }
    p_ctx->line_word = word;
    if (LINE_RL_CENTRED != line_state_rl(word))
    {
        /* one sensor on it (rl 10 or 01): at the line's edge */
        /* one sensor alone for MOTION_LF_EDGE_MIN_TICKS: the line is
           sliding off that edge (a curve).  Shorter: the two sensors met a
           white tile gap one after the other, an uneven cut (car 12:34:
           72 ms of one sensor from a straight, centred car, then both white;
           taken as an edge, the car swerved off) */
        if (LINE_RL_JUNCTION != line_state_rl(word))
        {
            if (p_ctx->lf_single_ticks < 1000u)
            {
                p_ctx->lf_single_ticks++;
            }
        }
        else
        {
            p_ctx->lf_single_ticks = 0u;
        }
        p_ctx->b_lf_edge = (LINE_RL_JUNCTION != line_state_rl(word))
                           && (p_ctx->lf_single_ticks >= MOTION_LF_EDGE_MIN_TICKS);
        /* only a clear edge reading: as a sensor fades to white the
           error passes 0 and flipped sign (car 22:20, -59 then +17) */
        if (p_ctx->b_lf_edge
            && ((line_state_error(word) >= MOTION_LS_DIR_MIN)
                || (line_state_error(word) <= -MOTION_LS_DIR_MIN)))
        {
            p_ctx->lf_edge_err = line_state_error(word);
        }
    }
    if (LINE_RL_CENTRED == line_state_rl(word))
    {
        /* Off the line: two white readings say nothing about the side. */
    }
    else if (line_state_error(word) >= MOTION_LS_DIR_MIN)
    {
        p_ctx->line_dir = 1;
    }
    else if (line_state_error(word) <= -MOTION_LS_DIR_MIN)
    {
        p_ctx->line_dir = -1;
    }
    else
    {
        /* Too small to say which side: keep the last. */
    }

    /* Both sensors inside the line: centred reads rl 11, off the line 00.
       The dark time bridges the short 00 of a wobble. */
    p_in->b_line_seen = (LINE_RL_CENTRED != line_state_rl(word))
                        || (line_state_dark_ms(word) < MOTION_LF_SEEN_MS);
    p_ctx->line_seen_run = p_in->b_line_seen ? (p_ctx->line_seen_run + 1u) : 0u;
    p_in->b_line_found   = (p_ctx->line_seen_run >= MOTION_LF_FOUND_TICKS);
    /* On a hump the line may be missing 2 x t_lost (M3, motion_cfg.h). */
    p_ctx->b_on_hump = (0u != IMU_W_GET(imu_word_read(), HUMP));  /* 0 = none */
    p_ctx->dark_sat_ticks
        = (line_state_dark_ms(word)
           >= (LINE_ST_DARK_MASK * LINE_ST_DARK_UNIT_MS))
              ? (p_ctx->dark_sat_ticks + 1u)
              : 0u;
    p_in->b_line_missing
        = (p_ctx->b_on_hump
               ? ((p_ctx->dark_sat_ticks * 10u)
                  >= (MOTION_HUMP_LOST_MS
                      - (LINE_ST_DARK_MASK * LINE_ST_DARK_UNIT_MS)))
               : line_state_lost(word))
          || line_state_fault(word)
                           || (p_ctx->line_same >= LINE_ST_STALE_TICKS);
#if MOTION_LF_STRADDLE
    /* Straddle layout: both white is the car on the line, so "lost" is
       MOTION_ST_LOST_MM driven forward with neither sensor touching it
       (a pivot adds nothing).  b_line_seen stays a real touch: the bypass
       rejoin needs one.  Found also when a touch was under
       MOTION_ST_FOUND_MM ago (after a bypass or a turn the car ends
       centred, both white), so Line Search does not sweep for nothing. */
    {
        int32_t fwd = p_ctx->d_l + p_ctx->d_r; /* 2 x counts this tick */

        if (LINE_RL_CENTRED != line_state_rl(word))
        {
            p_ctx->st_nt_um = 0u;
        }
        else if ((fwd > 0) && (p_ctx->st_nt_um < 100000000u))
        {
            p_ctx->st_nt_um += (uint32_t)ST_UM_2C(fwd);
        }
        else
        {
            /* still, backing or pivoting */
        }
        p_in->b_line_found = p_in->b_line_found
                             || (p_ctx->st_nt_um < (MOTION_ST_FOUND_MM * 1000u));
        p_in->b_line_missing
            = (p_ctx->st_nt_um >= (MOTION_ST_LOST_MM * 1000u))
              || ((0 != p_ctx->st_out)
                  && (p_ctx->st_out_um >= (MOTION_ST_OUT_MAX_MM * 1000u)))
              || (p_ctx->b_st_after_recov
                  && (p_ctx->st_nt_um >= (MOTION_ST_RECOV_LOST_MM * 1000u)))
              || line_state_fault(word)
              || (p_ctx->line_same >= LINE_ST_STALE_TICKS);
    }
#endif
    p_in->b_search_over  = p_ctx->b_search_over;
}

/* Encoder counts each wheel turns for a pivot of deg degrees on the spot:
   arc = deg/360 x pi x wheelbase, counts = arc / (pi x wheel diameter) x
   counts per turn, so pi cancels. */
#define PIVOT_COUNTS(deg)                                                      \
    (((deg) * MOTION_WHEELBASE_MM * MOTION_COUNTS_PER_REV)                     \
     / (360 * MOTION_WHEEL_DIA_MM))

/* LINE_SEARCH: pivot toward the line's last side up to the sweep angle,
   then back to the same angle the other side; then the search is over. */
static void
search_drive (motion_ctx_t *p_ctx)
{
    int32_t turned;
    int32_t dir;

    /* counter-clockwise counts per wheel since the search began, signed
       so + is toward search_dir */
    turned = (((p_ctx->pos_r - p_ctx->man_start_r)
               - (p_ctx->pos_l - p_ctx->man_start_l))
              / 2)
             * p_ctx->search_dir;

    if ((0u == p_ctx->search_phase)
        && (turned >= PIVOT_COUNTS(MOTION_LS_SWEEP_DEG)))
    {
        p_ctx->search_phase = 1u;
        motion_pid_reset(&p_ctx->pid_l);
        motion_pid_reset(&p_ctx->pid_r);
    }
    else if ((1u == p_ctx->search_phase)
             && (turned <= -PIVOT_COUNTS(MOTION_LS_SWEEP_DEG)))
    {
        p_ctx->b_search_over = true;
    }
    else
    {
        /* Still sweeping. */
    }

    if (p_ctx->b_search_over)
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }
    else
    {
        dir = (0u == p_ctx->search_phase) ? p_ctx->search_dir
                                          : -p_ctx->search_dir;
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, -dir * MOTION_LS_SPEED,
                                       p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, dir * MOTION_LS_SPEED,
                                       p_ctx->d_r);
    }
}

/* TURNING (FR-08): straight until the axle is over the crossing, then a
   pivot on the spot by encoder count: 90 deg left or right, 180 deg
   counter-clockwise for a U-turn.  Straight on only drives across.  Each
   phase measures from its own start, so a stop and resume keeps it. */
static void
turn_drive (motion_ctx_t *p_ctx)
{
    int32_t moved_mm;
    int32_t turned;
    int32_t deg;
    int32_t dir = (BARCODE_ACT_RIGHT == p_ctx->turn_action) ? -1 : 1;

    if (TURN_PH_FORWARD == p_ctx->turn_phase)
    {
        moved_mm = motion_ds_dist_mm(p_ctx->pos_l - p_ctx->turn_start_l,
                                     p_ctx->pos_r - p_ctx->turn_start_r);
        if (moved_mm >= ((BARCODE_ACT_FORWARD == p_ctx->turn_action)
                             ? MOTION_TURN_CROSS_MM
                             : MOTION_TURN_FWD_MM))
        {
            p_ctx->turn_phase   = (BARCODE_ACT_FORWARD == p_ctx->turn_action)
                                      ? TURN_PH_DONE
                                      : TURN_PH_PIVOT;
#if MOTION_TURN_IMU
            {
                /* 90 (180) deg from the line's own direction, learnt over
                   Line Following, so the car ends lined up with the new
                   line; from the heading now if none was learnt */
                int32_t h0 = p_ctx->b_hd_line
                                 ? (int32_t)(p_ctx->hd_line_udeg / 1000000)
                                 : (int32_t)ev_heading_deg();

                p_ctx->turn_h_target
                    = h0 + (dir * ((BARCODE_ACT_UTURN == p_ctx->turn_action)
                                       ? 180 : 90));
                p_ctx->b_turn_lead = false;
            }
#endif
            p_ctx->turn_start_l = p_ctx->pos_l;
            p_ctx->turn_start_r = p_ctx->pos_r;
            motion_pid_reset(&p_ctx->pid_l);
            motion_pid_reset(&p_ctx->pid_r);
        }
    }
    else if (TURN_PH_PIVOT == p_ctx->turn_phase)
    {
        deg    = (BARCODE_ACT_UTURN == p_ctx->turn_action) ? 180 : 90;
        turned = (((p_ctx->pos_r - p_ctx->turn_start_r)
                   - (p_ctx->pos_l - p_ctx->turn_start_l))
                  / 2)
                 * dir;
#if MOTION_TURN_IMU
        {
            /* deg still to turn, on the IMU (left +); the counts only as a
               cap (wheel slip made 90 deg of counts 81, car 13:02) */
            int32_t rem    = (p_ctx->turn_h_target - (int32_t)ev_heading_deg()) * dir;
            bool    b_done = false;
#if MOTION_LF_STRADDLE
            /* Stop ON the new line (car 14:39:13: stopped at 89 deg on the
               IMU with the new line just outside the right sensor; it then
               drove off beside it).  In the last MOTION_TURN_WIN_DEG the
               line sweeps in over the leading sensor (the left one in a
               left turn); once it has left that sensor it is between the
               two: stop.  The trailing sensor on it: stop too (still under
               a sensor).  Never seen: MOTION_TURN_OVER_DEG past the target,
               then Line Search. */
            uint32_t rl    = line_state_rl(p_ctx->line_word);
            uint32_t lead  = (dir > 0) ? LINE_RL_LEFT_DEV : LINE_RL_RIGHT_DEV;
            uint32_t trail = (dir > 0) ? LINE_RL_RIGHT_DEV : LINE_RL_LEFT_DEV;

            if (rem <= MOTION_TURN_WIN_DEG)
            {
                if (0u != (rl & lead))
                {
                    p_ctx->b_turn_lead = true;
                }
                else if (p_ctx->b_turn_lead
                         && ((LINE_RL_CENTRED == rl) || (0u != (rl & trail))))
                {
                    b_done = true;
                }
                else
                {
                    /* not there yet */
                }
            }
            if (rem <= -MOTION_TURN_OVER_DEG)
            {
                b_done = true;
            }
#else
            b_done = (rem <= MOTION_HD_LEAD_DEG);
#endif
            if (b_done || (turned >= PIVOT_COUNTS(deg + 30)))
            {
                p_ctx->turn_phase = TURN_PH_DONE;
            }
        }
#else
        if (turned >= PIVOT_COUNTS(deg))
        {
            p_ctx->turn_phase = TURN_PH_DONE;
        }
#endif
    }
    else
    {
        /* Done: braked until the statechart leaves Turning. */
    }

    if (TURN_PH_FORWARD == p_ctx->turn_phase)
    {
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, MOTION_TURN_FWD_SPEED,
                                       p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, MOTION_TURN_FWD_SPEED,
                                       p_ctx->d_r);
    }
    else if (TURN_PH_PIVOT == p_ctx->turn_phase)
    {
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, -dir * MOTION_TURN_SPEED,
                                       p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, dir * MOTION_TURN_SPEED,
                                       p_ctx->d_r);
    }
    else
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }
}

/* ------------------------------------------------------------------ *
 *  IMU heading reference for the bypass (MOTION_HD_*, motion_cfg.h)
 * ------------------------------------------------------------------ */

/* round(1000 x sin(deg)), deg = 0..90 */
static int16_t const g_sin1000[91] = {
    0,   17,  35,  52,  70,  87,  105, 122, 139, 156, 174, 191, 208, 225,
    242, 259, 276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454,
    469, 485, 500, 515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656,
    669, 682, 695, 707, 719, 731, 743, 755, 766, 777, 788, 799, 809, 819,
    829, 839, 848, 857, 866, 875, 883, 891, 899, 906, 914, 921, 927, 934,
    940, 946, 951, 956, 961, 966, 970, 974, 978, 982, 985, 988, 990, 993,
    995, 996, 998, 999, 999, 1000, 1000
};

/* 1000 x sin(deg) for any whole degree. */
static int32_t
sin1000_deg (int32_t deg)
{
    int32_t d    = deg % 360;
    int32_t sign = 1;

    if (d < 0)
    {
        d += 360;
    }
    if (d >= 180)
    {
        d -= 180;
        sign = -1;
    }
    if (d > 90)
    {
        d = 180 - d;
    }
    return sign * (int32_t)g_sin1000[d];
}

/* Sideways distance from the line, mm, + = left.  hd_lat_acc sums
   (d_l + d_r) x sin1000 each tick: 2 x counts x 1000. */
static int32_t
hd_lat_mm (motion_ctx_t const *p_ctx)
{
    return (int32_t)(((int64_t)p_ctx->hd_lat_acc * MOTION_WHEEL_DIA_MM * 355)
                     / ((int64_t)113 * MOTION_COUNTS_PER_REV * 2000));
}

/* Distance along the line since the car left it (where it braked for the
   obstacle), mm, same units as hd_lat_mm. */
static int32_t
hd_lon_mm (motion_ctx_t const *p_ctx)
{
    return (int32_t)(((int64_t)p_ctx->hd_lon_acc * MOTION_WHEEL_DIA_MM * 355)
                     / ((int64_t)113 * MOTION_COUNTS_PER_REV * 2000));
}

/* Once per tick, before man_drive: learn the line's direction in Line
   Following, sum the sideways move off the line, and say whether the
   heading can steer the bypass this tick (b_hd_ok). */
/* MOTION_HD_LINE_MM / MOTION_HD_MIN_MM in the units hd_tick sums each tick,
   d_l + d_r (2 x counts): 2 x mm x counts per turn / (pi x diameter),
   pi as 355/113. */
#define HD_2COUNTS(mm)                                                         \
    ((int32_t)((2 * (int64_t)(mm) * MOTION_COUNTS_PER_REV * 113)               \
               / ((int64_t)355 * MOTION_WHEEL_DIA_MM)))

static void
hd_tick (motion_ctx_t *p_ctx, motion_state_t state)
{
#if MOTION_HD_ENABLE
    int32_t h     = (int32_t)ev_heading_deg();
    bool    b_imu = (0u == IMU_W_GET(imu_word_read(), FAULT));
    int32_t jump  = motion_abs(h - p_ctx->hd_prev_deg);
    int32_t fwd   = p_ctx->d_l + p_ctx->d_r; /* 2 x counts this tick */

    p_ctx->hd_prev_deg = h;
    if (!b_imu || (MOTION_ST_IDLE == state) || (MOTION_ST_TURNING == state)
        || (MOTION_ST_FAULT == state)
        || ((MOTION_ST_LINE_FOLLOW != state) && (jump > MOTION_HD_JUMP_DEG)))
    {
        /* New mission, a junction turn, a fault or a heading reset: the
           old line direction no longer holds. */
        p_ctx->b_hd_line  = false;
        p_ctx->hd_travel  = 0;
        p_ctx->hd_lat_acc = 0;
        p_ctx->hd_lon_acc = 0;
    }
    else if (MOTION_ST_LINE_FOLLOW == state)
    {
        if (!p_ctx->b_hd_line || (jump > MOTION_HD_JUMP_DEG))
        {
            p_ctx->hd_line_udeg = (int64_t)h * 1000000;
            p_ctx->hd_travel    = 0;
            p_ctx->b_hd_line    = true;
            p_ctx->hd_snap_trav = 0;
        }
#if MOTION_HD_SNAP_DEG > 0
        else if ((motion_abs(h - (int32_t)(p_ctx->hd_line_udeg / 1000000))
                  > MOTION_HD_SNAP_DEG)
                 && ((p_ctx->hd_snap_trav + fwd) >= HD_2COUNTS(MOTION_HD_SNAP_MM)))
        {
            /* Off the average for MOTION_HD_SNAP_MM running: a curve (a
               wobble swings back sooner).  The line now runs this way. */
            p_ctx->hd_line_udeg = (int64_t)h * 1000000;
            p_ctx->hd_snap_trav = 0;
        }
#endif
        else if (fwd > 0)
        {
#if MOTION_HD_SNAP_DEG > 0
            if (motion_abs(h - (int32_t)(p_ctx->hd_line_udeg / 1000000))
                > MOTION_HD_SNAP_DEG)
            {
                p_ctx->hd_snap_trav += fwd;
            }
            else
            {
                p_ctx->hd_snap_trav = 0;
            }
#endif
            /* Distance-weighted average: each tick pulls the line direction
               toward the heading by the share of MOTION_HD_LINE_MM driven. */
            p_ctx->hd_line_udeg
                += ((((int64_t)h * 1000000) - p_ctx->hd_line_udeg) * fwd)
                   / HD_2COUNTS(MOTION_HD_LINE_MM);
            if (p_ctx->hd_travel < HD_2COUNTS(MOTION_HD_MIN_MM))
            {
                p_ctx->hd_travel += fwd;
            }
        }
        else
        {
            /* Not moving forward: nothing new about the line. */
        }
        p_ctx->hd_lat_acc = 0; /* on the line */
        p_ctx->hd_lon_acc = 0;
    }
    else if (p_ctx->b_hd_line)
    {
        /* Off the line (Braked, Bypass, Reversing, Line Search): sideways
           and along it, from where it left the line. */
        int32_t err = h - (int32_t)(p_ctx->hd_line_udeg / 1000000);

        p_ctx->hd_lat_acc += fwd * sin1000_deg(err);
        p_ctx->hd_lon_acc += fwd * sin1000_deg(90 - err);
    }
    else
    {
        /* No line direction yet: nothing to measure from. */
    }
    p_ctx->hd_err_deg = h - (int32_t)(p_ctx->hd_line_udeg / 1000000);
    p_ctx->b_hd_ok    = b_imu && p_ctx->b_hd_line
                     && (p_ctx->hd_travel >= HD_2COUNTS(MOTION_HD_MIN_MM));
#else
    (void)state;
    p_ctx->b_hd_ok = false;
#endif
}

/* Heading target of each bypass pivot with the IMU, as (heading - line)
   x side in degrees; the straight and braked phases are not used.  The
   arc's OUT uses arc_deg instead. */
static int8_t const g_byp_hd_target[BYP_PH_DONE] = {
    MOTION_BYPASS_SIDE_DEG, 0, 0, 0, 0, 0, 0, -45, 0, 0, 0,
    -MOTION_BYPASS_SIDE_DEG, 0, 0, 0, 0, 0, 0, 0, 0
};

/* Integer square root (floor). */
static int32_t
isqrt32 (int32_t v)
{
    int32_t r = 0;

    while (((r + 1) * (r + 1)) <= v)
    {
        r++;
    }
    return r;
}

/* The obstacle's reach from its centre and the circle (MOTION_OBS_*,
   MOTION_ARC_*, motion_cfg.h). */
#if MOTION_OBS_ROUND
#define ARC_REACH_MM()  (MOTION_OBS_W_MM / 2)
#define ARC_HALF_LEN_MM (MOTION_OBS_W_MM / 2)
#else
#define ARC_REACH_MM()                                                         \
    isqrt32(((MOTION_OBS_W_MM / 2) * (MOTION_OBS_W_MM / 2))                    \
            + ((MOTION_OBS_L_MM / 2) * (MOTION_OBS_L_MM / 2)))
#define ARC_HALF_LEN_MM (MOTION_OBS_L_MM / 2)
#endif

/* At a new BYPASS: run the arc (MOTION_BYPASS_ARC) if the heading is valid
   and the car is still on the line, and work out its circle (centred on
   the obstacle, which may sit off the line), the tangent angle and leg. */
static void
arc_plan (motion_ctx_t *p_ctx)
{
#if MOTION_BYPASS_ARC
    int32_t cy = MOTION_ARC_STOP_MM + MOTION_SENSOR_AXLE_MM + ARC_HALF_LEN_MM;
    int32_t cx = p_ctx->bypass_off_mm - MOTION_ARC_OFFSET_CENTRED;
    int32_t r  = ARC_REACH_MM() + ((MOTION_WHEELBASE_MM + 1) / 2)
                 + MOTION_ARC_GAP_MM;
    int32_t d2;
    int32_t phi;
    int32_t ang;

    p_ctx->b_arc = p_ctx->b_hd_ok && (motion_abs(hd_lat_mm(p_ctx)) < 30);
    if (!p_ctx->b_arc)
    {
        return;
    }
    /* how far the centre sits toward (+) or away from (-) the bypass side,
       from scan_task's offset (45 + C_pass when centred) */
    if (cx > MOTION_ARC_MAX_SHIFT_MM)
    {
        cx = MOTION_ARC_MAX_SHIFT_MM;
    }
    if (cx < -MOTION_ARC_MAX_SHIFT_MM)
    {
        cx = -MOTION_ARC_MAX_SHIFT_MM;
    }
    d2 = (cx * cx) + (cy * cy);
    /* phi: direction of the centre off the line, atan(cx / cy) */
    phi = 0;
    while ((phi < 89) && ((sin1000_deg(phi) * cy) < (cx * sin1000_deg(90 - phi))))
    {
        phi++;
    }
    while ((phi > -89) && ((sin1000_deg(phi) * cy) > (cx * sin1000_deg(90 - phi))))
    {
        phi--;
    }
    /* ang: half the angle the circle fills seen from the car, asin(r / d) */
    ang = 0;
    while ((ang < 89)
           && (((int64_t)sin1000_deg(ang) * sin1000_deg(ang) * d2)
               < ((int64_t)r * r * 1000000)))
    {
        ang++;
    }
    p_ctx->arc_r   = r;
    p_ctx->arc_cx  = cx;
    p_ctx->arc_cy  = cy;
    p_ctx->arc_deg = ((phi + ang) > 80) ? 80 : (phi + ang);
    p_ctx->arc_leg = isqrt32(d2 - (r * r));
    p_ctx->arc_xt  = (p_ctx->arc_leg * sin1000_deg(p_ctx->arc_deg)) / 1000;
    p_ctx->arc_yt  = (p_ctx->arc_leg * sin1000_deg(90 - p_ctx->arc_deg)) / 1000;
#else
    p_ctx->b_arc = false;
#endif
}

/* Angle of (y, x) from the +x axis toward +y, whole degrees, -180..180. */
static int32_t
atan2_deg (int32_t y, int32_t x)
{
    int32_t ax = motion_abs(x);
    int32_t ay = motion_abs(y);
    int32_t a  = 0;

    while ((a < 90) && ((sin1000_deg(a) * ax) < (ay * sin1000_deg(90 - a))))
    {
        a++;
    }
    if (x < 0)
    {
        a = 180 - a;
    }
    return (y < 0) ? -a : a;
}

/* Following the arc (BYP_PH_ARC).  Positions and headings are taken toward
   the bypass side (x), along the line (y), headings from the line toward
   the side.  At the car's place relative to the centre, p = (dx, dy), the
   circle runs along t = (-dy, dx); the heading wanted is t's direction plus
   MOTION_ARC_K_X10 / 10 deg per mm the car is inside the circle (outward),
   at most MOTION_ARC_K_MAX_DEG.  The wheels: the turn the circle needs
   (feed-forward, b / 2R of the speed) plus one count per
   MOTION_ARC_STEER_DEG of heading error.  A distance-only trim overshot in
   the sim (177 mm from the centre on a 243 mm circle). */
#define MOTION_ARC_EARLY_MM  (20) /* start the arc this far before the circle */
#define MOTION_ARC_K_X10     (3)  /* 0.3 deg per mm off the circle            */
#define MOTION_ARC_K_MAX_DEG (30)
#define MOTION_ARC_STEER_DEG (3)

static void
arc_drive (motion_ctx_t *p_ctx)
{
    int32_t side = p_ctx->bypass_side;
    int32_t r    = p_ctx->arc_r;
    int32_t dx   = (hd_lat_mm(p_ctx) * side) - p_ctx->arc_cx;
    int32_t dy   = hd_lon_mm(p_ctx) - p_ctx->arc_cy;
    int32_t dist = isqrt32((dx * dx) + (dy * dy));
    int32_t corr = ((r - dist) * MOTION_ARC_K_X10) / 10;
    int32_t want;
    int32_t err;
    int32_t turn;

    if (corr > MOTION_ARC_K_MAX_DEG)
    {
        corr = MOTION_ARC_K_MAX_DEG;
    }
    if (corr < -MOTION_ARC_K_MAX_DEG)
    {
        corr = -MOTION_ARC_K_MAX_DEG;
    }
    /* heading of t from the line toward the side: atan2(t_x, t_y) */
    want = atan2_deg(-dy, dx) + corr;
    err  = want - (p_ctx->hd_err_deg * side);
    while (err > 180)
    {
        err -= 360;
    }
    while (err < -180)
    {
        err += 360;
    }
    /* + = turn toward the side; the circle itself turns away from it */
    turn = (err / MOTION_ARC_STEER_DEG)
           - (((MOTION_BYPASS_SPEED * MOTION_WHEELBASE_MM) + r) / (2 * r));
    if (turn > MOTION_BYPASS_SPEED)
    {
        turn = MOTION_BYPASS_SPEED;
    }
    if (turn < -MOTION_BYPASS_SPEED)
    {
        turn = -MOTION_BYPASS_SPEED;
    }
    /* toward the side = left for a left bypass (+1): the right wheel faster */
    p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                              MOTION_BYPASS_SPEED - (turn * side), p_ctx->d_l);
    p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                              MOTION_BYPASS_SPEED + (turn * side), p_ctx->d_r);
}

/* Smooth bypass (MOTION_BYPASS_SMOOTH, motion_cfg.h).  Sideways place
   toward the side, mm: the measured sideways move plus where the axle sat
   at the start. */
#define SM_S_SEEN_MM (p_ctx->sm_s_in - MOTION_SM_LOOK_MM)

/* Side-watch outcomes for the console (print_byp), not phases. */
#define BYP_EV_SEEN  (BYP_PH_DONE + 1u)
#define BYP_EV_CLEAR (BYP_PH_DONE + 2u)
#define BYP_EV_MISS  (BYP_PH_DONE + 3u)

static void
sm_watch_end (motion_ctx_t *p_ctx)
{
    ID scan_id = scan_if_flag();

    if (p_ctx->b_sm_watch && (scan_id > 0))
    {
        (void)tk_set_flg(scan_id, SCAN_FLG_WATCH_END);
    }
    p_ctx->b_sm_watch = false;
}

static void
sm_watch_report (motion_ctx_t *p_ctx, uint32_t ev, int32_t a, int32_t b)
{
    motion_report_t rep;

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BYP);
    rep.val_l  = (int32_t)ev;
    rep.val_r  = a;
    rep.rate_l = b;
    rep.rate_r = 1;
    report_send(p_ctx, &rep);
}

/* One tick of the side watch.  MOTION_SM_WATCH 2: 90 deg, square on to the
   box's side (motion_cfg.h): seen, then clear twice -> the way back in
   starts where the box ends; never seen -> the 15 cm-deep fallback.
   MOTION_SM_WATCH 1 (45 deg) only ever pushed the way back later. */
static void
sm_watch_tick (motion_ctx_t *p_ctx, int32_t s, int32_t x)
{
    int32_t mm;
    int32_t th = p_ctx->hd_err_deg * p_ctx->bypass_side;
    /* the line's distance along the beam (MOTION_SM_WATCH_DEG off the
       car's heading): nearer than that is the box */
    int32_t thr = (p_ctx->sm_y * 1000) / sin1000_deg(MOTION_SM_WATCH_DEG);
    bool    b_along = (x >= (p_ctx->sm_y - 40))
                      && (motion_abs(th) <= MOTION_SM_WATCH_PAR_DEG);

    if (!p_ctx->b_sm_watch)
    {
        return;
    }
    if (p_ctx->b_sm_new)
    {
        p_ctx->b_sm_new = false;
        mm = (int32_t)p_ctx->sm_watch_mm;
        if (b_along && (mm > 1) && (mm < thr))
        {
            /* nearer than the line: the box beside the car.  Where the
               echo came from, in the line frame (along s, out toward the
               side): the sensor sits MOTION_SENSOR_AXLE_MM ahead of the
               axle, the beam MOTION_SM_WATCH_DEG off the heading toward
               the line (bdeg: from the line's direction, + = out). */
            int32_t sxs  = x + ((MOTION_SENSOR_AXLE_MM * sin1000_deg(th)) / 1000);
            int32_t sss  = s + ((MOTION_SENSOR_AXLE_MM * sin1000_deg(90 - th)) / 1000);
            int32_t bdeg = th - MOTION_SM_WATCH_DEG;
            int32_t plat = sxs + ((mm * sin1000_deg(bdeg)) / 1000);
            int32_t ps   = sss + ((mm * sin1000_deg(90 - bdeg)) / 1000);
            int32_t need = plat + MOTION_SM_CAR_HALF_MM + MOTION_SM_WIDEN_GAP_MM;

            if (!p_ctx->b_sm_seen)
            {
                p_ctx->b_sm_seen = true;
                p_ctx->sm_seen_s = s;
                sm_watch_report(p_ctx, BYP_EV_SEEN, s, mm);
            }
            p_ctx->sm_clear_n = 0u;
            if (ps > p_ctx->sm_far_seen)
            {
                p_ctx->sm_far_seen = ps; /* the box goes on at least to here */
            }
            if (plat > p_ctx->sm_edge_max)
            {
                p_ctx->sm_edge_max = plat;
            }
            /* widen the pass (their side watch): two echoes in a row say
               the side is nearer than planned -> out by the smaller, at
               most MOTION_SM_WIDEN_MAX_MM past the plan; never in */
            if ((0 != p_ctx->sm_need_prev) && (need > p_ctx->sm_y))
            {
                int32_t y = (need < p_ctx->sm_need_prev) ? need : p_ctx->sm_need_prev;

                if (y > (p_ctx->sm_y_plan + MOTION_SM_WIDEN_MAX_MM))
                {
                    y = p_ctx->sm_y_plan + MOTION_SM_WIDEN_MAX_MM;
                }
                if (y > p_ctx->sm_y)
                {
                    p_ctx->sm_y = y;
                }
            }
            p_ctx->sm_need_prev = need;
        }
        else if (b_along && p_ctx->b_sm_seen && (mm >= thr))
        {
            /* at or beyond the line (511 = nothing in range): past it */
            p_ctx->sm_need_prev = 0;
            p_ctx->sm_clear_n++;
            if (p_ctx->sm_clear_n >= 2u)
            {
                /* The far end: at least the furthest echo (+ margin), and at
                   least where the beam's nearest ray meets the side line now
                   (the beam stopped hitting the box: it ends behind that,
                   MOTION_SM_CLEAR_MIN_MM ahead of the axle).  Earliest safe
                   return (theirs, simplified): the turn may start before the
                   far end by half the side gap beyond MOTION_SM_EARLY_GAP_MM,
                   at most MOTION_SM_EARLY_MAX_MM; never behind the car. */
                int32_t far   = p_ctx->sm_far_seen + MOTION_SM_FAR_MARGIN_MM;
                int32_t gap   = (p_ctx->sm_y - MOTION_SM_CAR_HALF_MM) - p_ctx->sm_edge_max;
                int32_t early = (gap - MOTION_SM_EARLY_GAP_MM) / 2;
                int32_t s_in;

                if (far < (s + MOTION_SM_CLEAR_MIN_MM))
                {
                    far = s + MOTION_SM_CLEAR_MIN_MM;
                }
                early = (early < 0) ? 0
                        : ((early > MOTION_SM_EARLY_MAX_MM) ? MOTION_SM_EARLY_MAX_MM : early);
                s_in = far - early + MOTION_SM_LOOK_MM;
                if (s_in < (s + MOTION_SM_LOOK_MM))
                {
                    s_in = s + MOTION_SM_LOOK_MM; /* turn now */
                }
                if (s_in > MOTION_SM_S_IN_MAX_MM)
                {
                    s_in = MOTION_SM_S_IN_MAX_MM;
                }
                p_ctx->sm_s_in = s_in;
                sm_watch_report(p_ctx, BYP_EV_CLEAR, far, s_in);
                sm_watch_end(p_ctx);
            }
        }
        else
        {
            /* not alongside yet, or no echo (1): nothing learnt */
        }
    }
    if (p_ctx->b_sm_watch && !p_ctx->b_sm_seen
        && (s >= (p_ctx->sm_fb_s_in - MOTION_SM_LOOK_MM)))
    {
        /* never saw the box's side: back in after a 15 cm-deep box */
        p_ctx->sm_s_in = p_ctx->sm_fb_s_in;
        sm_watch_report(p_ctx, BYP_EV_MISS, s, p_ctx->sm_fb_s_in);
        sm_watch_end(p_ctx);
    }
    if (p_ctx->b_sm_watch
        && (s >= (MOTION_SM_S_IN_MAX_MM - MOTION_SM_LOOK_MM)))
    {
        sm_watch_end(p_ctx);
    }
}

static int32_t
sm_x_mm (motion_ctx_t const *p_ctx)
{
    return (hd_lat_mm(p_ctx) * p_ctx->bypass_side) + p_ctx->sm_x0;
}

/* Set up at the start of a BYPASS: smooth only with a heading, from the
   line (as the arc). */
static void
sm_plan (motion_ctx_t *p_ctx)
{
#if MOTION_BYPASS_SMOOTH
    int32_t shift = p_ctx->bypass_off_mm - MOTION_ARC_OFFSET_CENTRED;
    int32_t th0;

#if MOTION_SM_HD_FALLBACK
    if (!p_ctx->b_hd_ok && (0u == IMU_W_GET(imu_word_read(), FAULT)))
    {
        /* No line direction learned yet (car 20:44: the run started 25 cm
           from the box, before MOTION_HD_MIN_MM of Line Following, so the
           zig-zag ran).  The car is braked on the line facing along it:
           its heading is the line's. */
        p_ctx->hd_line_udeg = (int64_t)ev_heading_deg() * 1000000;
        p_ctx->b_hd_line    = true;
        p_ctx->hd_travel    = HD_2COUNTS(MOTION_HD_MIN_MM);
        p_ctx->hd_lat_acc   = 0;
        p_ctx->hd_lon_acc   = 0;
        p_ctx->hd_err_deg   = 0;
        p_ctx->b_hd_ok      = true;
    }
#endif
#if MOTION_SM_BOX_DIR
    {
        /* The line's direction from the box itself: it sits on the line,
           so from the car (its line sensors on the line) to the box's
           centre is the way the line runs there.  The learnt average lags
           in a curve (car 14:47: braked coming out of a left curve, the
           box from -35 to +10 deg, the average said 6 deg off, the box
           ~12: the path came out rotated, crossed a junction's arm 20 cm
           out and took it for the line). */
        uint32_t cb = (scan_profile_word() >> 24) & 0xFFu;

        if ((0u != cb) && (0u == IMU_W_GET(imu_word_read(), FAULT)))
        {
            int32_t c = (int32_t)cb - 128; /* box centre, deg, - = left */

            if (motion_abs(c) <= MOTION_SM_BOX_DIR_MAX)
            {
                int32_t h = (int32_t)ev_heading_deg();  /* left + */

                p_ctx->hd_line_udeg = (int64_t)(h - c) * 1000000;
                p_ctx->b_hd_line    = true;
                p_ctx->hd_travel    = HD_2COUNTS(MOTION_HD_MIN_MM);
                p_ctx->hd_lat_acc   = 0;
                p_ctx->hd_lon_acc   = 0;
                p_ctx->hd_err_deg   = c;     /* heading - line */
                p_ctx->b_hd_ok      = true;
            }
        }
    }
#endif
    /* MOTION_SM_HD_FALLBACK 0: no line direction learned (the run started
       at the box) -> b_smooth false -> the zig-zag, which measures the box
       itself (car 23:21: started 25 cm from the box, its own heading taken
       as the line's, the smooth path came out rotated) */
    th0 = p_ctx->hd_err_deg * p_ctx->bypass_side;
    p_ctx->b_smooth = p_ctx->b_hd_ok && (motion_abs(hd_lat_mm(p_ctx)) < 30);
    if (!p_ctx->b_smooth)
    {
        return;
    }
    p_ctx->b_arc = false;
    if (shift < MOTION_SM_SHIFT_MIN)
    {
        shift = MOTION_SM_SHIFT_MIN;
    }
    if (shift > MOTION_SM_SHIFT_MAX)
    {
        shift = MOTION_SM_SHIFT_MAX;
    }
    p_ctx->sm_y     = MOTION_SM_Y_MM + shift;
    /* the line sensors are on the line: the axle sits behind them */
    p_ctx->sm_x0    = -((MOTION_LINE_SENSOR_MM * sin1000_deg(th0)) / 1000);
    p_ctx->sm_rem   = 0;
    p_ctx->b_sm_out = false;
    p_ctx->sm_touch_um = 0u;
    p_ctx->b_sm_cross  = false;
    p_ctx->sm_s_in  = MOTION_SM_S_IN_MM;
#if MOTION_SM_ADAPT
    {
        uint32_t        pw  = scan_profile_word();
        int32_t         d_c = (int32_t)(pw & 0xFFFu);
        int32_t         w   = (int32_t)((pw >> 12) & 0xFFFu);
        motion_report_t rep;

        if ((d_c >= 100) && (d_c <= 400))
        {
            /* the turn starts (S_IN - look) only once the axle is at the
               far end of a MOTION_SM_BOX_LEN_MM deep box (car 22:07: it
               began 6.5 cm before the far end with a tight swing-out and
               the inner side met the far corner) */
            int32_t s_in = d_c + MOTION_SENSOR_AXLE_MM + MOTION_SM_BOX_LEN_MM
                           + MOTION_SM_LOOK_MM - MOTION_SM_TURN_EARLY_MM;

            p_ctx->sm_s_in = (s_in < 450) ? 450 : ((s_in > 750) ? 750 : s_in);
        }
        if ((w >= 50) && (w <= 300))
        {
            int32_t y = (w / 2) + MOTION_SM_CAR_HALF_MM + MOTION_SM_SIDE_GAP_MM;

            y = (y < MOTION_SM_Y_MIN_MM) ? MOTION_SM_Y_MIN_MM
                : ((y > MOTION_SM_Y_MAX_MM) ? MOTION_SM_Y_MAX_MM : y);
            /* only ever wider: the scan reads the box ~1-2 cm left almost
               every time (the servo's zero, likely), which narrowed the
               right-hand swing-out (car 22:07: 166 mm) */
            p_ctx->sm_y = y + ((shift > 0) ? shift : 0);
        }
        report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BYP);
        rep.val_l  = (int32_t)(BYP_PH_DONE + 5u);
        rep.val_r  = p_ctx->sm_y;
        rep.rate_l = p_ctx->sm_s_in;
        rep.rate_r = (int32_t)pw;
        report_send(p_ctx, &rep);
    }
#endif
    p_ctx->b_sm_watch = false;
    p_ctx->b_sm_seen  = false;
    p_ctx->b_sm_new   = false;
    p_ctx->sm_clear_n = 0u;
    p_ctx->b_sm_settle = false;
#if (MOTION_SM_WATCH == 2)
    {
        uint32_t pw  = scan_profile_word();
        int32_t  d_c = (int32_t)(pw & 0xFFFu);

        p_ctx->sm_fb_s_in = ((d_c >= 100) && (d_c <= 400))
                                ? (d_c + MOTION_SENSOR_AXLE_MM
                                   + MOTION_SM_BOX_LEN_MM + MOTION_SM_LOOK_MM)
                                : 650;
        p_ctx->sm_s_in = MOTION_SM_S_IN_MAX_MM; /* alongside until it knows */
        p_ctx->sm_y_plan    = p_ctx->sm_y;
        p_ctx->sm_far_seen  = 0;
        p_ctx->sm_edge_max  = -1000;
        p_ctx->sm_need_prev = 0;
    }
#endif
#else
    p_ctx->b_smooth = false;
#endif
}

/* The path's sideways place at s mm along the line. */
static int32_t
sm_path_mm (motion_ctx_t const *p_ctx, int32_t s)
{
    int32_t y = p_ctx->sm_y;
    int32_t x;

    if (s <= 0)
    {
        x = 0;
    }
    else if (s < MOTION_SM_S_OUT_MM)
    {
        x = (y * (1000 - sin1000_deg(90 - ((180 * s) / MOTION_SM_S_OUT_MM))))
            / 2000;
    }
    else if (s < p_ctx->sm_s_in)
    {
        x = y;
    }
    else if (s < (p_ctx->sm_s_in + MOTION_SM_L_IN_MM))
    {
        x = (y * (1000 + sin1000_deg(90 - ((180 * (s - p_ctx->sm_s_in))
                                           / MOTION_SM_L_IN_MM))))
            / 2000;
    }
    else
    {
        x = -(((s - p_ctx->sm_s_in - MOTION_SM_L_IN_MM)
               * sin1000_deg(MOTION_SM_GLIDE_DEG))
              / sin1000_deg(90 - MOTION_SM_GLIDE_DEG));
    }
    return x;
}

/* One tick on the curve: aim at the path MOTION_SM_LOOK_MM ahead (pure
   pursuit), one count of wheel difference per MOTION_SM_STEER_DEG of
   heading error (the fraction carried), soft start. */
static void
sm_drive (motion_ctx_t *p_ctx)
{
    int32_t side = p_ctx->bypass_side;
    int32_t s    = hd_lon_mm(p_ctx);
    int32_t x    = sm_x_mm(p_ctx);
    int32_t v    = 3 + (int32_t)(p_ctx->look_ticks / 5u);
    int32_t want;
    int32_t err;
    int32_t turn;
    int32_t cap;

    if (v > MOTION_SM_SPEED)
    {
        v = MOTION_SM_SPEED;
    }
    want = atan2_deg(sm_path_mm(p_ctx, s + MOTION_SM_LOOK_MM) - x,
                     MOTION_SM_LOOK_MM);
    if (want > MOTION_SM_MAX_DEG)
    {
        want = MOTION_SM_MAX_DEG;
    }
    if (want < -MOTION_SM_MAX_DEG)
    {
        want = -MOTION_SM_MAX_DEG;
    }
    err = want - (p_ctx->hd_err_deg * side);
    p_ctx->sm_rem += (err * 16) / MOTION_SM_STEER_DEG;
    turn = p_ctx->sm_rem / 16;
    p_ctx->sm_rem -= turn * 16;
    cap = v - MOTION_SM_MIN_WHEEL;
    if (turn > cap)
    {
        turn = cap;
    }
    if (turn < -cap)
    {
        turn = -cap;
    }
    /* + = toward the side: for a left bypass (+1) the right wheel faster */
    p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, v - (turn * side),
                              p_ctx->d_l);
    p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, v + (turn * side),
                              p_ctx->d_r);
}

/* One EVT_BYP line: the phase just entered and where the car is. */
static void
byp_report (motion_ctx_t *p_ctx)
{
    motion_report_t rep;
    int32_t         side = p_ctx->bypass_side;

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BYP);
    rep.val_l  = (int32_t)p_ctx->bypass_phase;
    rep.val_r  = hd_lat_mm(p_ctx) * side;
    rep.rate_l = p_ctx->hd_err_deg * side;
    rep.rate_r = p_ctx->b_hd_ok ? 1 : 0;
    report_send(p_ctx, &rep);
}

/* Next bypass phase: its own start positions and a fresh PID. */
static void
byp_next (motion_ctx_t *p_ctx, uint32_t ph)
{
    p_ctx->bypass_phase   = (uint8_t)ph;
    p_ctx->bypass_start_l = p_ctx->pos_l;
    p_ctx->bypass_start_r = p_ctx->pos_r;
    p_ctx->look_ticks     = 0u;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
    byp_report(p_ctx);
}

/* Ask scan_task to look (SCAN_FLG_LOOK or SCAN_FLG_LOOK_AHEAD); the answer
   comes back as an AVOID_LOOK command (take_commands). */
static void
byp_ask (motion_ctx_t *p_ctx, UINT bit)
{
    ID scan_id = scan_if_flag();

    p_ctx->look_state = LOOK_ASKED;
    if (scan_id > 0)
    {
        (void)tk_set_flg(scan_id, bit);
    }
}

/* A look is over when answered or timed out; a timeout counts as "something
   there".  true when over, with *p_near set when it read below near_mm. */
static bool
byp_look_done (motion_ctx_t *p_ctx, int32_t near_mm, bool *p_near)
{
    bool b_done;

    p_ctx->look_ticks++;
    b_done = (LOOK_ANSWERED == p_ctx->look_state)
             || (p_ctx->look_ticks >= MOTION_BYPASS_LOOK_TICKS);
    if (b_done)
    {
        *p_near = (LOOK_ANSWERED != p_ctx->look_state)
                  || ((int32_t)p_ctx->look_mm < near_mm);
        p_ctx->look_state = LOOK_NONE;
    }
    return b_done;
}

/* After a probe, back on the heading: go past, give up, or another step. */
static void
byp_after_probe (motion_ctx_t *p_ctx)
{
    if (p_ctx->b_look_present)
    {
        p_ctx->b_bypass_seen = true;
    }
    if (!p_ctx->b_look_present
        && (p_ctx->b_bypass_seen
            || (p_ctx->bypass_pass_mm >= MOTION_BYPASS_UNSEEN_MM)))
    {
        byp_next(p_ctx, BYP_PH_CLEAR);
    }
    else if (p_ctx->bypass_pass_mm >= MOTION_BYPASS_MAX_PASS_MM)
    {
        /* Still beside it: no safe way back in (FR-18 "otherwise stop"). */
        byp_next(p_ctx, BYP_PH_DONE);
        p_ctx->line_dir = -p_ctx->bypass_side;
    }
    else
    {
        byp_next(p_ctx, BYP_PH_STEP);
    }
}

/* BYPASS (FR-18, D14): the phases in motion_cfg.h.  It ends with the axle
   on the line and the car on its old heading, so Line Search finds the
   line at once. */
static void
bypass_drive (motion_ctx_t *p_ctx)
{
    uint32_t ph    = p_ctx->bypass_phase;
    int32_t  side  = p_ctx->bypass_side;
    int32_t  dl    = p_ctx->pos_l - p_ctx->bypass_start_l;
    int32_t  dr    = p_ctx->pos_r - p_ctx->bypass_start_r;
    int32_t  moved = motion_ds_dist_mm(dl, dr);
    int32_t  leg   = BYP_LEG_MM(p_ctx->bypass_off_mm);
    int32_t  lane  = leg + MOTION_BYPASS_LANE_MARGIN_MM;
    int32_t  turned;
    int32_t  dir;
    bool     b_pivot_done;
    bool     b_near;
    ID       scan_id;

    dir          = (ph < BYP_PH_DONE) ? ((int32_t)g_byp_turn[ph] * side) : 0;
    turned       = ((dr - dl) / 2) * dir;
    if (p_ctx->b_hd_ok && (0 != dir))
    {
        /* To a heading relative to the line, MOTION_HD_LEAD_DEG early;
           twice the encoder count stops it if the IMU misleads. */
        /* the arc's OUT: theta + the lead, so it ends near theta (stopping
           MOTION_HD_LEAD_DEG short aimed the leg inside the circle) */
        int32_t target = (p_ctx->b_smooth && (BYP_PH_OUT == ph))
                             ? MOTION_SM_PIVOT_DEG
                         : (p_ctx->b_arc && (BYP_PH_OUT == ph))
                             ? (p_ctx->arc_deg + MOTION_HD_LEAD_DEG)
                             : (int32_t)g_byp_hd_target[ph];

        b_pivot_done
            = (((target - (p_ctx->hd_err_deg * side))
                * (int32_t)g_byp_turn[ph])
               <= MOTION_HD_LEAD_DEG)
              || (turned >= (2 * BYP_PIVOT_COUNTS((int32_t)g_byp_deg[ph])));
    }
    else
    {
        b_pivot_done = (0 != dir)
                       && (turned >= BYP_PIVOT_COUNTS((int32_t)g_byp_deg[ph]));
    }
    if (lane > (int32_t)MOTION_AVOID_DIST_MAX)
    {
        lane = (int32_t)MOTION_AVOID_DIST_MAX; /* 511 = nothing in range */
    }

    switch (ph)
    {
        case BYP_PH_OUT:
            if (b_pivot_done && p_ctx->b_smooth)
            {
                p_ctx->sm_rem = 0;
                byp_next(p_ctx, BYP_PH_SMOOTH);
#if MOTION_SM_WATCH
                if (scan_if_flag() > 0)
                {
                    (void)tk_set_flg(scan_if_flag(), SCAN_FLG_WATCH);
                    p_ctx->b_sm_watch = true;
                }
#endif
            }
            else if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_LANE_LOOK);
                byp_ask(p_ctx, SCAN_FLG_LOOK_AHEAD);
            }
            break;
        case BYP_PH_SMOOTH:
            {
                int32_t x = sm_x_mm(p_ctx);
                int32_t s = hd_lon_mm(p_ctx);

                if (x > (p_ctx->sm_y / 2))
                {
                    p_ctx->b_sm_out = true;
                }
                sm_watch_tick(p_ctx, s, x);
#if MOTION_SM_CROSS_CHECK_MM > 0
                /* A crossing line, not the line (car 14:58: a junction's arm
                   beside the box, both sensors black at once while within
                   12 deg of the line's direction).  The 25 mm line covers
                   both sensors (~35 mm apart) only when crossed at 45 deg or
                   more; nearly side-on it touches one first.  So both black
                   within MOTION_SM_CROSS_DEG of the line's direction is a
                   line across it.  A touch is taken only after
                   MOTION_SM_CROSS_CHECK_MM, time for both to show. */
                if (LINE_RL_CENTRED != line_state_rl(p_ctx->line_word))
                {
                    int32_t f2 = p_ctx->d_l + p_ctx->d_r;

                    if ((f2 > 0) && (p_ctx->sm_touch_um < 100000000u))
                    {
                        p_ctx->sm_touch_um += (uint32_t)ST_UM_2C(f2);
                    }
                    if ((LINE_RL_JUNCTION == line_state_rl(p_ctx->line_word))
                        && (motion_abs(p_ctx->hd_err_deg) < MOTION_SM_CROSS_DEG))
                    {
                        p_ctx->b_sm_cross = true;
                    }
                }
                else
                {
                    p_ctx->sm_touch_um = 0u;
                    p_ctx->b_sm_cross  = false;
                }
#endif
                /* only near where the line should be (car 21:43, twice: at
                   18 cm out the line sensors met black beside the box,
                   probably a barcode sheet, took it for the line, lined
                   up there and swept into the box) */
                /* and heading in toward it: black met while running parallel
                   beside the box is not the line.  A distance-only gate (12 cm)
                   turned the real line away when the heading estimate was a few
                   degrees off (simulation, 21 Nov) */
                if (p_ctx->b_sm_out && (s >= SM_S_SEEN_MM)
                    && (x <= MOTION_SM_SEEN_X_MM)
#if MOTION_SM_CROSS_CHECK_MM > 0
                    && !p_ctx->b_sm_cross
                    && (p_ctx->sm_touch_um >= (MOTION_SM_CROSS_CHECK_MM * 1000u))
#endif
                    && ((p_ctx->hd_err_deg * side) <= -MOTION_SM_SEEN_DEG)
                    && (p_ctx->line_seen_run >= MOTION_LF_FOUND_TICKS))
                {
                    /* back on the line: Line Following takes it from here;
                       crossed from the side, so lost again it is that way */
                    /* MOTION_SM_ONTO (user, 20 Nov): on to the line the
                       sensors just met (ONTO: forward the sensor-to-axle
                       distance), then turn by the angle it came in at
                       (ALONG, to the line's heading) and only then Line
                       Following: a 40 deg entry straight into Line
                       Following set off a weave (car 23:02). */
                    sm_watch_end(p_ctx);
                    byp_next(p_ctx, MOTION_SM_ONTO ? BYP_PH_ONTO : BYP_PH_DONE);
                    p_ctx->line_dir = side;
                }
                else if (!p_ctx->b_hd_ok || (x < -MOTION_SM_MISS_X_MM)
                         || (s > (p_ctx->sm_s_in + MOTION_SM_L_IN_MM
                                  + MOTION_SM_GIVEUP_MM)))
                {
                    sm_watch_end(p_ctx);
                    /* no heading, or past where the line should be with no
                       line taken: stop and pivot to look (Hanis, 22 Nov: was
                       25 cm past before searching), first back toward the
                       side it came from, where a line crossed but not taken
                       would be */
                    byp_next(p_ctx, BYP_PH_DONE);
                    p_ctx->line_dir = (x > 0) ? -side : side;
#if MOTION_LF_STRADDLE
                    /* a real search: not the "touched it lately" shortcut
                       (that touch was a line not taken) */
                    p_ctx->st_nt_um = 100000000u;
#endif
                }
                else
                {
                    p_ctx->look_ticks++; /* soft start and the report */
                    if (0u == (p_ctx->look_ticks % 50u))
                    {
                        byp_report(p_ctx);
                    }
                }
            }
            break;
        case BYP_PH_LANE_LOOK:
            if (byp_look_done(p_ctx, lane, &b_near))
            {
                byp_next(p_ctx, b_near ? BYP_PH_LANE_BACK
                                       : (p_ctx->b_arc ? BYP_PH_ARC_LEG
                                                       : BYP_PH_SIDESTEP));
            }
            break;
        case BYP_PH_ARC_LEG:
            /* straight out to the circle: at the tangent point, or as soon
               as it is within R of the box centre (heading a little steep) */
            {
                int32_t x  = hd_lat_mm(p_ctx) * side;
                int32_t dx = x - p_ctx->arc_cx;
                int32_t dy = hd_lon_mm(p_ctx) - p_ctx->arc_cy;

                if ((hd_lon_mm(p_ctx) >= p_ctx->arc_yt)
                    || (x >= p_ctx->arc_xt)
                    || (((dx * dx) + (dy * dy))
                        <= ((p_ctx->arc_r + MOTION_ARC_EARLY_MM)
                            * (p_ctx->arc_r + MOTION_ARC_EARLY_MM)))
                    || (moved >= (2 * p_ctx->arc_leg)))
                {
                    byp_next(p_ctx, BYP_PH_ARC);
                }
            }
            break;
        case BYP_PH_ARC:
            /* round the box until heading theta back toward the line;
               then the measured sideways distance takes it to the line */
            if ((((p_ctx->hd_err_deg * side) + p_ctx->arc_deg)
                 <= MOTION_HD_LEAD_DEG)
                || !p_ctx->b_hd_ok
                || (moved >= (4 * p_ctx->arc_r)))
            {
                byp_next(p_ctx, BYP_PH_STEP_IN);
            }
            break;
        case BYP_PH_LANE_BACK:
            if (b_pivot_done)
            {
                /* Back on the heading scan_task planned in: it re-plans
                   the other side from that profile, or Blocked. */
                byp_next(p_ctx, BYP_PH_SWITCH);
                scan_id = scan_if_flag();
                if (scan_id > 0)
                {
                    (void)tk_set_flg(scan_id, SCAN_FLG_SIDE_BLOCKED);
                }
            }
            break;
        case BYP_PH_SWITCH:
            /* A new BYPASS (after STOP_SCAN) starts over; none: give up. */
            p_ctx->look_ticks++;
            if (p_ctx->look_ticks >= MOTION_BYPASS_SWITCH_TICKS)
            {
                byp_next(p_ctx, BYP_PH_DONE);
                p_ctx->line_dir = -side;
            }
            break;
        case BYP_PH_SIDESTEP:
            /* With the IMU: until the planned sideways distance from the
               line (whatever angle the car stopped at), 2 x leg at most. */
            if (p_ctx->b_hd_ok
                    ? ((hd_lat_mm(p_ctx) * side >= p_ctx->byp_lat_target)
                       || (moved >= (2 * leg)))
                    : (moved >= leg))
            {
                byp_next(p_ctx, BYP_PH_PARALLEL);
            }
            break;
        case BYP_PH_PARALLEL:
            if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_STEP);
            }
            break;
        case BYP_PH_STEP:
#if MOTION_BYPASS_KNOWN
            /* Known obstacle: straight past it by the calculated distance
               (MOTION_KNOWN_PASS_MM along the line from where the car
               braked), no probes beside it.  Without the IMU, alongside
               by that minus what the sidestep already went forward. */
            if (p_ctx->b_hd_ok
                    ? (hd_lon_mm(p_ctx)
                       >= (MOTION_KNOWN_PASS_MM + p_ctx->byp_pass_more))
                    : (moved >= ((0 == p_ctx->byp_pass_more)
                                     ? (MOTION_KNOWN_PASS_MM
                                        - ((leg * 342) / 1000)) /* cos 70 */
                                     : MOTION_KNOWN_MORE_MM)))
            {
                p_ctx->bypass_pass_mm += moved;
                byp_next(p_ctx, BYP_PH_TURN_IN);
            }
#else
            if (moved >= ((0 == p_ctx->bypass_pass_mm) ? MOTION_BYPASS_FIRST_MM
                                                       : MOTION_BYPASS_STEP_MM))
            {
                p_ctx->bypass_pass_mm += moved;
                byp_next(p_ctx, BYP_PH_PROBE_IN);
            }
#endif
            break;
        case BYP_PH_PROBE_IN:
            if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_LOOK);
                byp_ask(p_ctx, SCAN_FLG_LOOK);
            }
            break;
        case BYP_PH_LOOK:
            if (byp_look_done(p_ctx, MOTION_BYPASS_SEEN_MM, &b_near))
            {
                p_ctx->b_look_present = b_near;
                byp_next(p_ctx, BYP_PH_PROBE_OUT);
            }
            break;
        case BYP_PH_PROBE_OUT:
            if (b_pivot_done)
            {
                byp_after_probe(p_ctx);
            }
            break;
        case BYP_PH_CLEAR:
            if (moved >= MOTION_BYPASS_CLEAR_MM)
            {
                byp_next(p_ctx, BYP_PH_TURN_IN);
            }
            break;
        case BYP_PH_TURN_IN:
            if (b_pivot_done)
            {
#if MOTION_BYPASS_KNOWN
                /* Facing the way back: is it clear of the obstacle? */
                byp_next(p_ctx, BYP_PH_IN_LOOK);
                byp_ask(p_ctx, SCAN_FLG_LOOK_AHEAD);
#else
                byp_next(p_ctx, BYP_PH_STEP_IN);
#endif
            }
            break;
        case BYP_PH_IN_LOOK:
            if (byp_look_done(p_ctx, MOTION_KNOWN_IN_LOOK_MM, &b_near))
            {
                byp_next(p_ctx, b_near ? BYP_PH_IN_BACK : BYP_PH_STEP_IN);
            }
            break;
        case BYP_PH_IN_BACK:
            if (b_pivot_done)
            {
                /* Not past it yet: along again, a little further. */
                p_ctx->byp_pass_more += MOTION_KNOWN_MORE_MM;
                if (p_ctx->byp_pass_more > MOTION_KNOWN_MORE_MAX_MM)
                {
                    byp_next(p_ctx, BYP_PH_DONE); /* give up: Line Search */
                    p_ctx->line_dir = -side;
                }
                else
                {
                    byp_next(p_ctx, BYP_PH_STEP);
                }
            }
            break;
        case BYP_PH_STEP_IN:
            /* LINE_RL_CENTRED is "both white": anything else is the line.
               A confirmed black within MOTION_LF_SEEN_MS also counts (the
               same test as b_line_seen): crossing at ~60 deg the filtered
               rl never left 00 while line_task confirmed the line, so the
               car drove over it to the end of the leg (car, 4 Nov 2026). */
            /* With the IMU, also when the line sensors (MOTION_LINE_SENSOR_MM
               ahead of the axle) are back over the line's position: the
               way back is the sideways distance driven out, measured. */
            if ((LINE_RL_CENTRED != line_state_rl(p_ctx->line_word))
                || (line_state_dark_ms(p_ctx->line_word) < MOTION_LF_SEEN_MS)
                || (p_ctx->b_hd_ok
                        ? (((hd_lat_mm(p_ctx)
                             + ((MOTION_LINE_SENSOR_MM
                                 * sin1000_deg(p_ctx->hd_err_deg)) / 1000))
                            * side <= -MOTION_HD_PAST_MM)
                           || (moved >= (2 * (leg + MOTION_BYPASS_IN_EXTRA_MM))))
                        : (moved >= (leg + MOTION_BYPASS_IN_EXTRA_MM))))
            {
                byp_next(p_ctx, BYP_PH_ONTO);
            }
            break;
        case BYP_PH_ONTO:
            /* smooth: a little short of the sensor distance, so after
               ALONG the sensors are on the line, not just past it (car
               20:19: 2 cm past, a 3 s Line Search) */
            if (moved >= (p_ctx->b_smooth ? MOTION_SM_ONTO_MM
                                          : MOTION_LINE_SENSOR_MM))
            {
                byp_next(p_ctx, BYP_PH_ALONG);
            }
            break;
        case BYP_PH_ALONG:
            /* smooth: lined up, then braked MOTION_SM_SETTLE_TICKS before
               Line Search / Line Following, so they start from rest (car
               20:36: started still turning from the pivot, crossed the
               line and swung across it until lost) */
            if (b_pivot_done && p_ctx->b_smooth && !p_ctx->b_sm_settle)
            {
                p_ctx->b_sm_settle = true;
                p_ctx->look_ticks  = 0u;
            }
            if (p_ctx->b_sm_settle)
            {
                p_ctx->look_ticks++;
                if (p_ctx->look_ticks >= MOTION_SM_SETTLE_TICKS)
                {
                    p_ctx->b_sm_settle = false;
                    byp_next(p_ctx, BYP_PH_DONE);
                    p_ctx->line_dir = -side;
                }
            }
            else if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_DONE);
                p_ctx->line_dir = -side;
            }
            else
            {
                /* still turning */
            }
            break;
        default:
            /* Done: braked until the statechart leaves Bypass. */
            break;
    }

    ph  = p_ctx->bypass_phase;
    dir = (ph < BYP_PH_DONE) ? ((int32_t)g_byp_turn[ph] * side) : 0;
    if ((ph >= BYP_PH_DONE) || (BYP_PH_LOOK == ph)
        || ((BYP_PH_ALONG == ph) && p_ctx->b_sm_settle)
        || (BYP_PH_LANE_LOOK == ph) || (BYP_PH_SWITCH == ph)
        || (BYP_PH_IN_LOOK == ph))
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }
    else if (BYP_PH_ARC == ph)
    {
        arc_drive(p_ctx);
    }
    else if (BYP_PH_SMOOTH == ph)
    {
        sm_drive(p_ctx);
    }
    else if (0 != dir)
    {
        p_ctx->cmd_l = pid_capped(
            p_ctx, &p_ctx->pid_l, -dir * MOTION_BYPASS_TURN_SPEED, p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(
            p_ctx, &p_ctx->pid_r, dir * MOTION_BYPASS_TURN_SPEED, p_ctx->d_r);
    }
    else
    {
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, MOTION_BYPASS_SPEED,
                                       p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, MOTION_BYPASS_SPEED,
                                       p_ctx->d_r);
    }
}

/* Barcode re-read (MOTION_BC_RETRY, motion_cfg.h). */
#define RT_NONE  (0u)
#define RT_BRAKE (1u)
#define RT_ALIGN (2u)
#define RT_BACK  (3u)
#define RT_SLOW  (4u)
#define BYP_EV_RETRY (BYP_PH_DONE + 4u)

extern uint32_t barcode_retry_req(void); /* line_task/barcode_task.c */

static void
rt_next (motion_ctx_t *p_ctx, uint8_t ph)
{
    p_ctx->rt_phase = ph;
    p_ctx->rt_ticks = 0u;
    p_ctx->rt_l0    = p_ctx->pos_l;
    p_ctx->rt_r0    = p_ctx->pos_r;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
}

/* One tick in Line Following.  true: it set the wheels (brake, line up,
   back up); false: Line Following drives (RT_SLOW only slows it). */
static bool
rt_drive (motion_ctx_t *p_ctx)
{
    bool     b_bar = line_state_barcode(p_ctx->line_word);
    uint32_t req   = barcode_retry_req();
    int32_t  moved = motion_ds_dist_mm(p_ctx->pos_l - p_ctx->rt_l0,
                                       p_ctx->pos_r - p_ctx->rt_r0);
    bool     b_wheels = false;

    if (b_bar && !p_ctx->b_rt_bar)
    {
        p_ctx->rt_read_l = p_ctx->pos_l; /* a read begins here */
        p_ctx->rt_read_r = p_ctx->pos_r;
        if (RT_SLOW == p_ctx->rt_phase)
        {
            p_ctx->b_rt_seen = true;
        }
    }
    if (!b_bar && p_ctx->b_rt_bar && (RT_SLOW == p_ctx->rt_phase)
        && p_ctx->b_rt_seen)
    {
        rt_next(p_ctx, RT_NONE); /* read again: back to normal speed */
    }
    p_ctx->b_rt_bar = b_bar;

    if (req != p_ctx->rt_req_seen)
    {
        p_ctx->rt_req_seen = req;
        if ((RT_NONE == p_ctx->rt_phase) && !p_ctx->b_rt_used)
        {
            int32_t back = motion_ds_dist_mm(p_ctx->pos_l - p_ctx->rt_read_l,
                                             p_ctx->pos_r - p_ctx->rt_read_r)
                           + MOTION_RT_MARGIN_MM;
            motion_report_t rep;

            if (back < MOTION_RT_MIN_MM)
            {
                back = MOTION_RT_MIN_MM;
            }
            if (back > MOTION_RT_MAX_MM)
            {
                back = MOTION_RT_MAX_MM;
            }
            p_ctx->rt_back_mm = back;
            p_ctx->b_rt_used  = true;
            p_ctx->b_rt_seen  = false;
            rt_next(p_ctx, RT_BRAKE);
            moved = 0;
            report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BYP);
            rep.val_l  = (int32_t)BYP_EV_RETRY;
            rep.val_r  = back;
            rep.rate_l = p_ctx->hd_err_deg;
            rep.rate_r = p_ctx->b_hd_ok ? 1 : 0;
            report_send(p_ctx, &rep);
        }
    }

    p_ctx->rt_ticks++;
    switch (p_ctx->rt_phase)
    {
        case RT_BRAKE:
            p_ctx->cmd_l = 0;
            p_ctx->cmd_r = 0;
            if (p_ctx->rt_ticks >= 20u)
            {
                rt_next(p_ctx, RT_ALIGN);
            }
            b_wheels = true;
            break;
        case RT_ALIGN:
            /* straighten to the line's heading (IMU); none: as it is */
            if (!p_ctx->b_hd_ok || (motion_abs(p_ctx->hd_err_deg) <= 3)
                || (p_ctx->rt_ticks >= 200u))
            {
                rt_next(p_ctx, RT_BACK);
                p_ctx->cmd_l = 0;
                p_ctx->cmd_r = 0;
            }
            else
            {
                int32_t dir = (p_ctx->hd_err_deg > 0) ? -1 : 1; /* + = left */

                p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                                          -dir * MOTION_BYPASS_TURN_SPEED,
                                          p_ctx->d_l);
                p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                                          dir * MOTION_BYPASS_TURN_SPEED,
                                          p_ctx->d_r);
            }
            b_wheels = true;
            break;
        case RT_BACK:
            if ((moved <= -p_ctx->rt_back_mm) || (p_ctx->rt_ticks >= 1500u))
            {
                rt_next(p_ctx, RT_SLOW);
                lf_reset(p_ctx);
                p_ctx->cmd_l = 0;
                p_ctx->cmd_r = 0;
            }
            else
            {
                /* straight back; with a heading, hold the line's */
                int32_t c = 0;

                if (p_ctx->b_hd_ok)
                {
                    c = p_ctx->hd_err_deg / 3;
                    if (c > 2)
                    {
                        c = 2;
                    }
                    if (c < -2)
                    {
                        c = -2;
                    }
                }
                p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                                          -MOTION_RT_BACK_SPEED + c, p_ctx->d_l);
                p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                                          -MOTION_RT_BACK_SPEED - c, p_ctx->d_r);
            }
            b_wheels = true;
            break;
        case RT_SLOW:
            if (moved >= (p_ctx->rt_back_mm + 200))
            {
                rt_next(p_ctx, RT_NONE); /* went through it: done */
            }
            break;
        default:
            if (p_ctx->b_rt_used && !b_bar && (moved >= MOTION_RT_FORGET_MM))
            {
                p_ctx->b_rt_used = false; /* the next barcode may retry */
            }
            break;
    }
    return b_wheels;
}

#if MOTION_LF_STRADDLE
/* Straddle Line Following (MOTION_LF_STRADDLE, motion_cfg.h), after the
   other team's b3_line_step (subsystem3, used with permission): the two
   sensors sit either side of the line, both white = on it.  A black side
   = the line has slid under that sensor: a nudge back (self-tuned: a touch
   on the other side soon after shrinks it, the same side again grows it),
   and a side that stays black teaches a curve memory, so a bend is driven
   as one smooth arc; on white the memory fades to straight.  Tile gaps
   and cracks are white: nothing changes.  Curvatures are 1e6 / radius mm,
   + = left; speeds mm/s, turned into counts per tick with a remainder. */
static int32_t
st_abs (int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t
st_clamp (int32_t v, int32_t lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

static int32_t
st_curve_speed (int32_t base, int32_t curv)
{
    int32_t mag = st_abs(curv);

    if (base <= MOTION_ST_CURVE_MMPS)
    {
        return base;
    }
    if (mag > MOTION_ST_FULL_CURV)
    {
        mag = MOTION_ST_FULL_CURV;
    }
    return base - (((base - MOTION_ST_CURVE_MMPS) * mag) / MOTION_ST_FULL_CURV);
}

/* the inner wheel's minimum: MOTION_ST_MIN_WHEEL_MMPS, or a share of
   the speed when slower (at the 24 mm/s barcode speed a fixed 20 left
   4 mm/s to steer with: a 38 cm radius at best, in the S-curve) */
static int32_t
st_min_wheel (int32_t v)
{
    int32_t share = (v * MOTION_ST_MIN_WHEEL_PCT) / 100;

    return (share < MOTION_ST_MIN_WHEEL_MMPS) ? share : MOTION_ST_MIN_WHEEL_MMPS;
}

/* tightest curve at speed v that keeps the inner wheel at the minimum */
static int32_t
st_curve_limit (int32_t v)
{
    int32_t mw = st_min_wheel(v);

    if (v <= mw)
    {
        return 0;
    }
    return (int32_t)(((int64_t)2000000 * (v - mw))
                     / ((int64_t)v * MOTION_WHEELBASE_MM));
}

static void
st_tune_kick (motion_ctx_t *p_ctx, int8_t side)
{
    int64_t kick = p_ctx->st_kick;

    if ((0 != p_ctx->st_last_touch)
        && (p_ctx->st_touch_um < (MOTION_ST_OVERSHOOT_MM * 1000u)))
    {
        kick = (p_ctx->st_last_touch != side)
                   ? ((kick * MOTION_ST_SHRINK_PCT) / 100)
                   : ((kick * MOTION_ST_GROW_PCT) / 100);
    }
    else if (p_ctx->st_last_touch == side)
    {
        kick = (kick * MOTION_ST_GROW_PCT) / 100;
    }
    else
    {
        /* first touch, or long after the last: keep it */
    }
    if (kick < MOTION_ST_KICK_MIN_CURV)
    {
        kick = MOTION_ST_KICK_MIN_CURV;
    }
    if (kick > MOTION_ST_KICK_CURV)
    {
        kick = MOTION_ST_KICK_CURV;
    }
    p_ctx->st_kick       = (int32_t)kick;
    p_ctx->st_last_touch = side;
    p_ctx->st_touch_um   = 0u;
}

/* mm/s to counts per tick, the remainder kept for the next tick */
static int32_t
st_cpt (int32_t *p_rem, int32_t mmps)
{
    int32_t out;

    *p_rem += mmps * (MOTION_COUNTS_PER_REV * 113);
    out     = *p_rem / (355 * MOTION_WHEEL_DIA_MM * 100);
    *p_rem -= out * (355 * MOTION_WHEEL_DIA_MM * 100);
    return out;
}

static void
st_drive (motion_ctx_t *p_ctx, int32_t base)
{
    uint32_t rl    = line_state_rl(p_ctx->line_word);
    bool     l_blk = (0u != (rl & LINE_RL_LEFT_DEV));
    bool     r_blk = (0u != (rl & LINE_RL_RIGHT_DEV));
    int8_t   error = 0; /* -1 = left black = steer left */
    int32_t  fwd   = p_ctx->d_l + p_ctx->d_r;
    int32_t  ds_um = (fwd > 0) ? ST_UM_2C(fwd) : 0;
    int32_t  target;
    int32_t  limit;
    int32_t  v;
    int32_t  corr;
    bool     bend_bb;
    bool     escape;

    /* Over a barcode (line_state's barcode bit, or the slow re-read): no
       line steering, the car holds the heading it had when the read began
       (IMU), so the bars pass straight under the barcode sensor and a bar
       under a line sensor is no touch.  The curve memory waits. */
    /* MOTION_ST_BC_HOLD 0 (default): steering stays live over a barcode,
       as the other team's.  The barcode sensor also takes a curve's line
       for a barcode (car 13:50:47..13:51:01, S-curve: three "reads", all
       rejected), and holding the heading there drove the car straight off
       the curve, twice cutting short a recovery.  On a straight nothing
       touches the line, so live steering is straight driving anyway. */
    if (MOTION_ST_BC_HOLD
        && (line_state_barcode(p_ctx->line_word)
            || (MOTION_BC_RETRY && (RT_SLOW == p_ctx->rt_phase))))
    {
        int32_t herr;

        if (!p_ctx->b_st_bc)
        {
            p_ctx->b_st_bc  = true;
            p_ctx->st_bc_h0 = (int32_t)ev_heading_deg();
        }
        herr = p_ctx->st_bc_h0 - (int32_t)ev_heading_deg(); /* + = left */
        while (herr > 180)
        {
            herr -= 360;
        }
        while (herr < -180)
        {
            herr += 360;
        }
        target = st_clamp(herr * MOTION_ST_BC_KP, MOTION_ST_BC_MAX);
        v      = base;
        {
            int32_t step = (MOTION_ST_SLEW_CURV * ds_um) / 1000;

            target = st_clamp(target - p_ctx->st_steer, step) + p_ctx->st_steer;
        }
        p_ctx->st_steer = target;
        corr = (int32_t)(((int64_t)p_ctx->st_steer * v * MOTION_WHEELBASE_MM)
                         / 2000000);
        corr = st_clamp(corr, (v > st_min_wheel(v)) ? (v - st_min_wheel(v)) : 0);
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                                  st_cpt(&p_ctx->st_rem_l, v - corr), p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                                  st_cpt(&p_ctx->st_rem_r, v + corr), p_ctx->d_r);
        return;
    }
    p_ctx->b_st_bc = false;
    /* the turn the wheels make, averaged over ~8 ticks (+ = left) */
    p_ctx->st_dd16 += (((p_ctx->d_r - p_ctx->d_l) * 16) - p_ctx->st_dd16) / 8;
    p_ctx->st_ds16 += (((p_ctx->d_r + p_ctx->d_l) * 16) - p_ctx->st_ds16) / 8;
    if (l_blk && !r_blk)
    {
        error = -1;
    }
    else if (r_blk && !l_blk)
    {
        error = 1;
    }
    else
    {
        /* both white (on it) or both black (a crossing): straight */
    }
    /* The line crossing the whole pair (car 13:17:21: a hard left learnt
       in a curve, the line went from the left sensor to the right one
       12 mm later and on out past it; both white then read "on it" and
       the car drove off).  A touch that begins within MOTION_ST_TRAVERSE_MM
       of the other side's ending is the line sweeping across; when it ends
       too, the line is outside that sensor: st_out, steer back hard until
       any touch. */
    if (l_blk || r_blk)
    {
        if ((0 != error) && (error != p_ctx->st_side))
        {
            p_ctx->b_st_trav = (p_ctx->st_end_side == -error)
                               && (p_ctx->st_end_um < (MOTION_ST_TRAVERSE_MM * 1000u));
        }
        if (0 != p_ctx->st_out)
        {
            p_ctx->b_st_recov = true; /* this touch ends a recovery pivot */
        }
        p_ctx->b_st_after_recov = false;
        p_ctx->st_out = 0;
    }
    else
    {
        if (0 != p_ctx->st_side)
        {
            /* a single touch has just ended */
            if (p_ctx->b_st_trav)
            {
                p_ctx->st_out = p_ctx->st_side;
            }
            /* Which way did the line go?  The car turning AWAY from the
               sensor that just lost it swings that sensor off past the
               line: the line is now outside it (car 13:17:22, 13:50:56).
               Turning toward it, the line came back in between. */
            if (p_ctx->st_ds16 > 0)
            {
                int32_t k = (int32_t)(((int64_t)2000000 * p_ctx->st_dd16)
                                      / ((int64_t)MOTION_WHEELBASE_MM
                                         * p_ctx->st_ds16)); /* + = left */
                int32_t ts = (k >= MOTION_ST_EXIT_CURV) ? -1
                             : ((k <= -MOTION_ST_EXIT_CURV) ? 1 : 0);

                if ((0 != ts) && (ts == -p_ctx->st_side))
                {
                    p_ctx->st_out = p_ctx->st_side;
                }
            }
            if (0 != p_ctx->st_out)
            {
                p_ctx->st_out_um = 0u;
                p_ctx->st_mem    = 0; /* that curve was wrong */
            }
            /* after a recovery pivot the car is unsure: no touch again
               within MOTION_ST_RECOV_LOST_MM is Line Search (car 15:46:03:
               found the line, drove on 7 s at one heading without it) */
            p_ctx->b_st_after_recov = p_ctx->b_st_recov;
            p_ctx->b_st_recov       = false;
            /* on the outside of a learnt curve (the side against it) */
            p_ctx->b_st_outer  = (st_abs(p_ctx->st_mem) >= MOTION_ST_FULL_CURV)
                                 && (((int64_t)p_ctx->st_side * p_ctx->st_mem) > 0);
            p_ctx->st_end_side = p_ctx->st_side;
            p_ctx->st_end_um   = 0u;
            p_ctx->b_st_trav   = false;
        }
        else if (p_ctx->st_end_um < 100000000u)
        {
            p_ctx->st_end_um += (uint32_t)ds_um;
            /* the outside sensor touched, then nothing for
               MOTION_ST_OUTER_MM in the curve: the line slipped out past
               it (on a straight, or after an inside touch, nothing) */
            if (p_ctx->b_st_outer && (0 == p_ctx->st_out)
                && (p_ctx->st_end_um >= (MOTION_ST_OUTER_MM * 1000u)))
            {
                p_ctx->st_out     = p_ctx->st_end_side;
                p_ctx->b_st_outer = false;
                p_ctx->st_out_um  = 0u;
                p_ctx->st_mem     = 0;
            }
        }
        else
        {
            /* long ago */
        }
    }
    /* both black in a learnt bend is the bend's tape seen at a steep
       angle, not a crossing: keep the curve */
    bend_bb = l_blk && r_blk
              && (st_abs(p_ctx->st_mem) >= MOTION_ST_FULL_CURV)
              && (((int64_t)p_ctx->st_side * p_ctx->st_mem) <= 0);
    if (!bend_bb && (error != p_ctx->st_side))
    {
        if ((0 == error) && (((int64_t)p_ctx->st_side * p_ctx->st_mem) > 0)
            && (p_ctx->st_black_um >= (MOTION_ST_CURVE_END_MM * 1000u)))
        {
            /* the side against the learnt curve stayed black: the curve
               has ended, keep the steering it left */
            p_ctx->st_mem = p_ctx->st_steer;
        }
        p_ctx->st_side     = error;
        p_ctx->st_black_um = 0u;
        if (0 != error)
        {
            st_tune_kick(p_ctx, error);
        }
    }
    if (p_ctx->st_touch_um < (MOTION_ST_OVERSHOOT_MM * 1000u))
    {
        p_ctx->st_touch_um += (uint32_t)ds_um;
    }
    if (!(l_blk && r_blk))
    {
        p_ctx->st_bb_um = 0u;
    }
    else if (p_ctx->st_bb_um < (MOTION_ST_BB_CROSS_MM * 1000u))
    {
        p_ctx->st_bb_um += (uint32_t)ds_um;
    }
    else
    {
        /* long enough */
    }

    if ((0 != p_ctx->st_out) && !l_blk && !r_blk)
    {
        /* back toward the side the line went out of: pivot on the inner
           wheel (stopped), the outer at 2 x the curve speed, until a sensor
           finds the line; line_inputs gives up to Line Search after
           MOTION_ST_OUT_MAX_MM */
        int32_t vp = (base < MOTION_ST_CURVE_MMPS) ? base : MOTION_ST_CURVE_MMPS;

        if (p_ctx->st_out_um < 100000000u)
        {
            p_ctx->st_out_um += (uint32_t)ds_um;
        }
        p_ctx->st_steer = -p_ctx->st_out * MOTION_ST_KICK_CURV;
        corr            = -p_ctx->st_out * vp;
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                                  st_cpt(&p_ctx->st_rem_l, vp - corr), p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                                  st_cpt(&p_ctx->st_rem_r, vp + corr), p_ctx->d_r);
        return;
    }
    else if (bend_bb)
    {
        target             = p_ctx->st_mem;
        p_ctx->st_black_um = 0u;
        if (p_ctx->st_bb_um >= (MOTION_ST_BB_CROSS_MM * 1000u))
        {
            target += (p_ctx->st_mem > 0) ? p_ctx->st_kick : -p_ctx->st_kick;
        }
    }
    else if (0 != error)
    {
        int64_t side_pct = ((error < 0) && (p_ctx->st_mem >= MOTION_ST_FULL_CURV))
                               ? (int64_t)MOTION_ST_LEFT_PCT : 100;
        int64_t base_rate;
        int64_t rate;
        int64_t kick = p_ctx->st_kick;

        p_ctx->st_black_um += (uint32_t)ds_um;
        if (((int64_t)error * p_ctx->st_mem) > 0)
        {
            /* black against the learnt curve: forget it */
            p_ctx->st_mem -= (int32_t)(((int64_t)p_ctx->st_mem * ds_um)
                                       / ((int64_t)MOTION_ST_FORGET_MM * 1000));
        }
        base_rate = ((((int64_t)error * p_ctx->st_mem) < 0)
                     && (st_abs(p_ctx->st_mem) >= MOTION_ST_TIGHT_FROM_CURV))
                        ? MOTION_ST_LEARN_TIGHT_CURV : MOTION_ST_LEARN_CURV;
        rate = (base_rate * side_pct) / 100;
        p_ctx->st_mem -= (int32_t)(((int64_t)error * rate * ds_um
                                    * (((int64_t)MOTION_ST_ESCAPE_MM * 1000)
                                       + (int64_t)p_ctx->st_black_um))
                                   / ((int64_t)10000 * MOTION_ST_ESCAPE_MM * 1000));
        if (p_ctx->st_black_um >= (MOTION_ST_KICK_RAMP_MM * 1000u))
        {
            kick = MOTION_ST_KICK_CURV;
        }
        else
        {
            kick += ((MOTION_ST_KICK_CURV - kick) * (int64_t)p_ctx->st_black_um)
                    / ((int64_t)MOTION_ST_KICK_RAMP_MM * 1000);
        }
        kick   = (kick * side_pct) / 100;
        target = p_ctx->st_mem - (error * (int32_t)kick);
    }
    else
    {
        /* fades to straight, rounded away from 0 so it gets there */
        int64_t fade = ((int64_t)p_ctx->st_mem * ds_um)
                       / ((int64_t)MOTION_ST_FORGET_MM * 1000);

        if ((0 == fade) && (0 != p_ctx->st_mem) && (ds_um > 0))
        {
            fade = (p_ctx->st_mem > 0) ? 1 : -1;
        }
        p_ctx->st_mem -= (int32_t)fade;
        target = p_ctx->st_mem;
    }

    v             = st_curve_speed(base, p_ctx->st_mem);
    limit         = st_curve_limit(v);
    p_ctx->st_mem = st_clamp(p_ctx->st_mem, limit);
    target        = st_clamp(target, limit);
    {
        int32_t step = (MOTION_ST_SLEW_CURV * ds_um) / 1000;

        if (target > (p_ctx->st_steer + step))
        {
            target = p_ctx->st_steer + step;
        }
        if (target < (p_ctx->st_steer - step))
        {
            target = p_ctx->st_steer - step;
        }
    }
    p_ctx->st_steer = st_clamp(target, limit);

    corr   = (int32_t)(((int64_t)p_ctx->st_steer * v * MOTION_WHEELBASE_MM)
                       / 2000000);
    escape = (0 != error)
             && (p_ctx->st_black_um >= (MOTION_ST_ESCAPE_MM * 1000u))
             && (st_abs(p_ctx->st_mem) >= limit) && (v > 0);
    if (escape)
    {
        corr = -error * v; /* hairpin: stop the inner wheel, pivot on it */
    }
    else if (corr > (v - st_min_wheel(v)))
    {
        corr = (v > st_min_wheel(v)) ? (v - st_min_wheel(v)) : 0;
    }
    else if (corr < -(v - st_min_wheel(v)))
    {
        corr = (v > st_min_wheel(v)) ? -(v - st_min_wheel(v)) : 0;
    }
    else
    {
        /* inside the limits */
    }
    p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                              st_cpt(&p_ctx->st_rem_l, v - corr), p_ctx->d_l);
    p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                              st_cpt(&p_ctx->st_rem_r, v + corr), p_ctx->d_r);
}
#endif

/* Realign after a search (MOTION_LF_REALIGN, motion_cfg.h): found the line
   again more than MOTION_RA_MIN_DEG off the heading it had when the line
   went white -> onto it (axle over the line), pivot back to that heading,
   a short stop, then Line Following.  The same as the bypass's way back. */
#define RA_NONE   (0u)
#define RA_ONTO   (1u)
#define RA_ALIGN  (2u)
#define RA_SETTLE (3u)

static void
ra_next (motion_ctx_t *p_ctx, uint8_t ph)
{
    p_ctx->ra_phase = ph;
    p_ctx->ra_ticks = 0u;
    p_ctx->ra_l0    = p_ctx->pos_l;
    p_ctx->ra_r0    = p_ctx->pos_r;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
}

/* true: it set the wheels this tick. */
static bool
ra_drive (motion_ctx_t *p_ctx)
{
    int32_t err = p_ctx->ra_h_pre - (int32_t)ev_heading_deg(); /* + = turn left */
    int32_t moved;

    if (p_ctx->b_ra_pending)
    {
        p_ctx->b_ra_pending = false;
        if (motion_abs(err) >= MOTION_RA_MIN_DEG)
        {
            motion_report_t rep;

            ra_next(p_ctx, RA_ONTO);
            report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BYP);
            rep.val_l  = (int32_t)(BYP_PH_DONE + 6u);
            rep.val_r  = err;
            rep.rate_l = 0;
            rep.rate_r = 1;
            report_send(p_ctx, &rep);
        }
    }
    if (RA_NONE == p_ctx->ra_phase)
    {
        return false;
    }
    p_ctx->ra_ticks++;
    moved = motion_ds_dist_mm(p_ctx->pos_l - p_ctx->ra_l0,
                              p_ctx->pos_r - p_ctx->ra_r0);
    if (RA_ONTO == p_ctx->ra_phase)
    {
        if ((moved >= MOTION_SM_ONTO_MM) || (p_ctx->ra_ticks >= 300u))
        {
            ra_next(p_ctx, RA_ALIGN);
        }
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, MOTION_LF_BASE, p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, MOTION_LF_BASE, p_ctx->d_r);
    }
    else if (RA_ALIGN == p_ctx->ra_phase)
    {
        if ((motion_abs(err) <= MOTION_HD_LEAD_DEG) || (p_ctx->ra_ticks >= 300u))
        {
            ra_next(p_ctx, RA_SETTLE);
            p_ctx->cmd_l = 0;
            p_ctx->cmd_r = 0;
        }
        else
        {
            int32_t dir = (err > 0) ? 1 : -1; /* + = left */

            p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l,
                                      -dir * MOTION_BYPASS_TURN_SPEED, p_ctx->d_l);
            p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r,
                                      dir * MOTION_BYPASS_TURN_SPEED, p_ctx->d_r);
        }
    }
    else
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
        if (p_ctx->ra_ticks >= 15u)
        {
            ra_next(p_ctx, RA_NONE);
            lf_reset(p_ctx);
        }
    }
    return true;
}

/* What the wheels do this tick in the current state. */
static void
man_drive (motion_ctx_t *p_ctx, motion_state_t state)
{
    int32_t corr;

    if (MOTION_ST_REVERSING == state)
    {
        p_ctx->cmd_l =
            pid_capped(p_ctx, &p_ctx->pid_l, -MOTION_REV_TARGET, p_ctx->d_l);
        p_ctx->cmd_r =
            pid_capped(p_ctx, &p_ctx->pid_r, -MOTION_REV_TARGET, p_ctx->d_r);
    }
    else if (MOTION_ST_LINE_FOLLOW == state)
    {
#if MOTION_LF_REALIGN
        if (ra_drive(p_ctx))
        {
            return; /* onto the line and back to its heading */
        }
#endif
#if MOTION_BC_RETRY
        if (rt_drive(p_ctx))
        {
            return; /* braking, lining up or backing up for a re-read */
        }
#endif
#if MOTION_LF_STRADDLE
        {
            int32_t st_base = MOTION_ST_BASE_MMPS;

            if (line_state_barcode(p_ctx->line_word)
                || (MOTION_BC_RETRY && (RT_SLOW == p_ctx->rt_phase)))
            {
                st_base = MOTION_ST_READ_MMPS; /* slower over a barcode */
            }
            st_drive(p_ctx, st_base);
            return;
        }
#endif
        /* correction > 0: turn left (slow the left wheel, speed up the
           right).  With LINE_STEER_INVERT that is the left sensor darker.
           The remainder carries to the next tick, so a correction below
           one count still steers on average (S4: an error below 13 gave
           no correction at all). */
#if MOTION_LF_GAP_MM > 0
        if ((LINE_RL_CENTRED == line_state_rl(p_ctx->line_word))
            && !line_state_barcode(p_ctx->line_word))
        {
            if (0 == p_ctx->lf_gap2c)
            {
                p_ctx->lf_gap_h0   = (int32_t)ev_heading_deg();
                p_ctx->b_gap_start = !p_ctx->b_lf_edge;
            }
            p_ctx->lf_gap2c += ((p_ctx->d_l + p_ctx->d_r) > 0)
                                   ? (p_ctx->d_l + p_ctx->d_r) : 1;
        }
        else
        {
#if MOTION_LF_REALIGN
            /* the line back after a gap the car swerved in: line up with
               the heading it had before the gap (not after an edge slip:
               in a curve the line has turned, and Line Following follows) */
            if ((p_ctx->lf_gap2c > 0) && p_ctx->b_gap_start
                && (motion_abs(p_ctx->lf_gap_h0 - (int32_t)ev_heading_deg())
                    >= MOTION_RA_MIN_DEG))
            {
                p_ctx->ra_h_pre     = p_ctx->lf_gap_h0;
                p_ctx->b_ra_pending = true;
            }
#endif
            p_ctx->lf_gap2c = 0;
        }
#endif
        p_ctx->lf_rem += lf_correction(p_ctx, lf_error(p_ctx),
                                       MOTION_LF_HOLD_ON_BARCODE
                                       && line_state_barcode(p_ctx->line_word));
        corr = p_ctx->lf_rem / MOTION_LF_SCALE;
        p_ctx->lf_rem -= corr * MOTION_LF_SCALE;
        {
            /* TEST IMAGE (20 Nov): slower over a barcode (line_state's
               barcode bit, LINE_BARCODE_HOLD 1), so the slow analog
               barcode sensor sees thin bars darker.  Widths are measured
               in mm (Boon's reader), so the change of speed does not
               distort them.  0: off. */
            int32_t base = MOTION_LF_BASE;
            int32_t c;
#if MOTION_LF_BASE_CURVE > 0
            /* Curve speed (21 Nov, car 20:20: in a tight right curve the
               line sat on one sensor's edge for 2 s, error ~-60, and the
               correction that gives at base 6 turned too wide; the line
               slid off and was lost).  Line on one sensor's edge with a
               clear error, or lost from an edge: MOTION_LF_BASE_CURVE for
               MOTION_LF_CURVE_TICKS, the correction kept whole, so the same
               error turns much tighter.  Straights stay at the base. */
            {
                uint32_t rl = line_state_rl(p_ctx->line_word);
                int32_t  e  = line_state_error(p_ctx->line_word);

                /* only a line that stays at one edge MOTION_LF_CURVE_RUN ticks
                   is a curve (car 20:57: at base 8 every wobble touched an
                   edge, curve mode held a hard turn over each white patch:
                   a pivot every time, and the line lost) */
                if (((LINE_RL_LEFT_DEV == rl) || (LINE_RL_RIGHT_DEV == rl))
                    && ((e >= MOTION_LF_CURVE_ERR) || (e <= -MOTION_LF_CURVE_ERR)))
                {
                    if (p_ctx->lf_edge_run < 1000u)
                    {
                        p_ctx->lf_edge_run++;
                    }
                }
                else if (LINE_RL_CENTRED != rl)
                {
                    p_ctx->lf_edge_run = 0u;
                }
                else
                {
                    /* white: keep the count (a gap inside a curve) */
                }
                if (p_ctx->lf_edge_run >= MOTION_LF_CURVE_RUN)
                {
                    p_ctx->lf_curve_ticks = MOTION_LF_CURVE_TICKS;
                }
                else if ((LINE_RL_CENTRED == rl) && p_ctx->b_lf_edge
                         && (p_ctx->lf_curve_ticks > 0u))
                {
                    p_ctx->lf_curve_ticks = MOTION_LF_CURVE_TICKS;
                }
                else if (p_ctx->lf_curve_ticks > 0u)
                {
                    p_ctx->lf_curve_ticks--;
                }
                else
                {
                    /* on a straight */
                }
            }
            c = corr;
            if (p_ctx->lf_curve_ticks > 0u)
            {
                /* and steer harder (car 20:45: at base 4 the error ~-60
                   still turned ~15 cm radius, wider than the S-curve) */
                base = MOTION_LF_BASE_CURVE;
                c    = (corr * MOTION_LF_CURVE_BOOST_X10) / 10;
                if (c > (base + 2))
                {
                    c = base + 2;
                }
                if (c < -(base + 2))
                {
                    c = -(base + 2);
                }
            }
#endif
#if defined(MOTION_LF_BASE_READ) && (MOTION_LF_BASE_READ > 0)
            if (line_state_barcode(p_ctx->line_word)
                || (MOTION_BC_RETRY && (RT_SLOW == p_ctx->rt_phase)))
            {
                base = MOTION_LF_BASE_READ;
                /* the gain and cap are tuned for MOTION_LF_BASE: at the
                   read base scale the correction down with the speed, or
                   the same error turns twice as tight and the inner wheel
                   reverses (car, 20 Nov: wobble over barcodes) */
                c = (corr * base) / MOTION_LF_BASE;
            }
#endif

            p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, base - c,
                                           p_ctx->d_l);
            p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, base + c,
                                           p_ctx->d_r);
        }
    }
    else if (MOTION_ST_LINE_SEARCH == state)
    {
        search_drive(p_ctx);
    }
    else if (MOTION_ST_TURNING == state)
    {
        turn_drive(p_ctx);
    }
    else if (MOTION_ST_BYPASS == state)
    {
        bypass_drive(p_ctx);
    }
    else
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }
}

/* Faults that send the statechart to Motion Fault (FR-30); a lost tick or
   a failed set-up are handled elsewhere. */
#define SM_FAULTS                                                              \
    (MOTION_FAULT_STALL_L | MOTION_FAULT_STALL_R | MOTION_FAULT_ENC_L          \
     | MOTION_FAULT_ENC_R)

/* Command intake, once per tick before the statechart step: avoidance
   first, then navigation (design Section 4).  Both receives poll, so a
   tick takes at most one command from each buffer and a second one waits
   a tick.  Nothing waits on CMD (every tick looks), so it is only cleared,
   before the receives, so a command placed after it is still read.

   A JUNCTION goes to the statechart with its action kept for turn_drive();
   outside Line Following the statechart refuses it (CMD_REJECTED).  Here
   it is only "received": tick_statechart() reports the outcome.
   A BYPASS keeps its side and offset for bypass_drive(). */
static void
report_cmd (motion_ctx_t *p_ctx, int32_t src, uint32_t raw, int32_t res)
{
    motion_report_t rep;

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_CMD);
    rep.val_l  = src;
    rep.val_r  = (int32_t)raw;
    rep.rate_l = res;
    report_send(p_ctx, &rep);
}

static void
take_commands (motion_ctx_t *p_ctx, motion_sm_in_t *p_in)
{
    uint16_t word = 0u;
    UB       msg[CMD_MSG_BYTES];
    uint8_t  op;
    uint8_t  side;
    uint16_t mm;
    int32_t  res;

    (void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_CMD);

    if ((MOTION_SM_AVOID_NONE == (motion_sm_avoid_t)p_ctx->avoid_cmd)
        && ((INT)CMD_MSG_BYTES == tk_rcv_mbf(g_avoid_mbfid, &word, TMO_POL)))
    {
        res = CMD_RES_INVALID;
        if (motion_avoid_cmd_unpack(word, &op, &side, &mm)
            && (MOTION_AVOID_OP_LOOK == op))
        {
            /* A probe's answer (D14), not a manoeuvre: kept only while
               a probe is waiting for it. */
            res = CMD_RES_TAKEN;
            if (LOOK_ASKED == p_ctx->look_state)
            {
                p_ctx->look_mm    = mm;
                p_ctx->look_state = LOOK_ANSWERED;
            }
            else if (p_ctx->b_sm_watch)
            {
                p_ctx->sm_watch_mm = mm;
                p_ctx->b_sm_new    = true;
            }
        }
        else if (motion_avoid_cmd_unpack(word, &op, &side, &mm))
        {
            res              = CMD_RES_TAKEN;
            p_ctx->avoid_mm   = mm;
            p_ctx->avoid_side = side;
            p_ctx->avoid_cmd = (uint8_t)((MOTION_AVOID_OP_REVERSE == op)
                                             ? MOTION_SM_AVOID_REVERSE
                                             : MOTION_SM_AVOID_BYPASS);
        }
        report_cmd(p_ctx, CMD_SRC_AVOID, word, res);
    }

    if ((INT)CMD_MSG_BYTES == tk_rcv_mbf(g_nav_mbfid, msg, TMO_POL))
    {
        res = CMD_RES_TAKEN;
        if ((NAV_CMD_FAULT_CLEAR == msg[0]) && (FAULT_CLEAR_IMU == msg[1]))
        {
            p_in->nav = MOTION_SM_NAV_CLEAR_IMU;
        }
        else if ((NAV_CMD_FAULT_CLEAR == msg[0])
                 && (FAULT_CLEAR_MOTION == msg[1]))
        {
            p_in->nav = MOTION_SM_NAV_CLEAR_MOTION;
        }
        else if ((NAV_CMD_JUNCTION == msg[0])
                 && (msg[1] <= (UB)BARCODE_ACT_UTURN))
        {
            res             = CMD_RES_RECEIVED;
            p_in->nav       = MOTION_SM_NAV_JUNCTION;
            p_ctx->turn_req = msg[1]; /* the turn's, if the statechart
                                         takes it (tick_statechart) */
        }
        else
        {
            res = CMD_RES_INVALID;
        }
        report_cmd(p_ctx, CMD_SRC_NAV, ((uint32_t)msg[0] << 8) | msg[1], res);
    }
}

/* fault_flg bits (S.faults) for motion's FR-30 faults. */
static UINT
fault_flg_bits (uint8_t faults)
{
    UINT bits = 0u;

    if (0u != ((uint32_t)faults & (MOTION_FAULT_STALL_L | MOTION_FAULT_STALL_R)))
    {
        bits |= FAULT_MOTOR_STALL;
    }
    if (0u != ((uint32_t)faults & (MOTION_FAULT_ENC_L | MOTION_FAULT_ENC_R)))
    {
        bits |= FAULT_ENCODER;
    }

    return bits;
}

/* A refused E motion is tried again this many ticks later (100 ms), then
   once a second while tele_mbf still refuses it. */
#define TELE_RETRY_TICKS (10u)

/* E motion (MQTT_SCHEMA.md 5, k = 0), on every statechart change and once
   a second while a wheel moves (tick_telemetry): the state, each wheel's
   speed over the distance_state window (50 ms, to the nearest cm/s; one
   tick's counts would truncate 6 counts to 4 cm/s instead of ~4.8), the
   distance travelled in this mission, the turn rate and the motion faults
   now.  The distance counts forward and backward travel alike, so it
   never goes down: comms_task takes S.dst as a modulo-32768 difference,
   and a reverse would otherwise come out as about 32 m, which S.dst clamps
   to 255 cm.  comms_tele_send() polls, so the tick never waits; a send
   tele_mbf refuses is counted (timing line, NFR-10) and tried again with
   the values then (TELE_RETRY_TICKS, then once a second). */
static void
tele_motion (motion_ctx_t *p_ctx)
{
    UB      body[COMM_W_INTAKE_SIZE_MOTION];
    int32_t speed_l;
    int32_t speed_r;
    int32_t mission_mm; /* the packer keeps 15 bits, modulo 32768 */

    motion_ds_win_speed_cms(&p_ctx->ds_win, &speed_l, &speed_r);
    mission_mm = motion_ds_dist_mm((int32_t)p_ctx->mission_counts, 0);
    if (E_OK
        == comms_tele_send(
            body, (INT)comm_w_pack_motion(
                      body, (uint32_t)motion_sm_state(&p_ctx->sm), speed_l,
                      speed_r, (uint32_t)mission_mm, p_ctx->ds_turn,
                      (uint32_t)fault_flg_bits(p_ctx->faults), 0u)))
    {
        p_ctx->tele_refused = 0u;
        p_ctx->tele_pos_l   = p_ctx->pos_l;
        p_ctx->tele_pos_r   = p_ctx->pos_r;
    }
    else
    {
        if (p_ctx->tele_refused < UINT8_MAX)
        {
            p_ctx->tele_refused++;
        }
        p_ctx->tele_dropped++;
    }
    p_ctx->tele_ticks = 0u;
}

/* One statechart step per tick, after the stall check.  The mission state
   comes from nav_ctrl_task (one aligned load), commands from
   take_commands().  Every driving state is built: line following, the
   line search, the turn, the reverse and the bypass.
   distance_state is never reset: imu_task works from differences, and a
   jump would read as a fake movement.  The distance motion reports in
   telemetry (mission_counts) starts again at a new mission (Section 4:
   Ready -> Running; Paused -> Running is a resume and keeps it).  nav
   enters Running only from those two, so any other way in (Finished,
   Fault or Calibrating seen last) is a new mission too, even when Ready
   lasted less than a tick.  A new mission also forgets the side the line
   was last seen on (line_dir), so its first search does not sweep the
   last mission's side first.  Paused -> Finished -> Ready -> Running inside
   one tick (three MQTT commands in one comms window) still reads as a
   resume; catching it would need a mission number from nav. */
static void
tick_statechart (motion_ctx_t *p_ctx)
{
    motion_sm_in_t  in      = { 0 };
    motion_sm_out_t out     = { 0u, false, false, false };
    motion_state_t  before  = motion_sm_state(&p_ctx->sm);
    mission_state_t mission = nav_mission_state();
    motion_report_t rep;
    motion_report_t fault_rep;
    bool            b_fault_in;

    take_commands(p_ctx, &in);
    if ((MS_RUNNING == mission)
        && (MS_RUNNING != (mission_state_t)p_ctx->mission_prev)
        && (MS_PAUSED != (mission_state_t)p_ctx->mission_prev))
    {
        p_ctx->mission_counts = 0u; /* a new mission */
        p_ctx->line_dir       = 0;  /* no side seen in it yet (L9) */
        p_ctx->st_nt_um       = 0u; /* straddle: placed on the line */
    }
    p_ctx->mission_prev = (uint8_t)mission;
    in.b_running        = (MS_RUNNING == mission);
    in.b_stop_scan   = (0u != (p_ctx->stop_bits & MOTION_EVT_STOP_SCAN));
    in.b_stop_imu    = (0u != (p_ctx->stop_bits & MOTION_EVT_STOP_IMU));
    in.b_fault       = (0u != ((uint32_t)p_ctx->faults & SM_FAULTS));
    in.avoid         = (motion_sm_avoid_t)p_ctx->avoid_cmd;
    in.b_done        = man_done(p_ctx, before);
    p_ctx->avoid_cmd = (uint8_t)MOTION_SM_AVOID_NONE; /* taken */
    line_inputs(p_ctx, &in);

    motion_sm_step(&p_ctx->sm, &in, &out);
#if MOTION_BC_RETRY
    g_state_word = ((p_ctx->rt_phase >= RT_BRAKE) && (p_ctx->rt_phase <= RT_BACK))
                       ? (uint32_t)MOTION_ST_REVERSING /* gate the reader */
                       : (uint32_t)motion_sm_state(&p_ctx->sm); /* one store */
#else
    g_state_word = (uint32_t)motion_sm_state(&p_ctx->sm); /* one store */
#endif

    /* FAULT_CLEAR(MOTION) took it out of Motion Fault: forget the fault,
       or the next driving tick would raise it again. */
    if ((MOTION_ST_FAULT == before)
        && (MOTION_ST_FAULT != motion_sm_state(&p_ctx->sm)))
    {
        p_ctx->faults = (uint8_t)((uint32_t)p_ctx->faults & ~SM_FAULTS);
        fault_clear(FAULT_MOTOR_STALL | FAULT_ENCODER);
    }

    if (out.b_clear_stop_imu)
    {
        (void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_IMU);
    }
    /* Why it entered Motion Fault, for the console (W3).  Filled before
       the brake, which zeroes the duty, and man_start(), which resets
       the stall windows; sent with the state line below. */
    b_fault_in = out.b_changed
                 && (MOTION_ST_FAULT == motion_sm_state(&p_ctx->sm));
    if (b_fault_in)
    {
        report_fill(p_ctx, &fault_rep, REP_EVENT, (uint8_t)EVT_SM_FAULT);
        fault_rep.val_l  = p_ctx->stall_l.net;
        fault_rep.val_r  = p_ctx->stall_r.net;
        fault_rep.rate_l = (int32_t)p_ctx->stall_l.idle_ticks;
        fault_rep.rate_r = (int32_t)p_ctx->stall_r.idle_ticks;
    }
    if (out.b_changed && !motion_sm_driving(motion_sm_state(&p_ctx->sm)))
    {
        motor_brake_all(); /* before any reply, e.g. MOTION_FAULT */
    }
    hd_tick(p_ctx, motion_sm_state(&p_ctx->sm));
    if (out.b_changed)
    {
        /* A new turn takes the JUNCTION's action.  A refused JUNCTION (any
           state but Line Following, a turn braked half way included)
           never changes the turn underway, and a resumed turn keeps its
           own. */
        if ((MOTION_ST_TURNING == motion_sm_state(&p_ctx->sm))
            && !out.b_resumed)
        {
            p_ctx->turn_action = p_ctx->turn_req;
        }
        man_start(p_ctx, motion_sm_state(&p_ctx->sm), out.b_resumed);
#if MOTION_LF_STRADDLE
        /* Back on the line after a bypass: check it is really there (car
           15:53: lined up 4 cm past the line, no touch for 3.5 s, ran beside
           it).  No touch within MOTION_ST_RECOV_LOST_MM -> Line Search, first
           back toward the side it came from (line_dir). */
        if (MOTION_ST_BYPASS == motion_sm_state(&p_ctx->sm))
        {
            p_ctx->b_post_bypass = true;
        }
        else if ((MOTION_ST_LINE_FOLLOW == motion_sm_state(&p_ctx->sm))
                 && p_ctx->b_post_bypass)
        {
            p_ctx->b_post_bypass    = false;
            p_ctx->b_st_after_recov = true;
            p_ctx->st_nt_um         = 0u;
        }
        else if (MOTION_ST_LINE_SEARCH != motion_sm_state(&p_ctx->sm))
        {
            p_ctx->b_post_bypass = false;
        }
        else
        {
            /* searching after the bypass: keep it for Line Following */
        }
#endif
#if MOTION_LF_REALIGN
        if ((MOTION_ST_LINE_FOLLOW == before)
            && (MOTION_ST_LINE_SEARCH == motion_sm_state(&p_ctx->sm)))
        {
            /* the heading the car had when the line went white: the line's */
            p_ctx->ra_h_pre   = (p_ctx->lf_gap2c > 0) ? p_ctx->lf_gap_h0
                                                      : (int32_t)ev_heading_deg();
            p_ctx->b_ra_valid = (p_ctx->lf_gap2c > 0) && p_ctx->b_gap_start;
        }
        else if ((MOTION_ST_LINE_SEARCH == before)
                 && (MOTION_ST_LINE_FOLLOW == motion_sm_state(&p_ctx->sm)))
        {
            p_ctx->b_ra_pending = p_ctx->b_ra_valid;
            p_ctx->b_ra_valid   = false;
        }
        else
        {
            p_ctx->b_ra_valid = false; /* a bypass or a turn: they line up */
        }
#endif
    }
    man_drive(p_ctx, motion_sm_state(&p_ctx->sm));
    if (0u != (out.replies & MOTION_SM_REP_MOTION_FAULT))
    {
        fault_set(fault_flg_bits(p_ctx->faults)); /* S.faults, before nav */
    }
    sm_send_replies(out.replies);
    if (out.b_changed)
    {
        tele_motion(p_ctx);
    }

    if (MOTION_SM_NAV_JUNCTION == in.nav)
    {
        /* What became of the JUNCTION (critic C3): accepted only if a new
           turn started this tick, else refused with CMD_REJECTED.  Both
           states go out: a STOP, fault or pause in the same tick, or a
           braked turn resuming, changes the state although the JUNCTION
           was refused (review L1). */
        report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_CMD);
        rep.val_l  = CMD_SRC_NAV;
        rep.val_r  = (int32_t)(((uint32_t)NAV_CMD_JUNCTION << 8)
                               | (uint32_t)p_ctx->turn_req);
        rep.rate_l = (out.b_changed && !out.b_resumed
                      && (MOTION_ST_TURNING == motion_sm_state(&p_ctx->sm)))
                         ? CMD_RES_ACCEPTED
                         : CMD_RES_REFUSED;
        rep.rate_r = (int32_t)((uint32_t)before
                               | ((uint32_t)motion_sm_state(&p_ctx->sm)
                                  << CMD_WENT_SHIFT));
        report_send(p_ctx, &rep);
    }
    if (b_fault_in)
    {
        report_send(p_ctx, &fault_rep);
    }
    if (out.b_changed || (0u != out.replies))
    {
        report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_STATE);
        rep.val_l  = (int32_t)motion_sm_state(&p_ctx->sm);
        rep.val_r  = (int32_t)before;
        rep.rate_l = (int32_t)out.replies;
        report_send(p_ctx, &rep);
    }
}

/* After tick_distance: add this tick's travel to the mission distance
   (both wheels' counts, forward or back), then send the E motion record
   once a second while a wheel moves (plan 1.8), or again after a refused
   send (100 ms, then once a second).  Between state changes this is what lets the laptop, and
   comms_task's S.dst (cm since the last motion change), see the distance
   grow.  Nothing while the car stands still. */
static void
tick_telemetry (motion_ctx_t *p_ctx)
{
    int32_t sum = p_ctx->pos_l + p_ctx->pos_r;

    p_ctx->mission_counts += (uint32_t)motion_abs(sum - p_ctx->odo_prev);
    p_ctx->odo_prev = sum;

    p_ctx->tele_ticks++;
    if (((0u != p_ctx->tele_refused)
         && (p_ctx->tele_ticks >= ((1u == p_ctx->tele_refused)
                                       ? TELE_RETRY_TICKS
                                       : (uint32_t)TICKS_PER_S)))
        || ((p_ctx->tele_ticks >= (uint32_t)TICKS_PER_S)
            && ((p_ctx->pos_l != p_ctx->tele_pos_l)
                || (p_ctx->pos_r != p_ctx->tele_pos_r))))
    {
        tele_motion(p_ctx);
    }
}
#endif

static void
motion_tick (motion_ctx_t *p_ctx)
{
    uint32_t t_start = now_us();
    uint32_t t_end;
    uint32_t work;

    tick_timing(p_ctx, t_start);
    tick_encoders(p_ctx);

#if MOTION_BENCH
    if (button_pressed(p_ctx))
    {
        if (PHASE_IDLE != p_ctx->phase)
        {
            bench_stop(p_ctx, EVT_ABORT);
        }
        else if (0u == p_ctx->stop_bits)
        {
            bench_start(p_ctx);
        }
        else
        {
            /* A STOP bit is set: no test starts. */
        }
    }
#endif

    stall_check(p_ctx);

    switch (p_ctx->phase)
    {
        case PHASE_STEPS:
            bench_steps(p_ctx);
            break;

        case PHASE_RAMP:
            bench_ramp(p_ctx);
            break;

        case PHASE_PID:
            bench_pid(p_ctx);
            break;

#if MOTION_BENCH && MOTION_TEST_STOP
        case PHASE_STOPTEST:
            bench_stoptest(p_ctx);
            break;
#endif

        default:
            p_ctx->cmd_l = 0;
            p_ctx->cmd_r = 0;
            break;
    }

#if !MOTION_BENCH
    tick_statechart(p_ctx); /* the statechart sets cmd_l, cmd_r last */
#endif

    /* Braked for as long as any STOP bit is set, except a reverse, which
       runs while STOP_SCAN stays set (design 6.2; never under STOP_IMU). */
    if ((0u != p_ctx->stop_bits)
        && !((MOTION_ST_REVERSING == motion_sm_state(&p_ctx->sm))
             && (0u == (p_ctx->stop_bits & MOTION_EVT_STOP_IMU))))
    {
        p_ctx->cmd_l = 0;
        p_ctx->cmd_r = 0;
    }

    motor_set(MOTOR_LEFT, p_ctx->cmd_l);
    motor_set(MOTOR_RIGHT, p_ctx->cmd_r);

    tick_distance(p_ctx);
#if !MOTION_BENCH
    tick_telemetry(p_ctx);
#endif
    tick_report(p_ctx);

    t_end = now_us();
    work  = t_end - t_start;
    if (work > p_ctx->work_max_us)
    {
        p_ctx->work_max_us = work;
    }

    tick_feed_watchdog(p_ctx, t_end);
}

/* ------------------------------------------------------------------ *
 *  Handlers and tasks
 * ------------------------------------------------------------------ */

/* Cyclic handler, every MOTION_TICK_MS. */
static void
motion_cyclic_isr (void *exinf)
{
    (void)exinf;
    (void)tk_set_flg(g_flgid, MOTION_EVT_TICK);
}

#if MOTION_BENCH && MOTION_TEST_STOP
/* Stop test stand-in for scan_task and imu_task.  Once motion_task says
   the wheels are turning, it sets STOP_SCAN or STOP_IMU in turn, the way
   they will (stamp, then tk_set_flg), and clears it again after
   MOTION_TEST_STOP_HOLD ticks.  On the car only FAULT_CLEAR(IMU) clears
   STOP_IMU; the stand-in clears it itself so the test can go on. */
typedef struct
{
    UINT     bit;     /* the STOP bit it has set, 0 = none */
    uint32_t hold;    /* ticks it has held it              */
    uint32_t n_stops; /* stops requested so far         */
} standin_t;

static void
standin_step (standin_t *p_st)
{
    if (0u != p_st->bit)
    {
        p_st->hold++;
        if (p_st->hold >= MOTION_TEST_STOP_HOLD)
        {
            (void)tk_clr_flg(g_flgid, ~p_st->bit);
            p_st->bit = 0u;
        }
    }
    else if (gb_st_ready)
    {
        p_st->bit  = (0u == (p_st->n_stops % STOP_SOURCES))
                         ? MOTION_EVT_STOP_SCAN
                         : MOTION_EVT_STOP_IMU;
        p_st->hold = 0u;
        p_st->n_stops++;
        motion_stop_stamp(p_st->bit);
        (void)tk_set_flg(g_flgid, p_st->bit);
    }
    else
    {
        /* Waiting for the wheels to turn. */
    }
}

static void
stop_standin_task (INT stacd, void *exinf)
{
    standin_t st = { 0u, 0u, 0u };

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_dly_tsk((TMO)MOTION_TICK_MS);
        standin_step(&st);
    }
}
#endif

/* Hardware and state set-up.  Motors first, so they are braked before
   anything else happens.  Returns the first error. */
static ER
motion_init (motion_ctx_t *p_ctx)
{
    ER ercd;

    ercd = motor_init();
    if (E_OK == ercd)
    {
        ercd = gpio_irq_init();
    }
    if (E_OK == ercd)
    {
        ercd = encoder_init();
    }
#if MOTION_BENCH
    button_init(p_ctx);
#endif

    motion_pid_init(&p_ctx->pid_l, MOTION_PID_KP_Q8, MOTION_PID_KI_Q8,
                    MOTION_PID_KD_Q8, MOTION_PID_OUT_LIMIT);
    motion_pid_init(&p_ctx->pid_r, MOTION_PID_KP_Q8, MOTION_PID_KI_Q8,
                    MOTION_PID_KD_Q8, MOTION_PID_OUT_LIMIT);
    motion_stall_init(&p_ctx->stall_l, MOTION_STALL_TICKS,
                      MOTION_STALL_MIN_COUNTS, MOTION_STALL_ENC_EDGES);
    motion_stall_init(&p_ctx->stall_r, MOTION_STALL_TICKS,
                      MOTION_STALL_MIN_COUNTS, MOTION_STALL_ENC_EDGES);
    motion_ds_win_reset(&p_ctx->ds_win);
    motion_sm_init(&p_ctx->sm);

    encoder_read_counts(&p_ctx->prev_l, &p_ctx->prev_r);
    p_ctx->prev_edges_l = encoder_read_edges_left();
    p_ctx->prev_edges_r = encoder_read_edges_right();
    p_ctx->per_min_us = UINT32_MAX;
    p_ctx->phase      = PHASE_IDLE;

    if (ercd != E_OK)
    {
        p_ctx->faults |= MOTION_FAULT_INIT;
        motor_brake_all();
    }

    return ercd;
}

/* Everything before the first tick, in this order: why the chip last
   started (read before arming, which rewrites the marker it uses), the
   hardware set-up, then the watchdog.  It is armed only if the set-up
   passed and the tick exists (motor_task_start), so a car that cannot run
   stays braked instead of rebooting over and over: with no tick nothing
   would feed it, and the chip would reset every MOTION_WDT_MS. */
static ER
motion_start_up (motion_ctx_t *p_ctx, INT prc)
{
    motion_boot_t   boot     = motion_wdt_boot_cause();
    int32_t         hang_us  = -1;
    int32_t         armed_ms = 0;
    ER              ercd;
    motion_report_t rep;
#if MOTION_BENCH
    uint32_t us;

    if (motion_wdt_take_hang_us(&us))
    {
        hang_us = (int32_t)us;
    }
#endif

    ercd = motion_init(p_ctx);
    if ((E_OK == ercd) && (E_OK != g_tick_ercd))
    {
        ercd = g_tick_ercd; /* no tick: a set-up failure too */
        p_ctx->faults |= MOTION_FAULT_INIT;
        motor_brake_all();
    }
    if (E_OK == ercd)
    {
        p_ctx->wdt_fed_us = now_us();
        motion_wdt_start();
        armed_ms = (int32_t)MOTION_WDT_MS;
    }

    report_event(p_ctx, EVT_READY, ercd, prc);

    report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_BOOT);
    rep.val_l  = (int32_t)boot;
    rep.val_r  = hang_us;
    rep.rate_l = armed_ms;
    report_send(p_ctx, &rep);

    return ercd;
}

/* Wait for one tick and run it.  No tick within MOTION_TICK_TIMEOUT_MS
   (20 ms, below the 30 ms watchdog) means the cyclic handler stopped:
   brake.  This timeout runs on the same kernel timer as the tick, so if
   that timer stops (core 0 stalled) it never fires.  The watchdog covers
   that case: no tick, no feed. */
static void
motion_cycle (motion_ctx_t *p_ctx)
{
    ER   ercd;
    UINT ptn = 0u;
    UINT waiptn;

    /* Wait on the tick and on every STOP bit that is not set yet, with no
       clear: TWF_BITCLR would clear every bit waited on, and motion must
       never clear STOP_SCAN or STOP_IMU.  A new STOP wakes motion at once,
       also while it reverses under STOP_SCAN (design 6.2).  A STOP bit
       already set stays out of the wait (it would return at once,
       forever) and is looked at on every tick.  The tick is cleared by
       hand. */
    waiptn = MOTION_EVT_TICK | (MOTION_EVT_STOPS & ~(UINT)p_ctx->stop_bits);

    ercd = tk_wai_flg(g_flgid, waiptn, TWF_ORW, &ptn, MOTION_TICK_TIMEOUT_MS);
    if (E_OK == ercd)
    {
        stop_check(p_ctx, ptn);

        if (0u != (ptn & MOTION_EVT_TICK))
        {
            (void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_TICK);
        }
        if (0u != (ptn & MOTION_EVT_TICK)) /* only a tick runs the work */
        {
            motion_tick(p_ctx);
        }
    }
    else
    {
        motor_brake_all();
        p_ctx->lost_ticks++;
        if (p_ctx->phase != PHASE_IDLE)
        {
            p_ctx->faults |= MOTION_FAULT_TICK;
            bench_stop(p_ctx, EVT_FAULT);
        }
    }
}

/* A failed set-up (motion_start_up), for nav_ctrl_task and S.faults
   (W7): MOTION_FAULT sends the mission to Fault.  No fault bit is just
   for it, so both motion bits are set, motor and encoder. */
static void
init_fail_report (void)
{
    ID nav_id = nav_flag();

    fault_set(FAULT_MOTOR_STALL | FAULT_ENCODER);
    if (nav_id > 0)
    {
        (void)tk_set_flg(nav_id, NAV_FLG_MOTION_FAULT);
    }
}

void
motor_task (INT stacd, void *exinf)
{
    motion_ctx_t *p_ctx = &g_ctx;
    ER            ercd;
    INT           prc = MOTION_PRC_SINGLE;

    (void)stacd;
    (void)exinf;

#if TK_SUPPORT_SMP
    prc = tk_get_prc();
#endif
    ercd = motion_start_up(p_ctx, prc);

    if (ercd != E_OK)
    {
        /* Without encoders, PWM or the tick nothing here is safe to run:
           stay braked and do nothing else (the watchdog is not armed).
           Tell nav, or a mission would run with no motor and no fault
           (W7): now, and again whenever the mission is Running (a fault
           reset recalibrates and allows a start). */
        init_fail_report();
        for (;;)
        {
            (void)tk_dly_tsk((TMO)INIT_FAIL_POLL_MS);
            if (MS_RUNNING == nav_mission_state())
            {
                init_fail_report();
            }
        }
    }

    /* Start the period measurement from a fresh tick.  Clear only TICK:
       a STOP bit set before now must still brake the car. */
    (void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_TICK);

    for (;;)
    {
        motion_cycle(p_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Report task (core 0): all printing happens here
 * ------------------------------------------------------------------ */

static void
print_line (UB const *p_line)
{
    (void)tm_printf((UB *)"%s\n", p_line);
}

static void print_step_hints(UB *p_line, motion_report_t const *p_rep);

/* "[motion] FAULT 0x01: left wheel stalled  <tail>": every fault bit by
   name. */
static void
print_fault_bits (UB *p_line, uint8_t faults, char const *p_tail)
{
    (void)tm_sprintf(
        p_line, (UB *)"[motion] FAULT 0x%02x:%s%s%s%s%s%s  %s", (UINT)faults,
        ((faults & MOTION_FAULT_STALL_L) != 0u) ? " left wheel stalled" : "",
        ((faults & MOTION_FAULT_STALL_R) != 0u) ? " right wheel stalled" : "",
        ((faults & MOTION_FAULT_ENC_L) != 0u) ? " left encoder fault" : "",
        ((faults & MOTION_FAULT_ENC_R) != 0u) ? " right encoder fault" : "",
        ((faults & MOTION_FAULT_TICK) != 0u) ? " tick lost" : "",
        ((faults & MOTION_FAULT_INIT) != 0u) ? " init failed" : "", p_tail);
    print_line(p_line);
}

static void
print_faults (UB *p_line, motion_report_t const *p_rep)
{
    uint8_t faults = p_rep->faults;

    print_fault_bits(p_line, faults,
                     "motors braked. Fix, then press the button to run"
                     " again.");

    (void)tm_sprintf(
        p_line,
        (UB *)"[motion]   at the fault: duty L%+d R%+d, moved L%+d R%+d"
              " counts in this step",
        (INT)p_rep->duty_l, (INT)p_rep->duty_r, (INT)p_rep->val_l,
        (INT)p_rep->val_r);
    print_line(p_line);

    if (0u != (faults & (MOTION_FAULT_ENC_L | MOTION_FAULT_ENC_R)))
    {
        print_line((UB *)"[motion]   encoder fault: one phase kept switching"
                         " but the count went nowhere. Check the A and B"
                         " wires of that encoder.");
    }
    print_step_hints(p_line, p_rep);
}

/* EVT_SM_FAULT (normal image): why the statechart entered Motion Fault,
   with what the stall check judged.  Each wheel's stall window restarts
   whenever it makes progress (MOTION_STALL_MIN_COUNTS the commanded
   way), so a wheel that kept moving shows 0 ms or close to it.  It also
   restarts on a change of direction and on every tick under the dead
   band (MOTION_DEADBAND_DUTY), so 0 ms can mean not driven.  Right-wheel
   counts are in left-wheel units (tick_encoders). */
static void
print_sm_fault (UB *p_line, motion_report_t const *p_rep)
{
    print_fault_bits(p_line, p_rep->faults,
                     "motors braked until nav sends FAULT_CLEAR(MOTION).");
    (void)tm_sprintf(
        p_line,
        (UB *)"[motion]   at the fault: duty L%+d R%+d; since each wheel"
              " last made progress: L%+d counts in %d ms, R%+d counts in"
              " %d ms; missed L%u R%u",
        (INT)p_rep->duty_l, (INT)p_rep->duty_r, (INT)p_rep->val_l,
        (INT)(p_rep->rate_l * (int32_t)MOTION_TICK_MS), (INT)p_rep->val_r,
        (INT)(p_rep->rate_r * (int32_t)MOTION_TICK_MS), (UINT)p_rep->inv_l,
        (UINT)p_rep->inv_r);
    print_line(p_line);
}

static void
print_step_hints (UB *p_line, motion_report_t const *p_rep)
{
    int32_t driven;
    int32_t other;
    bool    b_left = (p_rep->duty_l != 0);

    /* A one-wheel step that moves the other wheel: plugs swapped. */
    if ((0 == p_rep->duty_l) != (0 == p_rep->duty_r))
    {
        driven = b_left ? p_rep->val_l : p_rep->val_r;
        other  = b_left ? p_rep->val_r : p_rep->val_l;
        if ((motion_abs(other) >= CROSSTALK_MIN_COUNTS)
            && (motion_abs(other) > (motion_abs(driven) / CROSSTALK_RATIO)))
        {
            (void)tm_sprintf(
                p_line,
                (UB *)"[motion]   WARNING: the %s encoder moved on a %s-only"
                      " step. Motor or encoder plugs swapped?",
                b_left ? "right" : "left", b_left ? "left" : "right");
            print_line(p_line);
        }
    }

    /* Count sign against commanded sign. */
    if (((p_rep->duty_l > 0) && (p_rep->val_l < 0))
        || ((p_rep->duty_l < 0) && (p_rep->val_l > 0))
        || ((p_rep->duty_r > 0) && (p_rep->val_r < 0))
        || ((p_rep->duty_r < 0) && (p_rep->val_r > 0)))
    {
        print_line(
            (UB *)"[motion]   count went the opposite way to the command."
                  " See README step 2 for which INVERT flag to set.");
    }

    if (((p_rep->duty_l != 0) && (0 == p_rep->val_l))
        || ((p_rep->duty_r != 0) && (0 == p_rep->val_r)))
    {
        print_line(
            (UB *)"[motion]   a driven wheel counted 0: encoder not wired,"
                  " not powered, or the wheel did not turn.");
    }
}

static void
print_ready (UB *p_line, motion_report_t const *p_rep)
{
    bool b_init_ok = (0 == p_rep->val_l);

#if MOTION_BENCH
    (void)tm_sprintf(
        p_line,
        (UB *)"[motion] ready on processor %d, init %s (er=%d)."
              " PWM %u Hz on GP%u-GP%u, encoders GP%u-GP%u%s,"
              " button GP%u, test=%s, MOTION_PACKED=%d",
        (INT)p_rep->val_r, b_init_ok ? "OK" : "FAILED", (INT)p_rep->val_l,
        (UINT)MOTION_PWM_HZ, (UINT)MOTION_PIN_M1A, (UINT)MOTION_PIN_M2B,
        (UINT)MOTION_PIN_ENC_L_A, (UINT)MOTION_PIN_ENC_R_B,
        encoder_left_enabled() ? "" : " (LEFT OFF: UART console)",
        (UINT)MOTION_PIN_BUTTON,
        (0 != MOTION_TEST_STOP)
            ? "stop (NFR-18)"
            : ((0 != MOTION_TEST_PID) ? "closed-loop" : "open-loop"),
        (INT)MOTION_PACKED);
    print_line(p_line);

    if (b_init_ok)
    {
        print_line((UB *)"[motion] wheels OFF the ground, then press the"
                         " button to start. Turn a wheel by hand to see"
                         " its count.");
    }
#else
    (void)tm_sprintf(p_line,
                     (UB *)"[motion] ready on processor %d, init %s (er=%d)."
                           " PWM %u Hz on GP%u-GP%u, encoders GP%u-GP%u%s,"
                           " bench OFF (MOTION_BENCH=0): motors stay braked,"
                           " MOTION_PACKED=%d",
                     (INT)p_rep->val_r, b_init_ok ? "OK" : "FAILED",
                     (INT)p_rep->val_l, (UINT)MOTION_PWM_HZ,
                     (UINT)MOTION_PIN_M1A, (UINT)MOTION_PIN_M2B,
                     (UINT)MOTION_PIN_ENC_L_A, (UINT)MOTION_PIN_ENC_R_B,
                     encoder_left_enabled() ? "" : " (LEFT OFF: UART console)",
                     (INT)MOTION_PACKED);
    print_line(p_line);
#endif

    /* Which counts per turn this image was built with (W1: the right
       wheel's value is still disputed), and the L4 switch (a -D, which
       an in-place rebuild keeps), so a log shows both. */
    (void)tm_sprintf(p_line,
                     (UB *)"[motion] counts per wheel turn: left %d"
                           " (MOTION_COUNTS_PER_REV), right %d"
                           " (MOTION_COUNTS_PER_REV_R)%s;"
                           " MOTION_LF_LOST_DIR_MS=%u",
                     (INT)MOTION_COUNTS_PER_REV, (INT)MOTION_COUNTS_PER_REV_R,
                     (MOTION_COUNTS_PER_REV_R != MOTION_COUNTS_PER_REV)
                         ? ", right counts scaled to left units"
                         : ", no scaling",
                     (UINT)MOTION_LF_LOST_DIR_MS);
    print_line(p_line);
}

static void
print_boot (UB *p_line, motion_report_t const *p_rep)
{
    switch (p_rep->val_l)
    {
        case MOTION_BOOT_WATCHDOG:
            print_line((UB *)"[motion] last reset: WATCHDOG. motion_task"
                             " stopped feeding it (task hung or tick"
                             " stopped). Motors stay braked until a"
                             " command.");
            break;

        case MOTION_BOOT_REBOOT:
            print_line((UB *)"[motion] last reset: a reboot by the boot ROM"
                             " or a debugger (such as after a UF2 copy),"
                             " not a fault.");
            break;

        default:
            print_line((UB *)"[motion] last reset: power-on or RUN pin.");
            break;
    }

    if (p_rep->rate_l > 0)
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] watchdog armed: the chip resets if"
                               " a tick does not complete within %d ms.",
                         (INT)p_rep->rate_l);
        print_line(p_line);
    }
    else
    {
        print_line((UB *)"[motion] watchdog NOT armed (init failed)."
                         " Motors stay braked.");
    }

    if (p_rep->val_r >= 0)
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] watchdog test: the reset came %d us"
                               " after the last feed (set to %u us).",
                         (INT)p_rep->val_r, (UINT)(MOTION_WDT_MS * US_PER_MS));
        print_line(p_line);
    }
}

/* "[motion] state LINE_FOLLOW -> BRAKED", plus the replies it sent. */
static void
print_state (UB *p_line, motion_report_t const *p_rep)
{
    uint32_t replies = (uint32_t)p_rep->rate_l;

    (void)tm_sprintf(
        p_line, (UB *)"[motion] state %s -> %s%s%s%s%s%s%s",
        motion_sm_name((uint32_t)p_rep->val_r),
        motion_sm_name((uint32_t)p_rep->val_l),
        (0u != (replies & MOTION_SM_REP_MOVE_DONE)) ? ", sent MOVE_DONE" : "",
        (0u != (replies & MOTION_SM_REP_CMD_REJECTED)) ? ", sent CMD_REJECTED"
                                                       : "",
        (0u != (replies & MOTION_SM_REP_MOTION_FAULT)) ? ", sent MOTION_FAULT"
                                                       : "",
        (0u != (replies & MOTION_SM_REP_LINE_FOUND)) ? ", sent LINE_FOUND" : "",
        (0u != (replies & MOTION_SM_REP_LINE_LOST)) ? ", sent LINE_LOST" : "",
        (0u != (replies & MOTION_SM_REP_REVERSE_DONE)) ? ", sent REVERSE_DONE"
                                                       : "");
    print_line(p_line);
}

/* EVT_BYP, e.g. "[motion] bypass SIDESTEP: 12 mm out, heading +68 deg to
   the line, IMU". */
#ifdef BYP_PH_DONE /* the statechart image only (MOTION_BENCH 0) */
static void
print_byp (UB *p_line, motion_report_t const *p_rep)
{
    static char const *const names[BYP_PH_DONE + 1u] = {
        "OUT",   "LANE_LOOK", "LANE_BACK", "SWITCH",  "SIDESTEP",
        "PARALLEL", "STEP",   "PROBE_IN",  "LOOK",    "PROBE_OUT",
        "CLEAR", "TURN_IN",   "STEP_IN",   "ONTO",    "ALONG",
        "IN_LOOK", "IN_BACK", "ARC_LEG", "ARC", "SMOOTH", "DONE"
    };
    uint32_t    ph   = (uint32_t)p_rep->val_l;
    char const *name = (ph <= BYP_PH_DONE) ? names[ph] : "?";

    if (BYP_EV_SEEN == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] side sensor: box seen at %d mm"
                         " along, %d mm away", (INT)p_rep->val_r,
                         (INT)p_rep->rate_l);
        print_line(p_line);
        return;
    }
    if (BYP_EV_CLEAR == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] side sensor: box ends at %d mm"
                         " along, back in from %d mm",
                         (INT)p_rep->val_r, (INT)p_rep->rate_l);
        print_line(p_line);
        return;
    }
    if ((BYP_PH_DONE + 6u) == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] line found %+d deg off its"
                         " heading before the loss: onto it, line up",
                         (INT)p_rep->val_r);
        print_line(p_line);
        return;
    }
    if ((BYP_PH_DONE + 5u) == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] bypass plan: box %d mm away,"
                         " face %d mm -> out %d mm, back in from %d mm",
                         (INT)((uint32_t)p_rep->rate_r & 0xFFFu),
                         (INT)(((uint32_t)p_rep->rate_r >> 12) & 0xFFFu),
                         (INT)p_rep->val_r, (INT)p_rep->rate_l);
        print_line(p_line);
        return;
    }
    if ((BYP_PH_DONE + 4u) == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] barcode re-read: line up (heading"
                         " %+d deg), back %d mm, then through it slowly",
                         (INT)p_rep->rate_l, (INT)p_rep->val_r);
        print_line(p_line);
        return;
    }
    if (BYP_EV_MISS == ph)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] side sensor: box never seen by"
                         " %d mm along, back in by distance (%d)",
                         (INT)p_rep->val_r, (INT)p_rep->rate_l);
        print_line(p_line);
        return;
    }
    (void)tm_sprintf(p_line,
                     (UB *)"[motion] bypass %s: %d mm out, heading %+d deg"
                           " to the line, %s",
                     name, (INT)p_rep->val_r, (INT)p_rep->rate_l,
                     (0 != p_rep->rate_r) ? "IMU" : "encoders only");
    print_line(p_line);
}
#endif

/* EVT_CMD, e.g. "[motion] nav_cmd FAULT_CLEAR(IMU): taken".  A JUNCTION
   prints twice: "received" at the read, then "accepted, TURNING" or
   "REFUSED in <state>, sent CMD_REJECTED" once the statechart decided;
   "REFUSED (<state> -> <state>)" if the state changed in that tick all
   the same (a STOP, fault or pause, or a braked turn resuming). */
static void
print_cmd (UB *p_line, motion_report_t const *p_rep)
{
    uint32_t    raw = (uint32_t)p_rep->val_r;
    uint32_t    met;  /* a JUNCTION's outcome: the state it met */
    uint32_t    went; /* and the state the statechart went to */
    char const *what;
    char const *res;
    uint8_t     op;
    uint8_t     side;
    uint16_t    mm = 0u;

    if (CMD_SRC_NAV == p_rep->val_l)
    {
        what = ((raw >> 8) == NAV_CMD_JUNCTION)       ? "JUNCTION"
               : (raw == ((NAV_CMD_FAULT_CLEAR << 8) | FAULT_CLEAR_IMU))
                   ? "FAULT_CLEAR(IMU)"
               : (raw == ((NAV_CMD_FAULT_CLEAR << 8) | FAULT_CLEAR_MOTION))
                   ? "FAULT_CLEAR(MOTION)"
                   : "unknown";
    }
    else
    {
        what = "unknown";
        if (motion_avoid_cmd_unpack((uint16_t)raw, &op, &side, &mm))
        {
            what = (MOTION_AVOID_OP_REVERSE == op) ? "REVERSE"
                   : (MOTION_AVOID_OP_LOOK == op)     ? "LOOK"
                   : (MOTION_AVOID_SIDE_LEFT == side) ? "BYPASS left"
                                                      : "BYPASS right";
        }
    }
    res  = (CMD_RES_TAKEN == p_rep->rate_l)      ? "taken"
           : (CMD_RES_RECEIVED == p_rep->rate_l) ? "received"
           : (CMD_RES_ACCEPTED == p_rep->rate_l) ? "accepted, TURNING"
                                                 : "dropped (invalid)";
    met  = (uint32_t)p_rep->rate_r & CMD_STATE_MASK;
    went = ((uint32_t)p_rep->rate_r >> CMD_WENT_SHIFT) & CMD_STATE_MASK;

    if ((CMD_RES_REFUSED == p_rep->rate_l) && (met == went))
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] nav_cmd %s (raw 0x%04x): REFUSED in"
                               " %s, sent CMD_REJECTED",
                         what, (UINT)raw, motion_sm_name(met));
    }
    else if (CMD_RES_REFUSED == p_rep->rate_l)
    {
        /* The state changed in that tick, but not for this JUNCTION. */
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] nav_cmd %s (raw 0x%04x): REFUSED"
                               " (%s -> %s), sent CMD_REJECTED",
                         what, (UINT)raw, motion_sm_name(met),
                         motion_sm_name(went));
    }
    else if (CMD_SRC_NAV == p_rep->val_l)
    {
        (void)tm_sprintf(p_line, (UB *)"[motion] nav_cmd %s (raw 0x%04x): %s",
                         what, (UINT)raw, res);
    }
    else
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] avoid_cmd %s %u mm (raw 0x%04x): %s",
                         what, (UINT)mm, (UINT)raw, res);
    }
    print_line(p_line);
}

static char const *
stop_name (int32_t bit)
{
    return (MOTION_EVT_STOP_IMU == (uint32_t)bit) ? "STOP_IMU" : "STOP_SCAN";
}

/* EVT_STOP, EVT_GO and EVT_STOPTEST. */
static void
print_stop (UB *p_line, motion_report_t const *p_rep)
{
    bool b_test = (PHASE_STOPTEST == (motion_phase_t)p_rep->phase);

    if (EVT_STOP == (report_event_t)p_rep->code)
    {
        if (p_rep->val_r < 0)
        {
            (void)tm_sprintf(p_line,
                             (UB *)"[motion] %s set: motors braked (the"
                                   " sender did not stamp the request).",
                             stop_name(p_rep->val_l));
        }
        else if (0 != p_rep->rate_l)
        {
            (void)tm_sprintf(p_line,
                             (UB *)"[motion] %s set: brake written %d us"
                                   " after the request.",
                             stop_name(p_rep->val_l), (INT)p_rep->val_r);
        }
        else
        {
            (void)tm_sprintf(p_line,
                             (UB *)"[motion] %s set: seen %d us after the"
                                   " request (already braked).",
                             stop_name(p_rep->val_l), (INT)p_rep->val_r);
        }
        print_line(p_line);

        if ((PHASE_IDLE != (motion_phase_t)p_rep->phase) && !b_test)
        {
            print_line((UB *)"[motion]   the bench test ended on the STOP."
                             " Press the button to run again.");
        }
    }
    else if (EVT_GO == (report_event_t)p_rep->code)
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] every STOP bit clear, %d ms after"
                               " the first one was seen.%s",
                         (INT)p_rep->val_l, b_test ? " Driving again." : "");
        print_line(p_line);
    }
    else
    {
        (void)tm_sprintf(p_line,
                         (UB *)"[motion] STOP TEST %s: %d stops, request to"
                               " brake min %d / mean %d / max %d us"
                               " (NFR-18: within 20000 us)",
                         stop_name(p_rep->val_l), (INT)p_rep->val_r,
                         (INT)p_rep->duty_l, (INT)p_rep->rate_r,
                         (INT)p_rep->rate_l);
        print_line(p_line);
    }
}

static void
print_event (UB *p_line, motion_report_t const *p_rep)
{
    switch (p_rep->code)
    {
        case EVT_READY:
            print_ready(p_line, p_rep);
            break;

        case EVT_BOOT:
            print_boot(p_line, p_rep);
            break;

        case EVT_START:
            if (0 != p_rep->val_r)
            {
                (void)tm_sprintf(p_line,
                                 (UB *)"[motion] START stop test (NFR-18):"
                                       " both wheels at %d per-mille,"
                                       " STOP_SCAN and STOP_IMU %u times"
                                       " each (press the button to abort)",
                                 (INT)MOTION_TEST_DUTY,
                                 (UINT)MOTION_TEST_STOP_TRIALS);
                print_line(p_line);
            }
            else if (0 != p_rep->val_l)
            {
                (void)tm_sprintf(p_line,
                                 (UB *)"[motion] START closed-loop test,"
                                       " target %d c/s per wheel",
                                 (INT)p_rep->val_l);
                print_line(p_line);
            }
            else
            {
                print_line((UB *)"[motion] START open-loop test"
                                 " (press the button to abort)");
            }
            break;

        case EVT_DONE:
            print_line((UB *)"[motion] DONE, motors braked."
                             " Press the button to run again.");
            break;

        case EVT_ABORT:
            print_line((UB *)"[motion] ABORTED by button, motors braked.");
            break;

        case EVT_FAULT:
            print_faults(p_line, p_rep);
            break;

        case EVT_SM_FAULT:
            print_sm_fault(p_line, p_rep);
            break;

        case EVT_STOP:
        case EVT_GO:
        case EVT_STOPTEST:
            print_stop(p_line, p_rep);
            break;

        case EVT_STATE:
            print_state(p_line, p_rep);
            break;

        case EVT_CMD:
            print_cmd(p_line, p_rep);
            break;

#ifdef BYP_PH_DONE
        case EVT_BYP:
            print_byp(p_line, p_rep);
            break;
#endif

        case EVT_RAMP:
            (void)tm_sprintf(p_line,
                             (UB *)"[motion] DEAD BAND: left starts at %d,"
                                   " right starts at %d per-mille"
                                   " (0 = did not move by %d)",
                             (INT)p_rep->val_l, (INT)p_rep->val_r,
                             (INT)MOTION_RAMP_MAX_DUTY);
            print_line(p_line);
            break;

        default:
            break;
    }
}

/* The word imu_task reads, then each field.  The word is rebuilt here
   with the same motion_ds_pack() motion_task uses. */
static void
print_distance (UB *p_line, motion_report_t const *p_rep)
{
    static char const *const state_names[] = { "STEADY", "ACCELERATING",
                                               "DECELERATING" };
    char const              *p_state       = "?";

    if ((uint32_t)p_rep->code
        < (uint32_t)(sizeof(state_names) / sizeof(state_names[0])))
    {
        p_state = state_names[p_rep->code];
    }

    (void)tm_sprintf(
        p_line,
        (UB *)"[motion] distance_state 0x%08x: dist %u mm (net %+d mm),"
              " %s (%+d mm/s^2), update %d, turn %+d deg/s",
        (UINT)motion_ds_pack(p_rep->val_l, (motion_ds_state_t)p_rep->code,
                             (uint32_t)p_rep->rate_r, p_rep->val_r),
        (UINT)((uint32_t)p_rep->val_l & MOTION_DS_MM_MASK), (INT)p_rep->val_l,
        p_state, (INT)p_rep->rate_l, (INT)p_rep->rate_r, (INT)p_rep->val_r);
    print_line(p_line);
}

static void
print_report (motion_report_t const *p_rep)
{
    UB line[REPORT_LINE_LEN];

    switch (p_rep->kind)
    {
        case REP_STATUS:
            (void)tm_sprintf(
                line,
                (UB *)"[motion] t=%u.%02us  duty L%+d R%+d  count L%+d R%+d"
                      "  speed L%+d R%+d c/s  missed L%u R%u",
                (UINT)(p_rep->t_ms / (uint32_t)MS_PER_S),
                (UINT)((p_rep->t_ms % (uint32_t)MS_PER_S) / MOTION_TICK_MS),
                (INT)p_rep->duty_l, (INT)p_rep->duty_r, (INT)p_rep->val_l,
                (INT)p_rep->val_r, (INT)p_rep->rate_l, (INT)p_rep->rate_r,
                (UINT)p_rep->inv_l, (UINT)p_rep->inv_r);
            print_line(line);
            break;

        case REP_STEP:
            if (p_rep->code < STEP_COUNT)
            {
                (void)tm_sprintf(
                    line,
                    (UB *)"[motion] STEP %-8s duty L%+d R%+d -> moved L%+d R%+d"
                          " counts (L%+d R%+d c/s, L%+d R%+d per tick)",
                    g_steps[p_rep->code].p_name, (INT)p_rep->duty_l,
                    (INT)p_rep->duty_r, (INT)p_rep->val_l, (INT)p_rep->val_r,
                    (INT)p_rep->rate_l, (INT)p_rep->rate_r,
                    (INT)(p_rep->rate_l / TICKS_PER_S),
                    (INT)(p_rep->rate_r / TICKS_PER_S));
                print_line(line);
                print_step_hints(line, p_rep);
            }
            break;

        case REP_TIMING:
            (void)tm_sprintf(
                line,
                (UB *)"[motion] timing (5 s): tick period %u..%u us,"
                      " tick work max %u us, encoder irq %u/s (longest %u us),"
                      " lost ticks %d, dropped reports %d",
                (UINT)p_rep->per_min_us, (UINT)p_rep->per_max_us,
                (UINT)p_rep->work_max_us,
                (UINT)(p_rep->irq_count / TIMING_REPORT_S),
                (UINT)p_rep->isr_max_us, (INT)p_rep->val_l, (INT)p_rep->val_r);
            print_line(line);
            (void)tm_sprintf(line,
                             (UB *)"[motion] watchdog (5 s): longest gap"
                                   " between feeds %d us, resets at %u us;"
                                   " E motion sends refused %d",
                             (INT)p_rep->rate_l,
                             (UINT)(MOTION_WDT_MS * US_PER_MS),
                             (INT)p_rep->rate_r);
            print_line(line);
            break;

        case REP_DIST:
            print_distance(line, p_rep);
            break;

        case REP_EVENT:
        default:
            print_event(line, p_rep);
            break;
    }
}

static void
motion_report_task (INT stacd, void *exinf)
{
    motion_report_t rep;
    INT             size;
    INT             polls;

    (void)stacd;
    (void)exinf;

    /* Same enumeration wait the demo monitor uses: anything printed
       before the host opens the port is lost.  Records queue meanwhile. */
    for (polls = 0;
         (polls < CONSOLE_WAIT_POLLS) && (tm_usb_state() < CONSOLE_ENUMERATED);
         polls++)
    {
        (void)tk_dly_tsk(CONSOLE_WAIT_MS);
    }

    for (;;)
    {
        size = tk_rcv_mbf(g_mbfid, &rep, TMO_FEVR);
        if ((INT)sizeof(rep) == size)
        {
            print_report(&rep);
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Start-up
 * ------------------------------------------------------------------ */

ER
motor_task_start (void)
{
    ER ercd = E_OK;
    ID obj_id;

    obj_id = tk_cre_flg(&g_cflg);
    if (obj_id < E_OK)
    {
        ercd = obj_id;
    }
    else
    {
        g_flgid = obj_id;
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_mbf(&g_cmbf);
        if (obj_id < E_OK)
        {
            ercd = obj_id;
        }
        else
        {
            g_mbfid = obj_id;
        }
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_mbf(&g_cmbf_nav);
        if (obj_id < E_OK)
        {
            ercd = obj_id;
        }
        else
        {
            g_nav_mbfid = obj_id;
        }
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_mbf(&g_cmbf_avoid);
        if (obj_id < E_OK)
        {
            ercd = obj_id;
        }
        else
        {
            g_avoid_mbfid = obj_id;
        }
    }

#if !MOTION_PACKED
    if (E_OK == ercd)
    {
        obj_id = tk_cre_mtx(&g_cmtx_ds);
        if (obj_id < E_OK)
        {
            ercd = obj_id;
        }
        else
        {
            g_ds_mtxid = obj_id;
        }
    }
#endif

    if (E_OK == ercd)
    {
        obj_id = tk_cre_tsk(&g_ctsk_report);
        ercd   = (obj_id < E_OK) ? obj_id : tk_sta_tsk(obj_id, 0);
    }

    /* The tick before motion_task, which arms the watchdog only once the
       tick that feeds it exists.  The handler only sets TICK, which
       motion_task clears before its first wait.  If the tick cannot be
       made, motion_task still starts, to brake the motors, and reports
       init FAILED without arming; this function returns the error. */
    if (E_OK == ercd)
    {
        obj_id      = tk_cre_cyc(&g_ccyc);
        g_tick_ercd = (obj_id < E_OK) ? obj_id : E_OK;
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_tsk(&g_ctsk_motion);
        ercd   = (obj_id < E_OK) ? obj_id : tk_sta_tsk(obj_id, 0);
    }

#if MOTION_BENCH && MOTION_TEST_STOP
    if (E_OK == ercd)
    {
        obj_id = tk_cre_tsk(&g_ctsk_standin);
        ercd   = (obj_id < E_OK) ? obj_id : tk_sta_tsk(obj_id, 0);
    }
#endif

    if (E_OK == ercd)
    {
        ercd = g_tick_ercd;
    }

    return ercd;
}
