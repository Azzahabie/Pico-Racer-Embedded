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
                       gap between watchdog feeds, us         */
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
                     rate_l = CMD_RES_*                     */
    EVT_BYP       /* bypass phase: val_l = phase, val_r =
                     sideways mm toward the side, rate_l =
                     heading - line deg (x side), rate_r =
                     1 IMU heading in use, 0 encoders only  */
} report_event_t;

/* EVT_CMD fields */
#define CMD_SRC_NAV       (0)
#define CMD_SRC_AVOID     (1)
#define CMD_RES_TAKEN     (0) /* passed to the statechart          */
#define CMD_RES_INVALID   (2) /* unknown or malformed: dropped      */

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
    uint8_t turn_action;    /* barcode_action_t from nav            */
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

/* ------------------------------------------------------------------ *
 *  Kernel objects
 * ------------------------------------------------------------------ */

static ID g_flgid;
static ID g_mbfid;
static ID g_nav_mbfid;   /* nav_cmd_mbf:   nav_ctrl_task -> motion */
static ID g_avoid_mbfid; /* avoid_cmd_mbf: scan_task -> motion     */

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
#define BYP_PH_DONE      (19u)

/* Pivot direction of each phase, x the side (+1 = toward the bypass
   side), and its angle; 0 = straight or braked (the looks, SWITCH, DONE).
   BYP_PH_ARC is neither: arc_drive() sets the wheels. */
static int8_t const g_byp_turn[BYP_PH_DONE] = {
    1, 0, -1, 0, 0, -1, 0, -1, 0, 1, 0, -1, 0, 0, 1, 0, 1, 0, 0
};
static uint8_t const g_byp_deg[BYP_PH_DONE] = {
    MOTION_BYPASS_SIDE_DEG, 0, MOTION_BYPASS_SIDE_DEG, 0, 0,
    MOTION_BYPASS_SIDE_DEG, 0, 45, 0, 45, 0, MOTION_BYPASS_SIDE_DEG, 0, 0,
    MOTION_BYPASS_SIDE_DEG, 0, MOTION_BYPASS_SIDE_DEG, 0, 0
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

    p_ctx->lf_rem      = 0;
    p_ctx->lf_corr     = 0;
    p_ctx->lf_hist_i   = 0u;
    p_ctx->lf_hard_run = 0u;
    for (i = 0u; i < MOTION_LF_DAMP_TICKS; i++)
    {
        p_ctx->lf_hist[i] = err;
    }
}

/* The error Line Following steers by.  Off the line (rl 00, both sensors
   white) the word's error is only the difference between two white
   readings, so the car ran straight off the outside of a curve (S4
   attempt 7: about -30 for the 1.5 s until LOST).  Instead it keeps
   turning, at full error, to the side the line was last seen on.  Not
   during a barcode hold, which also reads rl 00 by design. */
static int32_t
lf_error (const motion_ctx_t *p_ctx)
{
    uint32_t word = p_ctx->line_word;

    if ((LINE_RL_CENTRED == line_state_rl(word))
        && !line_state_barcode(word) && (0 != p_ctx->line_dir))
    {
        return p_ctx->line_dir * LINE_ST_ERR_MAX;
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

    target = lf_clamp(target, cap);
    p_ctx->lf_corr += lf_clamp(target - p_ctx->lf_corr, MOTION_LF_SLEW);
    return p_ctx->lf_corr;
}

/* A state was just entered: set up its manoeuvre. */
static void
man_start (motion_ctx_t *p_ctx, motion_state_t state, bool b_resumed)
{
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
        if (turned >= PIVOT_COUNTS(deg))
        {
            p_ctx->turn_phase = TURN_PH_DONE;
        }
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
        }
        else if (fwd > 0)
        {
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
    -MOTION_BYPASS_SIDE_DEG, 0, 0, 0, 0, 0, 0, 0
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
        int32_t target = (p_ctx->b_arc && (BYP_PH_OUT == ph))
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
            if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_LANE_LOOK);
                byp_ask(p_ctx, SCAN_FLG_LOOK_AHEAD);
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
            if (moved >= MOTION_LINE_SENSOR_MM)
            {
                byp_next(p_ctx, BYP_PH_ALONG);
            }
            break;
        case BYP_PH_ALONG:
            if (b_pivot_done)
            {
                byp_next(p_ctx, BYP_PH_DONE);
                p_ctx->line_dir = -side;
            }
            break;
        default:
            /* Done: braked until the statechart leaves Bypass. */
            break;
    }

    ph  = p_ctx->bypass_phase;
    dir = (ph < BYP_PH_DONE) ? ((int32_t)g_byp_turn[ph] * side) : 0;
    if ((ph >= BYP_PH_DONE) || (BYP_PH_LOOK == ph)
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
        /* correction > 0: turn left (slow the left wheel, speed up the
           right).  With LINE_STEER_INVERT that is the left sensor darker.
           The remainder carries to the next tick, so a correction below
           one count still steers on average (S4: an error below 13 gave
           no correction at all). */
        p_ctx->lf_rem += lf_correction(p_ctx, lf_error(p_ctx),
                                       line_state_barcode(p_ctx->line_word));
        corr = p_ctx->lf_rem / MOTION_LF_SCALE;
        p_ctx->lf_rem -= corr * MOTION_LF_SCALE;
        p_ctx->cmd_l = pid_capped(p_ctx, &p_ctx->pid_l, MOTION_LF_BASE - corr,
                                       p_ctx->d_l);
        p_ctx->cmd_r = pid_capped(p_ctx, &p_ctx->pid_r, MOTION_LF_BASE + corr,
                                       p_ctx->d_r);
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
   outside Line Following the statechart refuses it (CMD_REJECTED).
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
            p_in->nav          = MOTION_SM_NAV_JUNCTION;
            p_ctx->turn_action = msg[1];
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

/* Counts per tick -> cm/s: d x pi x diameter mm per tick, x 100 ticks a
   second, / 10 mm a cm; pi as 31416 / 10000. */
#define CMS_FROM_COUNTS(d)                                                     \
    (((d) * 31416 * MOTION_WHEEL_DIA_MM)                                       \
     / (MOTION_COUNTS_PER_REV * 1000))

/* E motion (MQTT_SCHEMA.md 5, k = 0) on every statechart change: the new
   state, this tick's wheel speeds, the running distance and turn rate, and
   the motion faults now.  comms_tele_send() polls, so the tick never
   waits. */
static void
tele_motion (motion_ctx_t const *p_ctx)
{
    UB body[COMM_W_INTAKE_SIZE_MOTION];

    (void)comms_tele_send(
        body, (INT)comm_w_pack_motion(
                  body, (uint32_t)motion_sm_state(&p_ctx->sm),
                  CMS_FROM_COUNTS(p_ctx->d_l), CMS_FROM_COUNTS(p_ctx->d_r),
                  (uint32_t)p_ctx->ds_mm, p_ctx->ds_turn,
                  (uint32_t)fault_flg_bits(p_ctx->faults), 0u));
}

/* One statechart step per tick, after the stall check.  The mission state
   comes from nav_ctrl_task (one aligned load), commands from
   take_commands().  Every driving state is built: line following, the
   line search, the turn, the reverse and the bypass.
   distance_state is never reset: imu_task works from differences, and a
   jump would read as a fake movement. */
static void
tick_statechart (motion_ctx_t *p_ctx)
{
    motion_sm_in_t  in      = { 0 };
    motion_sm_out_t out     = { 0u, false, false, false };
    motion_state_t  before  = motion_sm_state(&p_ctx->sm);
    mission_state_t mission = nav_mission_state();
    motion_report_t rep;

    take_commands(p_ctx, &in);
    in.b_running     = (MS_RUNNING == mission);
    in.b_stop_scan   = (0u != (p_ctx->stop_bits & MOTION_EVT_STOP_SCAN));
    in.b_stop_imu    = (0u != (p_ctx->stop_bits & MOTION_EVT_STOP_IMU));
    in.b_fault       = (0u != ((uint32_t)p_ctx->faults & SM_FAULTS));
    in.avoid         = (motion_sm_avoid_t)p_ctx->avoid_cmd;
    in.b_done        = man_done(p_ctx, before);
    p_ctx->avoid_cmd = (uint8_t)MOTION_SM_AVOID_NONE; /* taken */
    line_inputs(p_ctx, &in);

    motion_sm_step(&p_ctx->sm, &in, &out);

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
    if (out.b_changed && !motion_sm_driving(motion_sm_state(&p_ctx->sm)))
    {
        motor_brake_all(); /* before any reply, e.g. MOTION_FAULT */
    }
    hd_tick(p_ctx, motion_sm_state(&p_ctx->sm));
    if (out.b_changed)
    {
        man_start(p_ctx, motion_sm_state(&p_ctx->sm), out.b_resumed);
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

    if (out.b_changed || (0u != out.replies))
    {
        report_fill(p_ctx, &rep, REP_EVENT, (uint8_t)EVT_STATE);
        rep.val_l  = (int32_t)motion_sm_state(&p_ctx->sm);
        rep.val_r  = (int32_t)before;
        rep.rate_l = (int32_t)out.replies;
        report_send(p_ctx, &rep);
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
   passed, so a car that cannot run stays braked instead of rebooting
   over and over. */
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
   means the cyclic handler stopped: brake.  This timeout runs on the
   same kernel timer as the tick, so if that timer stops (core 0 stalled)
   it never fires.  The watchdog covers that case: no tick, no feed. */
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
        /* Without encoders or PWM nothing here is safe to run: stay
           braked and do nothing else (the watchdog is not armed). */
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
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

static void
print_faults (UB *p_line, motion_report_t const *p_rep)
{
    uint8_t faults = p_rep->faults;

    (void)tm_sprintf(
        p_line,
        (UB *)"[motion] FAULT 0x%02x:%s%s%s%s%s%s  motors braked."
              " Fix, then press the button to run again.",
        (UINT)faults,
        ((faults & MOTION_FAULT_STALL_L) != 0u) ? " left wheel stalled" : "",
        ((faults & MOTION_FAULT_STALL_R) != 0u) ? " right wheel stalled" : "",
        ((faults & MOTION_FAULT_ENC_L) != 0u) ? " left encoder fault" : "",
        ((faults & MOTION_FAULT_ENC_R) != 0u) ? " right encoder fault" : "",
        ((faults & MOTION_FAULT_TICK) != 0u) ? " tick lost" : "",
        ((faults & MOTION_FAULT_INIT) != 0u) ? " init failed" : "");
    print_line(p_line);

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
        "IN_LOOK", "IN_BACK", "ARC_LEG", "ARC", "DONE"
    };
    uint32_t    ph   = (uint32_t)p_rep->val_l;
    char const *name = (ph <= BYP_PH_DONE) ? names[ph] : "?";

    (void)tm_sprintf(p_line,
                     (UB *)"[motion] bypass %s: %d mm out, heading %+d deg"
                           " to the line, %s",
                     name, (INT)p_rep->val_r, (INT)p_rep->rate_l,
                     (0 != p_rep->rate_r) ? "IMU" : "encoders only");
    print_line(p_line);
}
#endif

/* EVT_CMD, e.g. "[motion] nav_cmd FAULT_CLEAR(IMU): taken". */
static void
print_cmd (UB *p_line, motion_report_t const *p_rep)
{
    uint32_t    raw = (uint32_t)p_rep->val_r;
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
    res = (CMD_RES_TAKEN == p_rep->rate_l) ? "taken" : "dropped (invalid)";

    if (CMD_SRC_NAV == p_rep->val_l)
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
                                   " between feeds %d us, resets at %u us",
                             (INT)p_rep->rate_l,
                             (UINT)(MOTION_WDT_MS * US_PER_MS));
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
        obj_id = tk_cre_cyc(&g_ccyc);
        ercd   = (obj_id < E_OK) ? obj_id : E_OK;
    }

    return ercd;
}
