/*
 *  motor_task.c
 *  motion_task (Buddy 2), bench bring-up version.  See motor_task.h.
 *
 *  Two tasks:
 *
 *    motion_task  core 1, priority 11.  Woken every 10 ms by a cyclic
 *                 handler through an event flag.  Reads both encoder
 *                 counts with one load, runs the bench test and the stall
 *                 check, writes the motors.  Never prints and never waits
 *                 on anything except its event flag.
 *
 *    motion_report  core 0, low priority.  Receives fixed-size records
 *                 from motion_task through a message buffer and prints
 *                 them.  tm_printf masks interrupts on the calling core
 *                 for the whole line in the USB-CDC build, which on core 1
 *                 would delay the encoder interrupt and lose counts.
 *
 *  Bench test (wheels off the ground): press the Robo Pico button on
 *  MOTION_PIN_BUTTON to start, press again to abort.  Without a press the
 *  motors stay braked; turning a wheel by hand still prints its count.
 *  Steps and how to read the output: docs/motion/README.md.
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
#include "motion_cfg.h"
#include "motion_lib.h"
#include "motor.h"
#include "encoder.h"
#include "motor_task.h"

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
#define BUTTON_DEBOUNCE_TICKS (3u) /* 30 ms                   */
#define MS_PER_S              (1000)
#define CROSSTALK_RATIO       (4) /* other wheel > 1/4 of driven */
#define BUTTON_SETTLE_MS      (2) /* new pull-up lifts the pin   */
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
    PHASE_PID
} motion_phase_t;

/* ------------------------------------------------------------------ *
 *  Report records (motion_task -> motion_report)
 * ------------------------------------------------------------------ */

typedef enum
{
    REP_STATUS = 0, /* val = position, rate = counts/s        */
    REP_STEP,       /* val = moved in the step, rate = c/s    */
    REP_EVENT,      /* code = EVT_*, val/rate = arguments     */
    REP_TIMING      /* timing of the last 5 s                 */
} report_kind_t;

typedef enum
{
    EVT_READY = 0, /* val_l = init ercd, val_r = processor    */
    EVT_START,     /* val_l = PID target in counts/s (or 0)   */
    EVT_DONE,
    EVT_ABORT, /* button                                  */
    EVT_FAULT, /* faults byte says which                  */
    EVT_RAMP   /* val = first duty that moved, 0 = none   */
} report_event_t;

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
    int32_t  pos_r;
    int32_t  d_l;
    int32_t  d_r;

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

    /* button */
    bool     b_btn_raw;
    bool     b_btn_down;
    uint32_t btn_same;

    /* timing */
    uint32_t ticks;
    uint32_t last_us;
    uint32_t per_min_us;
    uint32_t per_max_us;
    uint32_t work_max_us;
    uint32_t lost_ticks;
    uint32_t irq_mark;

    /* reporting */
    int32_t  rep_l;
    int32_t  rep_r;
    uint32_t rep_ticks;
    uint32_t rep_dropped;
} motion_ctx_t;

static motion_ctx_t g_ctx;

/* ------------------------------------------------------------------ *
 *  Kernel objects
 * ------------------------------------------------------------------ */

static ID g_flgid;
static ID g_mbfid;

static UW g_report_buf[(REPORT_QUEUE_LEN
                        * (sizeof(motion_report_t) + MBF_HEADER_BYTES))
                       / sizeof(UW)];

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

static T_CCYC const g_ccyc = {
    .cycatr = TA_HLNG | TA_STA,
    .cychdr = (FP)motion_cyclic_isr,
    .cyctim = MOTION_TICK_MS,
    .cycphs = MOTION_TICK_MS,
};

static T_CTSK const g_ctsk_motion = {
    .itskpri = MOTION_TASK_PRI,
    .stksz   = MOTION_TASK_STACK,
    .task    = (FP)motor_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC2, /* core 1 */
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

static T_CTSK const g_ctsk_report = {
    .itskpri = REPORT_TASK_PRI,
    .stksz   = REPORT_TASK_STACK,
    .task    = (FP)motion_report_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1, /* core 0 */
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

static uint32_t
now_us (void)
{
    return in_w(TIMER_RAWL);
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
    p_rep->irq_count   = gpio_irq_count();
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
 *  Button (polled, debounced, active low)
 * ------------------------------------------------------------------ */

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

    report_send(p_ctx, &rep);
}

static void
bench_start (motion_ctx_t *p_ctx)
{
    p_ctx->faults = 0u;
    p_ctx->step   = 0u;
    motion_pid_reset(&p_ctx->pid_l);
    motion_pid_reset(&p_ctx->pid_r);
    bench_mark(p_ctx);

#if MOTION_TEST_PID
    p_ctx->phase = PHASE_PID;
    report_event(p_ctx, EVT_START, rate_cps(MOTION_PID_TARGET, 1u), 0);
#else
    p_ctx->phase = PHASE_STEPS;
    report_event(p_ctx, EVT_START, 0, 0);
#endif
}

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
        motion_pid_step(&p_ctx->pid_l, MOTION_PID_TARGET, p_ctx->d_l);
    p_ctx->cmd_r =
        motion_pid_step(&p_ctx->pid_r, MOTION_PID_TARGET, p_ctx->d_r);
    p_ctx->step_ticks++;

    if (p_ctx->step_ticks >= MOTION_PID_TICKS)
    {
        bench_stop(p_ctx, EVT_DONE);
    }
}

/* FR-30.  Judged on the duty that was applied during the tick just
   measured.  Open loop counts movement either way (polarity is not known
   yet); closed loop counts only movement in the commanded direction, so
   a wrong INVERT flag also ends as a stall instead of a runaway. */
static bool
stall_wheel (motion_stall_t *p_stall, motor_side_t side, int32_t delta,
             bool b_directional)
{
    int32_t duty     = motor_get_duty(side);
    bool    b_driven = (motion_abs(duty) >= MOTION_DEADBAND_DUTY);
    int32_t progress = motion_abs(delta);

    if (b_directional)
    {
        progress = (duty >= 0) ? delta : -delta;
    }

    return motion_stall_update(p_stall, b_driven, progress);
}

static void
stall_check (motion_ctx_t *p_ctx)
{
    bool b_dir = (PHASE_PID == p_ctx->phase);

    if ((PHASE_STEPS == p_ctx->phase) || (PHASE_PID == p_ctx->phase))
    {
        if (encoder_left_enabled()
            && stall_wheel(&p_ctx->stall_l, MOTOR_LEFT, p_ctx->d_l, b_dir))
        {
            p_ctx->faults |= MOTION_FAULT_STALL_L;
        }
        if (stall_wheel(&p_ctx->stall_r, MOTOR_RIGHT, p_ctx->d_r, b_dir))
        {
            p_ctx->faults |= MOTION_FAULT_STALL_R;
        }
        if (p_ctx->faults != 0u)
        {
            bench_stop(p_ctx, EVT_FAULT);
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

static void
tick_encoders (motion_ctx_t *p_ctx)
{
    uint32_t packed = encoder_read_packed(); /* both, one load */
    uint16_t now_l  = motion_unpack_left(packed);
    uint16_t now_r  = motion_unpack_right(packed);

    p_ctx->d_l    = motion_count_delta(now_l, p_ctx->prev_l);
    p_ctx->d_r    = motion_count_delta(now_r, p_ctx->prev_r);
    p_ctx->prev_l = now_l;
    p_ctx->prev_r = now_r;
    p_ctx->pos_l += p_ctx->d_l;
    p_ctx->pos_r += p_ctx->d_r;
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
        irq_now = gpio_irq_count();
        report_fill(p_ctx, &rep, REP_TIMING, 0u);
        rep.val_l      = (int32_t)p_ctx->lost_ticks;
        rep.val_r      = (int32_t)p_ctx->rep_dropped;
        rep.irq_count  = irq_now - p_ctx->irq_mark;
        rep.isr_max_us = gpio_irq_take_max_us();
        report_send(p_ctx, &rep);

        p_ctx->irq_mark    = irq_now;
        p_ctx->per_min_us  = UINT32_MAX;
        p_ctx->per_max_us  = 0u;
        p_ctx->work_max_us = 0u;
    }
}

static void
motion_tick (motion_ctx_t *p_ctx)
{
    uint32_t t_start = now_us();
    uint32_t work;

    tick_timing(p_ctx, t_start);
    tick_encoders(p_ctx);

    if (button_pressed(p_ctx))
    {
        if (PHASE_IDLE == p_ctx->phase)
        {
            bench_start(p_ctx);
        }
        else
        {
            bench_stop(p_ctx, EVT_ABORT);
        }
    }

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

        default:
            p_ctx->cmd_l = 0;
            p_ctx->cmd_r = 0;
            break;
    }

    motor_set(MOTOR_LEFT, p_ctx->cmd_l);
    motor_set(MOTOR_RIGHT, p_ctx->cmd_r);

    tick_report(p_ctx);

    work = now_us() - t_start;
    if (work > p_ctx->work_max_us)
    {
        p_ctx->work_max_us = work;
    }
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

/* Hardware and state set-up.  Motors first, so they are braked before
   anything else happens.  Returns the first error. */
static ER
motion_init (motion_ctx_t *p_ctx)
{
    ER       ercd;
    uint32_t packed;

    ercd = motor_init();
    if (E_OK == ercd)
    {
        ercd = gpio_irq_init(INTPRI_MAX_EXTINT_PRI);
    }
    if (E_OK == ercd)
    {
        ercd = encoder_init();
    }
    button_init(p_ctx);

    motion_pid_init(&p_ctx->pid_l, MOTION_PID_KP_Q8, MOTION_PID_KI_Q8,
                    MOTION_PID_KD_Q8, MOTION_PID_OUT_LIMIT);
    motion_pid_init(&p_ctx->pid_r, MOTION_PID_KP_Q8, MOTION_PID_KI_Q8,
                    MOTION_PID_KD_Q8, MOTION_PID_OUT_LIMIT);
    motion_stall_init(&p_ctx->stall_l, MOTION_STALL_TICKS,
                      MOTION_STALL_MIN_COUNTS);
    motion_stall_init(&p_ctx->stall_r, MOTION_STALL_TICKS,
                      MOTION_STALL_MIN_COUNTS);

    packed            = encoder_read_packed();
    p_ctx->prev_l     = motion_unpack_left(packed);
    p_ctx->prev_r     = motion_unpack_right(packed);
    p_ctx->per_min_us = UINT32_MAX;
    p_ctx->phase      = PHASE_IDLE;

    if (ercd != E_OK)
    {
        p_ctx->faults |= MOTION_FAULT_INIT;
        motor_brake_all();
    }

    return ercd;
}

/* Wait for one tick and run it.  No tick within MOTION_TICK_TIMEOUT_MS
   means the cyclic handler or kernel timer stopped: brake. */
static void
motion_cycle (motion_ctx_t *p_ctx)
{
    ER   ercd;
    UINT ptn;

    ercd = tk_wai_flg(g_flgid, MOTION_EVT_TICK, TWF_ORW | TWF_BITCLR, &ptn,
                      MOTION_TICK_TIMEOUT_MS);
    if (E_OK == ercd)
    {
        motion_tick(p_ctx);
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

    ercd = motion_init(p_ctx);

#if TK_SUPPORT_SMP
    prc = tk_get_prc();
#endif
    report_event(p_ctx, EVT_READY, ercd, prc);

    if (ercd != E_OK)
    {
        /* Without encoders or PWM nothing here is safe to run: stay
           braked and do nothing else. */
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }

    /* Start the period measurement from a fresh tick. */
    (void)tk_clr_flg(g_flgid, 0u);

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
        (UB *)"[motion] FAULT 0x%02x:%s%s%s%s  motors braked."
              " Fix, then press the button to run again.",
        (UINT)faults,
        ((faults & MOTION_FAULT_STALL_L) != 0u) ? " left wheel stalled" : "",
        ((faults & MOTION_FAULT_STALL_R) != 0u) ? " right wheel stalled" : "",
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

    (void)tm_sprintf(p_line,
                     (UB *)"[motion] ready on processor %d, init %s (er=%d)."
                           " PWM %u Hz on GP%u-GP%u, encoders GP%u-GP%u%s,"
                           " button GP%u, test=%s",
                     (INT)p_rep->val_r, b_init_ok ? "OK" : "FAILED",
                     (INT)p_rep->val_l, (UINT)MOTION_PWM_HZ,
                     (UINT)MOTION_PIN_M1A, (UINT)MOTION_PIN_M2B,
                     (UINT)MOTION_PIN_ENC_L_A, (UINT)MOTION_PIN_ENC_R_B,
                     encoder_left_enabled() ? "" : " (LEFT OFF: UART console)",
                     (UINT)MOTION_PIN_BUTTON,
                     (0 != MOTION_TEST_PID) ? "closed-loop" : "open-loop");
    print_line(p_line);

    if (b_init_ok)
    {
        print_line((UB *)"[motion] wheels OFF the ground, then press the"
                         " button to start. Turn a wheel by hand to see"
                         " its count.");
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

        case EVT_START:
            if (0 != p_rep->val_l)
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
        obj_id = tk_cre_tsk(&g_ctsk_report);
        ercd   = (obj_id < E_OK) ? obj_id : tk_sta_tsk(obj_id, 0);
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_tsk(&g_ctsk_motion);
        ercd   = (obj_id < E_OK) ? obj_id : tk_sta_tsk(obj_id, 0);
    }

    if (E_OK == ercd)
    {
        obj_id = tk_cre_cyc(&g_ccyc);
        ercd   = (obj_id < E_OK) ? obj_id : E_OK;
    }

    return ercd;
}
