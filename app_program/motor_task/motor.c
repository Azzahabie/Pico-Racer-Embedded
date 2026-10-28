/*
 *  motor.c
 *  Two DC motors on the Robo Pico H-bridge, driven by RP2040 PWM.
 *
 *  Robo Pico truth table per motor: A high / B low = one direction,
 *  A low / B high = the other, both low = brake.  GP8/GP9 are PWM slice 4
 *  channels A/B and GP10/GP11 are slice 5 channels A/B, so each motor is
 *  one slice and its CC register holds both inputs (A in bits 15:0, B in
 *  bits 31:16).  Writing CC once therefore updates both inputs together.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "motion_cfg.h"
#include "motion_lib.h"
#include "motor.h"

/* RP2040 PWM register layout (datasheet 4.5.3). */
#define PWM_SLICE_OF(pin)  (((pin) >> 1) & 0x7u)
#define PWM_SLICE_STRIDE   (0x14u)
#define PWM_DIV_ONE        (1u << 4) /* DIV is 8.4 fixed point */
#define PWM_CC_B_SHIFT     (16u)
#define PWM_PINS_PER_SLICE (2u) /* channel A, then channel B */
#define PWM_CC_MAX         (0xFFFFu)

/* Bounded wait for the reset controller (a few cycles in practice). */
#define RESET_SPIN_LIMIT (100000u)

/* Each motor's A pin must be the even (channel A) pin of a slice and its
   B pin the odd pin of the same slice, or the single-store update and the
   pin map disagree. */
_Static_assert((MOTION_PIN_M1A % 2u) == 0u, "M1A must be a PWM channel A pin");
_Static_assert(MOTION_PIN_M1B == (MOTION_PIN_M1A + 1u),
               "M1B must pair with M1A");
_Static_assert((MOTION_PIN_M2A % 2u) == 0u, "M2A must be a PWM channel A pin");
_Static_assert(MOTION_PIN_M2B == (MOTION_PIN_M2A + 1u),
               "M2B must pair with M2A");
_Static_assert((MOTION_PWM_TOP + 1u) <= PWM_CC_MAX,
               "PWM_TOP too large for 16-bit CC");

typedef struct
{
    uint32_t slice;
    uint32_t pin_a;
    bool     b_invert;
    int32_t  duty; /* last request, before inversion */
} motor_t;

static motor_t g_motor[MOTOR_COUNT] = {
    { PWM_SLICE_OF(MOTION_PIN_M1A), MOTION_PIN_M1A,
      (MOTION_MOTOR_L_INVERT != 0), 0 },
    { PWM_SLICE_OF(MOTION_PIN_M2A), MOTION_PIN_M2A,
      (MOTION_MOTOR_R_INVERT != 0), 0 }
};

/* Address of one register of one PWM slice. */
static uint32_t
pwm_reg (uint32_t slice, uint32_t offset)
{
    return PWM_BASE + (slice * PWM_SLICE_STRIDE) + offset;
}

static void
motor_write_cc (motor_t const *p_motor, uint32_t cc)
{
    out_w(pwm_reg(p_motor->slice, PWM_CHx_CC), cc);
}

ER
motor_init (void)
{
    ER       ercd = E_OK;
    uint32_t spin;
    uint32_t idx;
    uint32_t pin;

    /* hw_setting.c only releases the PWM block from reset when the
       physical-timer option (USE_PTMR) is on, and config.h has it off.
       Without this the slice registers ignore every write.  The atomic
       clear alias leaves every other peripheral's reset bit alone. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    for (spin = 0u;
         (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) && (E_OK == ercd);
         spin++)
    {
        if (spin >= RESET_SPIN_LIMIT)
        {
            ercd = E_TMOUT;
        }
    }

    for (idx = 0u; (idx < (uint32_t)MOTOR_COUNT) && (E_OK == ercd); idx++)
    {
        motor_t *p_motor = &g_motor[idx];

        out_w(pwm_reg(p_motor->slice, PWM_CHx_CSR), 0u); /* stop */
        out_w(pwm_reg(p_motor->slice, PWM_CHx_DIV), PWM_DIV_ONE);
        out_w(pwm_reg(p_motor->slice, PWM_CHx_TOP), MOTION_PWM_TOP);
        motor_write_cc(p_motor, 0u); /* brake */
        out_w(pwm_reg(p_motor->slice, PWM_CHx_CTR), 0u);
        p_motor->duty = 0;

        /* Pads: no pulls (the sample I2C driver set a pull-up on
           GP8/GP9 at boot), then route both pins to the PWM. */
        for (pin = p_motor->pin_a; pin < (p_motor->pin_a + PWM_PINS_PER_SLICE);
             pin++)
        {
            clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
            (void)pwm_set_pin(pin);
        }
    }

    if (E_OK == ercd)
    {
        for (idx = 0u; idx < (uint32_t)MOTOR_COUNT; idx++)
        {
            set_w(pwm_reg(g_motor[idx].slice, PWM_CHx_CSR), PWM_CH_CSR_EN);
        }
    }

    return ercd;
}

void
motor_set (motor_side_t side, int32_t duty_pm)
{
    motor_t *p_motor;
    int32_t  duty;
    uint32_t level;
    uint32_t cc;

    if ((uint32_t)side < (uint32_t)MOTOR_COUNT)
    {
        p_motor = &g_motor[side];

        duty = motion_clamp(duty_pm, -MOTION_DUTY_FULL, MOTION_DUTY_FULL);
        p_motor->duty = duty;

        if (p_motor->b_invert)
        {
            duty = -duty;
        }

        /* 0..1000 per-mille -> 0..TOP+1 counts (TOP+1 = always high). */
        level = ((uint32_t)motion_abs(duty) * (MOTION_PWM_TOP + 1u))
                / (uint32_t)MOTION_DUTY_FULL;

        if (duty > 0)
        {
            cc = level; /* A = PWM, B = low */
        }
        else if (duty < 0)
        {
            cc = level << PWM_CC_B_SHIFT; /* A = low, B = PWM */
        }
        else
        {
            cc = 0u; /* both low: brake  */
        }

        motor_write_cc(p_motor, cc);
    }
}

void
motor_brake_all (void)
{
    uint32_t idx;

    for (idx = 0u; idx < (uint32_t)MOTOR_COUNT; idx++)
    {
        motor_write_cc(&g_motor[idx], 0u);
        g_motor[idx].duty = 0;
    }
}

int32_t
motor_get_duty (motor_side_t side)
{
    int32_t duty = 0;

    if ((uint32_t)side < (uint32_t)MOTOR_COUNT)
    {
        duty = g_motor[side].duty;
    }

    return duty;
}
