/*
 *  encoder.c
 *  Both wheel encoders: quadrature, x4 decoding, GPIO edge interrupts.
 *  See encoder.h.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "../common/gpio_irq.h"
#include "motion_cfg.h"
#include "motion_lib.h"
#include "encoder.h"

#define ENC_L_PINS ((1u << MOTION_PIN_ENC_L_A) | (1u << MOTION_PIN_ENC_L_B))
#define ENC_R_PINS ((1u << MOTION_PIN_ENC_R_A) | (1u << MOTION_PIN_ENC_R_B))

/* Time for the new pull-ups to lift the inputs (pads reset with pull-down). */
#define ENC_SETTLE_MS (2)

/* Decoder state: touched only by the interrupt once encoder_init() ends. */
static motion_quad_t g_quad_l;
static motion_quad_t g_quad_r;

/* Published words: single writer (the interrupt), single 32-bit store. */
static volatile uint32_t g_packed;
static volatile uint32_t g_invalid_packed;

static bool gb_left_on = false;

static uint8_t
encoder_ab (uint32_t gpio_in, uint32_t pin_a, uint32_t pin_b, bool b_swap)
{
    return motion_quad_ab(gpio_in & (1u << pin_a), gpio_in & (1u << pin_b),
                          b_swap);
}

/* GPIO dispatcher callback, interrupt context.  Decoding both wheels from
   one snapshot is cheaper than working out which pin moved. */
static void
encoder_edge_isr (uint32_t pins, uint32_t gpio_in)
{
    (void)pins;

    if (gb_left_on)
    {
        (void)motion_quad_step(&g_quad_l,
                               encoder_ab(gpio_in, MOTION_PIN_ENC_L_A,
                                          MOTION_PIN_ENC_L_B,
                                          (MOTION_ENC_L_INVERT != 0)));
    }
    (void)motion_quad_step(&g_quad_r, encoder_ab(gpio_in, MOTION_PIN_ENC_R_A,
                                                 MOTION_PIN_ENC_R_B,
                                                 (MOTION_ENC_R_INVERT != 0)));

    g_packed         = motion_pack_counts(g_quad_l.count, g_quad_r.count);
    g_invalid_packed = motion_pack_counts(g_quad_l.invalid, g_quad_r.invalid);
}

static void
encoder_input (uint32_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN); /* SIO, output off, input on */
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE | GPIO_SHEMITT);
}

ER
encoder_init (void)
{
    ER       ercd;
    UINT     intsts;
    uint32_t pins = ENC_R_PINS;
    uint32_t gpio_in;

#if TM_CONSOLE_USB_CDC
    gb_left_on = true;
#else
    gb_left_on = false; /* GP0/GP1 are the UART console */
#endif

    if (gb_left_on)
    {
        encoder_input(MOTION_PIN_ENC_L_A);
        encoder_input(MOTION_PIN_ENC_L_B);
        pins |= ENC_L_PINS;
    }
    encoder_input(MOTION_PIN_ENC_R_A);
    encoder_input(MOTION_PIN_ENC_R_B);
    (void)tk_dly_tsk(ENC_SETTLE_MS);

    /* Attach and take the starting state with interrupts off on this
       core, so the first edge is decoded against the right state. */
    DI(intsts);
    ercd    = gpio_irq_attach(pins, GPIO_IRQ_EDGE_BOTH, encoder_edge_isr);
    gpio_in = in_w(GPIO_IN);
    motion_quad_init(&g_quad_l,
                     encoder_ab(gpio_in, MOTION_PIN_ENC_L_A, MOTION_PIN_ENC_L_B,
                                (MOTION_ENC_L_INVERT != 0)));
    motion_quad_init(&g_quad_r,
                     encoder_ab(gpio_in, MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B,
                                (MOTION_ENC_R_INVERT != 0)));
    g_packed         = 0u;
    g_invalid_packed = 0u;
    EI(intsts);

    return ercd;
}

uint32_t
encoder_read_packed (void)
{
    return g_packed;
}

uint32_t
encoder_invalid_packed (void)
{
    return g_invalid_packed;
}

bool
encoder_left_enabled (void)
{
    return gb_left_on;
}
