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

/* Per-phase edge counts: touched only by the interrupt once
   encoder_init() ends. */
static uint16_t g_edges_la;
static uint16_t g_edges_lb;
static uint16_t g_edges_ra;
static uint16_t g_edges_rb;

/* Published words: single writer (the interrupt), single 32-bit store. */
static volatile uint32_t g_packed;
static volatile uint32_t g_invalid_packed;
static volatile uint32_t g_edges_l;
static volatile uint32_t g_edges_r;

static bool gb_left_on = false;

/* One more edge on a phase if its pin is in this interrupt's pin set. */
static uint16_t
encoder_count_edge (uint16_t edges, uint32_t pins, uint32_t pin)
{
    uint16_t result = edges;

    if (0u != (pins & (1u << pin)))
    {
        result = (uint16_t)(edges + 1u);
    }

    return result;
}

static uint8_t
encoder_ab (uint32_t gpio_in, uint32_t pin_a, uint32_t pin_b, bool b_swap)
{
    return motion_quad_ab(gpio_in & (1u << pin_a), gpio_in & (1u << pin_b),
                          b_swap);
}

/* GPIO dispatcher callback, interrupt context.  Decoding both wheels from
   one snapshot is cheaper than working out which pin moved.  pins (the
   pins that had an edge) only feeds the per-phase edge counts. */
static void
encoder_edge_isr (uint32_t pins, uint32_t gpio_in)
{
    if (gb_left_on)
    {
        (void)motion_quad_step(&g_quad_l,
                               encoder_ab(gpio_in, MOTION_PIN_ENC_L_A,
                                          MOTION_PIN_ENC_L_B,
                                          (MOTION_ENC_L_INVERT != 0)));
        g_edges_la = encoder_count_edge(g_edges_la, pins, MOTION_PIN_ENC_L_A);
        g_edges_lb = encoder_count_edge(g_edges_lb, pins, MOTION_PIN_ENC_L_B);
    }
    (void)motion_quad_step(&g_quad_r, encoder_ab(gpio_in, MOTION_PIN_ENC_R_A,
                                                 MOTION_PIN_ENC_R_B,
                                                 (MOTION_ENC_R_INVERT != 0)));
    g_edges_ra = encoder_count_edge(g_edges_ra, pins, MOTION_PIN_ENC_R_A);
    g_edges_rb = encoder_count_edge(g_edges_rb, pins, MOTION_PIN_ENC_R_B);

    g_packed         = motion_pack_counts(g_quad_l.count, g_quad_r.count);
    g_invalid_packed = motion_pack_counts(g_quad_l.invalid, g_quad_r.invalid);
    g_edges_l        = motion_pack_counts(g_edges_la, g_edges_lb);
    g_edges_r        = motion_pack_counts(g_edges_ra, g_edges_rb);
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
    g_edges_la       = 0u;
    g_edges_lb       = 0u;
    g_edges_ra       = 0u;
    g_edges_rb       = 0u;
    g_edges_l        = 0u;
    g_edges_r        = 0u;
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

uint32_t
encoder_read_edges_left (void)
{
    return g_edges_l;
}

uint32_t
encoder_read_edges_right (void)
{
    return g_edges_r;
}
