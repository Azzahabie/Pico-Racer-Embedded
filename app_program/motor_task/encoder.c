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

/* Both edges on every encoder pin (Buddy 5's gpio_irq.h has no EDGE_BOTH). */
#define ENC_EDGES (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

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

/* Timing evidence, written only by the interrupt: interrupts that reached
   the encoder callback, and the longest time from the dispatcher's entry
   timestamp to the end of the callback. */
static uint32_t          g_irq_last_us;
static volatile uint32_t g_irq_count;
static volatile uint32_t g_irq_max_us;

static bool gb_left_on = false;

/* One more edge on a phase if this callback is for its pin. */
static uint16_t
encoder_count_edge (uint16_t edges, UINT pin, uint32_t phase_pin)
{
    uint16_t result = edges;

    if (phase_pin == pin)
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

/* Buddy 5's GPIO dispatcher calls this once per encoder pin that had an
   edge, in interrupt context on core 1, after it has cleared that pin's
   edge bits, so reading GPIO_IN here sees every later change.  Only the
   wheel that owns the pin is decoded; if both of its pins changed in one
   interrupt, the first call counts one missed edge and the second sees no
   change. */
static void
encoder_edge_isr (UINT pin, UW events, UW now_us)
{
    uint32_t gpio_in = in_w(GPIO_IN);
    uint32_t run_us;

    /* Not needed: the level read above gives this pin's net change since
       the last callback.  That is exact while the wheel's other pin has
       not changed; a burst of three or more transitions across both pins
       before the interrupt runs can decode as a smaller step. */
    (void)events;

    if ((MOTION_PIN_ENC_L_A == pin) || (MOTION_PIN_ENC_L_B == pin))
    {
        (void)motion_quad_step(&g_quad_l,
                               encoder_ab(gpio_in, MOTION_PIN_ENC_L_A,
                                          MOTION_PIN_ENC_L_B,
                                          (MOTION_ENC_L_INVERT != 0)));
        g_edges_la = encoder_count_edge(g_edges_la, pin, MOTION_PIN_ENC_L_A);
        g_edges_lb = encoder_count_edge(g_edges_lb, pin, MOTION_PIN_ENC_L_B);
    }
    else
    {
        (void)motion_quad_step(&g_quad_r,
                               encoder_ab(gpio_in, MOTION_PIN_ENC_R_A,
                                          MOTION_PIN_ENC_R_B,
                                          (MOTION_ENC_R_INVERT != 0)));
        g_edges_ra = encoder_count_edge(g_edges_ra, pin, MOTION_PIN_ENC_R_A);
        g_edges_rb = encoder_count_edge(g_edges_rb, pin, MOTION_PIN_ENC_R_B);
    }

    g_packed         = motion_pack_counts(g_quad_l.count, g_quad_r.count);
    g_invalid_packed = motion_pack_counts(g_quad_l.invalid, g_quad_r.invalid);
    g_edges_l        = motion_pack_counts(g_edges_la, g_edges_lb);
    g_edges_r        = motion_pack_counts(g_edges_ra, g_edges_rb);

    /* Callbacks from one interrupt share now_us: count interrupts. */
    if (now_us != g_irq_last_us)
    {
        g_irq_last_us = now_us;
        g_irq_count   = g_irq_count + 1u;
    }
    run_us = in_w(TIMER_TIMERAWL) - now_us;
    if (run_us > g_irq_max_us)
    {
        g_irq_max_us = run_us;
    }
}

/* Attach both pins of one wheel to the dispatcher. */
static ER
encoder_attach_pair (UINT pin_a, UINT pin_b)
{
    ER ercd = gpio_irq_attach(pin_a, ENC_EDGES, encoder_edge_isr);

    if (E_OK == ercd)
    {
        ercd = gpio_irq_attach(pin_b, ENC_EDGES, encoder_edge_isr);
    }

    return ercd;
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
    ER       ercd = E_OK;
    UINT     intsts;
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
    }
    encoder_input(MOTION_PIN_ENC_R_A);
    encoder_input(MOTION_PIN_ENC_R_B);
    (void)tk_dly_tsk(ENC_SETTLE_MS);

    /* Attach and take the starting state with interrupts off on this
       core, so the first edge is decoded against the right state. */
    DI(intsts);
    if (gb_left_on)
    {
        ercd = encoder_attach_pair(MOTION_PIN_ENC_L_A, MOTION_PIN_ENC_L_B);
    }
    if (E_OK == ercd)
    {
        ercd = encoder_attach_pair(MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B);
    }
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
    g_irq_last_us    = 0u;
    g_irq_count      = 0u;
    g_irq_max_us     = 0u;
    EI(intsts);

    return ercd;
}

uint32_t
encoder_irq_count (void)
{
    return g_irq_count;
}

uint32_t
encoder_take_irq_max_us (void)
{
    UINT     intsts;
    uint32_t max_us;

    DI(intsts);
    max_us       = g_irq_max_us;
    g_irq_max_us = 0u;
    EI(intsts);

    return max_us;
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
