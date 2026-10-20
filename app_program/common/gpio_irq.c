/*
 *  gpio_irq.c
 *  The one owner of the RP2040 GPIO bank interrupt.  See gpio_irq.h.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "gpio_irq.h"

/* RP2040 datasheet 2.3.2 (IRQ numbers) and 2.19.6.1 (IO_BANK0). */
#define GPIO_IRQ_INTNO  (13u) /* IO_IRQ_BANK0 */
#define GPIO_PIN_COUNT  (30u)
#define GPIO_VALID_MASK ((1u << GPIO_PIN_COUNT) - 1u)
#define IO_REG_COUNT    (4u) /* 8 pins per register   */
#define IO_PINS_PER_REG (8u)
#define IO_BITS_PER_PIN (4u) /* LOW, HIGH, FALL, RISE */
#define IO_PIN_EVENTS   (0xFu)
#define CORE_COUNT      (2u)
#define CORE_ID_MASK    (1u)

#define TIMER_RAWL (TIMER_BASE + 0x28u) /* 1 MHz, no side effects */

#define IO_INTR0       (IO_BANK0_BASE + 0x0F0u)
#define IO_PROC0_INTE0 (IO_BANK0_BASE + 0x100u)
#define IO_PROC0_INTS0 (IO_BANK0_BASE + 0x120u)
#define IO_PROC1_INTE0 (IO_BANK0_BASE + 0x130u)
#define IO_PROC1_INTS0 (IO_BANK0_BASE + 0x150u)
#define IO_REG_STRIDE  (4u)

typedef struct
{
    uint32_t      pin_mask;
    gpio_irq_cb_t p_callback;
} gpio_client_t;

/* Everything one core's interrupt needs.  Each core only ever touches
   its own entry, from its own tasks and its own handler. */
typedef struct
{
    gpio_client_t     clients[GPIO_IRQ_MAX_CLIENTS];
    volatile uint32_t client_count;
    volatile uint32_t reg_in_use; /* bit r: register r has pins */
    volatile uint32_t irq_count;
    volatile uint32_t irq_max_us; /* longest handler run */
    bool              b_ready;
} gpio_core_t;

static gpio_core_t g_cores[CORE_COUNT];
static uint32_t    g_claimed_pins; /* every core's pins */
static bool        gb_vector_set = false;

static uint32_t
this_core (void)
{
    return in_w(SIO_CPUID_ADDR) & CORE_ID_MASK;
}

static uint32_t
intr_reg (uint32_t reg)
{
    return IO_INTR0 + (IO_REG_STRIDE * reg);
}

static uint32_t
inte_reg (uint32_t core, uint32_t reg)
{
    uint32_t base = (0u != core) ? IO_PROC1_INTE0 : IO_PROC0_INTE0;

    return base + (IO_REG_STRIDE * reg);
}

static uint32_t
ints_reg (uint32_t core, uint32_t reg)
{
    uint32_t base = (0u != core) ? IO_PROC1_INTS0 : IO_PROC0_INTS0;

    return base + (IO_REG_STRIDE * reg);
}

/* Interrupt handler, shared by both cores (one vector slot).  Each core
   sees only the pins enabled in its own INTE registers.  Acknowledge
   first, then read the pins: an edge that lands after the read raises
   the interrupt again, so none is lost. */
static void
gpio_irq_isr (UINT intno)
{
    uint32_t     t_start = in_w(TIMER_RAWL);
    uint32_t     core    = this_core();
    gpio_core_t *p_core  = &g_cores[core];
    uint32_t     pins    = 0u;
    uint32_t     reg;
    uint32_t     slot;
    uint32_t     ints;
    uint32_t     gpio_in;
    uint32_t     count;
    uint32_t     idx;
    uint32_t     t_used;

    (void)intno;
    p_core->irq_count = p_core->irq_count + 1u;

    for (reg = 0u; reg < IO_REG_COUNT; reg++)
    {
        if ((p_core->reg_in_use & (1u << reg)) != 0u)
        {
            ints = in_w(ints_reg(core, reg));
            if (ints != 0u)
            {
                out_w(intr_reg(reg), ints); /* clear edges */
                for (slot = 0u; slot < IO_PINS_PER_REG; slot++)
                {
                    if (((ints >> (slot * IO_BITS_PER_PIN)) & IO_PIN_EVENTS)
                        != 0u)
                    {
                        pins |= 1u << ((reg * IO_PINS_PER_REG) + slot);
                    }
                }
            }
        }
    }

    gpio_in = in_w(GPIO_IN);
    count   = p_core->client_count;

    for (idx = 0u; idx < count; idx++)
    {
        if ((pins & p_core->clients[idx].pin_mask) != 0u)
        {
            p_core->clients[idx].p_callback(
                pins & p_core->clients[idx].pin_mask, gpio_in);
        }
    }

    t_used = in_w(TIMER_RAWL) - t_start;
    if (t_used > p_core->irq_max_us)
    {
        p_core->irq_max_us = t_used;
    }
}

ER
gpio_irq_init (INT level)
{
    ER       ercd = E_OK;
    T_DINT   dint;
    uint32_t core = this_core();
    uint32_t reg;

    if (g_cores[core].b_ready)
    {
        ercd = E_OBJ;
    }
    else
    {
        /* This core's enables belong to the dispatcher from now on. */
        for (reg = 0u; reg < IO_REG_COUNT; reg++)
        {
            out_w(inte_reg(core, reg), 0u);
        }

        /* One vector slot for both cores: define the handler once.
           (Defining it again from the other core would install the
           same function, so a start-up race is harmless.) */
        if (!gb_vector_set)
        {
            dint.intatr = TA_HLNG;
            dint.inthdr = (FP)gpio_irq_isr;
            ercd        = tk_def_int(GPIO_IRQ_INTNO, &dint);
            if (E_OK == ercd)
            {
                gb_vector_set = true;
            }
        }

        if (E_OK == ercd)
        {
            /* The NVIC is per core: this enables the interrupt on
               the calling core only. */
            ClearInt(GPIO_IRQ_INTNO);
            EnableInt(GPIO_IRQ_INTNO, level);
            g_cores[core].b_ready = true;
        }
    }

    return ercd;
}

ER
gpio_irq_attach (uint32_t pin_mask, uint32_t edges, gpio_irq_cb_t p_callback)
{
    ER           ercd = E_OK;
    UINT         intsts;
    uint32_t     core   = this_core();
    gpio_core_t *p_core = &g_cores[core];
    uint32_t     pin;
    uint32_t     reg;
    uint32_t     bits;

    if (!p_core->b_ready)
    {
        ercd = E_OBJ;
    }
    else if ((NULL == p_callback) || (0u == pin_mask)
             || ((pin_mask & ~GPIO_VALID_MASK) != 0u) || (0u == edges)
             || ((edges & ~GPIO_IRQ_EDGE_BOTH) != 0u))
    {
        ercd = E_PAR;
    }
    else
    {
        DI(intsts);

        if (p_core->client_count >= GPIO_IRQ_MAX_CLIENTS)
        {
            ercd = E_LIMIT;
        }
        else if ((g_claimed_pins & pin_mask) != 0u)
        {
            ercd = E_OBJ; /* pin already has an owner */
        }
        else
        {
            /* Complete the entry before publishing the count. */
            p_core->clients[p_core->client_count].pin_mask   = pin_mask;
            p_core->clients[p_core->client_count].p_callback = p_callback;
            __asm__ volatile("" ::: "memory"); /* keep order */
            p_core->client_count = p_core->client_count + 1u;
            g_claimed_pins |= pin_mask;

            for (pin = 0u; pin < GPIO_PIN_COUNT; pin++)
            {
                if ((pin_mask & (1u << pin)) != 0u)
                {
                    reg  = pin / IO_PINS_PER_REG;
                    bits = edges << ((pin % IO_PINS_PER_REG) * IO_BITS_PER_PIN);
                    out_w(intr_reg(reg), bits);       /* stale edges */
                    set_w(inte_reg(core, reg), bits); /* atomic */
                    p_core->reg_in_use = p_core->reg_in_use | (1u << reg);
                }
            }
        }

        EI(intsts);
    }

    return ercd;
}

uint32_t
gpio_irq_count (void)
{
    return g_cores[this_core()].irq_count;
}

uint32_t
gpio_irq_take_max_us (void)
{
    gpio_core_t *p_core = &g_cores[this_core()];
    UINT         intsts;
    uint32_t     max_us;

    DI(intsts);
    max_us             = p_core->irq_max_us;
    p_core->irq_max_us = 0u;
    EI(intsts);

    return max_us;
}
