/*
 *  motion_wdt.c
 *  RP2040 hardware watchdog for motion_task.  See motion_wdt.h.
 *
 *  The port has no Pico SDK watchdog, so this follows the SDK's
 *  watchdog_enable() (pico-sdk 2.2.0, hardware_watchdog) register by
 *  register.  Three RP2040 details (datasheet 2.13 and 4.7):
 *    - PSM WDSEL resets to 0, and then a timeout resets nothing.  Like the
 *      SDK, select every block except the two oscillators.
 *    - The counter goes down twice per 1 us tick (erratum RP2040-E1), so
 *      LOAD is 2 x the timeout in microseconds.
 *    - REASON also says "watchdog" after the boot ROM's own reboot (after
 *      a UF2 copy).  Like the SDK, we write a marker to SCRATCH4 when we
 *      arm, which the boot ROM overwrites when it reboots.
 *  The 1 us tick comes from the WATCHDOG TICK register, which cpu_clock.c
 *  sets at boot.  Never write it here: it also drives the 1 MHz TIMER.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "motion_cfg.h"
#include "motion_wdt.h"

/* Registers (RP2040 datasheet 4.7 and 2.13). */
#define WDT_BASE       (0x40058000u)
#define WDT_CTRL       (WDT_BASE + 0x00u)
#define WDT_LOAD       (WDT_BASE + 0x04u)
#define WDT_REASON     (WDT_BASE + 0x08u)
#define WDT_SCRATCH(n) (WDT_BASE + 0x0Cu + (4u * (n)))
#define PSM_WDSEL      (0x40010000u + 0x08u)

#define WDT_CTRL_ENABLE    (1u << 30)
#define WDT_CTRL_PAUSE_ALL (0x7u << 24) /* PAUSE_DBG1, _DBG0, _JTAG */
#define WDT_REASON_TIMER   (1u << 0)
#define PSM_WDSEL_ALL      (0x0001FFFFu)
#define PSM_WDSEL_OSC      (0x00000003u) /* ROSC and XOSC */

/* SCRATCH4 marker while armed: the SDK's WATCHDOG_NON_REBOOT_MAGIC, so
   the boot ROM does not take it for a "jump to this address" request. */
#define WDT_ARMED_MARK  WDT_SCRATCH(4u)
#define WDT_ARMED_MAGIC (0x6AB73121u)

#define US_PER_MS         (1000u)
#define WDT_COUNTS_PER_US (2u) /* erratum RP2040-E1 */
#define WDT_LOAD_MAX      (0x00FFFFFFu)
#define WDT_LOAD_VALUE    (MOTION_WDT_MS * US_PER_MS * WDT_COUNTS_PER_US)

_Static_assert(WDT_LOAD_VALUE <= WDT_LOAD_MAX, "MOTION_WDT_MS too long");
_Static_assert(MOTION_WDT_MS >= (2u * MOTION_TICK_MS),
               "the watchdog must allow at least one late tick");

#if MOTION_BENCH
/* Watchdog test record in SCRATCH0..2, which only a power-off clears. */
#define WDT_HANG_MARK  WDT_SCRATCH(0u)
#define WDT_HANG_FED   WDT_SCRATCH(1u) /* 1 MHz time of the last feed   */
#define WDT_HANG_LAST  WDT_SCRATCH(2u) /* 1 MHz time, until the reset   */
#define WDT_HANG_MAGIC (0x48414E47u)   /* "HANG"                        */
#define WDT_TIMER_RAWL (TIMER_BASE + 0x28u)
#endif

motion_boot_t
motion_wdt_boot_cause (void)
{
    uint32_t      reason = in_w(WDT_REASON);
    motion_boot_t cause  = MOTION_BOOT_POWER_ON;

    if (((reason & WDT_REASON_TIMER) != 0u)
        && (WDT_ARMED_MAGIC == in_w(WDT_ARMED_MARK)))
    {
        cause = MOTION_BOOT_WATCHDOG;
    }
    else if (reason != 0u)
    {
        cause = MOTION_BOOT_REBOOT;
    }
    else
    {
        /* REASON is 0 after a power-on, brown-out or RUN pin reset. */
    }

    return cause;
}

/* The order matters: disarm while setting up, the marker and LOAD before
   the enable, the enable last. */
void
motion_wdt_start (void)
{
    clr_w(WDT_CTRL, WDT_CTRL_ENABLE);
    out_w(WDT_ARMED_MARK, WDT_ARMED_MAGIC);
    set_w(PSM_WDSEL, PSM_WDSEL_ALL & ~PSM_WDSEL_OSC);

    /* Hold the count while a debugger has a core halted.  The boot ROM's
       reboot clears these bits, so set them every time. */
    set_w(WDT_CTRL, WDT_CTRL_PAUSE_ALL);

    out_w(WDT_LOAD, WDT_LOAD_VALUE);
    set_w(WDT_CTRL, WDT_CTRL_ENABLE);
}

void
motion_wdt_feed (void)
{
    out_w(WDT_LOAD, WDT_LOAD_VALUE);
}

#if MOTION_BENCH

void
motion_wdt_hang (uint32_t fed_us)
{
    out_w(WDT_HANG_FED, fed_us);
    out_w(WDT_HANG_LAST, fed_us);
    out_w(WDT_HANG_MARK, WDT_HANG_MAGIC);

    for (;;)
    {
        out_w(WDT_HANG_LAST, in_w(WDT_TIMER_RAWL));
    }
}

bool
motion_wdt_take_hang_us (uint32_t *p_us)
{
    bool b_found = (WDT_HANG_MAGIC == in_w(WDT_HANG_MARK));

    if (b_found)
    {
        *p_us = in_w(WDT_HANG_LAST) - in_w(WDT_HANG_FED); /* modulo 2^32 */
        out_w(WDT_HANG_MARK, 0u);
    }

    return b_found;
}

#endif /* MOTION_BENCH */
