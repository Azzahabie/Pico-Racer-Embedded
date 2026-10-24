/*
 *  motion_wdt.h
 *  RP2040 hardware watchdog, fed by motion_task (Buddy 2).
 *
 *  motion_task arms it once its hardware set-up has passed and feeds it at
 *  the end of every completed 10 ms tick.  If motion_task hangs, or its
 *  tick stops coming (core 0 stalled, kernel timer stopped, a fault
 *  handler spinning), nothing feeds it and the whole chip resets
 *  MOTION_WDT_MS after the last feed.  The reset puts GP8..GP11 back to
 *  inputs with pull-downs, so the motor driver sees both inputs low
 *  (brake), and after the reboot the motors stay braked until a command
 *  (PreTri1 shared-resource row 2, PostTri1 Version C row 9).
 *
 *  Only motion_task calls these.
 */

#ifndef MOTION_WDT_H
#define MOTION_WDT_H

#include <stdint.h>
#include <stdbool.h>

#include "motion_cfg.h"

/* Why the chip last started.  Read it before motion_wdt_start(), which
   rewrites the marker it depends on. */
typedef enum
{
    MOTION_BOOT_POWER_ON = 0, /* power-on, brown-out or the RUN pin   */
    MOTION_BOOT_WATCHDOG,     /* our watchdog ran out: motion stopped */
    MOTION_BOOT_REBOOT        /* another watchdog reboot, such as the
                                 boot ROM's after a UF2 copy: no fault */
} motion_boot_t;

motion_boot_t motion_wdt_boot_cause(void);

/* Arm the watchdog with MOTION_WDT_MS, starting from a full count. */
void motion_wdt_start(void);

/* Start the count again.  Call only at the end of a completed tick. */
void motion_wdt_feed(void);

#if MOTION_BENCH
/* Watchdog test (MOTION_TEST_WDT).  Stops feeding and spins, writing the
   time to a scratch register that survives the reset.  Never returns. */
void motion_wdt_hang(uint32_t fed_us);

/* After a motion_wdt_hang() reset: true once, with *p_us = how long the
   chip ran after the last feed.  Clears the marker. */
bool motion_wdt_take_hang_us(uint32_t *p_us);
#endif

#endif /* MOTION_WDT_H */
