/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	demo_tasks.h
 *	The console-independent liveness task.
 */

#ifndef DEMO_TASKS_H
#define DEMO_TASKS_H

/* Off on the car: BOARD_LED_PIN is GP16, which is the IMU's I2C0 SDA.  Flip
   to 1 and rebuild for the LED liveness / USB-diagnostic blink on a bare
   Pico.  Note the kernel port still makes GP16 an output at reset
   (kernel/sysdepend/pico_rp2040/hw_setting.c) until BOARD_LED_PIN moves. */
#ifndef TM_BLINK_LED
#define TM_BLINK_LED	0
#endif

#include <tk/tkernel.h>

/* Blink task entry.  Deliberately touches nothing the console touches: if
   output stops but the LED keeps blinking, the console is at fault; if the
   LED freezes, the kernel is. */
IMPORT void blink_task(INT stacd, void *exinf);

#endif /* DEMO_TASKS_H */
