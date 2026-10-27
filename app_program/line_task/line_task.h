/*
 * line_task.h - black-line reader task (uT-Kernel 3.0, RP2040)
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <tk/tkernel.h>

/* 1 = usermain() starts line_task.  Off by default: the left line sensor
 * (LINE_LEFT_GPIO, GP8) is also the left motor's PWM pin, and
 * line_reader_init() takes it over.  Turn on once the pins are sorted. */
#ifndef TM_LINE_TASK
#define TM_LINE_TASK	0
#endif

/* Task entry point. Created and started from usermain() in app_main.c. */
void line_task(INT stacd, void *exinf);

#endif /* LINE_TASK_H */