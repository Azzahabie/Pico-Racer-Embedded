/*
 * line_task.h - black-line reader task (uT-Kernel 3.0, RP2040)
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <tk/tkernel.h>

/* Task entry point. Created and started from usermain() in app_main.c. */
void line_task(INT stacd, void *exinf);

#endif /* LINE_TASK_H */