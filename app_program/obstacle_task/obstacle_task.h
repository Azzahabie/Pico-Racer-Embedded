#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#include <tk/tkernel.h>

/* Buddy 5: obstacle scan task (scan_task in the design doc, section 6.5).
 * Started from app_main.c usermain() with tk_cre_tsk / tk_sta_tsk.
 * Same signature as led_test_task and the other TA_HLNG tasks. */
EXPORT void obstacle_task(INT stacd, void *exinf);

#endif /* OBSTACLE_TASK_H */
