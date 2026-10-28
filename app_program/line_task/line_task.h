/*
 * line_task.h - ir_sense_task (Buddy 3): samples both line sensors and the
 * barcode sensor every 1 ms (uT-Kernel 3.0, RP2040).  Priority 10, core 1.
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>

/* 1 = usermain() starts ir_sense_task and barcode_task.  The line sensors
 * are on GP26/GP27 (ADC0/ADC1) and the barcode sensor on GP7, clear of the
 * motor pins GP8-GP11 (common/line_config.h). */
#ifndef TM_LINE_TASK
#define TM_LINE_TASK	1
#endif

/* ir_flg: nav_ctrl_task sets IR_FLG_CAL_REQ (its cal_req_bit) to ask for a
 * calibration; ir_sense_task answers NAV_FLG_CAL_OK_IR or NAV_FLG_CAL_FAIL. */
#define IR_FLG_CAL_REQ	0x0001U

/* Create ir_flg.  Call in usermain() before nav_bind(); returns its ID. */
ID ir_sense_init(void);

/* Task entry point. Created and started from usermain() in app_main.c. */
void ir_sense_task(INT stacd, void *exinf);

/* line_state for motion_task (line_state.h layout): one aligned load. */
uint32_t line_state_word(void);

#endif /* LINE_TASK_H */
