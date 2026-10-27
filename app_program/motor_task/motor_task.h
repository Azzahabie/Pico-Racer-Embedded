/*
 *  motor_task.h
 *  motion_task (Buddy 2): motors, wheel encoders, speed control.
 *
 *  The folder keeps the repo's scaffold name (motor_task); the design
 *  document calls the task motion_task.
 *
 *  This is the bench bring-up version: it owns the motors and encoders,
 *  runs a 10 ms tick on core 1, detects stalls (FR-30), brakes on
 *  STOP_SCAN and STOP_IMU (NFR-18), publishes distance_state for imu_task
 *  (read it through motion_ds.h), feeds the hardware watchdog
 *  (motion_wdt.c) and runs a button-started bench test.  The command
 *  interface from nav_ctrl_task and scan_task comes next.
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <tk/tkernel.h>
#include "../common/messages.h"

#include "../common/messages.h"

/* ------------------------------------------------------------------ *
 *  motion_flg (design Section 4)
 *
 *  The event flag motion_task waits on.  Its bits (MOTION_EVT_*) are
 *  defined once, in common/messages.h, shared with scan_task and imu_task:
 *
 *    TICK       0x01  set by motion's 10 ms cyclic handler.
 *    CMD        0x02  set by a command sender after it has placed a
 *                     command (step 1.7; motion does not read it yet).
 *    STOP_SCAN  0x04  set and cleared ONLY by scan_task: "obstacle".
 *    STOP_IMU   0x08  set by imu_task, which never clears it; motion_task
 *                     clears it when nav_ctrl_task sends FAULT_CLEAR(IMU)
 *                     (step 1.7; until then nothing clears it).
 *
 *  A STOP bit is a level: motion brakes as soon as it sees one and stays
 *  braked while any is set.  Senders use tk_set_flg (and scan_task
 *  tk_clr_flg) on motor_task_flag(); never wait on this flag (only
 *  motion_task may).
 * ------------------------------------------------------------------ */

/* motion_flg's ID, valid once motor_task_start() has returned E_OK
   (0 before, and always 0 in bench images, MOTION_BENCH 1, so scan_task
   stays print-only there).  For the STOP senders and, later, the command
   senders. */
ID motor_task_flag(void);

/* Optional, for the NFR-18 timing: a STOP sender calls this just before
   tk_set_flg, with the bit it is about to set.  It notes the 1 MHz time,
   so motion can report how long after the request it braked. */
void motion_stop_stamp(UINT stop_bit);

/* Motion fault bitmask: one bit per fault, packed in one byte. */
#define MOTION_FAULT_STALL_L (1u << 0) /* FR-30 motor stall, left    */
#define MOTION_FAULT_STALL_R (1u << 1) /* FR-30 motor stall, right   */
#define MOTION_FAULT_TICK    (1u << 2) /* 10 ms tick missing         */
#define MOTION_FAULT_INIT    (1u << 3) /* hardware set-up failed     */
#define MOTION_FAULT_ENC_L   (1u << 4) /* FR-30 encoder fault, left  */
#define MOTION_FAULT_ENC_R   (1u << 5) /* FR-30 encoder fault, right */

/* Task entry, kernel task signature. */
void motor_task(INT stacd, void *exinf);

/* Create and start everything motion needs: its event flag, the 10 ms
   cyclic handler, motion_task (priority 11, pinned to core 1 under SMP)
   and a low-priority report task on core 0 that does all the printing,
   so core 1 never sits in tm_printf.  Call once from usermain(). */
ER motor_task_start(void);

#endif /* MOTOR_TASK_H */
