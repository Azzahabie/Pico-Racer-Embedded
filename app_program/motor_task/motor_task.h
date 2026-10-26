/*
 *  motor_task.h
 *  motion_task (Buddy 2): motors, wheel encoders, speed control.
 *
 *  The folder keeps the repo's scaffold name (motor_task); the design
 *  document calls the task motion_task.
 *
 *  This is the bench bring-up version: it owns the motors and encoders,
 *  runs a 10 ms tick on core 1, detects stalls (FR-30), publishes
 *  distance_state for imu_task (read it through motion_ds.h), feeds the
 *  hardware watchdog (motion_wdt.c) and runs a button-started bench test.
 *  The command interface from nav_ctrl_task and STOP_SCAN / STOP_IMU come
 *  next.
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <tk/tkernel.h>

/* Event-flag bits motion_task waits on (design Section 4).  Only TICK is
   used so far; CMD, STOP_SCAN and STOP_IMU will take the next bits. */
#define MOTION_EVT_TICK (1u << 0)

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
