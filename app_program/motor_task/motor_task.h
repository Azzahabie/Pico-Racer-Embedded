/*
 *  motor_task.h
 *  motion_task (Buddy 2): motors, wheel encoders, speed control.
 *
 *  The folder keeps the repo's scaffold name (motor_task); the design
 *  document calls the task motion_task.
 *
 *  This version owns the motors and encoders, runs a 10 ms tick on core 1,
 *  detects stalls (FR-30), feeds the hardware watchdog (motion_wdt.c), and
 *  handles Buddy 5's urgent STOP_SCAN request. Other motion commands and
 *  STOP_IMU remain future integration work.
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <tk/tkernel.h>

/* Motion fault bitmask: one bit per fault, packed in one byte. */
#define MOTION_FAULT_STALL_L (1u << 0) /* FR-30 motor stall, left    */
#define MOTION_FAULT_STALL_R (1u << 1) /* FR-30 motor stall, right   */
#define MOTION_FAULT_TICK    (1u << 2) /* 10 ms tick missing         */
#define MOTION_FAULT_INIT    (1u << 3) /* hardware set-up failed     */
#define MOTION_FAULT_ENC_L   (1u << 4) /* FR-30 encoder fault, left  */
#define MOTION_FAULT_ENC_R   (1u << 5) /* FR-30 encoder fault, right */

/* Task entry, kernel task signature. */
void motor_task(INT stacd, void *exinf);

/* Event flag used by motion_task; valid after successful motor_task_start(). */
ID motor_task_flag(void);

/* Create and start everything motion needs: its event flag, the 10 ms
   cyclic handler, motion_task (priority 11, pinned to core 1 under SMP)
   and a low-priority report task on core 0 that does all the printing,
   so core 1 never sits in tm_printf.  Call once from usermain(). */
ER motor_task_start(void);

#endif /* MOTOR_TASK_H */
