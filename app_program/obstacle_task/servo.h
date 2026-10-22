/*
 *	servo.h
 *	SG90 scan-head servo for Buddy 5, uT-Kernel port.
 *
 *	Pin: GP12 (approved pin map) = PWM slice 6, channel A.
 *	50 Hz frame; pulse width sets the angle. Angle 0 = straight ahead.
 */

#ifndef SERVO_H
#define SERVO_H

#include <tk/tkernel.h>

#define SERVO_PIN		12U

#define SERVO_CENTRE_US		1500	/* 0 deg, straight ahead               */
#define SERVO_US_PER_DEG	11	/* SG90 ~500..2500 us over ~180 deg     */
#define SERVO_MIN_US		500	/* never go past these: the servo jams  */
#define SERVO_MAX_US		2500	/* against its end stop                 */
#define SERVO_LIMIT_DEG		60	/* scan range is -60..+60 deg           */
#define SERVO_SETTLE_MS		150	/* report budget per step (NFR-07)      */

/* Mounting calibration (see the calibration test in obstacle_task.c).
 * Fix big errors (> ~20 deg, or facing backwards) by refitting the horn.
 * Fix small errors here. */
#define SERVO_TRIM_US		55	/* +/- tens of us to make 0 deg = straight ahead */
#define SERVO_DIR		-1	/* 1 = normal, -1 = servo mounted mirrored       */

EXPORT void	servo_init(void);
EXPORT UW	servo_set_angle(INT angle_deg);	/* returns pulse in us */

#endif /* SERVO_H */
