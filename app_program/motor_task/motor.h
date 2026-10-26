/*
 *  motor.h
 *  Two DC motors on the Robo Pico H-bridge, driven by RP2040 PWM.
 *
 *  Single writer: only motion_task calls motor_set() / motor_brake_all().
 *  Any other task that needs the car stopped asks motion_task (event flag
 *  bit), it never writes the PWM registers itself.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    MOTOR_LEFT = 0, /* M1: GP8 / GP9   */
    MOTOR_RIGHT,    /* M2: GP10 / GP11 */
    MOTOR_COUNT
} motor_side_t;

/* Release the PWM block from reset, set both slices to MOTION_PWM_HZ with
   both inputs low (brake), and hand GP8..GP11 to the PWM.  Call once,
   before anything else touches the motors. */
ER motor_init(void);

/* duty_pm: signed per-mille, clamped to +-MOTION_DUTY_FULL.  Positive is
   "forward" after MOTION_MOTOR_x_INVERT is applied; 0 is brake.  Both
   inputs of the motor are updated by one 32-bit store, and the hardware
   applies it at the next PWM period, so there is never a half-updated
   state on the H-bridge. */
void motor_set(motor_side_t side, int32_t duty_pm);

/* Both motors to brake (both inputs low). */
void motor_brake_all(void);

/* Last duty passed to motor_set(), before inversion. */
int32_t motor_get_duty(motor_side_t side);

#endif /* MOTOR_H */
