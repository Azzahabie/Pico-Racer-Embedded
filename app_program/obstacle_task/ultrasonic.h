/*
 *	ultrasonic.h
 *	HC-SR04 driver for Buddy 5 (obstacle_task / scan_task), uT-Kernel port.
 *
 *	Pins (approved pin map, Grove 3): TRIG = GP4, ECHO = GP5.
 *
 *	Step 1 version: the echo pulse is timed by POLLING the 1 MHz hardware
 *	timer. The interrupt version (ECHO_ISR) comes once the team agrees the
 *	shared GPIO dispatcher with Buddy 2 -- only one handler can own
 *	IO_IRQ_BANK0 per core, so it must not be grabbed here unilaterally.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <tk/tkernel.h>

#define US_TRIG_PIN		4U
#define US_ECHO_PIN		5U

#define US_RISE_TIMEOUT_US	10000U	/* no rising edge by now: no echo  */
#define US_FALL_TIMEOUT_US	40000U	/* still high by now: nothing near */
#define US_FAR_WIDTH_US		38000U	/* sensor's "nothing in range" pulse */
#define US_CYCLE_MS		60	/* minimum gap between pings       */

typedef enum {
	US_OK = 0,	/* valid distance in *p_mm                     */
	US_FAR,		/* echo came back, nothing in range (valid)    */
	US_NO_ECHO	/* no echo at all: sensor dead or unplugged    */
} US_STATUS;

EXPORT void		us_init(void);
EXPORT US_STATUS	us_ping(UW *p_mm, UW *p_width_us);

#endif /* ULTRASONIC_H */
