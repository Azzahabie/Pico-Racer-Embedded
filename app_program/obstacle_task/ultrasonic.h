/*
 *	ultrasonic.h
 *	HC-SR04 driver for Buddy 5 (scan_task), uT-Kernel port.
 *
 *	Design (Week 6 / Tribunal 1 interface row "ultrasonic.c"):
 *	  - 10 us trigger on GP4, echo timed on GP5 (Grove 3, 3.3 V until A5).
 *	  - ECHO_ISR only timestamps the edges and wakes scan_task; no maths
 *	    in the ISR. Timestamp is taken first thing, through the shared
 *	    core-1 GPIO dispatcher (common/gpio_irq.c).
 *	  - One ping per ~60 ms.
 *	  - Max-range echo  -> US_OUT_OF_RANGE (valid "clear path").
 *	    No pulse at all -> US_NO_ECHO      (sensor fault).
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <tk/tkernel.h>

/* 1 = ECHO_ISR timestamps edges (design).
 * 0 = poll the echo pin (fallback if the interrupt path misbehaves). */
#define US_USE_IRQ		1

#define US_TRIG_PIN		4U
#define US_ECHO_PIN		5U

#define US_RISE_TIMEOUT_US	10000U	/* no rising edge by now: no echo  */
#define US_FALL_TIMEOUT_US	40000U	/* still high by now: out of range */
#define US_WAIT_MS		50	/* task sleep while waiting (IRQ)  */
#define US_FAR_WIDTH_US		38000U	/* sensor's "nothing in range" pulse */
#define US_CYCLE_MS		60	/* minimum gap between pings       */

typedef enum {
	US_OK = 0,		/* valid distance in *p_mm                      */
	US_OUT_OF_RANGE,	/* echo came back, nothing in range: valid far  */
	US_NO_ECHO		/* no echo at all: sensor dead or unplugged     */
} us_status_t;

/* Call from the task that will ping (scan_task, on core 1). */
EXPORT void		us_init(void);

/* Design path (Section 4): ECHO_ISR sets echo_bit on the given event flag
 * (scan_flg) and us_ping() waits on that bit. Without this call the ISR
 * wakes the task directly (test modes 0-3). */
EXPORT void		us_use_flag(ID flgid, UINT echo_bit);
EXPORT us_status_t	us_ping(UW *p_mm, UW *p_width_us);

#endif /* ULTRASONIC_H */
