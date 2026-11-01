#ifndef WORD_STRESS_H
#define WORD_STRESS_H

/*
 *	word_stress.h
 *	Stress test for the lock-free shared words (Tribunal 1, Version C row 2).
 *
 *	Flip TM_WORD_STRESS to 1 and rebuild to include the test.  While it is 0
 *	the .c file compiles to nothing and the car image is unchanged.
 *
 *	Needs a two-core build (SMP=1).  Build the car profile (Wi-Fi + MQTT) so
 *	the test runs with the radio busy on core 0.
 */

#ifndef TM_WORD_STRESS
#define TM_WORD_STRESS	0
#endif

#include <tk/tkernel.h>

/* Create the controller task, which runs every phase and prints the
   results.  Called from usermain() only when TM_WORD_STRESS is 1. */
EXPORT ER word_stress_start(void);

#endif /* WORD_STRESS_H */
