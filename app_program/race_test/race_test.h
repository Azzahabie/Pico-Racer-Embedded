#ifndef RACE_TEST_H
#define RACE_TEST_H

/*
 *	race_test.h
 *	Two-core demonstration of the shared packed-word hazard.
 *
 *	Flip TM_RACE_TEST to 1 and rebuild to include the test.  While it is 0 the
 *	.c file compiles to nothing and the demo is unchanged.
 *
 *	Needs a two-core build (SMP=1).  On a single-core build there is no second
 *	core to race against, so the test would only show the trivial case.
 */

#ifndef TM_RACE_TEST
#define TM_RACE_TEST	0
#endif

#include <tk/tkernel.h>

/* Create the objects and start the three test tasks.  Called from usermain()
   only when TM_RACE_TEST is 1. */
EXPORT ER race_test_start(void);

#endif /* RACE_TEST_H */