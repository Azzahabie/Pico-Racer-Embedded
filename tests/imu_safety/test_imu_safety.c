/*
 *	test_imu_safety.c
 *	Host tests for app_program/imu_task/imu_safety.c (Buddy 4).
 *
 *	    make -C tests/imu_safety
 *
 *	Feeds safety_step() 10 ms samples as imu_task would and checks when
 *	the abnormal-tilt (FR-28) and no-data (NFR-13) events fire.
 */

#include <stdio.h>
#include <math.h>

#include "imu_safety.h"

/* hump.c prints; nothing here needs its lines. */
INT tm_printf(const UB *format, ...)
{
	(void)format;
	return 0;
}

static int checks, failures;

static void expect(int cond, const char *what)
{
	checks++;
	if (!cond) {
		failures++;
		printf("FAIL  %s\n", what);
	}
}

/* A still car tilted by deg, nose up: gravity splits into x and z. */
static UINT tilt(double deg)
{
	double r = deg * M_PI / 180;
	return safety_step(TRUE, (W)lround(1000 * sin(r)), 0,
			   (W)lround(1000 * cos(r)));
}

/* Returns the sample (1-based) on which `bit` first fired, 0 if never. */
static int run_tilt(double deg, int samples, UINT bit)
{
	int i;

	for (i = 1; i <= samples; i++) {
		if (tilt(deg) & bit) return i;
	}
	return 0;
}

int main(void)
{
	int	i, at;
	UINT	ev;

	/* Level and on a hump: never a tilt event. */
	safety_reset();
	expect(run_tilt(0, 500, SAFETY_EV_TILT) == 0, "level: no tilt event");
	safety_reset();
	expect(run_tilt(10, 500, SAFETY_EV_TILT) == 0,
	       "10 deg (steepest hump seen): no tilt event");
	safety_reset();
	expect(run_tilt(19, 500, SAFETY_EV_TILT) == 0, "19 deg: no tilt event");

	/* Past T: fires on the Nth sample in a row, 30 ms (NFR-19: 40 ms). */
	safety_reset();
	at = run_tilt(25, 100, SAFETY_EV_TILT);
	expect(at == SAFETY_TILT_N, "25 deg: tilt event on sample N");
	expect(run_tilt(25, 100, SAFETY_EV_TILT) == 0,
	       "25 deg held: reported once only");
	expect(safety_tilted(), "25 deg held: still tilted");
	tilt(0);
	expect(!safety_tilted(), "back level: not tilted");
	safety_reset();
	tilt(25); tilt(25);
	expect(!safety_tilted(), "2 samples over: not tilted yet");
	tilt(25);
	expect(safety_tilted(), "3 samples over: tilted");

	/* Two samples over, one back under: the run starts again. */
	safety_reset();
	tilt(30); tilt(30); tilt(0);
	expect(run_tilt(30, 2, SAFETY_EV_TILT) == 0, "interrupted run: no event");
	expect(run_tilt(30, 1, SAFETY_EV_TILT) == 1, "then the 3rd in a row fires");

	/* A jolt (|a| 2 g, all sideways) is not a tilt and does not count. */
	safety_reset();
	for (i = 0, ev = 0; i < 20; i++) ev |= safety_step(TRUE, 2000, 0, 300);
	expect((ev & SAFETY_EV_TILT) == 0, "2 g jolt: no tilt event");

	/* ... and a jolt in the middle of a real tilt does not reset it. */
	safety_reset();
	tilt(30); tilt(30);
	safety_step(TRUE, 2000, 0, 300);
	expect(run_tilt(30, 1, SAFETY_EV_TILT) == 1,
	       "jolt mid-tilt: run carries on");

	/* Upside down (|a| 1 g, no x/y): not a tilt by this check. */
	safety_reset();
	for (i = 0, ev = 0; i < 20; i++) ev |= safety_step(TRUE, 0, 0, -1000);
	expect((ev & SAFETY_EV_TILT) == 0, "flat but flipped: no tilt event");

	/* 90 deg on its side: a tilt. */
	safety_reset();
	expect(run_tilt(90, 10, SAFETY_EV_TILT) == SAFETY_TILT_N,
	       "on its side: tilt event");

	/* Reads failing: re-init after 2, no data after 5 (50 ms). */
	safety_reset();
	ev = 0;
	for (i = 1; i <= SAFETY_BAD_N; i++) {
		UINT e = safety_step(FALSE, 0, 0, 0);
		if (e & SAFETY_EV_REINIT) expect(i == SAFETY_REINIT_N,
						 "re-init on the 2nd bad read");
		if (e & SAFETY_EV_NO_DATA) expect(i == SAFETY_BAD_N,
						  "no data on the 5th bad read");
		ev |= e;
	}
	expect((ev & SAFETY_EV_REINIT) != 0, "re-init requested");
	expect((ev & SAFETY_EV_NO_DATA) != 0, "no-data event fired");
	for (i = 0, ev = 0; i < 100; i++) ev |= safety_step(FALSE, 0, 0, 0);
	expect((ev & SAFETY_EV_NO_DATA) == 0, "still failing: reported once");
	expect(safety_no_data(), "still failing: no data");
	safety_step(TRUE, 0, 0, 1000);
	expect(!safety_no_data(), "a good read: data again");

	/* 4 bad, 1 good, 4 bad: never 5 in a row (rderr 31/s on the car). */
	safety_reset();
	for (i = 0, ev = 0; i < 50; i++) {
		ev |= safety_step((i % 5) == 4, 0, 0, 1000);
	}
	expect((ev & SAFETY_EV_NO_DATA) == 0, "4 bad in 5: no fault");

	printf("imu_safety: %d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
