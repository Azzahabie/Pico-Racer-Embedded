/*
 *	test_imu_safety.c
 *	Host tests for app_program/imu_task/imu_safety.c and imu_class.c
 *	(Buddy 4).
 *
 *	    make -C tests/imu_safety
 *
 *	Feeds safety_step() 10 ms samples as imu_task would and checks when
 *	the abnormal-tilt (FR-28) and no-data (NFR-13) events fire; then
 *	feeds mclass_step() and checks the motion classes (FR-27).
 */

#include <stdio.h>
#include <math.h>

#include "imu_safety.h"
#include "imu_class.h"
#include "imu_task.h"		/* hump_state word layout */

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
	expect(run_tilt(20, 500, SAFETY_EV_TILT) == 0, "20 deg: no tilt event");
	safety_reset();
	expect(run_tilt(24, 500, SAFETY_EV_TILT) == 0, "24 deg: no tilt event");

	/* Past T (25 deg): fires on the Nth sample in a row, 30 ms (NFR-19: 40 ms). */
	safety_reset();
	at = run_tilt(30, 100, SAFETY_EV_TILT);
	expect(at == SAFETY_TILT_N, "30 deg: tilt event on sample N");
	expect(run_tilt(30, 100, SAFETY_EV_TILT) == 0,
	       "30 deg held: reported once only");
	expect(safety_tilted(), "30 deg held: still tilted");
	tilt(0);
	expect(!safety_tilted(), "back level: not tilted");
	safety_reset();
	tilt(30); tilt(30);
	expect(!safety_tilted(), "2 samples over: not tilted yet");
	tilt(30);
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

	/* ---- motion classifier (FR-27) ---- */

	/* Still: no mm, no turn, 1 g. */
	mclass_reset();
	for (i = 0; i < 30; i++) mclass_step(TRUE, 1000, 0, 0, FALSE, 0);
	expect(mclass_get() == MC_STATIONARY, "class: still");

	/* Straight at ~50 mm/s: a mm every other sample, never "still". */
	for (i = 0; i < 50; i++) mclass_step(TRUE, 1000, i % 2, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: moving straight");

	/* Slow, 1 mm per 40 ms: still moving, not still. */
	for (i = 0; i < 50; i++) mclass_step(TRUE, 1000, (i % 4) == 0, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: slow is still moving");

	/* Turning 30 deg/s. */
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 30, FALSE, 0);
	expect(mclass_get() == MC_TURNING, "class: turning");

	/* Turning on the spot: no mm, but turning. */
	for (i = 0; i < 30; i++) mclass_step(TRUE, 1000, 0, -40, FALSE, 0);
	expect(mclass_get() == MC_TURNING, "class: turning on the spot");

	/* One sample of turn noise does not change the class. */
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: back to moving");
	mclass_step(TRUE, 1000, 1, 40, FALSE, 0);
	mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: one-sample spike ignored");

	/* A turn rate wandering 17-21 deg/s (2026-10-28 02:14:39) stays one
	   turn: hysteresis, then straight again below 12. */
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 25, FALSE, 0);
	expect(mclass_get() == MC_TURNING, "class: turn starts");
	for (i = 0; i < 40; i++) mclass_step(TRUE, 1000, 1, 17 + (i % 5), FALSE, 0);
	expect(mclass_get() == MC_TURNING, "class: 17-21 dps stays turning");
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 5, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: below 12 dps, straight");

	/* A 50 ms blip (distance_state ACCELERATING while stopping) is not
	   reported. */
	for (i = 0; i < 5; i++) mclass_step(TRUE, 1000, 1, 0, TRUE, 0);
	for (i = 0; i < 5; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: 50 ms accel blip ignored");

	/* Accelerating (distance_state says so). */
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 0, TRUE, 0);
	expect(mclass_get() == MC_ACCELERATING, "class: accelerating");

	/* On a hump, climbing then descending outrank a turn. */
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 30, FALSE, 1);
	expect(mclass_get() == MC_CLIMBING, "class: climbing");
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 2);
	expect(mclass_get() == MC_DESCENDING, "class: descending");

	/* Impact while moving: at once, held 500 ms, then back. */
	for (i = 0; i < 10; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	expect(mclass_step(TRUE, 2000, 1, 0, FALSE, 0) == TRUE
	       && mclass_get() == MC_IMPACT, "class: impact reported at once");
	for (i = 0; i < MC_IMPACT_HOLD - 2; i++) {
		mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	}
	expect(mclass_get() == MC_IMPACT, "class: impact held 500 ms");
	for (i = 0; i < 20; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	expect(mclass_get() == MC_MOVING, "class: impact over");

	/* Hit just after stopping (a crash brakes the car): still impact. */
	for (i = 0; i < 30; i++) mclass_step(TRUE, 1000, 0, 0, FALSE, 0);
	mclass_step(TRUE, 1800, 0, 0, FALSE, 0);
	expect(mclass_get() == MC_IMPACT, "class: impact 0.3 s after stopping");

	/* Carried by hand long after stopping: 2.4 g is not an impact. */
	mclass_reset();
	for (i = 0; i < 300; i++) mclass_step(TRUE, 1000, 0, 0, FALSE, 0);
	for (i = 0; i < 20; i++) mclass_step(TRUE, 2400, 0, 0, FALSE, 0);
	expect(mclass_get() == MC_STATIONARY, "class: handled while still");

	/* A failed read never counts as an impact. */
	mclass_reset();
	for (i = 0; i < 10; i++) mclass_step(TRUE, 1000, 1, 0, FALSE, 0);
	mclass_step(FALSE, 9000, 1, 0, FALSE, 0);
	expect(mclass_get() != MC_IMPACT, "class: bad read is not an impact");

	expect(mclass_name(MC_CLIMBING)[0] == 'c', "class: names");

	/* ---- hump_state word ---- */
	{
		UW w = imu_w_pack(1, MC_MOVING, 9, 3, 23, 1, 7);

		expect(IMU_W_GET(w, HUMP) == 1, "word: hump state");
		expect(IMU_W_GET(w, CLASS) == MC_MOVING, "word: class");
		expect(IMU_W_GET(w, HEIGHT) == 9, "word: height");
		expect(IMU_W_GET(w, COUNT) == 3, "word: count");
		expect(IMU_W_GET(w, MAX) == 23, "word: highest");
		expect(IMU_W_GET(w, FAULT) == 1, "word: fault");
		expect(IMU_W_GET(w, TILT) == 7, "word: tilt");

		/* Past the range: clamped, never spilling into the next field. */
		w = imu_w_pack(0, 0, 300, 20, 999, 0, 90);
		expect(IMU_W_GET(w, HEIGHT) == 255, "word: height clamped");
		expect(IMU_W_GET(w, COUNT) == 15, "word: count clamped");
		expect(IMU_W_GET(w, MAX) == 255, "word: highest clamped");
		expect(IMU_W_GET(w, TILT) == 63, "word: tilt clamped");
		expect(IMU_W_GET(w, FAULT) == 0 && IMU_W_GET(w, HUMP) == 0,
		       "word: clamping leaves the others alone");

		/* The fields fill exactly 32 bits. */
		expect(IMU_W_TILT_SHIFT + IMU_W_TILT_BITS == 32, "word: 32 bits");
	}

	printf("imu_safety: %d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
