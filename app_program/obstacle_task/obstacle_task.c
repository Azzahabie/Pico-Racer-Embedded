/*
 *	obstacle_task.c
 *	Buddy 5: obstacle scan task.
 *
 *	Step 1: range-test mode. Pings the HC-SR04 every 60 ms and prints a
 *	summary every 20 pings, same format as the bench test, so the A5 range
 *	test can be repeated on the real car and RTOS build.
 *
 *	Later steps replace the loop body with the 6.5 statechart
 *	(Monitoring -> Coarse Scan -> Fine Scan -> Planning -> Avoiding).
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "obstacle_task.h"
#include "ultrasonic.h"
#include "servo.h"
#include "../common/messages.h"

/* Test mode:
 *   0 = ultrasonic range test (A5)
 *   1 = servo sweep test (coarse + fine angles)
 *   2 = servo mounting calibration: holds CENTRE, LEFT, RIGHT for a few
 *       seconds each so you can check the sensor really points that way.
 *   3 = scan test: servo + sensor together. Sweeps -60..+60 deg, measures
 *       the distance at each angle, and reports the closest object. */
#define OBSTACLE_TEST_MODE	3

#define SAMPLES_PER_ROUND	20U

LOCAL void servo_go(INT deg)
{
	UW us = servo_set_angle(deg);

	tm_printf((UB *)"[servo] angle %d deg -> pulse %d us\n", deg, (INT)us);
	tk_dly_tsk(500);	/* generous for the test; report budget is 150 ms */
}

LOCAL void servo_test(void)
{
	static const INT coarse[5] = { -60, -30, 0, 30, 60 };
	INT i, a;

	servo_init();
	tm_printf((UB *)"[servo] SG90 test on GP%d\n", (INT)SERVO_PIN);
	tk_dly_tsk(1000);

	while(1) {
		tm_printf((UB *)"[servo] -- centre --\n");
		servo_go(0);
		tk_dly_tsk(1000);

		tm_printf((UB *)"[servo] -- coarse scan --\n");
		for(i = 0; i < 5; i++) servo_go(coarse[i]);

		tm_printf((UB *)"[servo] -- fine scan, 5 deg steps --\n");
		for(a = -20; a <= 20; a += 5) servo_go(a);

		tk_dly_tsk(2000);
	}
}

LOCAL void servo_hold(INT deg, const char *where, INT secs)
{
	UW us = servo_set_angle(deg);

	tm_printf((UB *)"[calib] angle %d deg (pulse %d us): sensor should point %s\n",
		  deg, (INT)us, where);
	tk_dly_tsk(secs * 1000);
}

LOCAL void servo_calibrate(void)
{
	servo_init();
	tm_printf((UB *)"[calib] trim %d us, dir %d. Look at the sensor from above.\n",
		  (INT)SERVO_TRIM_US, (INT)SERVO_DIR);

	while(1) {
		servo_hold(0,   "STRAIGHT AHEAD", 5);
		servo_hold(-45, "LEFT (45 deg)",  3);
		servo_hold(0,   "STRAIGHT AHEAD", 3);
		servo_hold(45,  "RIGHT (45 deg)", 3);
	}
}

/* One distance at the current servo angle: median of 3 pings, so a single
 * bad echo is ignored (6.5: "median of 3"). Returns 0 if no valid reading. */
LOCAL UW ping_median3(void)
{
	UW v[3], n = 0, i, mm, width_us, t;

	for(i = 0; i < 3; i++) {
		if(us_ping(&mm, &width_us) == US_OK) v[n++] = mm;
		tk_dly_tsk(US_CYCLE_MS);
	}
	if(n == 0) return 0;
	if(n == 1) return v[0];
	if(n == 2) return (v[0] < v[1]) ? v[0] : v[1];	/* be cautious: nearer */

	/* sort 3 values, take the middle */
	if(v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
	if(v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
	if(v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
	return v[1];
}

LOCAL void scan_test(void)
{
	INT	a, best_angle;
	UW	mm, best_mm;

	servo_init();
	us_init();
	tm_printf((UB *)"[scan] servo GP%d + HC-SR04 TRIG GP%d ECHO GP%d\n",
		  (INT)SERVO_PIN, (INT)US_TRIG_PIN, (INT)US_ECHO_PIN);

	while(1) {
		best_mm = 0xFFFFFFFFU;
		best_angle = 0;

		tm_printf((UB *)"[scan] ---- sweep (negative = left) ----\n");
		for(a = -SERVO_LIMIT_DEG; a <= SERVO_LIMIT_DEG; a += 15) {
			servo_set_angle(a);
			tk_dly_tsk(200);		/* let the servo settle */
			mm = ping_median3();

			if(mm == 0) {
				tm_printf((UB *)"[scan] angle %d: nothing in range\n", a);
			} else {
				tm_printf((UB *)"[scan] angle %d: %d mm\n", a, (INT)mm);
				if(mm < best_mm) { best_mm = mm; best_angle = a; }
			}
		}

		if(best_mm == 0xFFFFFFFFU) {
			tm_printf((UB *)"[scan] RESULT: nothing seen\n");
		} else {
			tm_printf((UB *)"[scan] RESULT: closest object %d mm at angle %d\n",
				  (INT)best_mm, best_angle);
		}

		servo_set_angle(0);
		tk_dly_tsk(3000);			/* time to move the object */
	}
}

EXPORT void obstacle_task(INT stacd, void *exinf)
{
	UW	round = 0;

	(void)stacd; (void)exinf;

#if OBSTACLE_TEST_MODE == 1
	servo_test();		/* never returns */
#elif OBSTACLE_TEST_MODE == 2
	servo_calibrate();	/* never returns */
#elif OBSTACLE_TEST_MODE == 3
	scan_test();		/* never returns */
#endif

	us_init();
	tm_printf((UB *)"[obstacle] HC-SR04 TRIG=GP%d ECHO=GP%d\n",
		  (INT)US_TRIG_PIN, (INT)US_ECHO_PIN);

	while(1) {
		UW	sum_mm = 0, min_mm = 0xFFFFFFFFU, max_mm = 0;
		UW	n_ok = 0, n_far = 0, n_noecho = 0;
		UW	i;

		for(i = 0; i < SAMPLES_PER_ROUND; i++) {
			UW mm = 0, width_us = 0;

			switch(us_ping(&mm, &width_us)) {
			case US_OK:
				n_ok++;
				sum_mm += mm;
				if(mm < min_mm) min_mm = mm;
				if(mm > max_mm) max_mm = mm;
				break;
			case US_FAR:
				n_far++;
				break;
			case US_NO_ECHO:
			default:
				n_noecho++;
				break;
			}

			tk_dly_tsk(US_CYCLE_MS);	/* sleeps: other tasks run */
		}

		round++;

		/* One tm_printf per line: it is atomic per call, not per line. */
		if(n_ok > 0) {
			tm_printf((UB *)"[obstacle] round %d: mean %d mm  min %d  max %d  "
				  "spread %d mm  (ok %d, far %d, no-echo %d)\n",
				  (INT)round, (INT)(sum_mm / n_ok), (INT)min_mm,
				  (INT)max_mm, (INT)(max_mm - min_mm),
				  (INT)n_ok, (INT)n_far, (INT)n_noecho);
		} else {
			tm_printf((UB *)"[obstacle] round %d: no valid readings  "
				  "(far %d, no-echo %d)\n",
				  (INT)round, (INT)n_far, (INT)n_noecho);
		}
	}
}
