/*
 *	test_imu_hump.c
 *	Host tests for app_program/imu_task/hump.c (Buddy 4).
 *
 *	Runs on a PC with plain gcc; no kernel, no IMU:
 *	    make -C tests/imu_hump          (make V=1 ... prints every line)
 *
 *	A simulated car is pushed along a track with real slopes.  Every
 *	100 ms window it hands hump_step() what imu_task would: the mean
 *	tilt the accelerometer reads on that slope (with a little noise), the
 *	whole mm the encoders moved, and STEADY except when it sets off.
 *	Each scenario is one of the passes run on the car on 2026-10-27, and
 *	checks how many humps were counted.  Every scenario runs in its own
 *	process, so hump.c starts fresh each time.
 *
 *	The ramp-length scenarios (19 Nov: the final ramp is the same height,
 *	a different width) expect what the hump.h build switches give, so
 *	the Makefile runs this file against each switch setting.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/wait.h>

#include "hump.h"

/* ---- tm_printf: kept for the report, echoed with V=1 ---- */

static int verbose;
static char last_lines[4096];

INT tm_printf(const UB *format, ...)
{
	char	line[256];
	va_list	ap;

	va_start(ap, format);
	vsnprintf(line, sizeof line, (const char *)format, ap);
	va_end(ap);
	if (verbose) printf("        %s", line);
	if (strlen(last_lines) + strlen(line) < sizeof last_lines)
		strcat(last_lines, line);
	return 0;
}

/* ---- track: height in mm at points along it ---- */

typedef struct { double s, z; } PT;

static const PT *trk;
static int ntrk;

/* Slope (rise per mm) of the piece of track the car is on.  At a corner
   the piece it is heading onto. */
static double slope_at(double s, int dir)
{
	int	i;

	for (i = 0; i + 1 < ntrk; i++) {
		double a = trk[i].s, b = trk[i + 1].s;
		if ((dir >= 0) ? (s >= a && s < b) : (s > a && s <= b))
			return (trk[i + 1].z - trk[i].z) / (b - a);
	}
	return 0.0;
}

/* Triangle hump 10 mm high, 120 mm up, 120 mm down. */
static const PT HUMP[] = {
	{ 0, 0 }, { 200, 0 }, { 320, 10 }, { 440, 0 }, { 2000, 0 },
};
/* The same with a 40 mm level crest. */
static const PT FLAT_TOP[] = {
	{ 0, 0 }, { 200, 0 }, { 320, 10 }, { 360, 10 }, { 480, 0 },
	{ 2000, 0 },
};
/* The practice ramp measured on 2026-10-27: 10 mm, with a level top
   longer than HUMP_END_MM (log 11: level from 300 to past 336). */
static const PT LONG_TOP[] = {
	{ 0, 0 }, { 200, 0 }, { 300, 10 }, { 400, 10 }, { 500, 0 },
	{ 2000, 0 },
};
/* Two wedges whose far side drops straight off: no way down to read. */
static const PT WEDGES[] = {
	{ 0, 0 }, { 200, 0 }, { 300, 10 }, { 400, 10 }, { 401, 0 },
	{ 800, 0 }, { 900, 10 }, { 1000, 10 }, { 1001, 0 }, { 2000, 0 },
};
/* The demo-day hump (the only one, from the prof): 195 mm long, 10 mm at
   the middle, up half way and down the other half. */
static const PT DEMO[] = {
	{ 0, 0 }, { 200, 0 }, { 297, 10 }, { 395, 0 }, { 2000, 0 },
};
/* The demo hump as the car read it (S6 attempt 1, 1 Nov): the climb, then
   about 0 tilt, never the way down (its body is as long as the hump). */
static const PT DEMO_AS_READ[] = {
	{ 0, 0 }, { 200, 0 }, { 297, 10 }, { 3000, 10 },
};

/* Two humps, 9 mm and 23 mm, 300 mm apart. */
static const PT TWO[] = {
	{ 0, 0 }, { 200, 0 }, { 310, 9 }, { 420, 0 }, { 720, 0 },
	{ 900, 23 }, { 1080, 0 }, { 2000, 0 },
};
/* The same, the higher one first. */
static const PT TWO_HIGH_FIRST[] = {
	{ 0, 0 }, { 200, 0 }, { 380, 23 }, { 560, 0 }, { 860, 0 },
	{ 970, 9 }, { 1080, 0 }, { 2000, 0 },
};

/* 19 Nov: the final ramp is the same height as the demo hump, a
   different width.  Along the track a longer ramp is gentler.  10 mm,
   up half way and down the other half (slope of the way up): */
static const PT SYM_300[] = {			/* 66 mg, 3.8 deg */
	{ 0, 0 }, { 200, 0 }, { 350, 10 }, { 500, 0 }, { 2000, 0 },
};
static const PT SYM_360[] = {			/* 55 mg, 3.2 deg */
	{ 0, 0 }, { 200, 0 }, { 380, 10 }, { 560, 0 }, { 2000, 0 },
};
static const PT SYM_440[] = {			/* 45 mg, 2.6 deg */
	{ 0, 0 }, { 200, 0 }, { 420, 10 }, { 640, 0 }, { 2000, 0 },
};
static const PT SYM_600[] = {			/* 33 mg, 1.9 deg */
	{ 0, 0 }, { 200, 0 }, { 500, 10 }, { 800, 0 }, { 2000, 0 },
};
/* 8 mm over 300 mm (53 mg) and 12 mm over 440 mm (54 mg). */
static const PT SYM8_300[] = {
	{ 0, 0 }, { 200, 0 }, { 350, 8 }, { 500, 0 }, { 2000, 0 },
};
static const PT SYM12_440[] = {
	{ 0, 0 }, { 200, 0 }, { 420, 12 }, { 640, 0 }, { 2000, 0 },
};
/* The practice ramp's slopes (100 mm up, 100 down) with a level top of
   200 and 400 mm: longer than HUMP_TOP_MAX_MM (120). */
static const PT TOP_200[] = {
	{ 0, 0 }, { 200, 0 }, { 300, 10 }, { 500, 10 }, { 600, 0 },
	{ 2000, 0 },
};
static const PT TOP_400[] = {
	{ 0, 0 }, { 200, 0 }, { 300, 10 }, { 700, 10 }, { 800, 0 },
	{ 2000, 0 },
};
/* The same 200 mm top, then a second ramp 300 mm on. */
static const PT TOP_THEN_HUMP[] = {
	{ 0, 0 }, { 200, 0 }, { 300, 10 }, { 500, 10 }, { 600, 0 },
	{ 900, 0 }, { 1000, 10 }, { 1100, 0 }, { 2000, 0 },
};
/* Gentle both ways (220 mm up and down, 45 mg) with a 200 mm top. */
static const PT GENTLE_TOP[] = {
	{ 0, 0 }, { 200, 0 }, { 420, 10 }, { 620, 10 }, { 840, 0 },
	{ 2000, 0 },
};
/* A climb read late and a way down that eases off part way: a toe at
   25 mg (never seen), 80 mg up, then down at 100 mg, 30 mg, 100 mg.  The
   height read (6.5 mm) runs out at 5.5 mm on the way down, where the
   30 mg stretch lets it close; the last 4.5 mm down then reads as a new
   climb.  A fixed stand-in for what the noise does at random on a ramp
   near the 50 mg start (sim: 10 mm over 400 mm, 5 of 20 runs). */
static const PT LATE_KINK[] = {
	{ 0, 0 }, { 200, 0 }, { 340, 3.5 }, { 421, 10 }, { 446, 7.5 },
	{ 546, 4.5 }, { 591, 0 }, { 2000, 0 },
};
/* The floor itself tilted 30 mg (the most measured between floor spots,
   1.7 deg) for 300 mm: under any start threshold, not a hump. */
static const PT FLOOR_30MG[] = {
	{ 0, 0 }, { 200, 0 }, { 500, 9 }, { 2000, 9 },
};
/* The way up too gentle to see (33 mg), the far side steep (100 mg),
   then a second ramp: each is read from its far side. */
static const PT STEEP_FAR[] = {
	{ 0, 0 }, { 200, 0 }, { 500, 10 }, { 600, 0 }, { 900, 0 },
	{ 1000, 10 }, { 1100, 0 }, { 2000, 0 },
};

#define USE(t)	(trk = (t), ntrk = (int)(sizeof (t) / sizeof (t)[0]))

/* ---- the car ---- */

static double pos;		/* mm along the track            */
static double enc;		/* mm the encoders have counted  */
static double enc_scale = 1;	/* < 1: a pivot, under-reads     */
static int hand;		/* mg added while carried        */
static int dir = 1;		/* last way it moved             */
static unsigned rng = 12345;

/* +/- 4 mg of sensor noise. */
static int noise(void)
{
	rng = rng * 1103515245u + 12345u;
	return (int)((rng >> 16) % 9) - 4;
}

/* One 100 ms window, moving by dmm.  The accelerometer is mounted at an
   angle, so uphill reads on both x and y (as on the car: x -60, y -80). */
static void window(double dmm, int steady)
{
	double	from = pos, efrom = enc, mid, sn;
	int	d;

	if (dmm != 0) dir = (dmm > 0) ? 1 : -1;
	pos += dmm;
	enc += dmm * enc_scale;
	mid = (dmm != 0) ? (from + pos) / 2 : pos;
	sn = slope_at(mid, dir);
	sn = sn / sqrt(1 + sn * sn);
	d = (int)lround(enc) - (int)lround(efrom);
	hump_step((W)lround(-600 * sn) + noise() + hand,
		  (W)lround(-800 * sn) + noise() + hand, d, steady);
}

/* Picked up and put down at `to`: the wheels do not turn, the tilt
   swings (log 13: x -40..+111 mg while carried). */
static void carry(double to)
{
	int	i;

	for (i = 0; i < 10; i++) {
		hand = (i % 2) ? 110 : -60;
		window(0, 1);
	}
	hand = 0;
	pos = to;
	for (i = 0; i < 10; i++) window(0, 1);	/* put down: 1 s still */
}

/* Drive to `to` at `speed` mm/s. */
static void drive(double to, double speed)
{
	double	step = speed / 10;
	int	first = 1;

	while (fabs(to - pos) > 1e-9) {
		double d = to - pos;
		if (fabs(d) > step) d = (d > 0) ? step : -step;
		window(d, !first);
		first = 0;
	}
}

static void stop(double seconds)
{
	int	n = (int)(seconds * 10);

	while (n-- > 0) window(0, 1);
}

/* ---- scenarios ---- */

/* The demo hump, over at the car's line-following speed (~48 mm/s). */
static void demo_over(void);
/* As the car read it: the hump must close within 160 mm of level after
   the crest (HUMP_TOP_MAX_MM 120 + windows), not 250 mm on (S6 attempt 1). */
static void demo_as_read(void);

typedef struct {
	const char	*name;
	void		(*run)(void);
	UW		humps;
} SCENARIO;

/* Straight over at 50 mm/s, no stops. */
static void over(void)
{
	USE(HUMP); stop(2); drive(700, 50); stop(2);
}

/* Up, stopped on top, backed down, stopped (log 3, HUMP 2 + 3). */
static void up_stop_back(void)
{
	USE(HUMP); stop(2); drive(310, 50); stop(2); drive(100, 60); stop(2);
}

/* Only 3.5 mm up, rounds to a counted 4 mm (log 5, HUMP 3 + 4). */
static void small_up_stop_back(void)
{
	USE(HUMP); stop(2); drive(245, 40); stop(2); drive(100, 40); stop(2);
}

/* Stopped on top, nudged 7 mm up the slope, then backed down (log 6). */
static void nudge_on_top(void)
{
	USE(HUMP); stop(2); drive(280, 50); stop(2);
	drive(287, 20); stop(2); drive(100, 60); stop(2);
}

/* As above, then a second climb from a stop at the foot (log 7). */
static void again_from_stop(void)
{
	USE(HUMP); stop(2); drive(300, 50); stop(2); drive(100, 60); stop(2);
	drive(300, 50); stop(2); drive(100, 60); stop(2);
}

/* Back down without stopping on top: the old path (log 2, HUMP 1). */
static void up_back_no_stop(void)
{
	USE(HUMP); stop(2); drive(300, 50); drive(100, 60); stop(2);
}

/* Stopped level on the crest, then over and down the far side. */
static void crest_stop(void)
{
	USE(FLAT_TOP); stop(2); drive(340, 50); stop(2); drive(700, 50); stop(2);
}

/* Reversed over the whole hump. */
static void reverse_over(void)
{
	USE(HUMP); pos = 700; stop(2); drive(0, 50); stop(2);
}

/* Pivoted up (the encoders count 60% of the climb), stopped on top,
   backed down straight: the way down reads longer than the way up and
   once ran out mid-slope, the rest counted as a climb (log 9, pass A). */
static void pivot_up_stop_back(void)
{
	USE(HUMP); stop(2);
	enc_scale = 0.6; drive(310, 50); stop(2);
	enc_scale = 1.0; drive(100, 60); stop(2);
}

/* Course humps, one run. */
static void two_humps(void)
{
	USE(TWO); stop(2); drive(1300, 50); stop(2);
}

/* The higher hump first: the highest (FR-12, what E imu and S.hgt carry)
   stays 23 mm after the lower one, which is only the last. */
static void two_humps_high_first(void)
{
	USE(TWO_HIGH_FIRST); stop(2); drive(1300, 50); stop(2);
	if (hump_get_max_mm() < 21 || hump_get_max_mm() > 25
	    || hump_get_last_mm() < 7 || hump_get_last_mm() > 11) _exit(3);
	hump_mission_reset();
	if (hump_get_max_mm() != 0) _exit(3);	/* a new mission: 0 again */
	stop(2); drive(1800, 50); stop(2);	/* flat after the humps */
}

/* Up, stop, back, then on over the hump: the climbs are one hump each. */
static void mixed(void)
{
	USE(HUMP); stop(2); drive(300, 50); stop(2); drive(100, 60); stop(2);
	drive(700, 50); stop(2);
}

/* Long level top: over and down the far side without stopping.  The
   hump must not end on the top and count the way down again. */
static void long_top_over(void)
{
	USE(LONG_TOP); stop(2); drive(700, 70); stop(2);
}

/* Long level top: up, stopped, nudged on, backed down over the top and
   the slope (log 11: HUMP 5, then the way down counted as HUMP 6). */
static void long_top_nudge_back(void)
{
	USE(LONG_TOP); stop(2); drive(325, 40); stop(2);
	drive(336, 20); stop(1); drive(107, 60); stop(2);
}

/* Log 11 whole: three climbs, each stopped on top and backed down, the
   middle one missed after the false count. */
static void long_top_three(void)
{
	USE(LONG_TOP); stop(2);
	drive(325, 40); stop(2); drive(336, 20); stop(1); drive(107, 60); stop(2);
	drive(330, 60); stop(1); drive(16, 80); stop(2);
	drive(320, 40); stop(2);
}

/* Up, stopped on the long top, carried back to the start, over again
   (log 14: the second climb was taken as still being on top). */
static void long_top_carry(void)
{
	USE(LONG_TOP); stop(2); drive(340, 50); stop(2);
	carry(100); drive(700, 50); stop(2);
}

/* A fault halfway up: the hump is dropped, then the next pass counts. */
static void abort_mid_climb(void)
{
	USE(HUMP); stop(2); drive(280, 50); hump_abort();
	drive(100, 60); stop(2); drive(700, 50); stop(2);
}

/* Stopped on top, the mission paused, then backed down (2026-10-28:
   the pause forgot it was on top, and the way down counted as HUMP 2). */
static void pause_on_top(void)
{
	USE(HUMP); stop(2); drive(310, 50); stop(2); hump_pause();
	drive(100, 60); stop(2);
}

/* Paused halfway up: that climb is dropped, then the next pass counts. */
static void pause_mid_climb(void)
{
	USE(HUMP); stop(2); drive(280, 50); hump_pause();
	drive(100, 60); stop(2); drive(700, 50); stop(2);
}

/* Stopped part way up, 2.5 mm, then pushed on over (2026-10-28: "ignored,
   2.4 mm", then the rest of the climb was only followed as "on top"). */
static void stop_on_slope_over(void)
{
	USE(HUMP); stop(2); drive(230, 50); stop(2); drive(700, 50); stop(2);
}

/* Stopped part way up, under 2 mm, then pushed on over (2026-10-28:
   "ignored, 2.1 mm", then a new climb from 0 was "ignored, 3.0 mm"). */
static void stop_low_on_slope_over(void)
{
	USE(HUMP); stop(2); drive(220, 50); stop(2); drive(700, 50); stop(2);
}

/* Stopped part way up, then backed down: still one hump at most, and
   too low to count. */
static void stop_on_slope_back(void)
{
	USE(HUMP); stop(2); drive(230, 50); stop(2); drive(100, 50); stop(2);
}

/* Off a drop the hump never reads as DESCENDING, so only the
   HUMP_TOP_MAX_MM cap ends it, before the next wedge. */
static void wedges(void)
{
	USE(WEDGES); stop(2); drive(1500, 70); stop(2);
}

/* Flat floor, back and forth: nothing to count. */
static void flat(void)
{
	USE(HUMP); stop(2); drive(150, 50); drive(20, 50); stop(2);
	drive(150, 80); stop(2);
}

static void demo_over(void)
{
	USE(DEMO); stop(2); drive(1000, 48); stop(2);
}

static void demo_as_read(void)
{
	USE(DEMO_AS_READ); stop(2); drive(297, 48);
	while (hump_get_state() != HUMP_NONE && pos < 297 + 400) window(4.8, 1);
	if (pos - 297 > 160) _exit(3);		/* closed too late: FAIL */
	drive(1000, 48); stop(2);
}

/* S6 crossing 4 (2 Nov): GP21 ~80 mm after the hump, before the 120 mm
   level close, dropped an 11 mm hump.  Past the top, a pause counts it,
   at the right height, and driving on does not count it again. */
static void demo_as_read_pause(void)
{
	USE(DEMO_AS_READ); stop(2); drive(297 + 80, 48);
	if (hump_get_state() == HUMP_NONE) _exit(3);	/* not open: no test */
	hump_pause();
	if (hump_get_count() != 1 || hump_get_last_mm() < 9
	    || hump_get_last_mm() > 11) _exit(3);
	stop(2); drive(1500, 48); stop(2);
}

/* The same at the end of the mission (the hump near the end of the line). */
static void demo_as_read_end(void)
{
	USE(DEMO_AS_READ); stop(2); drive(297 + 80, 48);
	hump_mission_end();
}

/* Paused on the way down the demo hump: counted once, not again after. */
static void demo_pause_way_down(void)
{
	USE(DEMO); stop(2); drive(360, 48); hump_pause();
	stop(2); drive(1000, 48); stop(2);
}

/* Mission ended half way up: a part height is not counted. */
static void end_mid_climb(void)
{
	USE(HUMP); stop(2); drive(280, 50); hump_mission_end();
}

/* ---- ramp length (19 Nov): line following at ~48 mm/s ---- */

/* NFR-05 target: the highest hump within 2 mm of h (20 % above 10 mm).
   Fails the scenario (no count reported) if not. */
static void height_ok(double h)
{
	double	tol = (h > 10) ? h / 5 : 2;

	if (hump_get_count() > 0
	    && fabs((double)hump_get_max_mm() - h) > tol) _exit(3);
}

static void over_at(const PT *t, int n, double to, double speed, double h)
{
	trk = t; ntrk = n;
	stop(2); drive(to, speed); stop(2);
	height_ok(h);
}

#define OVER(t, to, v, h)	over_at((t), (int)(sizeof (t) / sizeof (t)[0]), \
					(to), (v), (h))

static void sym_300(void)	{ OVER(SYM_300, 1000, 48, 10); }
static void sym_360(void)	{ OVER(SYM_360, 1000, 48, 10); }
static void sym_440(void)	{ OVER(SYM_440, 1200, 48, 10); }
static void sym_600(void)	{ OVER(SYM_600, 1400, 48, 10); }
static void sym8_300(void)	{ OVER(SYM8_300, 1000, 48, 8); }
static void sym12_440(void)	{ OVER(SYM12_440, 1200, 48, 12); }
static void top_200(void)	{ OVER(TOP_200, 1200, 48, 10); }
static void top_400(void)	{ OVER(TOP_400, 1400, 48, 10); }
static void top_200_fast(void)	{ OVER(TOP_200, 1200, 150, 10); }
static void top_then_hump(void)	{ OVER(TOP_THEN_HUMP, 1600, 48, 10); }
static void gentle_top(void)	{ OVER(GENTLE_TOP, 1500, 48, 10); }
/* Read low by design (the toe is never seen): the count is the test. */
static void late_kink(void)
{
	USE(LATE_KINK); stop(2); drive(1200, 48); stop(2);
}
static void floor_30mg(void)	{ OVER(FLOOR_30MG, 1200, 48, 0); }
static void steep_far(void)	{ OVER(STEEP_FAR, 1700, 48, 10); }
/* The demo hump and the practice ramp at the speed cap (150 mm/s). */
static void demo_fast(void)	{ OVER(DEMO, 1000, 150, 10); }
static void long_top_fast(void)	{ OVER(LONG_TOP, 1000, 150, 10); }

/* Up onto the 200 mm top, stopped there (an obstacle), backed down the
   way up: backing down is downhill, not a second hump. */
static void top_200_stop_back(void)
{
	USE(TOP_200); stop(2); drive(450, 48); stop(2); drive(100, 60); stop(2);
	height_ok(10);
}

/* Expected counts that follow the hump.h switches.  The defaults keep
   what the car build does today, limits included: a 45 mg climb is not
   seen, and a level top longer than HUMP_TOP_MAX_MM counts twice. */
#define SEEN_45MG	((HUMP_ENTER_MG <= 45) ? 1 : 0)
#define LONG_TOP_N	(HUMP_WAY_DOWN ? 1 : 2)

static const SCENARIO scenarios[] = {
	{ "straight over",			over,			1 },
	{ "up, stop on top, back down",		up_stop_back,		1 },
	{ "3.5 mm up, stop, back down",		small_up_stop_back,	1 },
	{ "stop on top, nudge, back down",	nudge_on_top,		1 },
	{ "twice up/back, stop at foot",	again_from_stop,	2 },
	{ "up, back without stopping",		up_back_no_stop,	1 },
	{ "stop on a level crest, go on",	crest_stop,		1 },
	{ "reverse over",			reverse_over,		1 },
	{ "pivot up, stop, back down",		pivot_up_stop_back,	1 },
	{ "two course humps",			two_humps,		2 },
	{ "higher first: highest kept, reset",	two_humps_high_first,	0 },
	{ "up/stop/back, then over",		mixed,			2 },
	{ "long top, straight over",		long_top_over,		1 },
	{ "long top, nudge, back down",		long_top_nudge_back,	1 },
	{ "long top, three up/back",		long_top_three,		3 },
	{ "two wedges, drop-off far sides",	wedges,			2 },
	{ "long top, stop, carried back, over",	long_top_carry,		2 },
	{ "fault mid-climb, then over",		abort_mid_climb,	1 },
	{ "stop on top, pause, back down",	pause_on_top,		1 },
	{ "pause mid-climb, then over",		pause_mid_climb,	1 },
	{ "stop 2.5 mm up the slope, on over",	stop_on_slope_over,	1 },
	{ "stop 1.7 mm up the slope, on over",	stop_low_on_slope_over,	1 },
	{ "stop part way up, back down",	stop_on_slope_back,	0 },
	{ "flat floor only",			flat,			0 },
	{ "demo hump 195 mm, 10 mm, over",	demo_over,		1 },
	{ "demo hump as read: closes in 160 mm",	demo_as_read,	1 },
	{ "demo as read, paused 80 mm after",	demo_as_read_pause,	1 },
	{ "demo as read, mission ends 80 mm after", demo_as_read_end,	1 },
	{ "demo hump, paused on the way down",	demo_pause_way_down,	1 },
	{ "mission ends mid-climb",		end_mid_climb,		0 },
	/* Ramp length (19 Nov).  Heights within NFR-05 where counted. */
	{ "10 mm over 300 mm (3.8 deg)",	sym_300,		1 },
	{ "10 mm over 360 mm (3.2 deg)",	sym_360,		1 },
	{ "10 mm over 440 mm (2.6 deg)",	sym_440,		SEEN_45MG },
	{ "10 mm over 600 mm (1.9 deg)",	sym_600,		0 },
	{ "8 mm over 300 mm (3.1 deg)",		sym8_300,		1 },
	{ "12 mm over 440 mm (3.1 deg)",	sym12_440,		1 },
	{ "demo hump at 150 mm/s",		demo_fast,		1 },
	{ "practice ramp at 150 mm/s",		long_top_fast,		1 },
	{ "200 mm level top",			top_200,		LONG_TOP_N },
	{ "400 mm level top",			top_400,		LONG_TOP_N },
	{ "200 mm level top at 150 mm/s",	top_200_fast,		LONG_TOP_N },
	{ "200 mm top, stop, back down",	top_200_stop_back,	LONG_TOP_N },
	{ "200 mm top, then a second ramp",	top_then_hump,	LONG_TOP_N + 1 },
	{ "gentle (2.6 deg), 200 mm top",	gentle_top,
	  SEEN_45MG ? LONG_TOP_N : 0 },
	{ "climb read late, way down eases",	late_kink,
	  HUMP_WAY_DOWN ? 1 : 2 },
	{ "floor tilted 30 mg for 300 mm",	floor_30mg,		0 },
	{ "unseen way up, steep far sides",	steep_far,		2 },
};

int main(int argc, char **argv)
{
	int	i, failures = 0, n = (int)(sizeof scenarios / sizeof scenarios[0]);

	verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("hump.c built with HUMP_ENTER_MG %d, HUMP_WAY_DOWN %d\n",
	       HUMP_ENTER_MG, HUMP_WAY_DOWN);

	for (i = 0; i < n; i++) {
		int	fd[2], status;
		UW	got = (UW)-1;
		pid_t	pid;

		if (pipe(fd) != 0) return 2;
		if (verbose) printf("--- %s\n", scenarios[i].name);
		pid = fork();
		if (pid == 0) {
			close(fd[0]);
			scenarios[i].run();
			got = hump_get_count();
			if (write(fd[1], &got, sizeof got) != sizeof got) _exit(2);
			_exit(0);
		}
		close(fd[1]);
		if (read(fd[0], &got, sizeof got) != sizeof got) got = (UW)-1;
		close(fd[0]);
		waitpid(pid, &status, 0);

		if (got == scenarios[i].humps) {
			printf("ok    %-34s %u hump(s)\n", scenarios[i].name, got);
		} else {
			printf("FAIL  %-34s %d hump(s), expected %u\n",
			       scenarios[i].name, (int)got, scenarios[i].humps);
			failures++;
		}
	}
	printf("%d/%d scenarios passed\n", n - failures, n);
	return failures ? 1 : 0;
}
