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
/* Two humps, 9 mm and 23 mm, 300 mm apart. */
static const PT TWO[] = {
	{ 0, 0 }, { 200, 0 }, { 310, 9 }, { 420, 0 }, { 720, 0 },
	{ 900, 23 }, { 1080, 0 }, { 2000, 0 },
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
	{ "up/stop/back, then over",		mixed,			2 },
	{ "long top, straight over",		long_top_over,		1 },
	{ "long top, nudge, back down",		long_top_nudge_back,	1 },
	{ "long top, three up/back",		long_top_three,		3 },
	{ "two wedges, drop-off far sides",	wedges,			2 },
	{ "long top, stop, carried back, over",	long_top_carry,		2 },
	{ "flat floor only",			flat,			0 },
};

int main(int argc, char **argv)
{
	int	i, failures = 0, n = (int)(sizeof scenarios / sizeof scenarios[0]);

	verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
	setvbuf(stdout, NULL, _IONBF, 0);

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
