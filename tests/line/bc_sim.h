/* A model of the analog barcode sensor for the BARCODE_SENSOR_ANALOG tests
   (sim, nothing measured beyond the levels named): black segments on the
   sensor's path, a square spot BC_SIM_SPOT_MM wide that averages them, so
   the reading is white + (black - white) x the part of the spot on black,
   plus a small repeatable noise.  The levels are the lab's of 20 Nov (M):
   white about 200, thick bars 2400-3900.  Positions in mm. */
#ifndef TESTS_LINE_BC_SIM_H
#define TESTS_LINE_BC_SIM_H

#include <stdint.h>
#include <string.h>

#define BC_SIM_WHITE	(200.0)
#define BC_SIM_BLACK	(2600.0)
#define BC_SIM_SPOT_MM	(4.0)
#define BC_SIM_SEGS	(64)

static const char *const bc_sim_patterns[27] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"	/* '*' */
};

typedef struct {
	double a;
	double b;
	double level;		/* reading over it, fully covered */
} bc_sim_seg_t;

static bc_sim_seg_t bc_sim_seg[BC_SIM_SEGS];
static unsigned bc_sim_n;
static uint32_t bc_sim_rand = 12345u;

static void bc_sim_clear(void)
{
	bc_sim_n = 0u;
}

static void bc_sim_black(double a, double b, double level)
{
	if (bc_sim_n < BC_SIM_SEGS) {
		bc_sim_seg[bc_sim_n].a = a;
		bc_sim_seg[bc_sim_n].b = b;
		bc_sim_seg[bc_sim_n].level = level;
		bc_sim_n++;
	}
}

/* The widths of *L* (29 elements, bar first), read backwards if rev. */
static void bc_sim_widths(char letter, double n, double r, int rev, double *w)
{
	char text[3] = { '*', letter, '*' };
	unsigned k;
	unsigned i;
	unsigned o = 0u;

	for (k = 0u; k < 3u; ++k) {
		const char *p = bc_sim_patterns[(text[k] == '*') ? 26 : (text[k] - 'A')];

		for (i = 0u; i < 9u; ++i) {
			w[o++] = (p[i] == 'W') ? n * r : n;
		}
		if (k < 2u) {
			w[o++] = n;		/* inter-character gap */
		}
	}
	if (rev) {
		for (i = 0u; i < 14u; ++i) {
			double t = w[i];

			w[i] = w[28u - i];
			w[28u - i] = t;
		}
	}
}

/* A sheet *L* from x0: thick bars at BC_SIM_BLACK, narrow bars at
   thin_level; returns the end of its last bar. */
static double bc_sim_sheet(char letter, double n, double r, int rev, double x0,
			   double thin_level)
{
	double w[29];
	double x = x0;
	unsigned i;

	bc_sim_widths(letter, n, r, rev, w);
	for (i = 0u; i < 29u; ++i) {
		if ((i % 2u) == 0u) {
			bc_sim_black(x, x + w[i], (w[i] > n * 1.01) ? BC_SIM_BLACK : thin_level);
		}
		x += w[i];
	}
	return x;
}

/* The reading at x: the spot's average, plus noise of +-8 counts. */
static uint16_t bc_sim_read(double x)
{
	double h = BC_SIM_SPOT_MM / 2.0;
	double v = BC_SIM_WHITE;
	unsigned i;
	int noise;

	for (i = 0u; i < bc_sim_n; ++i) {
		double a = (bc_sim_seg[i].a > x - h) ? bc_sim_seg[i].a : x - h;
		double b = (bc_sim_seg[i].b < x + h) ? bc_sim_seg[i].b : x + h;

		if (b > a) {
			v += (bc_sim_seg[i].level - BC_SIM_WHITE) * (b - a) / BC_SIM_SPOT_MM;
		}
	}
	bc_sim_rand = bc_sim_rand * 1103515245u + 12345u;
	noise = (int)((bc_sim_rand >> 16) % 17u) - 8;
	v += (double)noise;
	if (v < 0.0) {
		v = 0.0;
	}
	if (v > 4095.0) {
		v = 4095.0;
	}
	return (uint16_t)v;
}

#endif
