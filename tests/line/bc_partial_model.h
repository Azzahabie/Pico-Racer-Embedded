/* A model of damaged analog reads of *X* for the partial-read decoder's
   property test (sim, test_bc_analog_partial.c).  Nothing here was
   measured beyond what is named: the three sheet scales (narrow 3.4 /
   5.1 / 6.9 mm), and from the car reads of 20 Nov (M) bars reading thinner
   and spaces wider by up to 0.7 narrow (17:44 and 18:00 reads), thin
   elements missed (a thin bar lost merges its two spaces, a thin space
   lost merges its two bars), the first or last element cut short, and
   specks.  Widths in um, bar first, in reading order.

   A read: the 29 printed elements of *X* (wide / narrow 2.2 to 3.0), then
     - `miss` narrow interior elements missed (with their neighbours, one
       measured element; two side by side give one of five);
     - in clip_pct % of reads 2 to clip_max printed elements never seen at
       one end, and in both_pct % of those 2 more at the other;
     - each element measured as printed - b (bar) or + b (space), b =
       b_lo..b_hi narrow, times a speed error drift across the read and a
       wobble (turning), times 1 + rel noise, plus s_lo..s_hi narrow noise;
     - in speck_pct % of reads a speck splits one element at least 1.5
       narrow wide: a light speck in a bar reads as a space (+b), a dark one
       in a space as a bar (-b), and is not seen if under 0.3 mm;
     - in part_pct % of reads the first or last element cut to 50-100 %. */
#ifndef TESTS_LINE_BC_PARTIAL_MODEL_H
#define TESTS_LINE_BC_PARTIAL_MODEL_H

#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t pm_rng = 1u;

static uint32_t pm_rnd(void)
{
	pm_rng ^= pm_rng << 13;
	pm_rng ^= pm_rng >> 17;
	pm_rng ^= pm_rng << 5;
	return pm_rng;
}

static double pm_u(void)
{
	return (double)pm_rnd() / 4294967296.0;
}

static double pm_gauss(void)
{
	double u = pm_u();
	double v = pm_u();

	u = (u < 1e-12) ? 1e-12 : u;
	return sqrt(-2.0 * log(u)) * cos(6.283185307179586 * v);
}

/* A-Z then '*', element order b s b s b s b s b, 1 = wide */
static const char *const pm_pat[27] = {
	"100001001", "001001001", "101001000", "000011001", "100011000", "001011000",
	"000001101", "100001100", "001001100", "000011100", "100000011", "001000011",
	"101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
	"001000110", "000010110", "110000001", "011000001", "111000000", "010010001",
	"110010000", "011010000", "010010100"
};

typedef struct {
	int missed;		/* thin elements missed */
	int specks;
	int c0, c1;		/* printed elements clipped, reading order */
} pm_truth_t;

typedef struct {
	double w;		/* printed width */
	int    bar;
	int    single_narrow;	/* one printed narrow element */
	int    first, last;	/* printed indexes, reading order */
} pm_el_t;

typedef struct {
	double r_lo, r_hi;	/* wide / narrow */
	double b_lo, b_hi;	/* blur bias, x narrow */
	double s_lo, s_hi;	/* noise, x narrow */
	double rel;		/* multiplicative noise, at most */
	double drift;		/* speed error across the read, at most */
	double wobble;		/* speed wobble, at most */
	int    miss;
	int    clip_pct;
	int    clip_max;
	int    both_pct;
	int    speck_pct;
	int    part_pct;
} pm_opt_t;

static void pm_opt_default(pm_opt_t *o)
{
	o->r_lo = 2.2;
	o->r_hi = 3.0;
	o->b_lo = 0.05;
	o->b_hi = 0.7;
	o->s_lo = 0.03;
	o->s_hi = 0.15;
	o->rel = 0.05;
	o->drift = 0.10;
	o->wobble = 0.0;
	o->miss = 0;
	o->clip_pct = 25;
	o->clip_max = 10;
	o->both_pct = 10;
	o->speck_pct = 10;
	o->part_pct = 15;
}

static int pm_read(char letter, int rev, const pm_opt_t *o, uint32_t *w, pm_truth_t *t)
{
	pm_el_t el[40];
	double  m[48];
	int     n = 29;
	int     k;
	int     q;
	int     sz = (int)(pm_rnd() % 3u);
	double  nar = ((sz == 0) ? 3.4 : ((sz == 1) ? 5.1 : 6.9)) * (0.9 + 0.2 * pm_u());
	double  r = o->r_lo + (o->r_hi - o->r_lo) * pm_u();
	double  b = nar * (o->b_lo + (o->b_hi - o->b_lo) * pm_u());
	double  sig = nar * (o->s_lo + (o->s_hi - o->s_lo) * pm_u());
	double  rel = o->rel * pm_u();
	double  drift = o->drift * (2.0 * pm_u() - 1.0);
	double  wob = o->wobble * pm_u();
	double  wf = 0.5 + 1.5 * pm_u();
	double  wph = 6.283185307179586 * pm_u();
	double  pos = 0.0;
	double  total = 0.0;

	memset(t, 0, sizeof(*t));
	for (k = 0; k < 29; k++) {
		int const   kk = rev ? 28 - k : k;
		const char *pp = pm_pat[((kk / 10) == 1) ? (letter - 'A') : 26];
		int const   wide = ((kk % 10) < 9) && (pp[kk % 10] == '1');

		el[k].w = wide ? r * nar : nar;
		el[k].bar = ((k % 2) == 0);
		el[k].single_narrow = !wide;
		el[k].first = k;
		el[k].last = k;
	}
	for (q = 0; q < o->miss; q++) {
		int tries;

		for (tries = 0; tries < 400; tries++) {
			int const a = 1 + (int)(pm_rnd() % (uint32_t)(n - 2));

			if (!el[a].single_narrow) {
				continue;
			}
			el[a - 1].w += el[a].w + el[a + 1].w;
			el[a - 1].last = el[a + 1].last;
			el[a - 1].single_narrow = 0;
			memmove(&el[a], &el[a + 2], (size_t)(n - a - 2) * sizeof(el[0]));
			n -= 2;
			t->missed++;
			break;
		}
	}
	if ((int)(pm_rnd() % 100u) < o->clip_pct) {
		int const c = 2 * (1 + (int)(pm_rnd() % (uint32_t)(o->clip_max / 2)));
		int       cut = 0;

		if (pm_rnd() & 1u) {
			while ((cut < n) && (el[cut].first < c)) {
				cut++;
			}
			cut += (cut % 2);		/* start on a bar */
			if ((n - cut) >= 9) {
				memmove(&el[0], &el[cut], (size_t)(n - cut) * sizeof(el[0]));
				n -= cut;
				t->c0 = el[0].first;
			}
		} else {
			while ((cut < n) && (el[n - 1 - cut].last > (28 - c))) {
				cut++;
			}
			cut += (cut % 2);
			if ((n - cut) >= 9) {
				n -= cut;
				t->c1 = 28 - el[n - 1].last;
			}
		}
		if (((int)(pm_rnd() % 100u) < o->both_pct) && (n > 11)) {
			if (t->c0 > 0) {
				n -= 2;
				t->c1 = 28 - el[n - 1].last;
			} else {
				memmove(&el[0], &el[2], (size_t)(n - 2) * sizeof(el[0]));
				n -= 2;
				t->c0 = el[0].first;
			}
		}
	}
	for (k = 0; k < n; k++) {
		total += el[k].w;
	}
	for (k = 0; k < n; k++) {
		double const mid = (pos + (el[k].w / 2.0)) / total;
		double const scale = (1.0 + (drift * ((2.0 * mid) - 1.0))) *
				     (1.0 + (wob * sin((6.283185307179586 * wf * mid) + wph)));
		double       x = ((el[k].w + (el[k].bar ? -b : b)) * scale * (1.0 + (rel * pm_gauss()))) +
				 (sig * pm_gauss());

		pos += el[k].w;
		m[k] = (x < 0.3) ? 0.3 : x;
	}
	if ((int)(pm_rnd() % 100u) < o->speck_pct) {
		int tries;

		for (tries = 0; tries < 100; tries++) {
			int const    a = (int)(pm_rnd() % (uint32_t)n);
			double const sp_p = nar * (0.15 + (0.35 * pm_u()));
			double const sp = ((a % 2) == 0) ? sp_p + b : sp_p - b;
			double       left;
			double       right;

			if (m[a] < (1.5 * nar)) {
				continue;
			}
			if (sp < 0.3) {
				break;			/* too faint to see */
			}
			left = (m[a] - sp) * (0.25 + (0.5 * pm_u()));
			right = m[a] - sp - left;
			if (right < 0.3) {
				break;
			}
			memmove(&m[a + 3], &m[a + 1], (size_t)(n - a - 1) * sizeof(m[0]));
			m[a] = left;
			m[a + 1] = sp;
			m[a + 2] = right;
			n += 2;
			t->specks++;
			break;
		}
	}
	if ((int)(pm_rnd() % 100u) < o->part_pct) {
		if (pm_rnd() & 1u) {
			m[0] *= 0.5 + (0.5 * pm_u());
		} else {
			m[n - 1] *= 0.5 + (0.5 * pm_u());
		}
	}
	for (k = 0; k < n; k++) {
		w[k] = (uint32_t)lround(((m[k] < 0.3) ? 0.3 : m[k]) * 1000.0);
	}
	return n;
}

/* Not a code: n odd 13..35; kind 0 any width 1-20 mm, 1 two widths
   (narrow, 2-3 x wide, a third wide) +-15 %, 2 the same with bars
   thinner and spaces wider (as the sensor). */
static int pm_noise(uint32_t *w, int kind)
{
	int const n = 13 + (2 * (int)(pm_rnd() % 12u));
	double    nar = 2.0 + (5.0 * pm_u());
	int       k;

	for (k = 0; k < n; k++) {
		double x;

		if (kind == 0) {
			x = 1.0 + (19.0 * pm_u());
		} else if (kind == 1) {
			x = ((pm_rnd() % 3u) == 0u) ? nar * (2.0 + pm_u()) : nar;
			x *= 0.85 + (0.3 * pm_u());
		} else {
			double const b = nar * 0.5 * pm_u();

			x = ((pm_rnd() % 3u) == 0u) ? nar * (2.0 + pm_u()) : nar;
			x += ((k % 2) == 0) ? -b : b;
			x = (x < 0.5) ? 0.5 : x;
			x *= 0.9 + (0.2 * pm_u());
		}
		w[k] = (uint32_t)lround(x * 1000.0);
	}
	return n;
}

#endif
