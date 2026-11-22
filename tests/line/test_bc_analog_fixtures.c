/* Host test: the car traces of 20 Nov (tests/line/fixtures, see the README
   there) through the whole analog barcode path, one 1 ms sample at a time
   as line_task.c runs it with BARCODE_SENSOR_ANALOG 1:
     ADC reading -> bc_analog_front.c (floor, darkness, travel from
     distance_state) -> the capture (bc_analog_capture.c) -> on BC_DONE
     bc_finish() (barcode_task's part, core 0) -> bc_resume().
   Nothing here runs on the car.

   The traces (M, car log, 20 Nov) are the largest reading of each 5 ms
   window, rebuilt from the test image's "[bc] trace" dumps; gaps between
   dumps are filled with white.  Here each window's value is held for its
   5 ms (sim).  The car's distance comes from "[motion] distance_state"
   lines about every 200 ms; it is interpolated and then published as
   motion_task does (whole mm every 10 ms, the counter +1), so the front
   end sees the same 1 mm steps as on the car (sim).  Before each trace:
   1.5 s of its first reading at 32 mm/s, so the capture sees a quiet
   zone; after it: 2.5 s of that reading at 32 mm/s, so the end
   of a code (40 mm of white) can come, as the traces stop soon after the
   last bar (sim, both synthetic).

   Expected: run3 (the car decoded 'Z' with the test image) -> 'Z';
   run1 ('Z' whose first four elements never rose above white) -> 'Z'
   with BC_REPAIR4 1; run4 (the track's sheet, faint thin bars) and run2
   (faint, aborted on the car) are reported, and may give no letter, but
   never a letter other than the one named. */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "../../app_program/line_task/bc_analog_front.h"
#include "../../app_program/line_task/bc_analog_capture.h"
#include "../../app_program/motor_task/motion_ds.h"

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures/"
#endif

#define MAX_V	(8000)
#define MAX_D	(400)
#define PRE_MS	(1500)
#define POST_MS	(2500)

typedef struct {
	int nv;
	int v[MAX_V];
	int nd;
	double dt[MAX_D];
	double dd[MAX_D];
} trace_t;

static trace_t tr;

/* The two arrays of the fixture's JSON: "values" and "dist_mm". */
static const char *after(const char *s, const char *key)
{
	const char *p = strstr(s, key);

	return (p == NULL) ? NULL : p + strlen(key);
}

static int load(const char *name)
{
	static char buf[65536];
	char path[256];
	FILE *f;
	size_t len;
	const char *p;
	char *e;
	long last = -1;
	long unwrap = 0;

	(void)snprintf(path, sizeof(path), "%s%s.json", FIXTURE_DIR, name);
	f = fopen(path, "r");
	if (f == NULL) {
		return 0;
	}
	len = fread(buf, 1u, sizeof(buf) - 1u, f);
	(void)fclose(f);
	buf[len] = '\0';
	tr.nv = 0;
	tr.nd = 0;
	p = after(buf, "\"values\":[");
	while ((p != NULL) && (*p != ']') && (tr.nv < MAX_V)) {
		tr.v[tr.nv++] = (int)strtol(p, &e, 10);
		p = (*e == ',') ? e + 1 : e;
	}
	p = after(buf, "\"dist_mm\":[");
	while ((p != NULL) && (*p == '[') && (tr.nd < MAX_D)) {
		long t = strtol(p + 1, &e, 10);
		long d = strtol(e + 1, &e, 10);

		/* dist_mm is modulo 32768 */
		if ((last >= 0) && (d < last - 16384)) {
			unwrap += 32768;
		} else if ((last >= 0) && (d > last + 16384)) {
			unwrap -= 32768;
		}
		last = d;
		tr.dt[tr.nd] = (double)t;
		tr.dd[tr.nd] = (double)(d + unwrap);
		tr.nd++;
		p = e + 1;			/* past ']' */
		p = (*p == ',') ? p + 1 : p;
	}
	return (tr.nv > 0) && (tr.nd > 2);
}

/* The car's distance (mm) at t ms from the trace start, interpolated. */
static double dist_at(double t)
{
	int i;

	for (i = 1; i < tr.nd; i++) {
		if (t <= tr.dt[i]) {
			break;
		}
	}
	if (i >= tr.nd) {
		i = tr.nd - 1;
	}
	return tr.dd[i - 1] + (t - tr.dt[i - 1]) * (tr.dd[i] - tr.dd[i - 1]) /
	       (tr.dt[i] - tr.dt[i - 1]);
}

#ifdef FIXTURE_DUMP
/* -DFIXTURE_DUMP: every reading of every code that ends (diagnosis) */
static void dump(const char *name, int t, const bc_capture_t *bc, bc_result_t r,
		 const bc_out_t *o)
{
	uint32_t w[BC_MAX_ELEMENTS + 1u];
	uint8_t k;
	uint16_t n;
	uint16_t j;

	(void)printf("  %s t=%d result %d ch '%c' reason %u pass %u\n", name, t, (int)r,
		     (o->ch != '\0') ? o->ch : '-', (unsigned)o->reason, (unsigned)o->pass);
	for (k = 0u; k < BC_READINGS; k++) {
		n = bc_reading(bc, k, w);
		(void)printf("    reading %u: %2u:", (unsigned)k, (unsigned)n);
		for (j = 0u; j < n; j++) {
			(void)printf(" %c%.1f", ((j & 1u) != 0u) ? 'w' : 'B', (double)w[j] / 1000.0);
		}
		(void)printf("\n");
	}
}
#endif

typedef struct {
	char letter;		/* first valid letter, '\0' none */
	unsigned valid;
	unsigned rejects;
	unsigned not_barcode;
	uint8_t pass;
	bool reversed;
	uint8_t last_reason;
	uint16_t last_elements;
	double travel_err_pct;	/* front end travel vs the interpolated one */
} result_t;

static void run(const char *name, unsigned span_check, result_t *res)
{
	bc_front_t f;
	bc_capture_t bc;
	bc_out_t o;
	uint32_t ds_ctr = 0u;
	uint32_t ds_word;
	double d0 = 0.0;
	double d_post0 = 0.0;
	double d_true = 0.0;
	double travel_um = 0.0;
	int t;
	int t_end;

	(void)span_check;
	(void)memset(res, 0, sizeof(*res));
	if (!load(name)) {
		(void)printf("  %s: fixture missing\n", name);
		CHECK(0);
		return;
	}
	bc_front_init(&f);
	bc_init(&bc);
	ds_word = motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0);
	t_end = tr.nv * 5 + POST_MS;
	d_post0 = dist_at((double)(tr.nv * 5));
	for (t = -PRE_MS; t < t_end; t++) {
		int raw;
		double d;
		uint32_t step;
		bc_result_t r;

		if (t < 0) {
			raw = tr.v[0];
			d = dist_at(0.0) + (double)t * 0.032;
		} else if (t < tr.nv * 5) {
			raw = tr.v[t / 5];
			d = dist_at((double)t);
		} else {
			raw = tr.v[0];
			d = d_post0 + (double)(t - tr.nv * 5) * 0.032;
		}
		if (t == -PRE_MS) {
			d0 = d;
		}
		if (((t + PRE_MS) % 10) == 0) {
			/* motion_task's publish: whole mm (floor), the counter +1 */
			double whole = d - d0 + 1000.0;

			ds_ctr++;
			ds_word = motion_ds_pack((int32_t)whole, MOTION_DS_STEADY, ds_ctr, 0);
		}
		bc_front_step(&f, (uint16_t)raw, ds_word, true, 1u);
		step = bc_front_take_step(&f);
		travel_um += (double)step;
		d_true = d - d0;
		r = bc_sample(&bc, f.norm, step, &o);
		if (r == BC_DONE) {
			r = bc_finish(&bc, &o);
#ifdef FIXTURE_DUMP
			dump(name, t, &bc, r, &o);
#endif
			bc_resume(&bc, r);
			if (r == BC_NONE) {
				res->not_barcode++;
			}
		}
		if (r == BC_VALID) {
			res->valid++;
			if (res->letter == '\0') {
				res->letter = o.ch;
				res->pass = o.pass;
				res->reversed = o.b_reversed;
			}
		} else if (r == BC_REJECT) {
			res->rejects++;
			res->last_reason = o.reason;
			res->last_elements = o.elements;
		} else {
			/* nothing finished */
		}
	}
	res->travel_err_pct = (d_true > 0.0) ? 100.0 * (travel_um / 1000.0 - d_true) / d_true : 0.0;
}

static void report(const char *name, const result_t *r)
{
	if (r->letter != '\0') {
		(void)printf("  %-30s '%c' (%s, pass %u)%s, travel %+.1f %%\n", name, r->letter,
			     r->reversed ? "rev" : "fwd", (unsigned)r->pass,
			     (r->rejects > 0u) ? ", and rejects" : "", r->travel_err_pct);
	} else if (r->rejects > 0u) {
		(void)printf("  %-30s no letter: rejected (reason %u, %u elements), travel %+.1f %%\n",
			     name, (unsigned)r->last_reason, (unsigned)r->last_elements,
			     r->travel_err_pct);
	} else {
		(void)printf("  %-30s no letter, no reject (%u not a barcode), travel %+.1f %%\n",
			     name, r->not_barcode, r->travel_err_pct);
	}
}

int main(void)
{
	result_t r1;
	result_t r2;
	result_t r3;
	result_t r4;

	(void)printf("analog fixtures (span %u, floor %u, repair4 %u, end on stop %u):\n",
		     (unsigned)BC_ANALOG_SPAN_RAW, (unsigned)BC_ANALOG_NORM_FLOOR,
		     (unsigned)BC_REPAIR4, (unsigned)BC_END_ON_STOP);
	run("run3_1509_decoded_Z", 0u, &r3);
	report("run3_1509_decoded_Z", &r3);
	CHECK(r3.letter == 'Z');
	CHECK(r3.valid == 1u);

	run("run1_1506_Z_missing_first4", 0u, &r1);
	report("run1_1506_Z_missing_first4", &r1);
#if BC_REPAIR4
	CHECK(r1.letter == 'Z');
#else
	CHECK(r1.letter == '\0');
#endif

	run("run4_1517_track_faint", 0u, &r4);
	report("run4_1517_track_faint", &r4);
	/* by hand from the readings (sim): *F* read backwards; no other
	   letter may come out */
	CHECK((r4.letter == '\0') || (r4.letter == 'F'));
#ifdef EXPECT_RUN4_F
	CHECK(r4.letter == 'F');
#endif

	run("run2_1508_faint_aborted", 0u, &r2);
	report("run2_1508_faint_aborted", &r2);
	CHECK(r2.letter == '\0');

	/* the front end's travel follows the car's within a few % */
	CHECK((r3.travel_err_pct > -5.0) && (r3.travel_err_pct < 5.0));
	CHECK((r1.travel_err_pct > -5.0) && (r1.travel_err_pct < 5.0));
	return check_report("test_bc_analog_fixtures");
}
