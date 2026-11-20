/*
 * bc_analog_front.c - the analog barcode sensor's front end, one call per
 * 1 ms sample.  Builds only with
 * BARCODE_SENSOR_ANALOG 1, so the digital car image is unchanged.
 */

#include "../common/line_config.h"

#if BARCODE_SENSOR_ANALOG

#include "bc_analog_front.h"
#include "../motor_task/motion_ds.h"

#define ADC_MAX		(4095u)
#define MS_MAX		(100000u)	/* saturation for the ms counters */

void bc_front_init(bc_front_t *f)
{
	f->dark = 0u;
	f->floor_v = (uint16_t)ADC_MAX;	/* snaps to the first reading */
	f->floor_ms = 0u;
	f->norm = 0u;
	f->black = false;
	f->seen = false;
	f->pending = 0u;
	f->ds_seen = false;
	f->ds_ok = false;
	f->ds_last = 0u;
	f->ctr_ms = MS_MAX;
	f->t_ms = 0u;
	f->travel_mm = 0u;
	f->ev_n = 0u;
	f->ev_head = 0u;
	f->speed_x16 = 0u;
	f->carry_x16 = 0u;
}

static uint32_t sat_add(uint32_t a, uint32_t b)
{
	return ((MS_MAX - a) < b) ? MS_MAX : a + b;
}

/* The level: floor, hysteresis, glitch filter, norm. */
static void level_step(bc_front_t *f, uint16_t raw, uint32_t elapsed_ms)
{
	uint32_t white;
	uint32_t scaled;

	if (raw <= ADC_MAX) {
		f->dark = (BC_ANALOG_BLACK_HIGH != 0u) ? raw
						       : (uint16_t)(ADC_MAX - raw);
	}
	/* the white floor: down at once, up one count per
	   BC_ANALOG_FLOOR_RISE_MS while white */
	if (f->dark < f->floor_v) {
		f->floor_v = f->dark;
		f->floor_ms = 0u;
	} else if (!f->black) {
		f->floor_ms = (uint16_t)(f->floor_ms + elapsed_ms);
		while ((f->floor_ms >= BC_ANALOG_FLOOR_RISE_MS) &&
		       (f->floor_v < f->dark)) {
			f->floor_ms = (uint16_t)(f->floor_ms - BC_ANALOG_FLOOR_RISE_MS);
			f->floor_v++;
		}
		if (f->floor_v >= f->dark) {
			f->floor_ms = 0u;
		}
	} else {
		/* black: the floor holds */
	}
	if (f->black) {
		if ((uint32_t)f->dark <= ((uint32_t)f->floor_v + BC_ANALOG_OFF_DELTA)) {
			f->black = false;
		}
	} else if ((uint32_t)f->dark >= ((uint32_t)f->floor_v + BC_ANALOG_ON_DELTA)) {
		f->black = true;
	} else {
		/* between the levels: keep */
	}
	/* the glitch filter, as bar_capture.c's: an edge counts once the
	   level has read the other way for more than BARCODE_GLITCH_TICKS ms */
	if (f->black == f->seen) {
		f->pending = 0u;
	} else {
		f->pending = sat_add(f->pending, elapsed_ms);
		if (f->pending > BARCODE_GLITCH_TICKS) {
			f->seen = f->black;
			f->pending = 0u;
		}
	}
	white = (BC_ANALOG_NORM_FLOOR != 0u) ? f->floor_v : BC_ANALOG_WHITE_RAW;
	if ((uint32_t)f->dark <= white) {
		f->norm = 0u;
	} else {
		scaled = (((uint32_t)f->dark - white) * 1000u) / BC_ANALOG_SPAN_RAW;
		f->norm = (uint16_t)((scaled > 1000u) ? 1000u : scaled);
	}
}

/* A whole-mm change of dist_mm: one more event. */
static void push_event(bc_front_t *f, uint32_t mm)
{
	f->travel_mm += mm;
	f->ev_head = (uint8_t)((f->ev_head + 1u) % BC_FRONT_EVENTS);
	f->ev_t[f->ev_head] = f->t_ms;
	f->ev_mm[f->ev_head] = f->travel_mm;
	if (f->ev_n < BC_FRONT_EVENTS) {
		f->ev_n++;
	}
}

/* The speed from the newest events (bc_front_h). */
static uint32_t speed_from_events(const bc_front_t *f)
{
	uint32_t t_new;
	uint32_t mm_new;
	uint32_t t_old = 0u;
	uint32_t mm_old = 0u;
	uint32_t span_ms;
	uint32_t since;
	uint32_t v;
	uint8_t i;
	uint8_t k;

	if (f->ev_n < 2u) {
		return 0u;
	}
	t_new = f->ev_t[f->ev_head];
	mm_new = f->ev_mm[f->ev_head];
	since = f->t_ms - t_new;
	if (since >= BC_FRONT_STOP_MS) {
		return 0u;
	}
	/* the oldest event in the window; at least the one before the newest */
	for (i = 1u; i < f->ev_n; i++) {
		k = (uint8_t)((f->ev_head + BC_FRONT_EVENTS - i) % BC_FRONT_EVENTS);
		if ((i > 1u) && ((t_new - f->ev_t[k]) > BC_FRONT_WIN_MS)) {
			break;
		}
		t_old = f->ev_t[k];
		mm_old = f->ev_mm[k];
	}
	span_ms = t_new - t_old;
	if ((span_ms == 0u) || (mm_new <= mm_old)) {
		return 0u;
	}
	v = ((mm_new - mm_old) * 16000u) / span_ms;
	/* longer than the mean interval since the last change: at most
	   1 mm over that time */
	if (((uint64_t)since * (mm_new - mm_old)) > span_ms) {
		uint32_t bound = 16000u / since;

		v = (bound < v) ? bound : v;
	}
	return v;
}

static void travel_step(bc_front_t *f, uint32_t ds, bool ds_up,
			uint32_t elapsed_ms)
{
	int32_t delta;
	uint32_t mm;

	f->t_ms += elapsed_ms;
	if (!f->ds_seen) {
		f->ds_seen = true;
		f->ds_last = ds;
		f->ctr_ms = 0u;
	} else if (motion_ds_counter(ds) != motion_ds_counter(f->ds_last)) {
		f->ctr_ms = 0u;
		delta = motion_ds_delta_mm(ds, f->ds_last);
		mm = (uint32_t)((delta < 0) ? -delta : delta);
		if (mm > (uint32_t)BC_FRONT_MAX_STEP_MM) {
			f->ev_n = 0u;		/* a reset of dist_mm, not travel */
		} else if (0u < mm) {
			push_event(f, mm);
		} else {
			/* the same whole mm */
		}
		f->ds_last = ds;
	} else {
		f->ctr_ms = sat_add(f->ctr_ms, elapsed_ms);
	}
	f->ds_ok = ds_up && (f->ctr_ms < LINE_SCAN_STALE_MS);
	if (f->ds_ok) {
		f->speed_x16 = speed_from_events(f);
	} else {
		f->ev_n = 0u;
		f->speed_x16 = BC_ANALOG_NO_DS_UM_PER_MS * 16u;
	}
}

void bc_front_step(bc_front_t *f, uint16_t raw, uint32_t ds, bool ds_up,
		   uint32_t elapsed_ms)
{
	level_step(f, raw, elapsed_ms);
	travel_step(f, ds, ds_up, elapsed_ms);
}

uint32_t bc_front_take_step(bc_front_t *f)
{
	uint32_t sum = f->carry_x16 + f->speed_x16;

	f->carry_x16 = sum & 15u;
	return sum >> 4;
}

uint32_t bc_front_speed_mm_s(const bc_front_t *f)
{
	return f->speed_x16 / 16u;
}

#else
typedef int bc_analog_front_off_t;	/* digital image: nothing built here */
#endif /* BARCODE_SENSOR_ANALOG */
