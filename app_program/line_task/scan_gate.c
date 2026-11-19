/*
 * scan_gate.c - the scan gate, one call per 1 ms sample.
 */

#include "scan_gate.h"
#include "../motor_task/motion_ds.h"

#ifndef LINE_SCAN_MAX_TURN_DPS
#error "LINE_SCAN_MAX_TURN_DPS must come from line_config.h"
#endif

/* Saturation for the ms counters: far above any limit they are compared
   with. */
#define MS_MAX (100000u)

void scan_gate_init(scan_gate_t *gate, uint32_t ds)
{
	gate->stage1 = 0;
	gate->stage2 = 0;
	gate->sample_ms = 0u;
	gate->counter = motion_ds_counter(ds);
	gate->stale_ms = 0u;
	gate->available = true;
	gate->open = true;
}

static uint32_t sat_add(uint32_t a, uint32_t b)
{
	return ((MS_MAX - a) < b) ? MS_MAX : a + b;
}

/* One step of a stage towards its input: 1/LINE_SCAN_AVG_DIV of the way,
   and at least one unit, so a steady input is reached exactly (the
   division alone truncates towards zero and would stop short). */
static int32_t low_pass(int32_t stage, int32_t input)
{
	int32_t diff = input - stage;
	int32_t step = diff / (int32_t)LINE_SCAN_AVG_DIV;

	if ((step == 0) && (diff != 0)) {
		step = (diff > 0) ? 1 : -1;
	}
	return stage + step;
}

bool scan_gate_step(scan_gate_t *gate, uint32_t ds, bool motion_up,
		    uint32_t elapsed_ms)
{
	uint32_t counter = motion_ds_counter(ds);
	int32_t input;
	int32_t size;

	/* distance_state: fresh while its counter keeps moving */
	if (counter != gate->counter) {
		gate->counter = counter;
		gate->stale_ms = 0u;
	} else {
		gate->stale_ms = sat_add(gate->stale_ms, elapsed_ms);
	}
	gate->available = motion_up && (gate->stale_ms < LINE_SCAN_STALE_MS);
	if (!gate->available) {
		/* fails open; the average starts again from 0 */
		gate->stage1 = 0;
		gate->stage2 = 0;
		gate->sample_ms = 0u;
		gate->open = true;
		return true;
	}

	input = motion_ds_turn_dps(ds) * SCAN_GATE_SCALE;
	gate->sample_ms = sat_add(gate->sample_ms, elapsed_ms);
	while (gate->sample_ms >= SCAN_GATE_SAMPLE_MS) {
		gate->sample_ms -= SCAN_GATE_SAMPLE_MS;
		gate->stage1 = low_pass(gate->stage1, input);
		gate->stage2 = low_pass(gate->stage2, gate->stage1);
	}
	size = (gate->stage2 < 0) ? -gate->stage2 : gate->stage2;
	gate->open = size < ((int32_t)LINE_SCAN_MAX_TURN_DPS * SCAN_GATE_SCALE);
	return gate->open;
}

int32_t scan_gate_avg_x10(const scan_gate_t *gate)
{
	return (gate->stage2 * 10) / SCAN_GATE_SCALE;
}
