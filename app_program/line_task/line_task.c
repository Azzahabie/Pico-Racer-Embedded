/*
 * line_task.c - two-sensor line reader and independent barcode scanner.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../usb_console_compat.h"
#include "line_task.h"
#include "line_logic.h"
#include "barcode_scanner.h"

typedef enum {
	LINE_STRAIGHT = 0,
	LINE_LEFT_DEVIATION,
	LINE_RIGHT_DEVIATION,
	LINE_JUNCTION
} LINE_STATE;

LOCAL const char *state_name(LINE_STATE state)
{
	switch (state) {
	case LINE_STRAIGHT:
#if LINE_STOP_ON_LOST
		return "LOST";
#else
		return "STRAIGHT";
#endif
	case LINE_LEFT_DEVIATION:
		return "LEFT";
	case LINE_RIGHT_DEVIATION:
		return "RIGHT";
	case LINE_JUNCTION:
		return "JUNCTION";
	default:
		return "UNKNOWN";
	}
}

LOCAL const char *cmd_name(LINE_STATE state)
{
	switch (state) {
	case LINE_STRAIGHT:
#if LINE_STOP_ON_LOST
		return "STOP";
#else
		return "DRIVE";
#endif
	case LINE_LEFT_DEVIATION:
#if LINE_STEER_INVERT
		return "STEER_LEFT";
#else
		return "STEER_RIGHT";
#endif
	case LINE_RIGHT_DEVIATION:
#if LINE_STEER_INVERT
		return "STEER_RIGHT";
#else
		return "STEER_LEFT";
#endif
	case LINE_JUNCTION:
	default:
		return "DRIVE";
	}
}

LOCAL LINE_STATE bits_to_state(UB bits)
{
	return (LINE_STATE)(bits & LINE_READER_OUTPUT_MASK);
}

LOCAL LINE_STATE filtered = LINE_STRAIGHT;
LOCAL LINE_STATE candidate = LINE_STRAIGHT;
LOCAL UINT candidate_count;

LOCAL LINE_STATE line_update(void)
{
	LINE_STATE raw = bits_to_state((UB)line_reader_read_2bit());
	UINT needed;

	if (raw == candidate) {
		if (candidate_count < 1000U) {
			candidate_count++;
		}
	} else {
		candidate = raw;
		candidate_count = 1u;
	}

	needed = (candidate == LINE_STRAIGHT)
		     ? LINE_LOST_SAMPLES : LINE_CONFIRM_SAMPLES;
	if (candidate_count >= needed) {
		filtered = candidate;
	}
	return filtered;
}

#if LINE_COMMISSIONING_MODE
LOCAL void commissioning_sample(const char *surface)
{
	UINT sample;
	line_reader_raw_t raw;

	tm_printf((UB *)"[commission] Place sensors over %s for 5 seconds\n",
		  surface);
	for (sample = 0u; sample < LINE_COMMISSION_SAMPLE_COUNT; ++sample) {
		raw = line_reader_read_raw();
		tm_printf((UB *)"[commission] GP%d=%d GP%d=%d\n",
			  (INT)LINE_LEFT_GPIO, (INT)raw.left_guard,
			  (INT)LINE_RIGHT_GPIO, (INT)raw.right_guard);
		tk_dly_tsk(LINE_COMMISSION_SAMPLE_PERIOD_MS);
	}
}
#endif

LOCAL void print_barcode_result(const barcode_decode_result_t *result)
{
	if (result->valid) {
		tm_printf((UB *)"[barcode] DECODED: \"%s\" (dir=%s, elems=%u)\n",
			  result->text, result->reversed ? "rev" : "fwd",
			  (UINT)result->elements);
	} else if (result->error_character != 0u) {
		tm_printf((UB *)"[barcode] ERROR: %s at char %u\n",
			  barcode_decode_error_name(result->error),
			  (UINT)result->error_character);
	} else {
		tm_printf((UB *)"[barcode] ERROR: %s\n",
			  barcode_decode_error_name(result->error));
	}
}

void line_task(INT stacd, void *exinf)
{
	LINE_STATE current;
	LINE_STATE last = (LINE_STATE)-1;
	line_reader_raw_t raw;
	barcode_decode_result_t barcode_result;
	char bit_string[3];
	UINT heartbeat = 0u;
	INT wait_count;

	(void)stacd;
	(void)exinf;
	line_reader_init(NULL);
	barcode_scanner_init(NULL);

	for (wait_count = 0; (wait_count < 200) && (tm_usb_state() < 2);
	     ++wait_count) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(800);

	tm_printf((UB *)"\n=== Line reader started ===\n");
	tm_printf((UB *)"Left sensor: GP%d, right sensor: GP%d\n",
		  (INT)LINE_LEFT_GPIO, (INT)LINE_RIGHT_GPIO);
	tm_printf((UB *)"Line bits are 0bRL; 1 = black. Barcode ADC: GP%d.\n",
		  (INT)BARCODE_GPIO);

#if LINE_COMMISSIONING_MODE
	commissioning_sample("white");
	commissioning_sample("black");
#endif

	while (1) {
		current = line_update();
		if (barcode_scanner_poll(&barcode_result)) {
			print_barcode_result(&barcode_result);
		}

		if (current != last) {
			line_reader_format_2bit((UB)current, bit_string);
			tm_printf((UB *)"[line] RL=%s | %s | cmd: %s\n",
				  bit_string, state_name(current), cmd_name(current));
			last = current;
		}

		heartbeat += LINE_SAMPLE_PERIOD_MS;
		if (heartbeat >= LINE_HEARTBEAT_MS) {
			heartbeat = 0u;
			raw = line_reader_get_last_raw();
			tm_printf((UB *)"[line] raw pins: GP%d=%d GP%d=%d\n",
				  (INT)LINE_LEFT_GPIO, (INT)raw.left_guard,
				  (INT)LINE_RIGHT_GPIO, (INT)raw.right_guard);
		}
		tk_dly_tsk(LINE_SAMPLE_PERIOD_MS);
	}
}
