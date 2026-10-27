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
		tm_printf((UB *)"[commission] %s ADC GP%d=%d GP%d=%d; barcode DO GP%d=%u\n",
			  surface,
			  (INT)LINE_SENSOR_LEFT_GPIO, (INT)raw.left_guard,
			  (INT)LINE_SENSOR_RIGHT_GPIO, (INT)raw.right_guard,
			  (INT)BARCODE_SENSOR_GPIO,
			  (UINT)barcode_scanner_read_raw());
		tk_dly_tsk(LINE_COMMISSION_SAMPLE_PERIOD_MS);
	}
}
#endif

LOCAL void barcode_report_scan(const barcode_decode_result_t *result)
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
	LINE_STATE pending_state = LINE_STRAIGHT;
	line_reader_raw_t raw;
	barcode_decode_result_t barcode_result;
	char bit_string[3];
	UINT heartbeat = 0u;
	INT wait_count;
	ER line_init_result;
	ER barcode_init_result;
	BOOL pending_line_log = FALSE;
	BOOL pending_heartbeat = FALSE;
	BOOL pending_adc_error = FALSE;
	ER adc_error;
	ER pending_adc_error_code = E_OK;

	(void)stacd;
	(void)exinf;
	line_init_result = line_reader_init(NULL);
	barcode_init_result = barcode_scanner_init(NULL);

	for (wait_count = 0; (wait_count < 200) && (tm_usb_state() < 2);
	     ++wait_count) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(800);

	if (line_init_result < E_OK) {
		tm_printf((UB *)"[line] ADC initialization failed, er=%d\n",
			  line_init_result);
		return;
	}
	if (barcode_init_result < E_OK) {
		tm_printf((UB *)"[barcode] GPIO/mutex initialization failed, er=%d\n",
			  barcode_init_result);
		return;
	}

	tm_printf((UB *)"\n=== Line reader started ===\n");
	tm_printf((UB *)"Left ADC: GP%d, right ADC: GP%d\n",
		  (INT)LINE_SENSOR_LEFT_GPIO, (INT)LINE_SENSOR_RIGHT_GPIO);
	tm_printf((UB *)"Line bits are 0bRL; 1 = black. Barcode DO: GP%d.\n",
		  (INT)BARCODE_SENSOR_GPIO);
	raw = line_reader_read_raw();
	tm_printf((UB *)"[line] raw ADC GP26=%d GP27=%d\n",
		  (INT)raw.left_guard, (INT)raw.right_guard);

#if LINE_COMMISSIONING_MODE
	commissioning_sample("white");
	commissioning_sample("black");
#endif

	while (1) {
		current = line_update();
		if (barcode_scanner_poll(&barcode_result)) {
			barcode_report_scan(&barcode_result);
			if (barcode_scanner_last_error() < E_OK) {
				tm_printf((UB *)"[barcode] publish failed, er=%d\n",
					  barcode_scanner_last_error());
			}
		}

		if (current != last) {
			if (barcode_scanner_scan_active()) {
				pending_state = current;
				pending_line_log = TRUE;
			} else {
				line_reader_format_2bit((UB)current, bit_string);
				tm_printf((UB *)"[line] RL=%s | %s | cmd: %s\n",
					  bit_string, state_name(current),
					  cmd_name(current));
			}
			last = current;
		}
		adc_error = line_reader_last_error();
		if (adc_error < E_OK) {
			pending_adc_error = TRUE;
			pending_adc_error_code = adc_error;
		}

		heartbeat += LINE_SAMPLE_PERIOD_MS;
		if (heartbeat >= LINE_HEARTBEAT_MS) {
			heartbeat = 0u;
			if (barcode_scanner_scan_active()) {
				pending_heartbeat = TRUE;
			} else {
				raw = line_reader_get_last_raw();
				tm_printf((UB *)"[line] raw ADC GP26=%d GP27=%d\n",
					  (INT)raw.left_guard, (INT)raw.right_guard);
			}
		}

		if (!barcode_scanner_scan_active()) {
			if (pending_adc_error) {
				tm_printf((UB *)"[line] ADC read failed, er=%d\n",
					  pending_adc_error_code);
				pending_adc_error = FALSE;
			}
			if (pending_line_log) {
				line_reader_format_2bit((UB)pending_state, bit_string);
				tm_printf((UB *)"[line] RL=%s | %s | cmd: %s\n",
					  bit_string, state_name(pending_state),
					  cmd_name(pending_state));
				pending_line_log = FALSE;
			}
			if (pending_heartbeat) {
				raw = line_reader_get_last_raw();
				tm_printf((UB *)"[line] raw ADC GP26=%d GP27=%d\n",
					  (INT)raw.left_guard, (INT)raw.right_guard);
				pending_heartbeat = FALSE;
			}
		}
		tk_dly_tsk(LINE_SAMPLE_PERIOD_MS);
	}
}
