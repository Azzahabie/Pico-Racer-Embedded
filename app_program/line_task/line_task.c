#include "line_task.h"
#include "../common/messages.h"

#include <bsp/libbsp.h>
#include <tm/tmonitor.h>

#include "bar_capture.h"
#include "barcode_decode.h"

#define SENSOR_POLL_MS 1u

static volatile UINT sensor_state;

#if BARCODE_SENSOR_AVAILABLE
static bar_cap_t barcode_capture;
static barcode_table_t barcode_table;
static barcode_asm_t barcode_assembler;
#endif

UINT line_task_get_state(void)
{
	return sensor_state;
}

static BOOL sensor_is_dark(UINT level)
{
	return (level == KY033_DARK_LEVEL) ? TRUE : FALSE;
}

static BOOL init_input(UINT gpio, UINT pull)
{
	ER er = gpio_set_pin(gpio, GPIO_MODE_IN);

	if(er < E_OK) {
		tm_printf((UB *)"[line] failed to configure GP%u, er=%d\n", gpio, er);
		return FALSE;
	}

	er = gpio_set_pull(gpio, pull);
	if(er < E_OK) {
		tm_printf((UB *)"[line] failed to set GP%u input pull, er=%d\n", gpio, er);
		return FALSE;
	}
	return TRUE;
}

#if BARCODE_SENSOR_AVAILABLE
static bool barcode_sink(bar_rec_t rec)
{
	barcode_result_t result;

	if(barcode_asm_push(&barcode_assembler, rec, &barcode_table, &result)) {
		if(result.valid) {
			if(result.character != '\0') {
				tm_printf((UB *)"[barcode] decoded %c (%s)\n",
					  result.character, result.pattern);
			} else {
				tm_printf((UB *)"[barcode] decoded %s\n",
					  barcode_action_name(result.action));
			}
		} else {
			tm_printf((UB *)"[barcode] rejected %s (%s)\n",
				  barcode_reject_name(result.reject), result.pattern);
		}
	}
	return true;
}
#endif

EXPORT void line_task(INT stacd, void *exinf)
{
	UINT previous = (UINT)-1;
	UINT next;
	UINT line_state;
	UINT inactive_pull = (KY033_DARK_LEVEL == 1u) ? GPIO_PULL_DOWN : GPIO_PULL_UP;
#if BARCODE_SENSOR_AVAILABLE
	UINT barcode_level;
	UINT barcode_inactive_pull = (B3_BARCODE_BAR_LEVEL != 0u) ? GPIO_PULL_DOWN : GPIO_PULL_UP;
#endif

	(void)stacd;
	(void)exinf;

	if(!init_input(LINE_LEFT_SENSOR_GPIO, inactive_pull) ||
	   !init_input(LINE_CENTRE_SENSOR_GPIO, inactive_pull)
#if BARCODE_SENSOR_AVAILABLE
	   || !init_input(BARCODE_SENSOR_GPIO, barcode_inactive_pull)
#endif
	   ) {
		tk_ext_tsk();
	}

#if BARCODE_SENSOR_AVAILABLE
	barcode_table_load_default(&barcode_table);
	barcode_asm_reset(&barcode_assembler);
	bar_cap_reset(&barcode_capture);
	tm_printf((UB *)"[barcode] enabled on GP%u; present a single-letter Code 39 code\n",
		  BARCODE_SENSOR_GPIO);
#endif

	while(1) {
		next = 0u;
		if(sensor_is_dark(gpio_get_val(LINE_LEFT_SENSOR_GPIO))) {
			next |= LINE_STATE_LEFT_ON_DARK;
		}
		if(sensor_is_dark(gpio_get_val(LINE_CENTRE_SENSOR_GPIO))) {
			next |= LINE_STATE_CENTRE_ON_DARK;
		}
#if BARCODE_SENSOR_AVAILABLE
		barcode_level = gpio_get_val(BARCODE_SENSOR_GPIO);
		if(barcode_level == B3_BARCODE_BAR_LEVEL) {
			next |= LINE_STATE_BARCODE_ON_DARK;
		}
		bar_cap_step(&barcode_capture, barcode_level != 0u, barcode_sink);
#endif

		sensor_state = next;
		line_state = next & (LINE_STATE_LEFT_ON_DARK | LINE_STATE_CENTRE_ON_DARK);
		if(line_state != previous) {
#if BARCODE_SENSOR_AVAILABLE
			tm_printf((UB *)"[sensors] left=%u centre=%u barcode=%u\n",
				  (next & LINE_STATE_LEFT_ON_DARK) != 0u,
				  (next & LINE_STATE_CENTRE_ON_DARK) != 0u,
				  (next & LINE_STATE_BARCODE_ON_DARK) != 0u);
#else
			tm_printf((UB *)"[sensors] left=%u centre=%u barcode=unavailable\n",
				  (next & LINE_STATE_LEFT_ON_DARK) != 0u,
				  (next & LINE_STATE_CENTRE_ON_DARK) != 0u);
#endif
			previous = line_state;
		}

		tk_dly_tsk(SENSOR_POLL_MS);
	}
}
