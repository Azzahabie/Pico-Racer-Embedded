/*
 *	led_test.c
 *	Pipeline validation task: mirrors GP21 (input) onto GP10 (output).
 *
 *	Confirms the recursive app_program/ build, the folder-per-task
 *	structure, and the real gpio_set_pin/gpio_get_val/gpio_set_val calls
 *	all work together before any buddy's real subsystem task is written.
 *
 *	No pull resistor is configured here -- libbsp.h exposes no pull-up/
 *	pull-down control, so GP21's button needs one in the circuit itself.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "led_test.h"

#define BTN_PIN		21U
#define LED_PIN		10U
#define POLL_MS		20

EXPORT void led_test_task(INT stacd, void *exinf)
{
	UINT	level;

	(void)stacd; (void)exinf;

	gpio_set_pin(BTN_PIN, GPIO_MODE_IN);
	gpio_set_pin(LED_PIN, GPIO_MODE_OUT);

	while(1) {
		level = gpio_get_val(BTN_PIN);
		gpio_set_val(LED_PIN, level);
		tk_dly_tsk(POLL_MS);
	}
}