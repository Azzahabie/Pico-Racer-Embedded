/*
 * line_task.c - black-line task (uT-Kernel 3.0, RP2040)
 *
 * Uses line_logic.c/.h to read the two sensors, filters the result, and
 * prints the line state over the console. No motors yet.
 *
 * Mount the two sensors closer together than the 18 mm line width so that
 * BOTH see black when the car is centred.
 *
 *   bits (R L)   state
 *   1 1          LINE_CENTRED   -> DRIVE
 *   0 1          LINE_LEFT      -> STEER_LEFT   (only left sees black)
 *   1 0          LINE_RIGHT     -> STEER_RIGHT  (only right sees black)
 *   0 0          LINE_LOST      -> STOP
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>


#include "../usb_console_compat.h"
#include "line_task.h"
#include "line_logic.h"
#include "barcode_scanner.h"

/* ------------------------- CONFIG: edit these ------------------------- */
#define SAMPLE_PERIOD_MS	5	/* sensors read every 5 ms               */
#define CONFIRM_SAMPLES		6	/* same reading N times in a row = accept */
#define LOST_SAMPLES		8	/* no line N times (~40 ms) = LOST        */
#define HEARTBEAT_MS		2000	/* raw pin levels printed this often     */
/* ---------------------------------------------------------------------- */

typedef enum {
	LINE_CENTRED = 0,
	LINE_LEFT,
	LINE_RIGHT,
	LINE_LOST
} LINE_STATE;

LOCAL const char *state_name(LINE_STATE s)
{
	switch(s) {
	case LINE_CENTRED:	return "CENTRED (both on line)";
	case LINE_LEFT:		return "LEFT    (line drifting left)";
	case LINE_RIGHT:	return "RIGHT   (line drifting right)";
	default:		return "LOST    (no line -> STOP)";
	}
}

LOCAL const char *cmd_name(LINE_STATE s)
{
	switch(s) {
	case LINE_CENTRED:	return "DRIVE";
	case LINE_LEFT:		return "STEER_LEFT";
	case LINE_RIGHT:	return "STEER_RIGHT";
	default:		return "STOP";
	}
}

/* Packed 0bRL value from line_reader -> line state. */
LOCAL LINE_STATE bits_to_state(UB bits)
{
	BOOL r = (bits & LINE_READER_RIGHT_GUARD_MASK) ? TRUE : FALSE;
	BOOL l = (bits & LINE_READER_LEFT_GUARD_MASK) ? TRUE : FALSE;

	if(l && r)	return LINE_CENTRED;
	if(l && !r)	return LINE_LEFT;
	if(!l && r)	return LINE_RIGHT;
	return LINE_LOST;
}

/*
 * A new state is only accepted after it has been read several times in a
 * row. LOST needs more samples, so a small gap or speck on the tape does
 * not stop the car, but a real end of line still does within ~40 ms.
 */
LOCAL LINE_STATE	filtered = LINE_LOST;	/* start "stopped" */
LOCAL LINE_STATE	candidate = LINE_LOST;
LOCAL UINT		candidate_cnt;

LOCAL LINE_STATE line_update(void)
{
	LINE_STATE raw = bits_to_state((UB)line_reader_read_2bit());
	UINT needed;

	if(raw == candidate) {
		if(candidate_cnt < 1000U) candidate_cnt++;
	} else {
		candidate = raw;
		candidate_cnt = 1;
	}

	needed = (candidate == LINE_LOST) ? LOST_SAMPLES : CONFIRM_SAMPLES;
	if(candidate_cnt >= needed) {
		filtered = candidate;
	}
	return filtered;
}

void line_task(INT stacd, void *exinf)
{
	LINE_STATE		s, last = (LINE_STATE)-1;
	line_reader_raw_t	raw;
	char			bitstr[3];
	UINT			since_beat = 0;
	BOOL			barcode_black, last_barcode_black;
	INT			barcode_raw;
	INT			i;

	(void)stacd; (void)exinf;

	line_reader_init(NULL);		/* Uses the configured sensor polarity. */
	barcode_scanner_init(NULL);

	/* Wait for the USB serial link, as monitor_task does, so the header
	   is not printed into a ring nobody is reading yet. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(800);

	tm_printf((UB *)"\n=== Line reader started ===\n");
	tm_printf((UB *)"Left sensor  : GP%d\n", (INT)LINE_READER_LEFT_GUARD_GPIO);
	tm_printf((UB *)"Right sensor : GP%d\n", (INT)LINE_READER_RIGHT_GUARD_GPIO);
	tm_printf((UB *)"Bits shown as RL, 1 = black\n\n");

	barcode_black = barcode_scanner_read_black() ? TRUE : FALSE;
	last_barcode_black = barcode_black;
	barcode_raw = (INT)barcode_scanner_get_last_raw();
	tm_printf((UB *)"[barcode] GP%d raw=%d black=%d\n",
		  (INT)BARCODE_SCANNER_GPIO, barcode_raw, (INT)barcode_black);


	while(1) {
		s = line_update();
		barcode_black = barcode_scanner_read_black() ? TRUE : FALSE;
		barcode_raw = (INT)barcode_scanner_get_last_raw();

		if(barcode_black != last_barcode_black) {
			tm_printf((UB *)"[barcode] GP%d raw=%d black=%d\n",
				  (INT)BARCODE_SCANNER_GPIO, barcode_raw,
				  (INT)barcode_black);
			last_barcode_black = barcode_black;
		}

		if(s != last) {
			line_reader_format_2bit((UB)line_reader_read_2bit(), bitstr);
			tm_printf((UB *)"[line] RL=%s | %s | cmd: %s\n",
				  bitstr, state_name(s), cmd_name(s));
			last = s;
		}

		/* Heartbeat with the raw pin levels, so a wiring or polarity
		   problem is visible even when the state never changes. */
		since_beat += SAMPLE_PERIOD_MS;
		if(since_beat >= HEARTBEAT_MS) {
			since_beat = 0;
			raw = line_reader_get_last_raw();
			tm_printf((UB *)"[line] raw pins: GP%d=%d GP%d=%d\n",
				  (INT)LINE_READER_LEFT_GUARD_GPIO, (INT)raw.left_guard,
				  (INT)LINE_READER_RIGHT_GUARD_GPIO, (INT)raw.right_guard);
			tm_printf((UB *)"[barcode] GP%d raw=%d black=%d\n",
				  (INT)BARCODE_SCANNER_GPIO, barcode_raw,
				  (INT)barcode_black);
		}

		tk_dly_tsk(SAMPLE_PERIOD_MS);
	}
}