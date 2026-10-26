/*
 *	test_motion_wdt.c
 *	Host test for app_program/motor_task/motion_wdt.c against fake
 *	watchdog, PSM and timer registers: the arming steps and their order
 *	(as the Pico SDK's watchdog_enable() does them), LOAD for 30 ms with
 *	erratum RP2040-E1, the feed, the reset cause, and the bench test
 *	record in the scratch registers.   make -C tests/motion_wdt
 */

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motor_task/motion_wdt.c"

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* ------------------------------------------------------------------ *
 *  Fake registers, with a log of every write in order
 * ------------------------------------------------------------------ */

#define WDT_TICK	(WDT_BASE + 0x2Cu)	/* must never be touched */

static uint32_t g_ctrl, g_load, g_reason, g_scratch[8], g_wdsel;
static uint32_t g_timer;
static int	g_tick_touched;

typedef struct { char op; UW addr; UW data; } wr_t;
static wr_t	g_wr[64];
static int	g_n_wr;

static jmp_buf	g_spin_jmp;
static int	g_spin_left;		/* SCRATCH2 writes before we stop it */

static uint32_t *
reg (UW addr)
{
	uint32_t *p = NULL;

	if (addr == WDT_CTRL) { p = &g_ctrl; }
	else if (addr == WDT_LOAD) { p = &g_load; }
	else if (addr == WDT_REASON) { p = &g_reason; }
	else if ((addr >= WDT_SCRATCH(0u)) && (addr <= WDT_SCRATCH(7u))) {
		p = &g_scratch[(addr - WDT_SCRATCH(0u)) / 4u];
	}
	else if (addr == PSM_WDSEL) { p = &g_wdsel; }
	else if (addr == WDT_TICK) { g_tick_touched = 1; }
	return p;
}

static void
log_wr (char op, UW addr, UW data)
{
	if (g_n_wr < (int)(sizeof g_wr / sizeof g_wr[0])) {
		g_wr[g_n_wr].op = op;
		g_wr[g_n_wr].addr = addr;
		g_wr[g_n_wr].data = data;
		g_n_wr++;
	}
}

UW in_w(UW addr)
{
	uint32_t *p = reg(addr);

	if (addr == WDT_TIMER_RAWL) { return g_timer++; }	/* 1 us per read */
	return (p != NULL) ? *p : 0u;
}

void out_w(UW addr, UW data)
{
	uint32_t *p = reg(addr);

	log_wr('=', addr, data);
	if (p != NULL) { *p = data; }
	if ((addr == WDT_HANG_LAST) && (g_spin_left > 0) && (--g_spin_left == 0)) {
		longjmp(g_spin_jmp, 1);		/* "the chip resets" */
	}
}

void set_w(UW addr, UW data)
{
	uint32_t *p = reg(addr);

	log_wr('|', addr, data);
	if (p != NULL) { *p |= data; }
}

void clr_w(UW addr, UW data)
{
	uint32_t *p = reg(addr);

	log_wr('&', addr, data);
	if (p != NULL) { *p &= ~data; }
}

/* Index of the first logged write matching op/addr/data, or -1. */
static int
find_wr (char op, UW addr, UW data)
{
	int i;

	for (i = 0; i < g_n_wr; i++) {
		if ((g_wr[i].op == op) && (g_wr[i].addr == addr) && (g_wr[i].data == data)) {
			return i;
		}
	}
	return -1;
}

/* ------------------------------------------------------------------ */

static void
test_boot_cause (void)
{
	/* REASON 0 is a power-on or RUN pin, whatever SCRATCH4 holds. */
	g_reason = 0u; g_scratch[4] = 0x6AB73121u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_POWER_ON);
	g_scratch[4] = 0u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_POWER_ON);

	/* Timeout while our marker was set: our watchdog. */
	g_reason = 1u; g_scratch[4] = 0x6AB73121u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_WATCHDOG);

	/* The boot ROM's reboot after a UF2 copy writes SCRATCH4 itself
	   (0, or 0xb007c0d3 with an address). */
	g_scratch[4] = 0u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_REBOOT);
	g_scratch[4] = 0xB007C0D3u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_REBOOT);

	/* A forced reset (CTRL TRIGGER) is not a timeout, marker or not. */
	g_reason = 2u; g_scratch[4] = 0x6AB73121u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_REBOOT);

	/* Reading the cause changes nothing. */
	CHECK(g_n_wr == 0);
}

static void
test_start (void)
{
	int i_disarm, i_mark, i_load, i_enable, i_wdsel, i_pause;

	/* As the boot ROM's reboot can leave it: enabled, pause bits off. */
	g_ctrl = WDT_CTRL_ENABLE;
	g_wdsel = 0u;
	g_n_wr = 0;
	motion_wdt_start();

	CHECK(g_load == 60000u);			/* 30 ms x 1000 x 2 */
	CHECK(g_wdsel == 0x0001FFFCu);			/* all but ROSC, XOSC */
	CHECK(g_scratch[4] == 0x6AB73121u);
	CHECK((g_ctrl & WDT_CTRL_ENABLE) != 0u);
	CHECK((g_ctrl & 0x07000000u) == 0x07000000u);	/* paused by a debugger */
	CHECK((g_ctrl & 0x80000000u) == 0u);		/* never TRIGGER */
	CHECK(!g_tick_touched);

	/* Order: disarm first, enable last, the marker and LOAD before it. */
	i_disarm = find_wr('&', WDT_CTRL, WDT_CTRL_ENABLE);
	i_mark   = find_wr('=', WDT_SCRATCH(4u), 0x6AB73121u);
	i_wdsel  = find_wr('|', PSM_WDSEL, 0x0001FFFCu);
	i_pause  = find_wr('|', WDT_CTRL, 0x07000000u);
	i_load   = find_wr('=', WDT_LOAD, 60000u);
	i_enable = find_wr('|', WDT_CTRL, WDT_CTRL_ENABLE);
	CHECK(i_disarm == 0);
	CHECK((i_mark > i_disarm) && (i_wdsel > i_disarm) && (i_pause > i_disarm));
	CHECK((i_load > i_disarm) && (i_load < i_enable));
	CHECK((i_mark < i_enable) && (i_wdsel < i_enable) && (i_pause < i_enable));
	CHECK(i_enable == g_n_wr - 1);

	/* Armed by us, so a timeout now would read as our watchdog. */
	g_reason = 1u;
	CHECK(motion_wdt_boot_cause() == MOTION_BOOT_WATCHDOG);
}

static void
test_feed (void)
{
	g_load = 123u;
	g_n_wr = 0;
	motion_wdt_feed();
	CHECK(g_load == 60000u);
	CHECK(g_n_wr == 1);				/* LOAD only */
	CHECK(!g_tick_touched);
}

static void
test_hang_record (void)
{
	uint32_t us = 0u;

	/* No record: nothing reported. */
	g_scratch[0] = 0u;
	CHECK(!motion_wdt_take_hang_us(&us));

	/* motion_wdt_hang() spins writing the time until the reset. */
	g_timer = 5000u;
	g_spin_left = 200;
	g_n_wr = 0;
	if (setjmp(g_spin_jmp) == 0) {
		motion_wdt_hang(4990u);
		CHECK(0);				/* never returns */
	}
	CHECK(g_scratch[0] == WDT_HANG_MAGIC);
	CHECK(g_scratch[1] == 4990u);
	CHECK(g_scratch[2] == 5000u + 198u);		/* last time written */
	CHECK(find_wr('=', WDT_LOAD, 60000u) < 0);	/* never fed again */

	/* The next start-up reads it once and clears the marker. */
	CHECK(motion_wdt_take_hang_us(&us));
	CHECK(us == 208u);
	CHECK(g_scratch[0] == 0u);
	CHECK(!motion_wdt_take_hang_us(&us));

	/* Across the timer's 32-bit wrap. */
	g_scratch[0] = WDT_HANG_MAGIC;
	g_scratch[1] = 0xFFFFF000u;
	g_scratch[2] = 0x00006000u;
	CHECK(motion_wdt_take_hang_us(&us));
	CHECK(us == 0x7000u);
}

int main(void)
{
	test_boot_cause();
	test_start();
	test_feed();
	test_hang_record();

	printf("motion_wdt: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
