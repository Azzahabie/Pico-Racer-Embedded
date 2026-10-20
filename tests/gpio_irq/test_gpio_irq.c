/*
 *	test_gpio_irq.c
 *	Host test for app_program/common/gpio_irq.c against a fake IO_BANK0:
 *	the INTE/INTS/INTR bit layout, write-1-to-clear, per-core delivery,
 *	pin ownership and limits.   make -C tests/gpio_irq
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../app_program/common/gpio_irq.c"

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* ---- fake hardware ------------------------------------------------- */

static uint32_t g_intr[4];		/* raw edge latches */
static uint32_t g_inte[2][4];		/* per core enables */
static uint32_t g_gpio_in;
static uint32_t g_cpu;			/* which core is "running" */
static uint32_t g_time;
static int	g_def_int_calls;
static FP	g_handler;
static int	g_enable_calls[2];

#define ALIAS_SET	0x2000u
#define ALIAS_CLR	0x3000u

static int reg_index(UW base, UW addr, uint32_t *p_idx)
{
	if (addr >= base && addr < base + 16u && ((addr - base) % 4u) == 0u) {
		*p_idx = (addr - base) / 4u;
		return 1;
	}
	return 0;
}

UW in_w(UW addr)
{
	uint32_t i;

	if (addr == SIO_CPUID_ADDR) return g_cpu;
	if (addr == GPIO_IN) return g_gpio_in;
	if (addr == TIMER_BASE + 0x28u) return g_time++;
	if (reg_index(IO_BANK0_BASE + 0x120u, addr, &i)) return g_intr[i] & g_inte[0][i];
	if (reg_index(IO_BANK0_BASE + 0x150u, addr, &i)) return g_intr[i] & g_inte[1][i];
	if (reg_index(IO_BANK0_BASE + 0x100u, addr, &i)) return g_inte[0][i];
	if (reg_index(IO_BANK0_BASE + 0x130u, addr, &i)) return g_inte[1][i];
	return 0u;
}

static void reg_write(UW addr, UW data, int op)	/* 0 = write, 1 = set, 2 = clr */
{
	uint32_t i, *p = NULL;

	if (reg_index(IO_BANK0_BASE + 0x0F0u, addr, &i)) {
		CHECK(op == 0);
		g_intr[i] &= ~(data & 0xCCCCCCCCu);	/* W1C, edge bits only */
		return;
	}
	if (reg_index(IO_BANK0_BASE + 0x100u, addr, &i)) p = &g_inte[0][i];
	if (reg_index(IO_BANK0_BASE + 0x130u, addr, &i)) p = &g_inte[1][i];
	CHECK(p != NULL);
	if (p == NULL) return;
	if (op == 0) *p = data; else if (op == 1) *p |= data; else *p &= ~data;
}

void out_w(UW a, UW d) { reg_write(a, d, 0); }
void set_w(UW a, UW d) { reg_write(a, d, 1); }
void clr_w(UW a, UW d) { reg_write(a, d, 2); }

ER tk_def_int(UINT intno, CONST T_DINT *p)
{
	CHECK(intno == 13u);
	CHECK(p->intatr == TA_HLNG);
	g_def_int_calls++;
	g_handler = p->inthdr;
	return E_OK;
}
void EnableInt(UINT intno, INT level) { CHECK(intno == 13u); CHECK(level == 1); g_enable_calls[g_cpu]++; }
void ClearInt(UINT intno) { CHECK(intno == 13u); }

static void run_irq(uint32_t core)
{
	g_cpu = core;
	((void (*)(UINT))g_handler)(13u);
}

/* ---- clients -------------------------------------------------------- */

static uint32_t g_last_pins[3], g_last_in[3];
static int g_calls[3];

static void cb_enc(uint32_t pins, uint32_t in) { g_calls[0]++; g_last_pins[0] = pins; g_last_in[0] = in; }
static void cb_echo(uint32_t pins, uint32_t in) { g_calls[1]++; g_last_pins[1] = pins; g_last_in[1] = in; }
static void cb_btn(uint32_t pins, uint32_t in) { g_calls[2]++; g_last_pins[2] = pins; g_last_in[2] = in; }
static void cb_dummy(uint32_t pins, uint32_t in) { (void)pins; (void)in; }

int main(void)
{
	/* Nothing works before init. */
	g_cpu = 1u;
	CHECK(gpio_irq_attach(0xFu, GPIO_IRQ_EDGE_BOTH, cb_enc) == E_OBJ);

	/* Core 1 takes the interrupt; stale enables are wiped. */
	g_inte[1][0] = 0xFFFFFFFFu;
	CHECK(gpio_irq_init(1) == E_OK);
	CHECK(g_def_int_calls == 1);
	CHECK(g_enable_calls[1] == 1);
	CHECK(g_inte[1][0] == 0u);
	CHECK(gpio_irq_init(1) == E_OBJ);

	/* Encoders GP0..GP3, both edges: 0xC in each nibble of PROC1_INTE0. */
	g_intr[0] = 0x0000CCCCu;		/* stale edges must be dropped */
	CHECK(gpio_irq_attach(0xFu, GPIO_IRQ_EDGE_BOTH, cb_enc) == E_OK);
	CHECK(g_inte[1][0] == 0x0000CCCCu);
	CHECK(g_inte[0][0] == 0u);
	CHECK(g_intr[0] == 0u);

	/* Echo GP5 on core 1, rising edge only: bit 3 of nibble 5. */
	CHECK(gpio_irq_attach(1u << 5, GPIO_IRQ_EDGE_RISE, cb_echo) == E_OK);
	CHECK(g_inte[1][0] == (0x0000CCCCu | (0x8u << 20)));

	/* GP2 rises: only the encoder client hears it, the latch is cleared. */
	g_gpio_in = 0x4u;
	g_intr[0] |= 0x8u << 8;
	run_irq(1u);
	CHECK(g_calls[0] == 1 && g_last_pins[0] == (1u << 2) && g_last_in[0] == 0x4u);
	CHECK(g_calls[1] == 0);
	CHECK(g_intr[0] == 0u);

	/* GP5 falls: not enabled (rise only), so nobody is called and the
	   latch is left for whoever owns it. */
	g_intr[0] |= 0x4u << 20;
	run_irq(1u);
	CHECK(g_calls[1] == 0);
	CHECK(g_intr[0] == (0x4u << 20));
	g_intr[0] = 0u;

	/* GP0 and GP5 together: both clients, each with only its own pins. */
	g_intr[0] |= (0x4u << 0) | (0x8u << 20);
	run_irq(1u);
	CHECK(g_calls[0] == 2 && g_last_pins[0] == 1u);
	CHECK(g_calls[1] == 1 && g_last_pins[1] == (1u << 5));

	/* A pin can have one owner, on either core. */
	CHECK(gpio_irq_attach(1u << 2, GPIO_IRQ_EDGE_RISE, cb_dummy) == E_OBJ);

	/* Bad arguments. */
	CHECK(gpio_irq_attach(1u << 30, GPIO_IRQ_EDGE_RISE, cb_dummy) == E_PAR);
	CHECK(gpio_irq_attach(1u << 6, 0u, cb_dummy) == E_PAR);
	CHECK(gpio_irq_attach(1u << 6, 0x3u, cb_dummy) == E_PAR);	/* level bits */
	CHECK(gpio_irq_attach(1u << 6, GPIO_IRQ_EDGE_RISE, NULL) == E_PAR);

	/* Core 0 joins for a button on GP20 (falling): no second tk_def_int,
	   enables land in PROC0_INTE2, nibble 4. */
	g_cpu = 0u;
	CHECK(gpio_irq_attach(1u << 20, GPIO_IRQ_EDGE_FALL, cb_btn) == E_OBJ);
	CHECK(gpio_irq_init(1) == E_OK);
	CHECK(g_def_int_calls == 1);
	CHECK(g_enable_calls[0] == 1);
	CHECK(gpio_irq_attach(1u << 20, GPIO_IRQ_EDGE_FALL, cb_btn) == E_OK);
	CHECK(g_inte[0][2] == (0x4u << 16));
	CHECK(g_inte[1][2] == 0u);

	/* GP20 falls: core 1's handler sees nothing, core 0's delivers it. */
	g_intr[2] |= 0x4u << 16;
	run_irq(1u);
	CHECK(g_calls[2] == 0);
	CHECK(g_intr[2] != 0u);
	run_irq(0u);
	CHECK(g_calls[2] == 1 && g_last_pins[2] == (1u << 20));
	CHECK(g_intr[2] == 0u);

	/* Counts are per core. */
	g_cpu = 1u;
	CHECK(gpio_irq_count() == 4u);
	g_cpu = 0u;
	CHECK(gpio_irq_count() == 1u);
	CHECK(gpio_irq_take_max_us() >= 1u);
	CHECK(gpio_irq_take_max_us() == 0u);

	/* Client limit per core. */
	CHECK(gpio_irq_attach(1u << 21, GPIO_IRQ_EDGE_FALL, cb_dummy) == E_OK);
	CHECK(gpio_irq_attach(1u << 22, GPIO_IRQ_EDGE_FALL, cb_dummy) == E_OK);
	CHECK(gpio_irq_attach(1u << 23, GPIO_IRQ_EDGE_FALL, cb_dummy) == E_OK);
	CHECK(gpio_irq_attach(1u << 24, GPIO_IRQ_EDGE_FALL, cb_dummy) == E_LIMIT);

	printf("gpio_irq: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
