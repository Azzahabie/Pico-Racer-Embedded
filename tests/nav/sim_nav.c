/* Host test for nav_ctrl_task's junction handling (app_program/nav_task/
 * nav_ctrl.c, FR-08): the real nav_ctrl.c is compiled in with the motion
 * sim's kernel stubs and small fakes for the board, barcode_task and
 * comms_task.  make -C tests/nav
 *
 * What it checks: the barcode command goes to motion_task once per
 * junction, and is used up only when motion turns (MOVE_DONE); a refusal
 * (CMD_REJECTED, motion not in Line Following, e.g. during a bypass) keeps
 * it for the next junction. */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/nav_task/nav_ctrl.c"

/* ---- checks ---------------------------------------------------------- */
static int g_checks;
static int g_failures;
#define CHECK(c)                                                              \
	do {                                                                  \
		g_checks++;                                                   \
		if (!(c)) {                                                   \
			g_failures++;                                         \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);   \
		}                                                             \
	} while (0)

/* ---- fakes: kernel --------------------------------------------------- */
static UINT g_flg[8];			/* event flags by ID 1..7 */
static ID   g_next_flg = 1;
static UB   g_sent[16][2];		/* messages on nav_cmd_mbf */
static int  g_sent_n;
static int  g_mbf_full;			/* tk_snd_mbf returns E_TMOUT */
#define NAV_CMD_MBF_ID	50

ID tk_cre_flg(CONST T_CFLG *p) { (void)p; return g_next_flg++; }
ER tk_set_flg(ID id, UINT ptn) { g_flg[id] |= ptn; return E_OK; }
ER tk_clr_flg(ID id, UINT ptn) { g_flg[id] &= ptn; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *p) { p->flgptn = g_flg[id]; return E_OK; }
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo)
{
	(void)mode; (void)tmo;
	*p_ptn = g_flg[id] & ptn;
	return (0U != *p_ptn) ? E_OK : E_TMOUT;
}
ER tk_snd_mbf(ID id, void *msg, INT size, TMO tmo)
{
	(void)tmo;
	if ((NAV_CMD_MBF_ID != id) || (2 != size) || g_mbf_full) {
		return E_TMOUT;
	}
	memcpy(g_sent[g_sent_n % 16], msg, 2);
	g_sent_n++;
	return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo) { (void)id; (void)msg; (void)tmo; return E_TMOUT; }
ER tk_dly_tsk(TMO tmo) { (void)tmo; return E_OK; }
ER tk_slp_tsk(TMO tmo) { (void)tmo; return E_OK; }

static char g_log[8192];
INT tm_printf(const UB *format, ...)
{
	va_list ap;
	size_t  len = strlen(g_log);

	va_start(ap, format);
	(void)vsnprintf(g_log + len, sizeof(g_log) - len, (const char *)format, ap);
	va_end(ap);
	return 0;
}

/* ---- fakes: board ---------------------------------------------------- */
UW   in_w(UW addr) { (void)addr; return 0xFFFFFFFFu; }
void out_w(UW addr, UW data) { (void)addr; (void)data; }
void clr_w(UW addr, UW data) { (void)addr; (void)data; }
ER   gpio_set_pin(UINT no, UINT mode) { (void)no; (void)mode; return E_OK; }
ER   gpio_set_val(UINT no, UINT val) { (void)no; (void)val; return E_OK; }
UINT gpio_get_val(UINT no) { (void)no; return 1U; }
ER   pwm_set_pin(UINT no) { (void)no; return E_OK; }
ER   pwm_set_enabled(UINT no, BOOL en) { (void)no; (void)en; return E_OK; }
ER   pwm_set_wrap(UINT no, UW wrap) { (void)no; (void)wrap; return E_OK; }
ER   pwm_set_cc(UINT no, UW cc) { (void)no; (void)cc; return E_OK; }
ER   gpio_irq_init(void) { return E_OK; }
ER   gpio_irq_attach(UINT pin, UW edges, gpio_isr_t p_isr) { (void)pin; (void)edges; (void)p_isr; return E_OK; }

/* ---- fakes: barcode_task and comms_task ------------------------------ */
static uint32_t g_bar_word = BARCODE_CMD_NONE;
uint32_t barcode_cmd_word(void) { return g_bar_word; }
ER comms_tele_send(const UB *body, INT length) { (void)body; (void)length; return E_OK; }

/* barcode_task's store: letter, action, sequence */
static void bar_store(uint32_t letter, uint32_t action, uint32_t seq)
{
	g_bar_word = barcode_cmd_pack(letter, action, seq);
}

/* ---- helpers --------------------------------------------------------- */
static void reset(void)
{
	nav_bind_t bind;

	memset(g_flg, 0, sizeof(g_flg));
	g_next_flg  = 1;
	g_nav_flgid = 0;
	g_fault_flgid = 0;
	(void)nav_ctrl_init();
	memset(&bind, 0, sizeof(bind));
	bind.nav_cmd_mbfid = NAV_CMD_MBF_ID;
	nav_bind(&bind);
	g_sent_n = 0;
	g_mbf_full = 0;
	g_bar_word = BARCODE_CMD_NONE;
	g_cmd_taken = 0U;
	g_move_open = false;
	g_cmd_sent = false;
	g_log[0] = '\0';
	g_mission = (uint32_t)MS_RUNNING;
}

/* the last JUNCTION nav sent is this action */
static bool sent_junction(uint32_t action)
{
	return (0 < g_sent_n) && (NAV_CMD_JUNCTION == g_sent[(g_sent_n - 1) % 16][0])
	       && (action == g_sent[(g_sent_n - 1) % 16][1]);
}

/* ---- scenarios ------------------------------------------------------- */

/* The bug: a junction motion refuses (here during a bypass) used to use up
   the barcode command, so the real junction after it went straight on. */
static void refused_keeps_command(void)
{
	reset();
	bar_store(11u, 1u, 1u);			/* 'L', LEFT, seq 1 */
	on_junction();				/* GP7 crosses a line mid-bypass */
	CHECK(1 == g_sent_n);
	CHECK(sent_junction(1u));
	on_motion_reply(NAV_FLG_CMD_REJECTED);	/* motion was in BYPASS */
	CHECK(NULL != strstr(g_log, "CMD_REJECTED, barcode command kept for the next junction"));
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();				/* the real junction */
	CHECK(2 == g_sent_n);
	CHECK(sent_junction(1u));		/* still LEFT */
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* turned */
	CHECK(!barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();				/* the next junction: no command */
	CHECK(2 == g_sent_n);
	CHECK(NULL != strstr(g_log, "no barcode command: straight on"));
}

/* MOVE_DONE uses it up at once, as before. */
static void done_uses_command(void)
{
	reset();
	bar_store(17u, 2u, 5u);			/* 'R', RIGHT, seq 5 */
	on_junction();
	CHECK(sent_junction(2u));
	on_motion_reply(NAV_FLG_MOVE_DONE);
	CHECK(!barcode_cmd_pending(g_bar_word, g_cmd_taken));
	CHECK(NULL == strstr(g_log, "kept"));
}

/* A new barcode read while the turn runs is for the next junction. */
static void newer_command_waits(void)
{
	reset();
	bar_store(11u, 1u, 1u);			/* LEFT, seq 1 */
	on_junction();
	bar_store(20u, 3u, 2u);			/* 'U', U_TURN, seq 2, read mid-turn */
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* the LEFT is done */
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	on_junction();
	CHECK(sent_junction(3u));		/* the U-turn, at the next junction */
}

/* A junction while a move is open is ignored; a full buffer sends nothing
   and keeps the command; a mission change drops the open move. */
static void edges(void)
{
	reset();
	bar_store(11u, 1u, 1u);
	on_junction();
	on_junction();				/* during the turn */
	CHECK(1 == g_sent_n);
	CHECK(NULL != strstr(g_log, "junction during a move: ignored"));

	reset();
	bar_store(11u, 1u, 1u);
	g_mbf_full = 1;
	on_junction();
	CHECK(0 == g_sent_n);
	CHECK(!g_move_open);
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));
	g_mbf_full = 0;
	on_junction();
	CHECK(sent_junction(1u));

	reset();
	bar_store(11u, 1u, 1u);
	on_junction();
	set_state(MS_FINISHED);			/* the open move ends with it */
	CHECK(!g_move_open && !g_cmd_sent);
	on_motion_reply(NAV_FLG_MOVE_DONE);	/* a late reply uses nothing */
	CHECK(barcode_cmd_pending(g_bar_word, g_cmd_taken));

	reset();
	g_mission = (uint32_t)MS_PAUSED;
	bar_store(11u, 1u, 1u);
	on_junction();
	CHECK(0 == g_sent_n);			/* not Running: ignored */
}

int main(void)
{
	refused_keeps_command();
	done_uses_command();
	newer_command_waits();
	edges();
	printf("nav junctions: %d checks, %d failures\n", g_checks, g_failures);
	return (0 == g_failures) ? 0 : 1;
}
