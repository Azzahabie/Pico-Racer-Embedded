/*
 *	scan_logic_test.c
 *	PC test for app_program/obstacle_task/scan_logic.c (pure C), NFR-16.
 *	Lives OUTSIDE app_program/ on purpose: everything under app_program/
 *	is compiled into the firmware, and this file has main().
 *
 *	Build and run (laptop, plain gcc, from the repo root):
 *	  gcc -Wall -Wextra -I app_program/obstacle_task -o scan_logic_test \
 *	      tests/scan_logic_test.c app_program/obstacle_task/scan_logic.c
 *	  ./scan_logic_test
 *
 *	Each test names the requirement it gives evidence for.
 */

#include <stdio.h>
#include <string.h>
#include "scan_logic.h"

static int g_fails = 0;

static void grid_clear(uint16_t grid[SL_SLOTS])
{
	int idx;

	for(idx = 0; idx < SL_SLOTS; idx++) {
		grid[idx] = SL_NOT_MEASURED;
	}
}

static void set(uint16_t grid[SL_SLOTS], int deg, uint16_t dist_mm)
{
	grid[SL_IDX(deg)] = dist_mm;
}

static void check(const char *p_name, int cond)
{
	printf("  [%s] %s\n", cond ? "PASS" : "FAIL", p_name);
	if(!cond) {
		g_fails++;
	}
}

static void show(const sl_profile_t *p_prof, sl_action_t action)
{
	printf("  found=%d in_path=%d d_c=%d mm @ %d deg, edges %d..%d deg, "
	       "x %d..%d mm, W=%d mm, offset L=%d R=%d, clear L=%d R=%d, "
	       "open L=%d R=%d -> %s\n",
	       (int)p_prof->found, (int)p_prof->in_path,
	       (int)p_prof->d_close_mm, (int)p_prof->close_deg,
	       (int)p_prof->left_deg, (int)p_prof->right_deg,
	       (int)p_prof->x_left_mm, (int)p_prof->x_right_mm,
	       (int)p_prof->width_mm,
	       (int)p_prof->offset_left_mm, (int)p_prof->offset_right_mm,
	       (int)p_prof->clear_left_mm, (int)p_prof->clear_right_mm,
	       (int)p_prof->left_open, (int)p_prof->right_open,
	       sl_action_name(action));
}

static sl_action_t run(const uint16_t grid[SL_SLOTS], int32_t d_obs_mm,
		       bool reverse_tried, sl_profile_t *p_prof)
{
	sl_action_t act;

	sl_profile(grid, d_obs_mm, p_prof);
	act = sl_decide(p_prof, reverse_tried);
	show(p_prof, act);
	return act;
}

int main(void)
{
	uint16_t	grid[SL_SLOTS];
	sl_profile_t	prof;
	sl_action_t	act;
	sl_runs_t	runs;

	/* 1. Real bench scan (20 Oct): object ~19 cm ahead, 15-deg coarse
	 *    readings only, far readings beyond the edges. */
	printf("Test 1: real bench scan, object straight ahead (FR-16, FR-17)\n");
	grid_clear(grid);
	set(grid, -45, 960);  set(grid, -40, SL_FAR); set(grid, -30, 206);
	set(grid, -15, 193);  set(grid, 0, 192);      set(grid, 15, 193);
	set(grid, 30, SL_FAR);		/* read 1907 mm: beyond D_obs */
	set(grid, 40, SL_FAR); set(grid, 45, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("object found and in the path", prof.found && prof.in_path);
	check("closest is ~192 mm near 0 deg", 192 == prof.d_close_mm && 0 == prof.close_deg);
	check("edges at -30 and +15", -30 == prof.left_deg && 15 == prof.right_deg);
	check("both sides open", prof.left_open && prof.right_open);
	check("width about 10 cm (coarse data underestimates)",
	      prof.width_mm > 80 && prof.width_mm < 130);
	check("passes on the right (equal clearance, smaller offset)", SL_BYPASS_RIGHT == act);

	/* 2. Nothing within D_obs -> false alarm */
	printf("Test 2: false alarm (FR-17 continue)\n");
	grid_clear(grid);
	set(grid, -45, SL_FAR); set(grid, -30, 900); set(grid, 0, 850);
	set(grid, 30, SL_FAR);  set(grid, 45, 1200);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("continue", SL_CONTINUE == act);

	/* 3. Very close -> reverse */
	printf("Test 3: too close (FR-17 reverse)\n");
	grid_clear(grid);
	set(grid, -30, SL_FAR); set(grid, 0, 80); set(grid, 30, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("reverse", SL_REVERSE == act);

	/* 4. Wall: hits all the way to both scan limits -> blocked */
	printf("Test 4: wall across the whole scan (FR-17 stop, FR-18)\n");
	grid_clear(grid);
	set(grid, -45, 350); set(grid, -30, 300); set(grid, 0, 250);
	set(grid, 30, 300);  set(grid, 45, 350);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("neither side open", !prof.left_open && !prof.right_open);
	check("blocked", SL_BLOCKED == act);

	/* 5. Object on the right, fine-scanned edges -> pass left */
	printf("Test 5: object on the right (NFR-07 side choice)\n");
	grid_clear(grid);
	set(grid, -45, SL_FAR); set(grid, -30, SL_FAR); set(grid, 0, SL_FAR);
	set(grid, 5, 310); set(grid, 10, 300); set(grid, 30, 290);
	set(grid, 40, 300); set(grid, 45, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("edges 5..40 deg", 5 == prof.left_deg && 40 == prof.right_deg);
	check("d_c is the true closest point (290 @ 30)",
	      290 == prof.d_close_mm && 30 == prof.close_deg);
	check("bypass left", SL_BYPASS_LEFT == act);

	/* 6. sin table spot checks */
	printf("Test 6: sin table\n");
	check("sin(0) = 0", 0 == sl_sin1000(0));
	check("sin(30) = 500", 500 == sl_sin1000(30));
	check("sin(-15) = -259", -259 == sl_sin1000(-15));

	/* 7. Bench log 21 Oct: object at the right edge, outside the car's path */
	printf("Test 7: object beside the path, not in it (FR-17 continue)\n");
	grid_clear(grid);
	set(grid, -45, SL_FAR); set(grid, -30, SL_FAR); set(grid, 0, SL_FAR);
	set(grid, 30, SL_FAR);  set(grid, 40, 310); set(grid, 45, 300);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("found but not in the path", prof.found && !prof.in_path);
	check("continue (was BYPASS LEFT before)", SL_CONTINUE == act);

	/* 8. Two objects at different depths are not merged */
	printf("Test 8: near box ahead, far box on the left (FR-16 depth split)\n");
	grid_clear(grid);
	set(grid, -30, 380); set(grid, -15, 380); set(grid, -5, 200);
	set(grid, 0, 200);   set(grid, 5, 205);   set(grid, 10, SL_FAR);
	set(grid, 30, SL_FAR); set(grid, 45, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("near object only: edges -5..+5", -5 == prof.left_deg && 5 == prof.right_deg);
	check("left side open (gap to the far box)", prof.left_open);
	check("left clearance measured, not 'open'",
	      prof.clear_left_mm > 0 && prof.clear_left_mm < SL_CLEAR_OPEN_MM);
	check("right clearance larger -> bypass right", SL_BYPASS_RIGHT == act);

	/* 9. Gap on the left too narrow for the car -> right, even though the
	 *    left move would be shorter */
	printf("Test 9: narrow gap on the left (FR-18 safe route only)\n");
	grid_clear(grid);
	set(grid, -30, SL_FAR); set(grid, -10, 260); set(grid, -5, SL_FAR);
	set(grid, 0, 250);      set(grid, 10, 250);  set(grid, 15, SL_FAR);
	set(grid, 30, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("left clearance below C_pass", prof.clear_left_mm < SL_C_PASS_MM);
	check("left offset is the shorter one", prof.offset_left_mm < prof.offset_right_mm);
	check("bypass right", SL_BYPASS_RIGHT == act);

	/* 10. Still too close after a reverse -> blocked, no second reverse */
	printf("Test 10: too close, reverse already tried (Statechart 5)\n");
	grid_clear(grid);
	set(grid, -30, SL_FAR); set(grid, 0, 80); set(grid, 30, SL_FAR);
	act = run(grid, SL_D_OBS_MM, true, &prof);
	check("blocked", SL_BLOCKED == act);

	/* 11. Rescan after a reverse uses D_obs + reverse distance */
	printf("Test 11: rescan after reversing (Statechart 5)\n");
	grid_clear(grid);
	set(grid, -30, SL_FAR); set(grid, 0, 480); set(grid, 30, SL_FAR);
	act = run(grid, SL_D_OBS_MM, true, &prof);
	check("normal D_obs: looks like a false alarm", SL_CONTINUE == act);
	act = run(grid, SL_D_OBS_MM + SL_REV_DIST_MM, true, &prof);
	check("D_obs + reverse: still an obstacle", prof.found && SL_CONTINUE != act);

	/* 12. Off-path object is closer, but the one ahead matters */
	printf("Test 12: closer object off to the side, another ahead (FR-13)\n");
	grid_clear(grid);
	set(grid, -45, 200); set(grid, -40, SL_FAR); set(grid, -30, SL_FAR);
	set(grid, -5, 350);  set(grid, 0, 340);      set(grid, 5, 350);
	set(grid, 10, SL_FAR); set(grid, 30, SL_FAR); set(grid, 45, SL_FAR);
	act = run(grid, SL_D_OBS_MM, false, &prof);
	check("profiles the object ahead", 0 == prof.close_deg && prof.in_path);
	check("left clearance limited by the side object", prof.clear_left_mm < SL_CLEAR_OPEN_MM);
	check("not 'continue'", SL_CONTINUE != act);

	/* 13. N-in-a-row counters */
	printf("Test 13: Monitoring / Blocked / Sensor Lost counters\n");
	sl_runs_reset(&runs);
	sl_runs_add(&runs, sl_classify(false, 200U));
	check("one close reading is not enough", runs.close_run < SL_N_STOP);
	sl_runs_add(&runs, sl_classify(false, 210U));
	check("two close in a row -> stop", runs.close_run >= SL_N_STOP);
	sl_runs_reset(&runs);
	sl_runs_add(&runs, sl_classify(false, 200U));
	sl_runs_add(&runs, sl_classify(true, 0U));
	sl_runs_add(&runs, sl_classify(false, 200U));
	check("a no-echo in between breaks the run", runs.close_run < SL_N_STOP);
	sl_runs_reset(&runs);
	sl_runs_add(&runs, sl_classify(true, 0U));
	sl_runs_add(&runs, sl_classify(true, 0U));
	sl_runs_add(&runs, sl_classify(true, 0U));
	check("three no-echo -> sensor lost", runs.no_echo_run >= SL_N_NO_ECHO);
	sl_runs_add(&runs, sl_classify(false, SL_FAR));
	check("out of range is a valid echo, not a fault",
	      0U == runs.no_echo_run && 1U == runs.echo_run && 1U == runs.clear_run);
	check("D_stop boundary: 250 mm is clear, 249 mm is close",
	      SL_PING_CLEAR == sl_classify(false, 250U) &&
	      SL_PING_CLOSE == sl_classify(false, 249U));

	printf("\n%s (%d failed)\n",
	       g_fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fails);
	return g_fails ? 1 : 0;
}
