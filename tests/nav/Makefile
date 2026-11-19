# Host test for nav_ctrl_task (app_program/nav_task/nav_ctrl.c): junction
# commands and calibration, the real nav_ctrl.c on the motion sim stubs.
# sim_nav_test is the same with -DNAV_TEST_JUNCTION_ACTION=1 (a TEST image),
# sim_nav_drop with -DNAV_KEEP_CMD_ON_REFUSAL=0 (a refused junction uses up
# the barcode command, the design rule).
#     make -C tests/nav          (in the build container on macOS)

SRC  := ../../app_program
CC   ?= gcc
SAN  ?= -fsanitize=undefined,address -fno-sanitize-recover=all
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) \
         -Istubs -I../motion_task_sim/stubs -I$(SRC)/nav_task

.PHONY: test clean
test: sim_nav sim_nav_test sim_nav_drop
	./sim_nav
	./sim_nav_test
	./sim_nav_drop

DEPS := sim_nav.c $(SRC)/nav_task/nav_ctrl.c $(SRC)/nav_task/nav_ctrl.h \
        $(SRC)/line_task/barcode_action.c $(SRC)/line_task/barcode_action.h \
        stubs/bsp/libbsp.h

sim_nav: $(DEPS)
	$(CC) $(FLAGS) -o $@ sim_nav.c $(SRC)/line_task/barcode_action.c

sim_nav_test: $(DEPS)
	$(CC) $(FLAGS) -DNAV_TEST_JUNCTION_ACTION=1 -o $@ sim_nav.c $(SRC)/line_task/barcode_action.c

sim_nav_drop: $(DEPS)
	$(CC) $(FLAGS) -DNAV_KEEP_CMD_ON_REFUSAL=0 -o $@ sim_nav.c $(SRC)/line_task/barcode_action.c

clean:
	rm -f sim_nav sim_nav_test sim_nav_drop

.DEFAULT_GOAL := test
