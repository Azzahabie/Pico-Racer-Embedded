# Host test for nav_ctrl_task's junction handling (app_program/nav_task/
# nav_ctrl.c): the real nav_ctrl.c with the motion sim's kernel stubs.
#     make -C tests/nav          (in the build container on macOS)

SRC  := ../../app_program
CC   ?= gcc
SAN  ?= -fsanitize=undefined,address -fno-sanitize-recover=all
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) \
         -Istubs -I../motion_task_sim/stubs -I$(SRC)/nav_task

.PHONY: test clean
test: sim_nav
	./sim_nav

sim_nav: sim_nav.c $(SRC)/nav_task/nav_ctrl.c $(SRC)/nav_task/nav_ctrl.h \
         $(SRC)/line_task/barcode_action.c $(SRC)/line_task/barcode_action.h \
         stubs/bsp/libbsp.h
	$(CC) $(FLAGS) -o $@ sim_nav.c $(SRC)/line_task/barcode_action.c

clean:
	rm -f sim_nav

.DEFAULT_GOAL := test
