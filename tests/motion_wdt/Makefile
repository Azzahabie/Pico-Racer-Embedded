# Host test for the watchdog driver (app_program/motor_task/motion_wdt.c)
# against fake registers.  Plain host gcc:  make -C tests/motion_wdt
# Built with MOTION_BENCH=1 so the watchdog test functions are checked too.

SRC   := ../../app_program/motor_task
CC    ?= gcc
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror \
         -fsanitize=undefined,address -fno-sanitize-recover=all \
         -I../motion_task_sim/stubs -I$(SRC)

.PHONY: test clean
test: test_motion_wdt
	./test_motion_wdt

test_motion_wdt: test_motion_wdt.c $(SRC)/motion_wdt.c $(wildcard $(SRC)/*.h)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -o $@ test_motion_wdt.c

clean:
	rm -f test_motion_wdt

.DEFAULT_GOAL := test
