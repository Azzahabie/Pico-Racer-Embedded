# Host simulation of motion_task (Buddy 2): the real motor_task.c and
# motion_lib.c against a simulated car.  Plain host gcc:
#     make -C tests/motion_task_sim          # every build mode
#     make -C tests/motion_task_sim V=1      # also print the console output

SRC   := ../../app_program/motor_task
CC    ?= gcc
SAN   := -fsanitize=undefined,address -fno-sanitize-recover=all
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-function -Werror $(SAN) \
         -Istubs -I$(SRC)
RUNARG := $(if $(V),-v,)
DEPS   := sim_motion.c $(SRC)/motor_task.c $(SRC)/motion_lib.c \
          $(wildcard $(SRC)/*.h $(SRC)/../common/*.h stubs/*/*.h)

.PHONY: test clean
test: sim_off sim_off_unpacked sim_open sim_pid sim_wdt
	./sim_off $(RUNARG)
	./sim_off_unpacked $(RUNARG)
	./sim_open $(RUNARG)
	./sim_pid $(RUNARG)
	./sim_wdt $(RUNARG)

# Integration build (the committed default): no bench code at all.
sim_off: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

# The same with the Week 6 (unpacked) layouts, MOTION_PACKED 0: the
# Week 13 "before" build, with distance_state behind a mutex.
sim_off_unpacked: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_PACKED=0 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

sim_open: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=0 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

sim_pid: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=1 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

# Watchdog test image (MOTION_TEST_WDT 1): hangs in L fwd on purpose.
sim_wdt: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=0 -DMOTION_TEST_WDT=1 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

clean:
	rm -f sim_off sim_off_unpacked sim_open sim_pid sim_wdt

.DEFAULT_GOAL := test
