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
DEPS   := sim_motion.c $(SRC)/motor_task.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c \
          $(wildcard $(SRC)/*.h $(SRC)/../common/*.h stubs/*/*.h)

.PHONY: test clean
test: sim_off sim_off_unpacked sim_known sim_arc sim_arc_cube sim_arc_cyl sim_open sim_pid sim_wdt sim_stop
	./sim_off $(RUNARG)
	./sim_off_unpacked $(RUNARG)
	./sim_known $(RUNARG)
	./sim_arc $(RUNARG)
	./sim_arc_cube $(RUNARG)
	./sim_arc_cyl $(RUNARG)
	./sim_open $(RUNARG)
	./sim_pid $(RUNARG)
	./sim_wdt $(RUNARG)
	./sim_stop $(RUNARG)

# Integration build (the committed default): no bench code at all.
sim_off: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# The same with the Week 6 (unpacked) layouts, MOTION_PACKED 0: the
# Week 13 "before" build, with distance_state behind a mutex.
sim_off_unpacked: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_PACKED=0 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# The straight-past bypass for the known demo box (MOTION_BYPASS_KNOWN 1):
# no side looks alongside, one look along the way back in.
sim_known: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_BYPASS_KNOWN=1 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# The arc bypass (MOTION_BYPASS_ARC 1, test option): round the known box.
sim_arc: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_BYPASS_ARC=1 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# The arc round other obstacles: a 200 mm cube and a 100 mm cylinder.
sim_arc_cube: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_BYPASS_ARC=1 -DMOTION_OBS_W_MM=200 -DMOTION_OBS_L_MM=200 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

sim_arc_cyl: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=0 -DMOTION_TEST_PID=0 -DMOTION_BYPASS_ARC=1 -DMOTION_OBS_ROUND=1 -DMOTION_OBS_W_MM=100 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

sim_open: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=0 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

sim_pid: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=1 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# Watchdog test image (MOTION_TEST_WDT 1): hangs in L fwd on purpose.
sim_wdt: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=0 -DMOTION_TEST_WDT=1 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

# Stop test image (MOTION_TEST_STOP 1): NFR-18 trials from the stand-in.
sim_stop: $(DEPS)
	$(CC) $(FLAGS) -DMOTION_BENCH=1 -DMOTION_TEST_PID=0 -DMOTION_TEST_STOP=1 -o $@ sim_motion.c $(SRC)/motion_lib.c $(SRC)/motion_sm.c -lm

clean:
	rm -f sim_off sim_off_unpacked sim_known sim_arc sim_arc_cube sim_arc_cyl sim_open sim_pid sim_wdt sim_stop

.DEFAULT_GOAL := test
