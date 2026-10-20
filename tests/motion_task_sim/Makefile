# Host simulation of motion_task (Buddy 2): the real motor_task.c and
# motion_lib.c against a simulated car.  Plain host gcc:
#     make -C tests/motion_task_sim          # both bench modes
#     make -C tests/motion_task_sim V=1      # also print the console output

SRC   := ../../app_program/motor_task
CC    ?= gcc
SAN   := -fsanitize=undefined,address -fno-sanitize-recover=all
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-function -Werror $(SAN) \
         -Istubs -I$(SRC)
RUNARG := $(if $(V),-v,)

.PHONY: test clean
test: sim_open sim_pid
	./sim_open $(RUNARG)
	./sim_pid $(RUNARG)

sim_open: sim_motion.c $(SRC)/motor_task.c $(SRC)/motion_lib.c $(wildcard $(SRC)/*.h stubs/*/*.h)
	$(CC) $(FLAGS) -DMOTION_TEST_PID=0 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

sim_pid: sim_motion.c $(SRC)/motor_task.c $(SRC)/motion_lib.c $(wildcard $(SRC)/*.h stubs/*/*.h)
	$(CC) $(FLAGS) -DMOTION_TEST_PID=1 -o $@ sim_motion.c $(SRC)/motion_lib.c -lm

clean:
	rm -f sim_open sim_pid

.DEFAULT_GOAL := test
