# Host test for scan_task (app_program/obstacle_task/obstacle_task.c,
# Statechart 5): the real obstacle_task.c and scan_logic.c on the motion
# sim's kernel stubs, with fakes for scan_if, the HC-SR04 and the servo.
#     make -C tests/obstacle          (in the build container on macOS)

SRC  := ../../app_program
CC   ?= gcc
SAN  ?= -fsanitize=undefined,address -fno-sanitize-recover=all
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) \
         -I../motion_task_sim/stubs -I$(SRC)/obstacle_task

.PHONY: test clean
test: sim_obstacle
	./sim_obstacle

sim_obstacle: sim_obstacle.c $(SRC)/obstacle_task/obstacle_task.c \
              $(SRC)/obstacle_task/scan_logic.c $(SRC)/obstacle_task/scan_logic.h \
              $(SRC)/obstacle_task/scan_if.h
	$(CC) $(FLAGS) -o $@ sim_obstacle.c $(SRC)/obstacle_task/scan_logic.c

clean:
	rm -f sim_obstacle

.DEFAULT_GOAL := test
