# Host test for the PWM motor driver (app_program/motor_task/motor.c)
# against fake registers.  Plain host gcc:  make -C tests/motor

SRC   := ../../app_program/motor_task
CC    ?= gcc
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror \
         -fsanitize=undefined,address -fno-sanitize-recover=all \
         -I../motion_task_sim/stubs -I$(SRC)

.PHONY: test clean
test: test_motor
	./test_motor

test_motor: test_motor.c $(SRC)/motor.c $(SRC)/motion_lib.c $(wildcard $(SRC)/*.h)
	$(CC) $(FLAGS) -o $@ test_motor.c $(SRC)/motion_lib.c

clean:
	rm -f test_motor

.DEFAULT_GOAL := test
