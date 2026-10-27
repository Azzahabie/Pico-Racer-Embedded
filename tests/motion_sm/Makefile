# Host unit tests for the motion statechart (app_program/motor_task/
# motion_sm.c, design 6.2).  Plain host gcc:  make -C tests/motion_sm
# The module is compiled with strict warnings, the test with -Wall -Wextra;
# both run under UBSan/ASan.

SRC_DIR  := ../../app_program/motor_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=c11 -O1 -g -Wall -Wextra -Werror $(SAN) -I$(SRC_DIR)
STRICT   := $(BASE) -Wconversion -Wsign-conversion -Wshadow -Wpedantic

.PHONY: test clean
test: test_motion_sm
	./test_motion_sm

motion_sm.o: $(SRC_DIR)/motion_sm.c $(SRC_DIR)/motion_sm.h
	$(CC) $(STRICT) -c -o $@ $<

test_motion_sm: test_motion_sm.c motion_sm.o $(SRC_DIR)/motion_sm.h
	$(CC) $(BASE) -o $@ test_motion_sm.c motion_sm.o

clean:
	rm -f test_motion_sm motion_sm.o

.DEFAULT_GOAL := test
