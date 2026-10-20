# Host unit tests for motion_lib (Buddy 2).  Plain host gcc, no ARM
# toolchain and no Docker needed:  make -C tests/motion_lib
# Lives outside app_program/ on purpose: everything under app_program/ is
# compiled into the firmware.
#
# The library is compiled with strict warnings; the test file only with
# -Wall -Wextra.  Both run under UBSan/ASan, so a signed overflow or an
# out-of-bounds table read fails the run.

SRC_DIR  := ../../app_program/motor_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=c11 -O1 -g -Wall -Wextra -Werror $(SAN) -I$(SRC_DIR)
STRICT   := $(BASE) -Wconversion -Wsign-conversion -Wshadow -Wpedantic

.PHONY: test clean
test: test_motion_lib
	./test_motion_lib

motion_lib.o: $(SRC_DIR)/motion_lib.c $(SRC_DIR)/motion_lib.h
	$(CC) $(STRICT) -c -o $@ $<

test_motion_lib: test_motion_lib.c motion_lib.o $(SRC_DIR)/motion_lib.h
	$(CC) $(BASE) -o $@ test_motion_lib.c motion_lib.o

clean:
	rm -f test_motion_lib motion_lib.o

.DEFAULT_GOAL := test
