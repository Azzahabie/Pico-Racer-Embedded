# Host tests for the IMU hump detector (Buddy 4).  Plain host gcc, no ARM
# toolchain and no Docker needed:  make -C tests/imu_hump
# make V=1 also prints every line hump.c would send to the console.
# Lives outside app_program/ on purpose: everything under app_program/ is
# compiled into the firmware.
#
# hump.c builds against stub/ in place of the kernel headers, with the
# same 32-bit W/UW as the RP2040, and under UBSan/ASan.

SRC_DIR  := ../../app_program/imu_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) -Istub -I$(SRC_DIR)

.PHONY: test clean
test: test_imu_hump
	./test_imu_hump $(if $(V),-v)

hump.o: $(SRC_DIR)/hump.c $(SRC_DIR)/hump.h
	$(CC) $(BASE) -Wno-format -c -o $@ $<

test_imu_hump: test_imu_hump.c hump.o $(SRC_DIR)/hump.h
	$(CC) $(BASE) -o $@ test_imu_hump.c hump.o -lm

clean:
	rm -f test_imu_hump hump.o

.DEFAULT_GOAL := test
