# Host tests for imu_task's abnormal-tilt and read-failure checks and the
# motion classifier (Buddy 4).  Plain host gcc, no ARM toolchain:
#     make -C tests/imu_safety
# Lives outside app_program/ on purpose: everything under app_program/ is
# compiled into the firmware.
#
# imu_safety.c, imu_class.c (and hump.c, for imu_isqrt) build against the stub kernel
# headers in ../imu_hump/stub, with the RP2040's 32-bit W/UW, under
# UBSan/ASan.

SRC_DIR  := ../../app_program/imu_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) \
            -I../imu_hump/stub -I$(SRC_DIR)

.PHONY: test clean
test: test_imu_safety
	./test_imu_safety

imu_class.o: $(SRC_DIR)/imu_class.c $(SRC_DIR)/imu_class.h
	$(CC) $(BASE) -c -o $@ $<

imu_safety.o: $(SRC_DIR)/imu_safety.c $(SRC_DIR)/imu_safety.h $(SRC_DIR)/hump.h
	$(CC) $(BASE) -c -o $@ $<

hump.o: $(SRC_DIR)/hump.c $(SRC_DIR)/hump.h
	$(CC) $(BASE) -Wno-format -c -o $@ $<

test_imu_safety: test_imu_safety.c imu_safety.o imu_class.o hump.o
	$(CC) $(BASE) -o $@ test_imu_safety.c imu_safety.o imu_class.o hump.o -lm

clean:
	rm -f test_imu_safety imu_safety.o imu_class.o hump.o

.DEFAULT_GOAL := test
