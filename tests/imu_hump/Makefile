# Host tests for the IMU hump detector (Buddy 4).  Plain host gcc, no ARM
# toolchain and no Docker needed:  make -C tests/imu_hump
# make V=1 also prints every line hump.c would send to the console.
# Lives outside app_program/ on purpose: everything under app_program/ is
# compiled into the firmware.
#
# hump.c builds against stub/ in place of the kernel headers, with the
# same 32-bit W/UW as the RP2040, and under UBSan/ASan.
#
# The same scenarios run again against each long-ramp build switch in
# hump.h, alone and together; the ramp-length scenarios expect what each
# setting gives (19 Nov: the final ramp is a different width).

SRC_DIR  := ../../app_program/imu_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) -Istub -I$(SRC_DIR)

VARIANTS      := enter40 waydown long
FLAGS_enter40 := -DHUMP_ENTER_MG=40
FLAGS_waydown := -DHUMP_WAY_DOWN=1
FLAGS_long    := -DHUMP_ENTER_MG=40 -DHUMP_WAY_DOWN=1

.PHONY: test clean
test: test_imu_hump $(VARIANTS:%=test_imu_hump_%)
	./test_imu_hump $(if $(V),-v)
	for v in $(VARIANTS); do ./test_imu_hump_$$v $(if $(V),-v) || exit 1; done

hump.o: $(SRC_DIR)/hump.c $(SRC_DIR)/hump.h
	$(CC) $(BASE) -Wno-format -c -o $@ $<

test_imu_hump: test_imu_hump.c hump.o $(SRC_DIR)/hump.h
	$(CC) $(BASE) -o $@ test_imu_hump.c hump.o -lm

hump_%.o: $(SRC_DIR)/hump.c $(SRC_DIR)/hump.h
	$(CC) $(BASE) $(FLAGS_$*) -Wno-format -c -o $@ $<

test_imu_hump_%: test_imu_hump.c hump_%.o $(SRC_DIR)/hump.h
	$(CC) $(BASE) $(FLAGS_$*) -o $@ test_imu_hump.c hump_$*.o -lm

clean:
	rm -f test_imu_hump hump.o $(VARIANTS:%=test_imu_hump_%) \
	      $(VARIANTS:%=hump_%.o)

.DEFAULT_GOAL := test
