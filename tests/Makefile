# Host tests (plain gcc on a PC, no ARM toolchain, no board):
#     make -C tests
# Each folder also runs on its own, e.g. make -C tests/motion_lib.
# These live outside app_program/ because every .c under app_program/ is
# compiled into the firmware.

SUITES := motion_lib motion_sm motor motion_wdt encoder encoder_dispatch motion_task_sim imu_hump imu_safety line nav

# Single-file tests at this level (no folder Makefile of their own).
CC   ?= gcc
FILE_TESTS := scan_logic host_messages

.PHONY: test clean $(SUITES) $(FILE_TESTS)
test: $(SUITES) $(FILE_TESTS)

$(SUITES):
	$(MAKE) -s -C $@

# Buddy 5: scan_task's pure logic (NFR-16, NFR-20, D14 lane check); again
# with the known-width sizing option (-DSL_KNOWN_W_MM=90, Test 20).
scan_logic:
	$(CC) -std=gnu11 -Wall -Wextra -Werror -I ../app_program/obstacle_task \
	    -o scan_logic_test scan_logic_test.c ../app_program/obstacle_task/scan_logic.c
	./scan_logic_test | tail -n 1
	$(CC) -std=gnu11 -Wall -Wextra -Werror -I ../app_program/obstacle_task -DSL_KNOWN_W_MM=90 \
	    -o scan_logic_test_known scan_logic_test.c ../app_program/obstacle_task/scan_logic.c
	./scan_logic_test_known | tail -n 1 | sed 's/^/SL_KNOWN_W_MM=90: /'

# Every task: messages.h packers against independent vectors (NFR-17).
host_messages:
	$(CC) -std=c99 -Wall -Wextra -Werror -o host/test_messages host/test_messages.c
	./host/test_messages

clean:
	for s in $(SUITES); do $(MAKE) -s -C $$s clean; done
	rm -f scan_logic_test scan_logic_test_known host/test_messages
