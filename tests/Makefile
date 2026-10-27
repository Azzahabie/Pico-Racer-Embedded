# Host tests (plain gcc on a PC, no ARM toolchain, no board):
#     make -C tests
# Each folder also runs on its own, e.g. make -C tests/motion_lib.
# These live outside app_program/ because every .c under app_program/ is
# compiled into the firmware.

SUITES := motion_lib motion_sm motor motion_wdt encoder encoder_dispatch motion_task_sim imu_hump

.PHONY: test clean $(SUITES)
test: $(SUITES)

$(SUITES):
	$(MAKE) -s -C $@

clean:
	for s in $(SUITES); do $(MAKE) -s -C $$s clean; done
