# Host tests for Buddy 3 (ir_sense_task, barcode_task): calibration, line
# tracking and junctions, barcode capture -> decode -> table, map updates
# (also through barcode_task.c's own loop), and line_state's contract with
# motion_task's statechart.  Plain host gcc,
# no ARM toolchain:
#     make -C tests/line
# Lives outside app_program/ on purpose: everything under app_program/ is
# compiled into the firmware.

SRC_DIR  := ../../app_program/line_task
SM_DIR   := ../../app_program/motor_task
CC       ?= gcc
SAN      := -fsanitize=undefined,address -fno-sanitize-recover=all
BASE     := -std=c11 -O1 -g -Wall -Wextra -Wpedantic -Werror $(SAN) -I$(SRC_DIR)

# The writeup's example table (A left, B right, C straight, D U-turn) for
# the compiled-in table builds, and one with a bad character.
TABLE_EX  := -DBARCODE_DEFAULT_TABLE_SET=1 -DBARCODE_DEFAULT_TABLE='"LRFUFFFFFFFFFFFFFFFFFFFFFF"'
TABLE_BAD := -DBARCODE_DEFAULT_TABLE_SET=1 -DBARCODE_DEFAULT_TABLE='"LRFXFFFFFFFFFFFFFFFFFFFFFF"' -DTEST_BAD_TABLE
# barcode_task.c itself, on the motion sim's kernel stand-ins.
TASK     := -std=gnu11 -O1 -g -Wall -Wextra -Werror $(SAN) -I../motion_task_sim/stubs -I$(SRC_DIR)
TASK_SRC := $(SRC_DIR)/bar_capture.c $(SRC_DIR)/barcode_rx.c $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c
TASK_DEP := test_barcode_task.c check.h $(SRC_DIR)/barcode_task.c $(SRC_DIR)/barcode_task.h $(TASK_SRC) \
            $(SRC_DIR)/barcode_action.h ../../app_program/common/line_config.h

BINS := test_barcode_decode test_barcode_action test_barcode_action_table test_ir_cal \
        test_line_track test_barcode_pipeline test_line_state \
        test_barcode_task test_barcode_task_table test_barcode_task_bad

.PHONY: test clean
test: $(BINS)
	@for t in $(BINS); do ./$$t || exit 1; done

test_barcode_decode: ../test_barcode_decode.c $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_decode.h
	$(CC) $(BASE) -o $@ ../test_barcode_decode.c $(SRC_DIR)/barcode_decode.c

test_barcode_action: test_barcode_action.c check.h $(SRC_DIR)/barcode_action.c $(SRC_DIR)/barcode_action.h
	$(CC) $(BASE) -o $@ test_barcode_action.c $(SRC_DIR)/barcode_action.c

test_barcode_action_table: test_barcode_action.c check.h $(SRC_DIR)/barcode_action.c $(SRC_DIR)/barcode_action.h
	$(CC) $(BASE) $(TABLE_EX) -o $@ test_barcode_action.c $(SRC_DIR)/barcode_action.c

test_barcode_task: $(TASK_DEP)
	$(CC) $(TASK) -o $@ test_barcode_task.c $(TASK_SRC)

test_barcode_task_table: $(TASK_DEP)
	$(CC) $(TASK) $(TABLE_EX) -o $@ test_barcode_task.c $(TASK_SRC)

test_barcode_task_bad: $(TASK_DEP)
	$(CC) $(TASK) $(TABLE_BAD) -o $@ test_barcode_task.c $(TASK_SRC)

test_ir_cal: test_ir_cal.c check.h $(SRC_DIR)/ir_cal.c $(SRC_DIR)/ir_cal.h
	$(CC) $(BASE) -o $@ test_ir_cal.c $(SRC_DIR)/ir_cal.c

test_line_track: test_line_track.c check.h $(SRC_DIR)/line_track.c $(SRC_DIR)/line_track.h $(SRC_DIR)/ir_cal.c $(SRC_DIR)/line_state.h $(SM_DIR)/motion_cfg.h
	$(CC) $(BASE) -o $@ test_line_track.c $(SRC_DIR)/line_track.c $(SRC_DIR)/ir_cal.c

test_barcode_pipeline: test_barcode_pipeline.c check.h $(SRC_DIR)/bar_capture.c $(SRC_DIR)/bar_capture.h $(SRC_DIR)/barcode_rx.c $(SRC_DIR)/barcode_rx.h $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c
	$(CC) $(BASE) -o $@ test_barcode_pipeline.c $(SRC_DIR)/bar_capture.c $(SRC_DIR)/barcode_rx.c $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c

test_line_state: test_line_state.c check.h $(SRC_DIR)/line_state.h $(SRC_DIR)/barcode_action.h $(SM_DIR)/motion_sm.c $(SM_DIR)/motion_sm.h
	$(CC) $(BASE) -o $@ test_line_state.c $(SM_DIR)/motion_sm.c

clean:
	rm -f $(BINS)

.DEFAULT_GOAL := test
