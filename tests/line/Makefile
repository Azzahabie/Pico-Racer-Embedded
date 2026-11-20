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
        test_barcode_task test_barcode_task_table test_barcode_task_bad \
        test_junction_arm test_junction_arm_nogate test_scan_gate \
        test_bc_analog test_bc_analog_fixtures test_bc_analog_chain \
        test_barcode_task_analog

# The analog barcode path (BARCODE_SENSOR_ANALOG 1): Boon's capture and
# decoders as adapted, our front end; the car traces of 20 Nov
# (fixtures/); the barcode-armed junction with it; barcode_task's side.
ANALOG   := -DBARCODE_SENSOR_ANALOG=1
BC_SRC   := $(SRC_DIR)/bc_analog_front.c $(SRC_DIR)/bc_analog_capture.c $(SRC_DIR)/bc_analog_code39.c
BC_DEPS  := $(BC_SRC) $(SRC_DIR)/bc_analog_front.h $(SRC_DIR)/bc_analog_capture.h \
            $(SRC_DIR)/bc_analog_code39.h $(SRC_DIR)/bc_analog_cfg.h $(SRC_DIR)/bc_analog_link.h \
            ../../app_program/common/line_config.h check.h bc_sim.h

# The chain from GP7 to the junction flag (test_junction_arm.c).
JA_SRC := $(SRC_DIR)/bar_capture.c $(SRC_DIR)/barcode_rx.c \
          $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c \
          $(SRC_DIR)/line_track.c $(SRC_DIR)/ir_cal.c $(SRC_DIR)/junction_arm.c \
          $(SRC_DIR)/scan_gate.c
JA_DEPS := test_junction_arm.c check.h $(JA_SRC) $(SRC_DIR)/bar_capture.h \
           $(SRC_DIR)/barcode_rx.h $(SRC_DIR)/junction_arm.h $(SRC_DIR)/line_track.h \
           $(SRC_DIR)/scan_gate.h ../../app_program/motor_task/motion_ds.h \
           $(SM_DIR)/motion_sm.h ../../app_program/common/line_config.h
# The scan gate (test_scan_gate.c): the average, motion's state, and where
# it acts.
SG_SRC := $(SRC_DIR)/scan_gate.c $(SRC_DIR)/bar_capture.c $(SRC_DIR)/barcode_decode.c \
          $(SRC_DIR)/line_track.c $(SRC_DIR)/ir_cal.c
SG_DEPS := test_scan_gate.c check.h $(SG_SRC) $(SRC_DIR)/scan_gate.h \
           $(SRC_DIR)/bar_capture.h $(SRC_DIR)/line_track.h \
           ../../app_program/motor_task/motion_ds.h $(SM_DIR)/motion_sm.h \
           ../../app_program/common/line_config.h
# The scan gate built out but expected by the tests: the gate tests FAIL
# (they see the behaviour before the gate).  Not in the default run.
SCAN_OFF := -DLINE_SCAN_GATE=0 -DTEST_EXPECT_GATE=1
# The scan gate with a 500 ms average (two stages of 250 ms): the weave on
# a straight shuts it, and the weave checks FAIL.  Not in the default run.
SCAN_SHORT := -DLINE_SCAN_AVG_DIV=25
# The branch's rule before the barcode-armed junction: length only, END
# after the quiet zone, quiet factor 4, one split in the decoder, no glitch
# filter, no scan gate.  Not in the default run: it is expected to FAIL
# the envelope checks.
OLD_RULE := -DLINE_JUNCTION_RULE=0 -DBARCODE_END_ON_STOP=0 \
            -DBARCODE_Q_WIDTH_FACTOR=4 -DBARCODE_DECODE_SPLIT=0 \
            -DBARCODE_GLITCH_TICKS=0 -DBARCODE_NOT_BAR_RATIO_X10=0 \
            -DLINE_SCAN_GATE=0
# The barcode-armed junction without the H1 fix (no glitch filter, no
# "not a bar" inside a read): expected to FAIL the H1 checks.
H1_OFF := -DBARCODE_GLITCH_TICKS=0 -DBARCODE_NOT_BAR_RATIO_X10=0

.PHONY: test clean junction_old junction_h1_off junction_scan_off scan_gate_short
test: $(BINS)
	@for t in $(BINS); do ./$$t || exit 1; done

test_barcode_decode: ../test_barcode_decode.c $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_decode.h ../../app_program/common/line_config.h
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

test_line_track: test_line_track.c check.h $(SRC_DIR)/line_track.c $(SRC_DIR)/line_track.h $(SRC_DIR)/ir_cal.c $(SRC_DIR)/line_state.h $(SM_DIR)/motion_cfg.h ../../app_program/common/line_config.h
	$(CC) $(BASE) -o $@ test_line_track.c $(SRC_DIR)/line_track.c $(SRC_DIR)/ir_cal.c

test_barcode_pipeline: test_barcode_pipeline.c check.h $(SRC_DIR)/bar_capture.c $(SRC_DIR)/bar_capture.h $(SRC_DIR)/barcode_rx.c $(SRC_DIR)/barcode_rx.h $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c ../../app_program/common/line_config.h
	$(CC) $(BASE) -o $@ test_barcode_pipeline.c $(SRC_DIR)/bar_capture.c $(SRC_DIR)/barcode_rx.c $(SRC_DIR)/barcode_decode.c $(SRC_DIR)/barcode_action.c

test_line_state: test_line_state.c check.h $(SRC_DIR)/line_state.h $(SRC_DIR)/barcode_action.h $(SM_DIR)/motion_sm.c $(SM_DIR)/motion_sm.h
	$(CC) $(BASE) -o $@ test_line_state.c $(SM_DIR)/motion_sm.c

test_junction_arm: $(JA_DEPS)
	$(CC) $(BASE) -o $@ test_junction_arm.c $(JA_SRC) -lm

# LINE_SCAN_GATE 0: everything before the gate, unchanged
test_junction_arm_nogate: $(JA_DEPS)
	$(CC) $(BASE) -DLINE_SCAN_GATE=0 -o $@ test_junction_arm.c $(JA_SRC) -lm

test_junction_old: $(JA_DEPS)
	$(CC) $(BASE) $(OLD_RULE) -o $@ test_junction_arm.c $(JA_SRC) -lm

test_junction_h1_off: $(JA_DEPS)
	$(CC) $(BASE) $(H1_OFF) -o $@ test_junction_arm.c $(JA_SRC) -lm

test_junction_scan_off: $(JA_DEPS)
	$(CC) $(BASE) $(SCAN_OFF) -o $@ test_junction_arm.c $(JA_SRC) -lm

test_bc_analog: test_bc_analog.c $(BC_DEPS)
	$(CC) $(BASE) $(ANALOG) -o $@ test_bc_analog.c $(BC_SRC)

test_bc_analog_fixtures: test_bc_analog_fixtures.c $(BC_DEPS) fixtures/run1_1506_Z_missing_first4.json
	$(CC) $(BASE) $(ANALOG) -DEXPECT_RUN4_F -o $@ test_bc_analog_fixtures.c $(BC_SRC)

test_bc_analog_chain: test_bc_analog_chain.c $(BC_DEPS) $(JA_DEPS)
	$(CC) $(BASE) $(ANALOG) -o $@ test_bc_analog_chain.c $(BC_SRC) \
	    $(SRC_DIR)/barcode_action.c $(SRC_DIR)/junction_arm.c $(SRC_DIR)/line_track.c $(SRC_DIR)/ir_cal.c

test_barcode_task_analog: $(TASK_DEP) $(BC_DEPS)
	$(CC) $(TASK) $(TABLE_EX) $(ANALOG) -o $@ test_barcode_task.c $(TASK_SRC) \
	    $(SRC_DIR)/bc_analog_capture.c $(SRC_DIR)/bc_analog_code39.c

test_scan_gate: $(SG_DEPS)
	$(CC) $(BASE) -o $@ test_scan_gate.c $(SG_SRC) -lm

test_scan_gate_short: $(SG_DEPS)
	$(CC) $(BASE) $(SCAN_SHORT) -o $@ test_scan_gate.c $(SG_SRC) -lm

test_junction_scan_short: $(JA_DEPS)
	$(CC) $(BASE) $(SCAN_SHORT) -o $@ test_junction_arm.c $(JA_SRC) -lm

# make -C tests/line junction_old: shows what the old rule gets wrong
junction_old: test_junction_old
	-./test_junction_old

# make -C tests/line junction_h1_off: shows what the H1 fix fixes
junction_h1_off: test_junction_h1_off
	-./test_junction_h1_off

# make -C tests/line junction_scan_off: the gate tests catch a missing gate
junction_scan_off: test_junction_scan_off
	-./test_junction_scan_off

# make -C tests/line scan_gate_short: why the average is not 500 ms
scan_gate_short: test_scan_gate_short test_junction_scan_short
	-./test_scan_gate_short
	-./test_junction_scan_short

clean:
	rm -f $(BINS) test_junction_old test_junction_h1_off test_junction_scan_off \
	      test_scan_gate_short test_junction_scan_short

.DEFAULT_GOAL := test
