# Host test: motion's encoder driver through Buddy 5's real GPIO dispatcher
# (app_program/common/gpio_irq.c, unchanged) on a fake IO_BANK0, in both
# console builds.  Plain host gcc:  make -C tests/encoder_dispatch
#     make -C tests/encoder_dispatch race   # opt-in: the attach race

SRC   := ../../app_program
CC    ?= gcc
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror \
         -fsanitize=undefined,address -fno-sanitize-recover=all \
         -I../motion_task_sim/stubs -I$(SRC)/motor_task -I$(SRC)/common
UNITS := test_encoder_dispatch.c $(SRC)/common/gpio_irq.c \
         $(SRC)/motor_task/encoder.c $(SRC)/motor_task/motion_lib.c
DEPS  := $(UNITS) $(wildcard $(SRC)/motor_task/*.h $(SRC)/common/gpio_irq.h ../motion_task_sim/stubs/*/*.h)

.PHONY: test race clean
test: test_dispatch_usb test_dispatch_uart
	./test_dispatch_usb
	./test_dispatch_uart

test_dispatch_usb: $(DEPS)
	$(CC) $(FLAGS) -DTM_CONSOLE_USB_CDC=1 -o $@ $(UNITS)

test_dispatch_uart: $(DEPS)
	$(CC) $(FLAGS) -DTM_CONSOLE_USB_CDC=0 -o $@ $(UNITS)

# Expected to fail until gpio_irq_attach() stops doing a read-modify-write
# of PROCx_INTE with interrupts on (a proposal to Buddy 5).
race: test_dispatch_race
	./test_dispatch_race

test_dispatch_race: $(DEPS)
	$(CC) $(FLAGS) -Wno-unused-function -DTM_CONSOLE_USB_CDC=1 -DTEST_ATTACH_RACE -o $@ $(UNITS)

clean:
	rm -f test_dispatch_usb test_dispatch_uart test_dispatch_race

.DEFAULT_GOAL := test
