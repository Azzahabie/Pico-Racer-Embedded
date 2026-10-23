# Host test for the encoder driver (app_program/motor_task/encoder.c), in
# both console builds.  Plain host gcc:  make -C tests/encoder

SRC   := ../../app_program/motor_task
CC    ?= gcc
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror \
         -fsanitize=undefined,address -fno-sanitize-recover=all \
         -I../motion_task_sim/stubs -I$(SRC)
DEPS  := test_encoder.c $(SRC)/encoder.c $(SRC)/motion_lib.c \
         $(wildcard $(SRC)/*.h $(SRC)/../common/*.h ../motion_task_sim/stubs/*/*.h)

.PHONY: test clean
test: test_encoder_usb test_encoder_uart
	./test_encoder_usb
	./test_encoder_uart

test_encoder_usb: $(DEPS)
	$(CC) $(FLAGS) -DTM_CONSOLE_USB_CDC=1 -o $@ test_encoder.c $(SRC)/motion_lib.c

test_encoder_uart: $(DEPS)
	$(CC) $(FLAGS) -DTM_CONSOLE_USB_CDC=0 -o $@ test_encoder.c $(SRC)/motion_lib.c

clean:
	rm -f test_encoder_usb test_encoder_uart

.DEFAULT_GOAL := test
