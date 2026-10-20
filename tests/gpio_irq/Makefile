# Host test for the GPIO interrupt dispatcher (app_program/common/gpio_irq.c)
# against a fake IO_BANK0.  Plain host gcc:  make -C tests/gpio_irq

CC    ?= gcc
FLAGS := -std=gnu11 -O1 -g -Wall -Wextra -Werror \
         -fsanitize=undefined,address -fno-sanitize-recover=all \
         -I../motion_task_sim/stubs

.PHONY: test clean
test: test_gpio_irq
	./test_gpio_irq

test_gpio_irq: test_gpio_irq.c ../../app_program/common/gpio_irq.c ../../app_program/common/gpio_irq.h
	$(CC) $(FLAGS) -o $@ test_gpio_irq.c

clean:
	rm -f test_gpio_irq

.DEFAULT_GOAL := test
