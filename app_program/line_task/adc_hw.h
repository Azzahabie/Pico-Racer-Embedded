#ifndef ROBOCAR_ADC_HW_H
#define ROBOCAR_ADC_HW_H

#include <stdint.h>

#include <tk/tkernel.h>

ER adc_hw_init(void);
void adc_hw_pin_init(uint32_t gpio);

/*
 * Returns a 12-bit average, or UINT16_MAX if a bounded hardware wait fails.
 * Only one task may call adc_hw_read(); callers with multiple ADC readers
 * must protect calls with a lock.
 */
uint16_t adc_hw_read(uint32_t channel);
ER adc_hw_last_error(void);

#endif
