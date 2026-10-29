#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "line_config.h"

#define LINE_SAMPLE_PERIOD_MS           (1u)
#define LINE_CONFIRM_SAMPLES            (6u)
#define LINE_LOST_SAMPLES               (8u)
#define LINE_HEARTBEAT_MS               (2000u)
#define LINE_COMMISSION_DURATION_MS     (5000u)
#define LINE_COMMISSION_SAMPLE_PERIOD_MS (100u)
#define LINE_COMMISSION_SAMPLE_COUNT    \
    (LINE_COMMISSION_DURATION_MS / LINE_COMMISSION_SAMPLE_PERIOD_MS)
/* Compile-time behavior switches. */
#ifndef LINE_STEER_INVERT
#define LINE_STEER_INVERT                (0)
#endif
#ifndef LINE_STOP_ON_LOST
#define LINE_STOP_ON_LOST                (0)
#endif
#ifndef LINE_COMMISSIONING_MODE
#define LINE_COMMISSIONING_MODE          (0)
#endif

#if (LINE_STEER_INVERT != 0) && (LINE_STEER_INVERT != 1)
#error "LINE_STEER_INVERT must be 0 or 1"
#endif
#if (LINE_STOP_ON_LOST != 0) && (LINE_STOP_ON_LOST != 1)
#error "LINE_STOP_ON_LOST must be 0 or 1"
#endif
#if (LINE_COMMISSIONING_MODE != 0) && (LINE_COMMISSIONING_MODE != 1)
#error "LINE_COMMISSIONING_MODE must be 0 or 1"
#endif

#if (LINE_LEFT_GPIO > 29u) || (LINE_RIGHT_GPIO > 29u) || \
    (BARCODE_GPIO > 29u) || (BATTERY_SENSE_GPIO > 29u)
#error "Configured RP2040 GPIO numbers must be in the range 0..29"
#endif
#if LINE_LEFT_GPIO == LINE_RIGHT_GPIO
#error "The left and right line sensors must use distinct GPIOs"
#endif
#if BARCODE_SENSOR_AVAILABLE && \
    ((BARCODE_GPIO == LINE_LEFT_GPIO) || (BARCODE_GPIO == LINE_RIGHT_GPIO))
#error "The barcode GPIO must not duplicate a line-sensor GPIO"
#endif
#if BATTERY_SENSE_GPIO != (26u + BATTERY_SENSE_ADC_CHANNEL)
#error "The battery-sense GPIO and ADC channel do not match"
#endif

#endif
