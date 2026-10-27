#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

/* Measure over white and at the midpoint of the 18 mm black line before use;
   set midpoint thresholds with a suitable hysteresis gap. */
#define LINE_SENSOR_LEFT_GPIO                    (26u)
#define LINE_SENSOR_LEFT_ADC_CHANNEL             (0u)
#define LINE_SENSOR_LEFT_BLACK_IS_HIGH           (1u)
#define LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD  (2500u)
#define LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD (2000u)

#define LINE_SENSOR_RIGHT_GPIO                    (27u)
#define LINE_SENSOR_RIGHT_ADC_CHANNEL             (1u)
#define LINE_SENSOR_RIGHT_BLACK_IS_HIGH           (1u)
#define LINE_SENSOR_RIGHT_BLACK_RISING_THRESHOLD  (2500u)
#define LINE_SENSOR_RIGHT_BLACK_FALLING_THRESHOLD (2000u)

#define BARCODE_SENSOR_GPIO                       (7u)
/* Verify the digital DO polarity on hardware. */
#define BARCODE_BLACK_IS_HIGH                     (1u)

/* GP28 remains reserved for battery sensing (ADC2). */
#define BATTERY_SENSE_GPIO                        (28u)
#define BATTERY_SENSE_ADC_CHANNEL                 (2u)

#define BARCODE_MAX_CHARACTERS                    (32u)
#define BARCODE_MAX_ELEMENTS                      ((BARCODE_MAX_CHARACTERS * 10u) + 19u)
#define BARCODE_MAX_WIDTHS                        (512u)
#define BARCODE_WIDE_NARROW_MIN_RATIO_X100        (150u)
#define BARCODE_GAP_MAX_NARROW_RATIO_X100         (300u)
#define BARCODE_SCAN_END_GAP_US                   (50000u)
#define BARCODE_SCAN_IDLE_US                      BARCODE_SCAN_END_GAP_US
#define BARCODE_SCAN_MAX_US                       (10000000u)

/* Compatibility names retained for the other board-configuration users. */
#define LINE_LEFT_GPIO                            LINE_SENSOR_LEFT_GPIO
#define LINE_RIGHT_GPIO                           LINE_SENSOR_RIGHT_GPIO
#define LINE_BLACK_IS_HIGH                        LINE_SENSOR_LEFT_BLACK_IS_HIGH
#define BARCODE_GPIO                              BARCODE_SENSOR_GPIO
#define KY033_DARK_LEVEL                          BARCODE_BLACK_IS_HIGH
#define BARCODE_SENSOR_AVAILABLE                  (1u)

#define MOTOR_DRIVER_GPIO_FIRST                   (8u)
#define MOTOR_DRIVER_GPIO_LAST                    (11u)

#if ((LINE_SENSOR_LEFT_GPIO >= MOTOR_DRIVER_GPIO_FIRST) && \
     (LINE_SENSOR_LEFT_GPIO <= MOTOR_DRIVER_GPIO_LAST)) || \
    ((LINE_SENSOR_RIGHT_GPIO >= MOTOR_DRIVER_GPIO_FIRST) && \
     (LINE_SENSOR_RIGHT_GPIO <= MOTOR_DRIVER_GPIO_LAST))
#define LINE_MOTOR_PIN_CONFLICT (1u)
#warning "A line sensor GPIO overlaps the motor-driver GPIO range"
#else
#define LINE_MOTOR_PIN_CONFLICT (0u)
#endif

#if (LINE_SENSOR_LEFT_GPIO > 29u) || (LINE_SENSOR_RIGHT_GPIO > 29u) || \
    (BARCODE_SENSOR_GPIO > 29u) || (BATTERY_SENSE_GPIO > 29u)
#error "Configured RP2040 GPIO numbers must be in the range 0..29"
#endif
#if LINE_SENSOR_LEFT_GPIO == LINE_SENSOR_RIGHT_GPIO
#error "The left and right line sensors must use distinct GPIOs"
#endif
#if (BARCODE_SENSOR_GPIO == LINE_SENSOR_LEFT_GPIO) || \
    (BARCODE_SENSOR_GPIO == LINE_SENSOR_RIGHT_GPIO)
#error "The barcode GPIO must not duplicate a line-sensor GPIO"
#endif
#if LINE_SENSOR_LEFT_GPIO != (26u + LINE_SENSOR_LEFT_ADC_CHANNEL)
#error "The left line GPIO and ADC channel do not match"
#endif
#if LINE_SENSOR_RIGHT_GPIO != (26u + LINE_SENSOR_RIGHT_ADC_CHANNEL)
#error "The right line GPIO and ADC channel do not match"
#endif
#if BATTERY_SENSE_GPIO != (26u + BATTERY_SENSE_ADC_CHANNEL)
#error "The battery-sense GPIO and ADC channel do not match"
#endif

#endif
