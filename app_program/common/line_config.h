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

/* ---- Barcode capture in ir_sense_task, in 1 ms sampling ticks ----
   Q: a read ends after this many white ticks, or BARCODE_Q_WIDTH_FACTOR x
   the narrowest width seen, whichever is longer (the existing 50 ms /
   4x-narrowest rule).  W_max: widths per read (32 letters).  W_long: any
   single bar or space longer than this aborts the read.  PROVISIONAL until
   measured on the car. */
#define BARCODE_Q_TICKS                (BARCODE_SCAN_END_GAP_US / 1000u)
#define BARCODE_Q_WIDTH_FACTOR                    (4u)
#define BARCODE_W_MAX                             BARCODE_MAX_ELEMENTS
#define BARCODE_W_LONG_TICKS                      (2000u)
#define BARCODE_READ_MAX_TICKS         (BARCODE_SCAN_MAX_US / 1000u)

/* Compiled-in letter -> movement table, used until a valid map arrives
   (MQTT_SCHEMA.md 8.3).  No approved table exists yet (schema section 12
   item 9), so it is off: until a map is loaded every letter is rejected
   as unknown and no junction command is sent.  To use one, set _SET to 1
   and give 26 characters, 'A' first: F forward, L left, R right, U U-turn. */
#define BARCODE_DEFAULT_TABLE_SET                 (0)
#define BARCODE_DEFAULT_TABLE                     ""

/* Junctions seen while a barcode is being read, or this soon after, are
   the barcode's own bars under the line sensors and are ignored. */
#define BARCODE_JUNCTION_HOLDOFF_MS               (300u)

/* ---- Junction detection (ticks of 1 ms) ----
   J_n: both line sensors black this many samples running = one junction.
   Rearm: both must leave black this many samples running first. */
#define LINE_JUNCTION_SAMPLES                     (3u)
#define LINE_JUNCTION_REARM_SAMPLES               (20u)

/* ---- Line loss: no confirmed black for this long (ms) = lost ---- */
#define LINE_LOST_MS                              (1500u)

/* ---- Calibration (CAL_REQ from nav_ctrl_task), ADC counts ----
   IR_CAL_SAMPLES ticks are recorded per sensor with the car still.  If a
   sensor saw both black and white (span >= IR_CAL_MIN_CONTRAST), its
   thresholds are set at the measured midpoint +/- IR_CAL_HYST_PCT of the
   span.  If it saw one colour only (the start pose straddles the line, so
   white), the tested thresholds above are kept and white must sit at least
   IR_CAL_WHITE_MARGIN below the falling threshold.  A sensor reading at an
   ADC rail (<= IR_RAIL_LOW or >= IR_RAIL_HIGH) for most of the window, or
   for IR_RAIL_FAULT_SAMPLES running in use, is stuck.  PROVISIONAL. */
#define IR_CAL_SAMPLES                            (1000u)
#define IR_CAL_MIN_CONTRAST                       (400u)
#define IR_CAL_HYST_PCT                           (10u)
#define IR_CAL_WHITE_MARGIN                       (200u)
#define IR_RAIL_LOW                               (2u)
#define IR_RAIL_HIGH                              (4093u)
#define IR_RAIL_FAULT_SAMPLES                     (200u)

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
