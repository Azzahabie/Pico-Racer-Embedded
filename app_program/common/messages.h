#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

/* Shared message structs, mailbox/queue IDs, and pin assignments.
 * Every buddy's task depends on this file -- route changes through
 * a pull request so nobody's build breaks silently. */

#define LINE_LEFT_SENSOR_GPIO       6u
#define LINE_CENTRE_SENSOR_GPIO     8u
#define BARCODE_SENSOR_GPIO         26u
#ifndef BARCODE_SENSOR_AVAILABLE
#define BARCODE_SENSOR_AVAILABLE    1u
#endif

/* Set to the measured DO level when a KY-033 is over the black line. */
#ifndef KY033_DARK_LEVEL
#define KY033_DARK_LEVEL            1u
#endif

#if (BARCODE_SENSOR_AVAILABLE != 0u) && (BARCODE_SENSOR_AVAILABLE != 1u)
#error "BARCODE_SENSOR_AVAILABLE must be 0 or 1"
#endif

#ifndef B3_BARCODE_GPIO
#define B3_BARCODE_GPIO             BARCODE_SENSOR_GPIO
#endif
#ifndef B3_BARCODE_BAR_LEVEL
#define B3_BARCODE_BAR_LEVEL        KY033_DARK_LEVEL
#endif

#if (KY033_DARK_LEVEL != 0u) && (KY033_DARK_LEVEL != 1u)
#error "KY033_DARK_LEVEL must be 0 or 1"
#endif

#endif /* COMMON_MESSAGES_H */
