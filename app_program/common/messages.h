#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

/* Shared message structs and mailbox/queue IDs.
 * Every buddy's task depends on this file -- route changes through
 * a pull request so nobody's build breaks silently. */

#include "robot_config.h"

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
