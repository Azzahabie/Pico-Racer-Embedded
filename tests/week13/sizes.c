/*
 *  sizes.c
 *  Never part of the firmware or the host tests.  tests/week13/measure.sh
 *  compiles it with the car's ARM compiler (not host gcc: enums and padding
 *  differ) and prints each array's size with nm -S.  Today's entries are
 *  plain constant expressions (calculated, not measured); the ARM compiler
 *  matters once structs or enums are added here.
 */

#include <stdint.h>
#include "messages.h"

/* A message buffer stores a 4-byte header plus the message rounded up to
   4 bytes (kernel/tkernel/messagebuf.h, ROUNDSZ and HEADERSZ). */
#define MBF_SLOT(bytes) (4u + ((((bytes) + 3u) / 4u) * 4u))

/* Motion's event on Buddy 1's tele_mbf, schema v3.  Not switched:
   it is the same in both builds. */
uint8_t msg_tele_motion_payload[COMM_W_INTAKE_SIZE_MOTION];
uint8_t msg_tele_motion_in_mbf[MBF_SLOT(COMM_W_INTAKE_SIZE_MOTION)];
uint8_t msg_tele_motion_on_wire[COMM_W_E_SIZE_MOTION];

/* end of file */
