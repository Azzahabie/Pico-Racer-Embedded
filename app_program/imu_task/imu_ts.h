#ifndef IMU_TS_H
#define IMU_TS_H

/* Tilt-state word: tilt and turn rate for the 1 s status line (FR-29),
 * from imu_task (Buddy 4) to comms_task (Buddy 1, core 0).  One writer
 * (imu_task, every 10 ms sample), built in a local and stored once; one
 * load to read, from either core, no mutex.  Masks and shifts, not
 * bit-fields.
 *
 *   bits  0-5   tilt   whole degrees from level, 0-62; 63 = not available
 *   bits  6-17  turn   deg/s, counter-clockwise (left) positive,
 *                      -2047..+2047 stored + 2048; 0 = not available
 *   bits 18-31  zero
 *
 * Both fields are stored exactly as record S carries them (S.tlt, S.trn,
 * MQTT_SCHEMA.md section 4), so comms_task copies them without converting.
 * Tilt is the same value as the hump_state word's (imu_task.h); that word
 * is full and has no "not available" code, hence this one.
 *
 * Plain C, no kernel calls, so it builds on a PC too. */

#include <stdbool.h>
#include <stdint.h>

#define IMU_TS_TILT_SHIFT	0u
#define IMU_TS_TILT_BITS	6u
#define IMU_TS_TILT_MAX		62u		/* largest real tilt           */
#define IMU_TS_TILT_NA		63u

#define IMU_TS_TURN_SHIFT	6u
#define IMU_TS_TURN_BITS	12u
#define IMU_TS_TURN_OFFSET	2048
#define IMU_TS_TURN_MAX		2047		/* -2048 is kept for NA        */
#define IMU_TS_TURN_NA		0u		/* stored value                */

#define IMU_TS_MASK(bits)	((1u << (bits)) - 1u)

#define IMU_TS_WORD_NA							\
	((IMU_TS_TILT_NA << IMU_TS_TILT_SHIFT)				\
	 | (IMU_TS_TURN_NA << IMU_TS_TURN_SHIFT))

_Static_assert((IMU_TS_TURN_SHIFT + IMU_TS_TURN_BITS) <= 32u,
	       "the fields fit one 32-bit word");
_Static_assert((IMU_TS_TURN_MAX + IMU_TS_TURN_OFFSET)
		   == (int32_t)IMU_TS_MASK(IMU_TS_TURN_BITS),
	       "turn rate offset matches its width");

/* The latest word, in one load.  Never blocks; any task, either core.
 * IMU_TS_WORD_NA until imu_task's first sample (or if it never runs). */
uint32_t imu_ts_read(void);

/* Stored tilt, 0..63 (63 = not available). */
static inline uint32_t imu_ts_tilt_raw(uint32_t w)
{
	return (w >> IMU_TS_TILT_SHIFT) & IMU_TS_MASK(IMU_TS_TILT_BITS);
}

/* Stored turn rate, 0..4095, offset kept (0 = not available). */
static inline uint32_t imu_ts_turn_raw(uint32_t w)
{
	return (w >> IMU_TS_TURN_SHIFT) & IMU_TS_MASK(IMU_TS_TURN_BITS);
}

/* Build the word (imu_task only).  tilt_deg is clamped to 0..62 and
 * turn_dps to -2047..+2047; a field whose _ok is false gets its NA code. */
static inline uint32_t imu_ts_pack(uint32_t tilt_deg, int32_t turn_dps,
				   bool tilt_ok, bool turn_ok)
{
	uint32_t tilt = IMU_TS_TILT_NA;
	uint32_t turn = IMU_TS_TURN_NA;

	if(tilt_ok) {
		tilt = (tilt_deg > IMU_TS_TILT_MAX) ? IMU_TS_TILT_MAX : tilt_deg;
	}
	if(turn_ok) {
		if(turn_dps > IMU_TS_TURN_MAX) turn_dps = IMU_TS_TURN_MAX;
		if(turn_dps < -IMU_TS_TURN_MAX) turn_dps = -IMU_TS_TURN_MAX;
		turn = (uint32_t)(turn_dps + IMU_TS_TURN_OFFSET);
	}
	return (tilt << IMU_TS_TILT_SHIFT) | (turn << IMU_TS_TURN_SHIFT);
}

#endif /* IMU_TS_H */
