#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <tk/tkernel.h>

/* imu_task (Buddy 4): reads the GY-511 accelerometer over I2C0
 * (GP16 = SDA, GP17 = SCL) every 10 ms, detects humps, and raises
 * IMU_FAULT / STOP_IMU on abnormal tilt or lost data. */

/* imu_flg bits.  nav_ctrl_task sets CAL_REQ (nav_bind_t.cal_req_bit);
 * imu_task answers with NAV_FLG_CAL_OK_IMU or NAV_FLG_CAL_FAIL on
 * nav_flg. [PROPOSED: agree the value with Buddies 3 and 5] */
#define IMU_FLG_CAL_REQ	0x0001U

/* hump_state word (Tribunal Version B/C): everything imu_task reports, in
 * one aligned 32-bit word.  One writer (imu_task, every 10 ms), readers
 * on core 1, no mutex: a whole-word load is atomic on the M0+.  Masks
 * and shifts, not bit-fields.  [Moves to common/messages.h with the
 * team PR.]
 *
 *   bits  0-1   hump state      0 none, 1 climbing, 2 descending
 *   bits  2-4   motion class    0 still, 1 turning, 2 climbing,
 *                               3 descending, 4 impact, 5 accel, 6 moving
 *   bits  5-12  height now      mm on the hump in progress, 0-255
 *   bits 13-16  hump count      this mission, 0-15
 *   bits 17-24  highest hump    mm this mission, 0-255 (FR-12)
 *   bit  25     IMU fault       1 from the fault until a calibration passes
 *   bits 26-31  tilt            whole degrees, 0-63
 * Values past a field's range are clamped.  Tilt and turn rate for
 * telemetry (FR-29), with "not available" codes, are in imu_ts.h. */
#define IMU_W_HUMP_SHIFT	0U
#define IMU_W_HUMP_BITS		2U
#define IMU_W_CLASS_SHIFT	2U
#define IMU_W_CLASS_BITS	3U
#define IMU_W_HEIGHT_SHIFT	5U
#define IMU_W_HEIGHT_BITS	8U
#define IMU_W_COUNT_SHIFT	13U
#define IMU_W_COUNT_BITS	4U
#define IMU_W_MAX_SHIFT		17U
#define IMU_W_MAX_BITS		8U
#define IMU_W_FAULT_SHIFT	25U
#define IMU_W_FAULT_BITS	1U
#define IMU_W_TILT_SHIFT	26U
#define IMU_W_TILT_BITS		6U

#define IMU_W_MASK(bits)	((1UL << (bits)) - 1UL)

/* One field into place, clamped to its width. */
static inline UW imu_w_field(UW v, UW shift, UW bits)
{
	return ((v > IMU_W_MASK(bits)) ? IMU_W_MASK(bits) : v) << shift;
}

static inline UW imu_w_pack(UW hump, UW cls, UW height_mm, UW count,
			    UW max_mm, UW fault, UW tilt_deg)
{
	return imu_w_field(hump, IMU_W_HUMP_SHIFT, IMU_W_HUMP_BITS)
	     | imu_w_field(cls, IMU_W_CLASS_SHIFT, IMU_W_CLASS_BITS)
	     | imu_w_field(height_mm, IMU_W_HEIGHT_SHIFT, IMU_W_HEIGHT_BITS)
	     | imu_w_field(count, IMU_W_COUNT_SHIFT, IMU_W_COUNT_BITS)
	     | imu_w_field(max_mm, IMU_W_MAX_SHIFT, IMU_W_MAX_BITS)
	     | imu_w_field(fault, IMU_W_FAULT_SHIFT, IMU_W_FAULT_BITS)
	     | imu_w_field(tilt_deg, IMU_W_TILT_SHIFT, IMU_W_TILT_BITS);
}

/* One field out of a word read with imu_word_read(). */
#define IMU_W_GET(word, name) \
	(((word) >> IMU_W_##name##_SHIFT) & IMU_W_MASK(IMU_W_##name##_BITS))

/* The current word (one aligned load; core 1 readers). */
EXPORT UW imu_word_read(void);

/* Create imu_flg.  Call in usermain() before nav_bind(), so its ID can
 * go in nav_bind_t.imu_flgid.  Returns the ID, or an error (< E_OK). */
EXPORT ID imu_task_init(void);

EXPORT void imu_task(INT stacd, void *exinf);

#endif /* IMU_TASK_H */
