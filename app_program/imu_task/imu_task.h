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

/* Create imu_flg.  Call in usermain() before nav_bind(), so its ID can
 * go in nav_bind_t.imu_flgid.  Returns the ID, or an error (< E_OK). */
EXPORT ID imu_task_init(void);

EXPORT void imu_task(INT stacd, void *exinf);

#endif /* IMU_TASK_H */
