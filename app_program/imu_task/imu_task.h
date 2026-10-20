#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <tk/tkernel.h>

/* GY-511 (LSM303DLHC or LSM303AGR) bring-up task: reads the accelerometer
 * and magnetometer over I2C0 (GP8 = SDA, GP9 = SCL) and prints them. */
EXPORT void imu_task(INT stacd, void *exinf);

#endif /* IMU_TASK_H */
