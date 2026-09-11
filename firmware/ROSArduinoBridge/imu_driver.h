/*********************************************************************
 * MPU9250 gyro + accelerometer driver (buddy robot).
 *
 * Raw I2C register access through Wire — no extra Arduino library to install,
 * and it works with the MPU6500 boards that are widely sold as "MPU9250".
 *
 * The magnetometer is deliberately unused. Indoors the motors, L298 and
 * chassis metal make a compass heading worse than no heading at all, so yaw
 * comes from the gyro and is fused with wheel odometry on the host.
 *
 * Readings are reported in the *chip* frame in SI units. Mount orientation
 * lives in the URDF imu_link rpy (buddy description/imu.xacro) so a sign error
 * has exactly one place to be fixed.
 *********************************************************************/
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

/* I2C bus — free on buddy: motors use 18/19/32/33, encoders 26/27/16/17. */
#define IMU_I2C_SDA        21
#define IMU_I2C_SCL        22
#define IMU_I2C_CLOCK      400000L

/* AD0 low = 0x68 (module default), AD0 high = 0x69. Both are probed. */
#define IMU_ADDR_PRIMARY   0x68
#define IMU_ADDR_SECONDARY 0x69

/* Internal sample rate. Host polls far slower, so samples between polls are
 * averaged — that is a mean angular rate over the interval, which is what the
 * EKF wants from a rate measurement, and it avoids aliasing motor vibration. */
#define IMU_SAMPLE_RATE    100
#define IMU_SAMPLE_INTERVAL (1000 / IMU_SAMPLE_RATE)

/* Samples averaged for the startup gyro bias estimate (robot must be still). */
#define IMU_CALIBRATION_SAMPLES 200

/* Serial transport uses integers in milli-units: gyro mrad/s, accel mm/s^2.
 * Integers keep the reply short and parsing free of float/locale surprises. */
#define IMU_SERIAL_SCALE   1000.0f

bool imuInit();
bool imuAvailable();

/* Sample the sensor. Call at IMU_SAMPLE_INTERVAL from loop(). */
void imuUpdate();

/* Average of the samples taken since the previous call, chip frame, SI units.
 * Repeats the last average when no new sample arrived. */
void imuReadAveraged(float gyro_rad_s[3], float accel_m_s2[3]);

/* Re-estimate gyro bias. Blocks for roughly half a second and must run with
 * the robot stationary. */
bool imuCalibrateGyroBias();

/* "gx gy gz ax ay az ok" in milli-units — the reply for "g" and the tail of "f". */
void imuPrintReading();

#endif  // IMU_DRIVER_H
