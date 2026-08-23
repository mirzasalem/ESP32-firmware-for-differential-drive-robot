/*********************************************************************
 * MPU9250 gyro + accelerometer implementation — see imu_driver.h.
 *********************************************************************/

#ifdef USE_IMU

#include <Wire.h>
#include <math.h>

#include "imu_driver.h"

/* MPU9250 / MPU6500 register map (accel + gyro block only). */
#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_ACCEL_CONFIG2  0x1D
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_PWR_MGMT_2     0x6C
#define MPU_REG_WHO_AM_I       0x75

/* +/- 250 deg/s: the robot turns near 7 deg/s, so the narrowest range gives
 * the best resolution. +/- 2 g likewise for a wheeled indoor robot. */
#define MPU_GYRO_LSB_PER_DPS   131.0f
#define MPU_ACCEL_LSB_PER_G    16384.0f
#define MPU_GRAVITY            9.80665f

static uint8_t imu_addr = 0;
static bool imu_ready = false;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

static double gyro_sum[3] = {0.0, 0.0, 0.0};
static double accel_sum[3] = {0.0, 0.0, 0.0};
static uint32_t imu_sample_count = 0;

static float last_gyro[3] = {0.0f, 0.0f, 0.0f};
static float last_accel[3] = {0.0f, 0.0f, 0.0f};

/* Helpers are non-static on purpose: the Arduino build concatenates every .ino
 * into one translation unit and auto-generates prototypes, and a generated
 * prototype for a static function that USE_IMU has compiled out would warn. */
bool imuWriteReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegs(uint8_t reg, uint8_t * buf, uint8_t len)
{
  Wire.beginTransmission(imu_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)imu_addr, (int)len, (int)true) != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool imuProbe(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

/* One burst read of the accel/temp/gyro block, converted to SI. */
bool imuReadRaw(float gyro_rad_s[3], float accel_m_s2[3])
{
  uint8_t buf[14];
  if (!imuReadRegs(MPU_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    return false;
  }

  const int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  const int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  const int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  /* buf[6..7] is temperature — unused. */
  const int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  const int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  const int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

  const float dps_to_rad = (float)M_PI / 180.0f;
  gyro_rad_s[0] = ((float)gx / MPU_GYRO_LSB_PER_DPS) * dps_to_rad;
  gyro_rad_s[1] = ((float)gy / MPU_GYRO_LSB_PER_DPS) * dps_to_rad;
  gyro_rad_s[2] = ((float)gz / MPU_GYRO_LSB_PER_DPS) * dps_to_rad;

  accel_m_s2[0] = ((float)ax / MPU_ACCEL_LSB_PER_G) * MPU_GRAVITY;
  accel_m_s2[1] = ((float)ay / MPU_ACCEL_LSB_PER_G) * MPU_GRAVITY;
  accel_m_s2[2] = ((float)az / MPU_ACCEL_LSB_PER_G) * MPU_GRAVITY;
  return true;
}

bool imuInit()
{
  imu_ready = false;
  imu_addr = 0;

  Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL, IMU_I2C_CLOCK);

  if (imuProbe(IMU_ADDR_PRIMARY)) {
    imu_addr = IMU_ADDR_PRIMARY;
  } else if (imuProbe(IMU_ADDR_SECONDARY)) {
    imu_addr = IMU_ADDR_SECONDARY;
  } else {
    return false;
  }

  /* WHO_AM_I is only logged, not enforced: 0x71 MPU9250, 0x73 MPU9255,
   * 0x70 MPU6500, and clone boards report other values while speaking the
   * same accel/gyro register map. */
  uint8_t who = 0;
  imuReadRegs(MPU_REG_WHO_AM_I, &who, 1);

  if (!imuWriteReg(MPU_REG_PWR_MGMT_1, 0x80)) {  // device reset
    return false;
  }
  delay(100);
  imuWriteReg(MPU_REG_PWR_MGMT_1, 0x01);  // clock from gyro PLL
  delay(10);
  imuWriteReg(MPU_REG_PWR_MGMT_2, 0x00);  // all axes enabled
  imuWriteReg(MPU_REG_CONFIG, 0x03);      // gyro DLPF 41 Hz
  imuWriteReg(MPU_REG_SMPLRT_DIV, (uint8_t)(1000 / IMU_SAMPLE_RATE - 1));
  imuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);    // +/- 250 deg/s
  imuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);   // +/- 2 g
  imuWriteReg(MPU_REG_ACCEL_CONFIG2, 0x03);  // accel DLPF 41 Hz
  delay(20);

  float gyro[3];
  float accel[3];
  if (!imuReadRaw(gyro, accel)) {
    return false;
  }

  imu_ready = true;
  imuCalibrateGyroBias();
  return true;
}

bool imuAvailable()
{
  return imu_ready;
}

void imuUpdate()
{
  if (!imu_ready) {
    return;
  }

  float gyro[3];
  float accel[3];
  if (!imuReadRaw(gyro, accel)) {
    return;
  }

  for (int i = 0; i < 3; i++) {
    gyro_sum[i] += (double)(gyro[i] - gyro_bias[i]);
    accel_sum[i] += (double)accel[i];
  }
  imu_sample_count++;
}

void imuReadAveraged(float gyro_rad_s[3], float accel_m_s2[3])
{
  if (imu_sample_count > 0) {
    const double n = (double)imu_sample_count;
    for (int i = 0; i < 3; i++) {
      last_gyro[i] = (float)(gyro_sum[i] / n);
      last_accel[i] = (float)(accel_sum[i] / n);
      gyro_sum[i] = 0.0;
      accel_sum[i] = 0.0;
    }
    imu_sample_count = 0;
  }

  for (int i = 0; i < 3; i++) {
    gyro_rad_s[i] = last_gyro[i];
    accel_m_s2[i] = last_accel[i];
  }
}

bool imuCalibrateGyroBias()
{
  if (!imu_ready) {
    return false;
  }

  double sum[3] = {0.0, 0.0, 0.0};
  uint32_t taken = 0;

  for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
    float gyro[3];
    float accel[3];
    if (imuReadRaw(gyro, accel)) {
      sum[0] += gyro[0];
      sum[1] += gyro[1];
      sum[2] += gyro[2];
      taken++;
    }
    delay(IMU_SAMPLE_INTERVAL / 10 + 1);
  }

  if (taken == 0) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    gyro_bias[i] = (float)(sum[i] / (double)taken);
    gyro_sum[i] = 0.0;
    accel_sum[i] = 0.0;
  }
  imu_sample_count = 0;
  return true;
}

void imuPrintReading()
{
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float accel[3] = {0.0f, 0.0f, 0.0f};
  imuReadAveraged(gyro, accel);

  for (int i = 0; i < 3; i++) {
    Serial.print((long)lroundf(gyro[i] * IMU_SERIAL_SCALE));
    Serial.print(" ");
  }
  for (int i = 0; i < 3; i++) {
    Serial.print((long)lroundf(accel[i] * IMU_SERIAL_SCALE));
    Serial.print(" ");
  }
  Serial.print(imu_ready ? 1 : 0);
}

#endif  // USE_IMU
