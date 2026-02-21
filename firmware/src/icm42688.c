#include "icm42688.h"
#include "gpio.h"
#include "spi.h"

#define ICM_REG_DEVICE_CONFIG      0x11
#define ICM_REG_ACCEL_DATA_X1      0x1F
#define ICM_REG_PWR_MGMT0          0x4E
#define ICM_REG_GYRO_CONFIG0       0x4F
#define ICM_REG_ACCEL_CONFIG0      0x50
#define ICM_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM_REG_WHOAMI             0x75

#define ICM42688_WHOAMI 0x47

static uint8_t icm_r(uint8_t reg) {
  uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0xFF};
  uint8_t rx[2] = {0};
  cs_imu(0);
  spi1_txrx(tx, rx, 2);
  cs_imu(1);
  return rx[1];
}

static void icm_w(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
  cs_imu(0);
  spi1_txrx(tx, 0, 2);
  cs_imu(1);
}

int icm42688_init(void) {
  if (icm_r(ICM_REG_WHOAMI) != ICM42688_WHOAMI) return -1;

  /* Soft reset */
  icm_w(ICM_REG_DEVICE_CONFIG, 0x01);
  for (volatile int i = 0; i < 120000; i++) { }
  if (icm_r(ICM_REG_WHOAMI) != ICM42688_WHOAMI) return -2;

  /* Configure full-scale and ODR before enabling sensors. */
  icm_w(ICM_REG_GYRO_CONFIG0, 0x06);       /* +/-2000 dps, 1kHz */
  icm_w(ICM_REG_ACCEL_CONFIG0, 0x06);      /* +/-16g, 1kHz */
  icm_w(ICM_REG_GYRO_ACCEL_CONFIG0, 0x00); /* default low-latency filters */

  /* Power up accel + gyro in low noise mode */
  icm_w(ICM_REG_PWR_MGMT0, 0x0F);
  for (volatile int i = 0; i < 50000; i++) { }
  return 0;
}

int icm42688_read6(int16_t acc[3], int16_t gyr[3]) {
  uint8_t tx = (uint8_t)(ICM_REG_ACCEL_DATA_X1 | 0x80);
  uint8_t rx[12] = {0};

  cs_imu(0);
  spi1_txrx(&tx, 0, 1);
  spi1_txrx(0, rx, 12);
  cs_imu(1);

  acc[0] = (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);
  acc[1] = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  acc[2] = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
  gyr[0] = (int16_t)(((uint16_t)rx[6] << 8) | rx[7]);
  gyr[1] = (int16_t)(((uint16_t)rx[8] << 8) | rx[9]);
  gyr[2] = (int16_t)(((uint16_t)rx[10] << 8) | rx[11]);
  return 0;
}
