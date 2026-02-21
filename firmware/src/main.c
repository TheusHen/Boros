#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bmp390.h"
#include "flash_w25q.h"
#include "gpio.h"
#include "icm42688.h"
#include "log.h"
#include "spi.h"
#include "systick.h"
#include "uart.h"

#define CPU_HZ 16000000u

#define SAMPLE_PERIOD_MS   10u
#define IMU_CAL_SAMPLES    256u
#define BARO_BASE_SAMPLES  128u
#define SENSOR_RETRY_MS    500u
#define LOG_FLUSH_MS       250u
#define HEALTH_REPORT_MS  1000u

#define BMP_MIN_VALID_PA     20000
#define BMP_MAX_VALID_PA    120000
#define TEMP_MIN_VALID_CENTI -5000
#define TEMP_MAX_VALID_CENTI 10000

/* ICM-42688 configured to +/-16g and +/-2000dps. */
#define ICM_ACC_LSB_PER_G        2048
#define ICM_GYR_DPS_X10_DIVISOR  164 /* raw * 100 / 164 = dps*10 */

typedef struct {
  int64_t sum_ax, sum_ay, sum_az;
  int64_t sum_gx, sum_gy, sum_gz;
  uint16_t count;
  int16_t ax_bias, ay_bias, az_bias;
  int16_t gx_bias, gy_bias, gz_bias;
  uint8_t done;
} imu_cal_t;

static void beep(int n) {
  for (int i = 0; i < n; i++) {
    buzzer_set(1);
    for (volatile int d = 0; d < 70000; d++) { }
    buzzer_set(0);
    for (volatile int d = 0; d < 70000; d++) { }
  }
}

static int16_t sat_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static uint32_t isqrt_u64(uint64_t x) {
  uint64_t res = 0;
  uint64_t bit = (uint64_t)1 << 62;
  while (bit > x) bit >>= 2;

  while (bit != 0) {
    if (x >= (res + bit)) {
      x -= (res + bit);
      res = (res >> 1) + bit;
    } else {
      res >>= 1;
    }
    bit >>= 2;
  }

  return (uint32_t)res;
}

static int bmp_sample_is_valid(const bmp390_sample_t* s) {
  if (!s) return 0;
  if ((s->press_pa < BMP_MIN_VALID_PA) || (s->press_pa > BMP_MAX_VALID_PA)) return 0;
  if ((s->temp_centi < TEMP_MIN_VALID_CENTI) || (s->temp_centi > TEMP_MAX_VALID_CENTI)) return 0;
  return 1;
}

static int32_t pressure_to_altitude_cm(int32_t pressure_pa, int32_t base_pa) {
  if ((pressure_pa <= 0) || (base_pa <= 0)) return 0;

  float ratio = (float)pressure_pa / (float)base_pa;
  if (ratio < 0.05f) ratio = 0.05f;
  if (ratio > 1.2f) ratio = 1.2f;

  /* ISA troposphere approximation. */
  float alt_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
  if (alt_m < 0.0f) alt_m = 0.0f;
  if (alt_m > 100000.0f) alt_m = 100000.0f;

  return (int32_t)(alt_m * 100.0f + 0.5f);
}

static void imu_cal_feed(imu_cal_t* c, const int16_t acc[3], const int16_t gyr[3]) {
  if (c->done) return;

  c->sum_ax += acc[0];
  c->sum_ay += acc[1];
  c->sum_az += acc[2];
  c->sum_gx += gyr[0];
  c->sum_gy += gyr[1];
  c->sum_gz += gyr[2];
  c->count++;

  if (c->count < IMU_CAL_SAMPLES) return;

  int32_t ax_m = (int32_t)(c->sum_ax / c->count);
  int32_t ay_m = (int32_t)(c->sum_ay / c->count);
  int32_t az_m = (int32_t)(c->sum_az / c->count);

  c->gx_bias = sat_i16((int32_t)(c->sum_gx / c->count));
  c->gy_bias = sat_i16((int32_t)(c->sum_gy / c->count));
  c->gz_bias = sat_i16((int32_t)(c->sum_gz / c->count));

  /* Remove static accel bias while preserving ~1g along measured direction. */
  uint64_t mag2 = (uint64_t)((int64_t)ax_m * ax_m + (int64_t)ay_m * ay_m + (int64_t)az_m * az_m);
  uint32_t mag = isqrt_u64(mag2);
  if (mag > 0) {
    c->ax_bias = sat_i16(ax_m - (int32_t)(((int64_t)ax_m * ICM_ACC_LSB_PER_G) / mag));
    c->ay_bias = sat_i16(ay_m - (int32_t)(((int64_t)ay_m * ICM_ACC_LSB_PER_G) / mag));
    c->az_bias = sat_i16(az_m - (int32_t)(((int64_t)az_m * ICM_ACC_LSB_PER_G) / mag));
  } else {
    c->ax_bias = 0;
    c->ay_bias = 0;
    c->az_bias = 0;
  }

  c->done = 1;
}

int main(void) {
  gpio_init_all();
  uart2_init(CPU_HZ, 115200);
  spi1_init();
  systick_init_1ms(CPU_HZ);

  uart2_puts("BOOT\r\n");
  beep(2);

  int log_ok = (log_init() == 0);
  uint32_t flash_id = 0;
  int flash_id_ok = log_ok && (w25q_read_jedec_id(&flash_id) == 0);
  if (flash_id_ok) {
    char id_line[40];
    int id_n = snprintf(id_line, sizeof(id_line), "FLASH ID 0x%06lX\r\n", (unsigned long)flash_id);
    if (id_n > 0) {
      uart2_puts(id_line);
    }
  } else {
    uart2_puts("FLASH ID FAIL\r\n");
  }
  uart2_puts(log_ok ? "LOG FLASH\r\n" : "LOG RAM FALLBACK\r\n");

  int bmp_ready = (bmp390_init() == 0);
  int imu_ready = (icm42688_init() == 0);
  uart2_puts(bmp_ready ? "BMP OK\r\n" : "BMP FAIL\r\n");
  uart2_puts(imu_ready ? "IMU OK\r\n" : "IMU FAIL\r\n");
  uart2_puts("SYS READY\r\n");

  imu_cal_t imu_cal = {0};
  int imu_cal_announced = 0;

  int64_t baro_sum_pa = 0;
  uint16_t baro_count = 0;
  int32_t baro_base_pa = 101325;
  int baro_base_ready = 0;

  bmp390_sample_t bmp_last = {0};
  bmp_last.press_pa = baro_base_pa;
  bmp_last.temp_centi = 2500;
  int bmp_have_last = 0;
  uint16_t bmp_stale_count = 0;

  int16_t acc_last[3] = {0, 0, ICM_ACC_LSB_PER_G};
  int16_t gyr_last[3] = {0, 0, 0};
  int imu_have_last = 0;
  uint16_t imu_stale_count = 0;

  uint32_t last_sample = 0;
  uint32_t last_retry = 0;
  uint32_t last_flush = 0;
  uint32_t last_health = 0;

  while (1) {
    int c = uart2_getc_nonblock();
    if ((c == 'D') || (c == 'd')) {
      beep(1);
      log_dump_uart();
      beep(3);
    } else if ((c == 'F') || (c == 'f')) {
      log_flush();
      uart2_puts("LOG FLUSH\r\n");
    }

    uint32_t now = millis();
    if ((now - last_sample) < SAMPLE_PERIOD_MS) {
      continue;
    }
    last_sample = now;

    if (((!bmp_ready) || (!imu_ready)) && ((now - last_retry) >= SENSOR_RETRY_MS)) {
      last_retry = now;
      if (!bmp_ready) bmp_ready = (bmp390_init() == 0);
      if (!imu_ready) imu_ready = (icm42688_init() == 0);
    }

    bmp390_sample_t bmp = bmp_last;
    int bmp_ok = 0;
    if (bmp_ready) {
      if ((bmp390_read_sample(&bmp) == 0) && bmp_sample_is_valid(&bmp)) {
        bmp_ok = 1;
        bmp_last = bmp;
        bmp_have_last = 1;
        bmp_stale_count = 0;
      } else {
        bmp_ready = 0;
      }
    }
    if (!bmp_ok) {
      if (bmp_have_last) {
        bmp = bmp_last;
      } else {
        bmp.press_pa = baro_base_pa;
        bmp.temp_centi = 0;
      }
      if (bmp_stale_count < 0xFFFFu) bmp_stale_count++;
    }

    int16_t acc_raw[3] = {0, 0, ICM_ACC_LSB_PER_G};
    int16_t gyr_raw[3] = {0, 0, 0};
    int imu_ok = 0;
    if (imu_ready) {
      if (icm42688_read6(acc_raw, gyr_raw) == 0) {
        imu_ok = 1;
        for (int i = 0; i < 3; ++i) {
          acc_last[i] = acc_raw[i];
          gyr_last[i] = gyr_raw[i];
        }
        imu_have_last = 1;
        imu_stale_count = 0;
      } else {
        imu_ready = 0;
      }
    }
    if (!imu_ok) {
      if (imu_have_last) {
        for (int i = 0; i < 3; ++i) {
          acc_raw[i] = acc_last[i];
          gyr_raw[i] = gyr_last[i];
        }
      }
      if (imu_stale_count < 0xFFFFu) imu_stale_count++;
    }

    if (imu_ok) {
      imu_cal_feed(&imu_cal, acc_raw, gyr_raw);
      if (imu_cal.done && !imu_cal_announced) {
        imu_cal_announced = 1;
        uart2_puts("IMU CAL OK\r\n");
      }
    }

    if (bmp_ok && !baro_base_ready) {
      baro_sum_pa += bmp.press_pa;
      baro_count++;
      if (baro_count >= BARO_BASE_SAMPLES) {
        baro_base_pa = (int32_t)(baro_sum_pa / baro_count);
        if (baro_base_pa < BMP_MIN_VALID_PA || baro_base_pa > BMP_MAX_VALID_PA) {
          baro_base_pa = 101325;
        }
        baro_base_ready = 1;
        uart2_puts("BARO BASE OK\r\n");
      }
    }

    int32_t ax_corr = acc_raw[0] - imu_cal.ax_bias;
    int32_t ay_corr = acc_raw[1] - imu_cal.ay_bias;
    int32_t az_corr = acc_raw[2] - imu_cal.az_bias;

    int32_t gx_corr = gyr_raw[0] - imu_cal.gx_bias;
    int32_t gy_corr = gyr_raw[1] - imu_cal.gy_bias;
    int32_t gz_corr = gyr_raw[2] - imu_cal.gz_bias;

    int16_t ax_mg = sat_i16((ax_corr * 1000) / ICM_ACC_LSB_PER_G);
    int16_t ay_mg = sat_i16((ay_corr * 1000) / ICM_ACC_LSB_PER_G);
    int16_t az_mg = sat_i16((az_corr * 1000) / ICM_ACC_LSB_PER_G);

    int16_t gx_dps_x10 = sat_i16((gx_corr * 100) / ICM_GYR_DPS_X10_DIVISOR);
    int16_t gy_dps_x10 = sat_i16((gy_corr * 100) / ICM_GYR_DPS_X10_DIVISOR);
    int16_t gz_dps_x10 = sat_i16((gz_corr * 100) / ICM_GYR_DPS_X10_DIVISOR);

    int32_t pressure_pa = bmp.press_pa;
    int16_t temp_centi = bmp.temp_centi;
    int32_t altitude_cm = 0;
    if (baro_base_ready) {
      altitude_cm = pressure_to_altitude_cm(pressure_pa, baro_base_pa);
    }

    uint16_t flags = 0;
    if (bmp_ok) flags |= LOG_FLAG_BMP_OK;
    if (imu_ok) flags |= LOG_FLAG_IMU_OK;
    if (!bmp_ok) flags |= LOG_FLAG_BMP_STALE;
    if (!imu_ok) flags |= LOG_FLAG_IMU_STALE;
    if (imu_cal.done) flags |= LOG_FLAG_IMU_CAL_DONE;
    if (baro_base_ready) flags |= LOG_FLAG_BARO_BASE_OK;
    if (usb_vbus_sense_read()) flags |= LOG_FLAG_USB_VBUS;
    if (chg_stat_read()) flags |= LOG_FLAG_CHG_STAT;
    if (imu_int1_read()) flags |= LOG_FLAG_IMU_INT1;
    if (imu_int2_read()) flags |= LOG_FLAG_IMU_INT2;
    if (log_is_flash_enabled()) {
      flags |= LOG_FLAG_FLASH_OK;
    } else {
      flags |= LOG_FLAG_FLASH_FALLBK;
    }
    if (log_dropped_records() != 0u) {
      flags |= LOG_FLAG_LOG_DROPPED;
    }

    log_rec_t rec = {0};
    rec.ms = now;
    rec.pressure_pa = pressure_pa;
    rec.altitude_cm = altitude_cm;
    rec.temp_centi = temp_centi;
    rec.ax_mg = ax_mg;
    rec.ay_mg = ay_mg;
    rec.az_mg = az_mg;
    rec.gx_dps_x10 = gx_dps_x10;
    rec.gy_dps_x10 = gy_dps_x10;
    rec.gz_dps_x10 = gz_dps_x10;
    rec.flags = flags;

    log_append(&rec);

    if ((now - last_flush) >= LOG_FLUSH_MS) {
      last_flush = now;
      log_flush();
    }

    if ((now - last_health) >= HEALTH_REPORT_MS) {
      last_health = now;
      char line[180];
      int n = snprintf(line, sizeof(line),
                       "STAT t=%lu bmp=%d imu=%d bStale=%u iStale=%u flash=%d drop=%lu\r\n",
                       (unsigned long)now,
                       bmp_ready ? 1 : 0,
                       imu_ready ? 1 : 0,
                       (unsigned)bmp_stale_count,
                       (unsigned)imu_stale_count,
                       log_is_flash_enabled(),
                       (unsigned long)log_dropped_records());
      if (n > 0) {
        uart2_puts(line);
      }
    }
  }
}
