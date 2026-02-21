#pragma once
#include <stdint.h>

enum {
  LOG_FLAG_BMP_OK       = (1u << 0),
  LOG_FLAG_IMU_OK       = (1u << 1),
  LOG_FLAG_IMU_CAL_DONE = (1u << 2),
  LOG_FLAG_BARO_BASE_OK = (1u << 3),
  LOG_FLAG_USB_VBUS     = (1u << 4),
  LOG_FLAG_CHG_STAT     = (1u << 5),
  LOG_FLAG_IMU_INT1     = (1u << 6),
  LOG_FLAG_IMU_INT2     = (1u << 7),
  LOG_FLAG_FLASH_OK     = (1u << 8),
  LOG_FLAG_FLASH_FALLBK = (1u << 9),
  LOG_FLAG_BMP_STALE    = (1u << 10),
  LOG_FLAG_IMU_STALE    = (1u << 11),
  LOG_FLAG_LOG_DROPPED  = (1u << 12),
};

typedef struct __attribute__((packed)) {
  uint32_t ms;
  uint32_t seq;

  int32_t pressure_pa;
  int32_t altitude_cm;

  int16_t temp_centi;

  int16_t ax_mg;
  int16_t ay_mg;
  int16_t az_mg;

  int16_t gx_dps_x10;
  int16_t gy_dps_x10;
  int16_t gz_dps_x10;

  uint16_t flags;
  uint16_t reserved;
  uint16_t crc;
} log_rec_t;

int log_init(void);
void log_append(const log_rec_t* r);
void log_flush(void);
void log_dump_uart(void);
int log_is_flash_enabled(void);
uint32_t log_dropped_records(void);
