#pragma once
#include <stdint.h>

void gpio_init_all(void);

void cs_flash(int level);
void cs_imu(int level);
void cs_baro(int level);

void buzzer_set(int on);

int usb_vbus_sense_read(void);
int chg_stat_read(void);
int imu_int1_read(void);
int imu_int2_read(void);
