#pragma once
#include <stdint.h>
int icm42688_init(void);
int icm42688_read6(int16_t acc[3], int16_t gyr[3]);