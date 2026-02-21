#pragma once
#include <stdint.h>

typedef struct {
  int32_t press_raw;
  int32_t temp_raw;
  int32_t press_pa;
  int16_t temp_centi;
} bmp390_sample_t;

int bmp390_init(void);
int bmp390_read_raw(int32_t* press_raw, int32_t* temp_raw);
int bmp390_read_sample(bmp390_sample_t* s);
