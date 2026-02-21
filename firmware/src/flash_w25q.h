#pragma once
#include <stdint.h>

int w25q_init(void);
int w25q_read_jedec_id(uint32_t* jedec_id);
int w25q_read(uint32_t addr, uint8_t* buf, uint32_t n);
int w25q_write(uint32_t addr, const uint8_t* buf, uint32_t n);
int w25q_erase_4k(uint32_t addr);
