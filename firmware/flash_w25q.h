#pragma once
#include <stdint.h>

void w25q_init(void);
uint32_t w25q_read_jedec_id(void);
void w25q_read(uint32_t addr, uint8_t* buf, uint32_t n);
void w25q_write(uint32_t addr, const uint8_t* buf, uint32_t n);
void w25q_erase_4k(uint32_t addr);
