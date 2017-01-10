#pragma once

#include <stddef.h>
#include <stdint.h>

void init_i2c(void);
int i2c_write(uint8_t addr, uint8_t reg, uint8_t value);
int i2c_read(uint8_t addr, uint8_t reg, uint8_t *value);
