#pragma once

#include <stddef.h>
#include <stdint.h>

#define I2C_ACCEL_REG_X 0x29
#define I2C_ACCEL_REG_Y 0x2B
#define I2C_ACCEL_REG_Z 0x2D

void init_i2c(void);
int i2c_accel_write(uint8_t reg, uint8_t value);
int i2c_accel_read(uint8_t reg, uint8_t *value);
