#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void (*handler_t)(void);

struct i2c_read_request_t {
    uint8_t addr;
    uint8_t reg;
    uint8_t *value;
    handler_t on_success;
    handler_t on_error;
};

struct i2c_write_request_t {
    uint8_t addr;
    uint8_t reg;
    uint8_t value;
    handler_t on_success;
    handler_t on_error;
};

void init_i2c(void);
int i2c_write(struct i2c_write_request_t request);
int i2c_read(struct i2c_read_request_t request);
