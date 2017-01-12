#pragma once

#define I2C_IRQ_LEVEL 2

#include <stddef.h>
#include <stdint.h>

typedef void (*handler_t)(void);

/* Asynchronous read request. */
struct i2c_read_request_t {
    /* Source device's address */
    uint8_t addr;

    /* Source register */
    uint8_t reg;

    /* Where to save the value */
    uint8_t *value;

    /* Callback to be called on success */
    handler_t on_success;

    /* Callback to be called on error */
    handler_t on_error;
};

/* Asynchronous write request. */
struct i2c_write_request_t {
    /* Target device's address */
    uint8_t addr;

    /* Target register */
    uint8_t reg;

    /* Value to be saved */
    uint8_t value;

    /* Callback to be called on success */
    handler_t on_success;

    /* Callback to be called on error */
    handler_t on_error;
};

/* Initiliases I2C */
void init_i2c(void);

/* Schedules an I2C write. Returns 0 on success, 1 if an I2C operation is already in progress */
int i2c_write(struct i2c_write_request_t request);

/* Schedules an I2C read. Returns 0 on success, 1 if an I2C operation is already in progress */
int i2c_read(struct i2c_read_request_t request);
