#include "i2c.h"
#include <irq.h>
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

#define IRQ_LEVEL 1

void init_i2c(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
            GPIO_Low_Speed, GPIO_PuPd_NOPULL,
            GPIO_AF_I2C1);
    GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
            GPIO_Low_Speed, GPIO_PuPd_NOPULL,
            GPIO_AF_I2C1);

    I2C1->CR1 = 0;

    NVIC_SetPriority(DMA1_Stream6_IRQn, IRQ_LEVEL);
    NVIC_SetPriority(DMA1_Stream5_IRQn, IRQ_LEVEL);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C1->CR2 = PCLK1_MHZ | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    I2C1->CCR = (PCLK1_MHZ * 1000 * 1000) / (I2C_SPEED_HZ << 1);
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;
}

static struct {
    enum {
        MODE_NONE, MODE_WRITE, MODE_READ
    } mode;

    union {
        struct i2c_read_request_t read;
        struct i2c_write_request_t write;
    };

    int state;
} state;

static void fail_request(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    switch (state.mode) {
        case MODE_WRITE:
            output("Write request failed\r\n");
            state.write.on_error();
            break;
        case MODE_READ:
            output("Read request failed\r\n");
            state.read.on_error();
            break;
        default:
            break;
    }
    state.mode = MODE_NONE;
}

static void write_receive_bit(uint8_t sr1);
static void read_receive_bit(uint8_t sr1);

static void receive_bit(uint8_t sr1) {
    switch (state.mode) {
        case MODE_WRITE:
            write_receive_bit(sr1);
            break;
        case MODE_READ:
            read_receive_bit(sr1);
            break;
        default:
            break;
    }
}

void I2C1_EV_IRQHandler(void) {
    uint8_t sr1 = I2C1->SR1;
    receive_bit(sr1);
}

void I2C1_ER_IRQHanlder(void) {
    I2C1->SR1;
    I2C1->SR2;

    fail_request();
}

#define START switch(state.state) { case 0:

#define WAIT_FOR(bit) \
    if (!(sr1 & bit)) return;

#define YIELD state.state = __LINE__; return; case __LINE__:

#define FINISH state.state = 0; state.mode = MODE_NONE; }

static void write_receive_bit(uint8_t sr1) {
    START WAIT_FOR(I2C_SR1_SB);
        I2C1->DR = state.write.addr << 1;
    YIELD WAIT_FOR(I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->DR = state.write.reg;
    YIELD WAIT_FOR(I2C_SR1_TXE);
        I2C1->DR = state.write.value;
    YIELD WAIT_FOR(I2C_SR1_BTF);
        I2C1->CR1 |= I2C_CR1_STOP;
    FINISH
        state.write.on_success();
}

static void read_receive_bit(uint8_t sr1) {
    START WAIT_FOR(I2C_SR1_SB);
        I2C1->DR = state.read.addr << 1;
    YIELD WAIT_FOR(I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->DR = state.read.reg;
    YIELD WAIT_FOR(I2C_SR1_BTF);
        I2C1->CR1 |= I2C_CR1_START;
    YIELD WAIT_FOR(I2C_SR1_SB);
        I2C1->DR = (state.read.addr << 1) | 1U;
        I2C1->CR1 &= ~I2C_CR1_ACK;
    YIELD WAIT_FOR(I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
    YIELD WAIT_FOR(I2C_SR1_RXNE);
        *state.read.value = I2C1->DR;
    FINISH
        state.read.on_success();
}

int i2c_write(struct i2c_write_request_t request) {
    irq_level_t irq_level = IRQprotect(IRQ_LEVEL);

    if (state.mode != MODE_NONE) {
        output("write aborted\r\n");
        IRQunprotect(irq_level);
        return 1;
    }

    state.mode = MODE_WRITE;
    state.write = request;
    state.state = 0;
    I2C1->CR1 |= I2C_CR1_START;

    IRQunprotect(irq_level);
    return 0;
}

int i2c_read(struct i2c_read_request_t request) {
    irq_level_t irq_level = IRQprotect(IRQ_LEVEL);

    if (state.mode != MODE_NONE) {
        output("read aborted\r\n");
        IRQunprotect(irq_level);
        return 1;
    }

    state.mode = MODE_READ;
    state.read = request;
    state.state = 0;
    I2C1->CR1 |= I2C_CR1_START;

    IRQunprotect(irq_level);
    return 0;
}
