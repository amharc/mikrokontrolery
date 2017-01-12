#include "i2c.h"
#include <irq.h>
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16


/* Initialises I2C */
void init_i2c(void) {
    /* Enable the clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* Configure lines */
    GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
            GPIO_Low_Speed, GPIO_PuPd_NOPULL,
            GPIO_AF_I2C1);
    GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
            GPIO_Low_Speed, GPIO_PuPd_NOPULL,
            GPIO_AF_I2C1);


    /* Disable I2C */
    I2C1->CR1 = 0;

    /* Enable interrupts */
    NVIC_SetPriority(DMA1_Stream6_IRQn, I2C_IRQ_LEVEL);
    NVIC_SetPriority(DMA1_Stream5_IRQn, I2C_IRQ_LEVEL);

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    /* Configure I2C, requesting all interrupts */
    I2C1->CR2 = PCLK1_MHZ | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    I2C1->CCR = (PCLK1_MHZ * 1000 * 1000) / (I2C_SPEED_HZ << 1);
    I2C1->TRISE = PCLK1_MHZ + 1;

    /* Start I2C */
    I2C1->CR1 |= I2C_CR1_PE;
}

/* State of the I2C subsystem */
static struct {
    enum {
        /* No I2C operation in progress */
        MODE_NONE,

        /* A write operation is in progress */
        MODE_WRITE,
        
        /* A read operation is in progress */
        MODE_READ
    } mode;

    union {
        /* The read operation in progress */
        struct i2c_read_request_t read;

        /* The write operation in progress */
        struct i2c_write_request_t write;
    };

    /* Line number the respective corotuine should start with */
    int state;
} state;

/* Called when something has failed, aborts the operation. */
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

/* The event interrupt handler */
void I2C1_EV_IRQHandler(void) {
    uint8_t sr1 = I2C1->SR1;
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

/* The error interrupt handler */
void I2C1_ER_IRQHanlder(void) {
    I2C1->SR1;
    I2C1->SR2;

    fail_request();
}

/* Below, write_receive_bit and read_receive_bit are implemented as (very) simple
 * coroutines that use C preprocessor liberally, namely: a custom domain specific
 * language for yielding and waiting for bits.
 *
 * A coroutine remember its state in the global state variable (i.e. its line number,
 * as a rudimentary subsitute for program counter), and resume execution when called
 * again.
 */

/* Starts a coroutine. */
#define START switch(state.state) { case 0:

/* Waits for a condition to be satisfied. */
#define WAIT_FOR(bit) if (!(cond)) return;

/* Saves the current state (i.e. program counter = line number) */
#define YIELD state.state = __LINE__; return; case __LINE__:

/* Ends a corotuine */
#define FINISH state.state = 0; state.mode = MODE_NONE; }

/* Coroutine performing a single write operation */
static void write_receive_bit(uint8_t sr1) {
    START WAIT_FOR(sr1 & I2C_SR1_SB);
        I2C1->DR = state.write.addr << 1;
    YIELD WAIT_FOR(sr1 & I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->DR = state.write.reg;
    YIELD WAIT_FOR(sr1 & I2C_SR1_TXE);
        I2C1->DR = state.write.value;
    YIELD WAIT_FOR(sr1 & I2C_SR1_BTF);
        I2C1->CR1 |= I2C_CR1_STOP;
    FINISH
        state.write.on_success();
}

/* Coroutine performing a single read operation */
static void read_receive_bit(uint8_t sr1) {
    START WAIT_FOR(sr1 & I2C_SR1_SB);
        I2C1->DR = state.read.addr << 1;
    YIELD WAIT_FOR(sr1 & I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->DR = state.read.reg;
    YIELD WAIT_FOR(sr1 & I2C_SR1_BTF);
        I2C1->CR1 |= I2C_CR1_START;
    YIELD WAIT_FOR(sr1 & I2C_SR1_SB);
        I2C1->DR = (state.read.addr << 1) | 1U;
        I2C1->CR1 &= ~I2C_CR1_ACK;
    YIELD WAIT_FOR(sr1 & I2C_SR1_ADDR);
        I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
    YIELD WAIT_FOR(sr1 & I2C_SR1_RXNE);
        *state.read.value = I2C1->DR;
    FINISH
        state.read.on_success();
}

int i2c_write(struct i2c_write_request_t request) {
    irq_level_t irq_level = IRQprotect(I2C_IRQ_LEVEL);

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
    irq_level_t irq_level = IRQprotect(I2C_IRQ_LEVEL);

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
