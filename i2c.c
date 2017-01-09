#include "i2c.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"
#include "leds.h"
#include "timer.h"

#define LIS35DE_ADDR 0x1C
#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16
#define I2C_ACCEL_REG_CTRL1 0x20
#define I2C_ACCEL_REG_CTRL3 0x22
#define I2C_ACCEL_REG_CLICK_CFG 0x38
#define I2C_ACCEL_REG_CLICK_SRC 0x39
#define I2C_ACCEL_REG_CLICK_THSY_X 0x3B
#define I2C_ACCEL_REG_CLICK_THSZ 0x3C
#define I2C_ACCEL_REG_CLICK_TIMELIMIT 0x3D
#define I2C_ACCEL_REG_CLICK_LATENCY 0x3E
#define I2C_ACCEL_REG_CLICK_WINDOW 0x3F
#define SINGLE_X (1 << 0)
#define DOUBLE_X (1 << 1)
#define SINGLE_Y (1 << 2)
#define DOUBLE_Y (1 << 3)
#define SINGLE_Z (1 << 4)
#define DOUBLE_Z (1 << 5)

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
    I2C1->CR2 = PCLK1_MHZ;
    I2C1->CCR = (PCLK1_MHZ * 1000 * 1000) / (I2C_SPEED_HZ << 1);
    I2C1->TRISE = PCLK1_MHZ + 1;
    I2C1->CR1 |= I2C_CR1_PE;
}

void init_accel(void) {
    i2c_accel_write(I2C_ACCEL_REG_CTRL1, 0);
    i2c_accel_write(I2C_ACCEL_REG_CTRL3, 0);

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOinConfigure(GPIOA, 1, GPIO_PuPd_UP,
            EXTI_Mode_Interrupt,
            EXTI_Trigger_Rising);
    EXTI->PR |= EXTI_PR_PR1;
    NVIC_SetPriority(EXTI1_IRQn, 2);
    NVIC_EnableIRQ(EXTI1_IRQn);

    i2c_accel_write(I2C_ACCEL_REG_CTRL1, (1 << 6) | (1 << 2) | (1 << 1) | (1 << 0));

#define THRESHOLD 0x8
    i2c_accel_write(I2C_ACCEL_REG_CLICK_THSY_X, THRESHOLD | (THRESHOLD << 4));
    i2c_accel_write(I2C_ACCEL_REG_CLICK_THSZ, THRESHOLD);
    i2c_accel_write(I2C_ACCEL_REG_CLICK_TIMELIMIT, 0x03);
    i2c_accel_write(I2C_ACCEL_REG_CLICK_LATENCY, 0x06);
    i2c_accel_write(I2C_ACCEL_REG_CLICK_WINDOW, 0xff);
    i2c_accel_write(I2C_ACCEL_REG_CLICK_CFG, SINGLE_Z | DOUBLE_Z | SINGLE_X | DOUBLE_X | SINGLE_Y | DOUBLE_Y);
    i2c_accel_write(I2C_ACCEL_REG_CTRL3, 7);
}

#define STR(x) #x
#define STR2(x) STR(x)
#define STR_LINE STR2(__LINE__)
#define MAX_TRIES (1000 * 1000 * 10)
#define BUSY_WAIT_FOR_BIT(cond) \
    {\
        int count; \
        for (count = 0; count < MAX_TRIES && !(I2C1->SR1 & (cond)); ++count) { /* busy wait */ } \
        if (count == MAX_TRIES) {\
            output("Unable to wait for " #cond " in " __FILE__ ":" STR_LINE "\r\n"); \
            I2C1->CR1 |= I2C_CR1_STOP; \
            return 1; \
        } \
    }

int i2c_accel_write(uint8_t reg, uint8_t value) {
    I2C1->CR1 |= I2C_CR1_START;
    BUSY_WAIT_FOR_BIT(I2C_SR1_SB);

    I2C1->DR = LIS35DE_ADDR << 1;
    BUSY_WAIT_FOR_BIT(I2C_SR1_ADDR);

    I2C1->SR2;
    I2C1->DR = reg;
    BUSY_WAIT_FOR_BIT(I2C_SR1_TXE);

    I2C1->DR = value;
    BUSY_WAIT_FOR_BIT(I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_STOP;

    return 0;
}

int i2c_accel_read(uint8_t reg, uint8_t *value) {
    I2C1->CR1 |= I2C_CR1_START;
    BUSY_WAIT_FOR_BIT(I2C_SR1_SB);

    I2C1->DR = LIS35DE_ADDR << 1;
    BUSY_WAIT_FOR_BIT(I2C_SR1_ADDR);

    I2C1->SR2;
    I2C1->DR = reg;
    BUSY_WAIT_FOR_BIT(I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_START;
    BUSY_WAIT_FOR_BIT(I2C_SR1_SB);

    I2C1->DR = (LIS35DE_ADDR << 1) | 1U;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    BUSY_WAIT_FOR_BIT(I2C_SR1_ADDR);
    I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_STOP;
    BUSY_WAIT_FOR_BIT(I2C_SR1_RXNE);

    *value = I2C1->DR;

    return 0;
}

static void handle_interrupt(void) {
    uint8_t src;
    i2c_accel_read(I2C_ACCEL_REG_CLICK_SRC, &src);

    output("Interrupt: ");

#define CHECK(MACRO) \
    if (src & MACRO) { \
        output(#MACRO "detected\r\n");\
    }

    CHECK(SINGLE_X);
    CHECK(DOUBLE_X);

    CHECK(SINGLE_Y);
    CHECK(DOUBLE_Y);

    CHECK(SINGLE_Z);
    CHECK(DOUBLE_Z);

    if (src & (SINGLE_X | SINGLE_Y | SINGLE_Z)) {
        reset_timer(TIMER_RED);
    }
    else {
        reset_timer(TIMER_GREEN);
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        handle_interrupt();
    }
}
