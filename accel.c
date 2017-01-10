#include "i2c.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"
#include "leds.h"
#include "timer.h"
#include "accel.h"

#define LIS35DE_ADDR 0x1C
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

void init_accel(void) {
    accel_write(I2C_ACCEL_REG_CTRL1, 0);
    accel_write(I2C_ACCEL_REG_CTRL3, 0);

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOinConfigure(GPIOA, 1, GPIO_PuPd_UP,
            EXTI_Mode_Interrupt,
            EXTI_Trigger_Rising);
    EXTI->PR |= EXTI_PR_PR1;
    NVIC_SetPriority(EXTI1_IRQn, 2);
    NVIC_EnableIRQ(EXTI1_IRQn);

    accel_write(I2C_ACCEL_REG_CTRL1, (1 << 6) | (1 << 2) | (1 << 1) | (1 << 0));

#define THRESHOLD 0x8
    accel_write(I2C_ACCEL_REG_CLICK_THSY_X, THRESHOLD | (THRESHOLD << 4));
    accel_write(I2C_ACCEL_REG_CLICK_THSZ, THRESHOLD);
    accel_write(I2C_ACCEL_REG_CLICK_TIMELIMIT, 0x03);
    accel_write(I2C_ACCEL_REG_CLICK_LATENCY, 0x06);
    accel_write(I2C_ACCEL_REG_CLICK_WINDOW, 0xff);
    accel_write(I2C_ACCEL_REG_CLICK_CFG, SINGLE_Z | DOUBLE_Z | SINGLE_X | DOUBLE_X | SINGLE_Y | DOUBLE_Y);
    accel_write(I2C_ACCEL_REG_CTRL3, 7);
}

static void handle_interrupt(void) {
    uint8_t src;
    accel_read(I2C_ACCEL_REG_CLICK_SRC, &src);

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

int accel_write(uint8_t reg, uint8_t value) {
    return i2c_write(LIS35DE_ADDR, reg, value);
}

int accel_read(uint8_t reg, uint8_t *value) {
    return i2c_read(LIS35DE_ADDR, reg, value);
}
