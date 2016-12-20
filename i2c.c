#include "i2c.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"

#define LIS35DE_ADDR 0x1C
#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16
#define I2C_ACCEL_REG_CTRL1 0x20

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

    output("Enabled I2c\r\n");
    {
        uint8_t reg;
        i2c_accel_read(0xff, &reg);
        output("Register: ");
        output_int(reg);
        output("\r\n");
    }
    i2c_accel_write(I2C_ACCEL_REG_CTRL1, (1 << 6) | (1 << 2) | (1 << 1) | (1 << 0));
    {
        uint8_t reg;
        i2c_accel_read(I2C_ACCEL_REG_CTRL1, &reg);
        output("Register: ");
        output_int(reg);
        output("\r\n");
    }
    output("Enabled I2c accel\r\n");
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
