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
#define CLICK_SRC_IA (1 << 6)

/* The code below is asynchronous and uses a custom domain specific lanaguage to
 * express asynchronicity as if the code was synchronous. The underlying mechanism
 * is akin to (very limited) coroutines -- a function serves as a coroutine,
 * which is basically static and not re-entrant, but saves its state in static variables
 * and resumes execution on next call.
 */

/* A coroutine initialising the accelerometer */
static void init_accel_phase2(void);

void init_accel(void) {
    init_accel_phase2();
}

/* State of a general coroutine */
struct coroutine_state_t {
    union {
        /* The write request currently performed */
        struct i2c_write_request_t write;

        /* The read request currently performed */
        struct i2c_read_request_t read;
    };

    /* The line number to resume execution on from */
    int line; 
};

/* Declares a corotuine */
#define ACCEL_COROUTINE(name) \
    /* Error handling function */ \
    static void name ## _error(void); \
    /* The coroutine itself */ \
    static void name(void); \
    static struct coroutine_state_t name ## _state =\
        { .write = \
            { .addr = LIS35DE_ADDR \
            , .reg = 0 \
            , .value = 0 \
            , .on_success = &name \
            , .on_error = &name ## _error\
            }\
        , .read =\
            { .addr = LIS35DE_ADDR \
            , .reg = 0 \
            , .value = NULL \
            , .on_success = &name \
            , .on_error = &name ## _error \
            }\
        , .line = 0\
        };\
    static void name ## _error(void) {\
        output("Error in " #name ", line"); \
        output_int(name ## _state . line); \
        output("\r\n"); \
    }\
    \
    static void name(void) { \
        static struct coroutine_state_t *state = &name ## _state; \
        switch (state->line) { \
            case 0:

#define ACCEL_END } state->line = 0; }

/* Schedules an asynchronous write and resumes execution of the coroutine when the
 * write has been finished */
#define ACCEL_WRITE(_reg, _value) \
        state->write.reg = (_reg); \
        state->write.value = (_value); \
        state->line = __LINE__; \
        i2c_write(state->write); \
        return; \
        case __LINE__: \

/* Schedules an asynchronous read and resumes execution of the coroutine when the
 * read has been finished */
#define ACCEL_READ(_reg, _value) \
        state->read.reg = (_reg); \
        state->read.value = &(_value); \
        state->line = __LINE__; \
        i2c_read(state->read); \
        return; \
        case __LINE__: \

/* Initialises the accelerometer */
ACCEL_COROUTINE(init_accel_phase2) {
    /* Turn it off */
    ACCEL_WRITE(I2C_ACCEL_REG_CTRL1, 0);
    ACCEL_WRITE(I2C_ACCEL_REG_CTRL3, 0);

    /* Enable the clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Configure lines */
    GPIOinConfigure(GPIOA, 1, GPIO_PuPd_UP,
            EXTI_Mode_Interrupt,
            EXTI_Trigger_Rising);

    /* Enable interrupts */
    EXTI->PR |= EXTI_PR_PR1;
    NVIC_SetPriority(EXTI1_IRQn, ACCEL_IRQ_LEVEL);
    NVIC_EnableIRQ(EXTI1_IRQn);


    /* Set default parameters */
    ACCEL_WRITE(I2C_ACCEL_REG_CTRL1, (1 << 6) | (1 << 2) | (1 << 1) | (1 << 0));
#define THRESHOLD 0x8
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_THSY_X, THRESHOLD | (THRESHOLD << 4));
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_THSZ, THRESHOLD);
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_TIMELIMIT, 0x03);
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_LATENCY, 0x06);
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_WINDOW, 0xff);
    ACCEL_WRITE(I2C_ACCEL_REG_CLICK_CFG, SINGLE_Z | DOUBLE_Z | SINGLE_X | DOUBLE_X | SINGLE_Y | DOUBLE_Y);
    ACCEL_WRITE(I2C_ACCEL_REG_CTRL3, 7);
}
ACCEL_END

ACCEL_COROUTINE(handle_interrupt) {
    static uint8_t src = 0;
    do {
        ACCEL_READ(I2C_ACCEL_REG_CLICK_SRC, src);
    } while (!(src & CLICK_SRC_IA));

    if (src & (SINGLE_X | SINGLE_Y | SINGLE_Z)) {
        reset_timer(TIMER_RED);
    }

    if (src & (DOUBLE_X | DOUBLE_Y | DOUBLE_Z)) {
        reset_timer(TIMER_GREEN);
    }
}
ACCEL_END

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        if (handle_interrupt_state.line != 0) {
            output("Interrupt handler is busy. This should not happen.\r\n");
        }
        else {
            handle_interrupt();
        }
    }
}
