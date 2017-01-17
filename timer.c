#include "timer.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"
#include "leds.h"

#define TIMERS(X)                                                              \
    X(TIM2, TIMER_RED, LED_RED)                                                \
    X(TIM5, TIMER_GREEN, LED_GREEN)

/* Initialises timers */
void init_timer(void) {
#define X(TIMER, CONSTANT, LED)                                                \
    {                                                                          \
        RCC->APB1ENR |= RCC_APB1ENR_##TIMER##EN;                               \
        TIMER->CR1 = 0;                                                        \
        TIMER->PSC = 16 * 3 - 1;                                               \
        TIMER->ARR = 1000000;                                                  \
        TIMER->EGR = TIM_EGR_UG;                                               \
        TIMER->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);                              \
        TIMER->DIER = TIM_DIER_UIE;                                            \
        NVIC_EnableIRQ(TIMER##_IRQn);                                          \
        NVIC_SetPriority(TIMER##_IRQn, TIMERS_IRQ_LEVEL);                      \
        TIMER->CR1 |= TIM_CR1_CEN;                                             \
    }

    TIMERS(X)
#undef X
}

void reset_timer(enum timer_select_t timer) {
    switch (timer) {
#define X(TIMER, CONSTANT, LED)                                                \
    case CONSTANT:                                                             \
        set_led(LED, 1);                                                       \
        output("led " #LED " on\r\n");                                         \
        TIMER->CNT = 0;                                                        \
        break;

        TIMERS(X)
#undef X
    }
}

#define X(TIMER, CONSTANT, LED)                                                \
    void TIMER##_IRQHandler(void) {                                            \
        uint32_t it_status = TIMER->SR & TIMER->DIER;                          \
        output(#TIMER " fired: resetting led " #LED "\r\n");                   \
        if (it_status & TIM_SR_UIF) {                                          \
            TIMER->SR = ~TIM_SR_UIF;                                           \
        }                                                                      \
        if (it_status & TIM_SR_CC1IF) {                                        \
            TIMER->SR = ~TIM_SR_CC1IF;                                         \
        }                                                                      \
        set_led(LED, 0);                                                       \
    }

TIMERS(X)

#undef X
