#include "timer.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include "usart.h"
#include "leds.h"

void init_timer(void) {
#define INIT(TIMER) \
    { \
        RCC->APB1ENR |= RCC_APB1ENR_ ## TIMER ## EN; \
        TIMER->CR1 = 0; \
        TIMER->PSC = 16 * 3 - 1; \
        TIMER->ARR = 1000000; \
        TIMER->EGR = TIM_EGR_UG; \
        TIMER->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF); \
        TIMER->DIER = TIM_DIER_UIE; \
        NVIC_EnableIRQ(TIMER ## _IRQn); \
        TIMER->CR1 |= TIM_CR1_CEN; \
    }

    INIT(TIM2);
    INIT(TIM5);
}

void reset_timer(enum timer_select_t timer) {
    switch (timer) {
        case TIMER_RED:
            set_led(LED_RED, 1);
            TIM2->CNT = 0;
            break;
        case TIMER_GREEN:
            set_led(LED_GREEN, 1);
            TIM5->CNT = 0;
            break;
    }
}

void TIM2_IRQHandler(void) {
    uint32_t it_status = TIM2->SR & TIM2->DIER;
    output("TIM2\r\n");
    if (it_status & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;
    }
    if (it_status & TIM_SR_CC1IF) {
        TIM2->SR = ~TIM_SR_CC1IF;
    }
    set_led(LED_RED, 0);
}

void TIM5_IRQHandler(void) {
    uint32_t it_status = TIM5->SR & TIM5->DIER;
    output("TIM5\r\n");
    if (it_status & TIM_SR_UIF) {
        TIM5->SR = ~TIM_SR_UIF;
    }
    if (it_status & TIM_SR_CC1IF) {
        TIM5->SR = ~TIM_SR_CC1IF;
    }
    set_led(LED_GREEN, 0);
}
