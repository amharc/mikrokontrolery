#include "button.h"
#include "leds.h"
#include "usart.h"
#include <stm32.h>
#include <gpio.h>

#define ALL_INTERRUPTS(MACRO) \
    MACRO(UP,      GPIOB,  5, EXTI_PR_PR5,  EXTI9_5_IRQn); \
    MACRO(DOWN,    GPIOB,  6, EXTI_PR_PR6,  EXTI9_5_IRQn); \
    MACRO(USER,    GPIOC, 13, EXTI_PR_PR13, EXTI15_10_IRQn); \
    MACRO(ACTION,  GPIOB, 10, EXTI_PR_PR10, EXTI15_10_IRQn); \
    MACRO(MODE,    GPIOA,  0, EXTI_PR_PR0,  EXTI0_IRQn); \
    MACRO(LEFT,    GPIOB,  3, EXTI_PR_PR3,  EXTI3_IRQn); \
    MACRO(RIGHT,   GPIOB,  4, EXTI_PR_PR4,  EXTI4_IRQn); \

void init_buttons(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

#define INIT_INTERRUPT(NAME, GPIO, PIN, FLAG, IRQn) \
    GPIOinConfigure(GPIO, PIN, GPIO_PuPd_UP, \
            EXTI_Mode_Interrupt, \
            EXTI_Trigger_Falling); \
    EXTI->PR |= FLAG; \
    NVIC_SetPriority(IRQn, 2); \
    NVIC_EnableIRQ(IRQn);

    ALL_INTERRUPTS(INIT_INTERRUPT)
}

static inline void generic_handler(int min, int max) {
    toggle_led(LED_BLUE);
#define PROCESS_INTERRUPT(NAME, GPIO, PIN, FLAG, IRQn) \
    if (min <= PIN && PIN <= max && (EXTI->PR & FLAG)) { \
        EXTI->PR = FLAG; \
        output(#NAME "\r\n"); \
    }
    
    ALL_INTERRUPTS(PROCESS_INTERRUPT)
}

void EXTI9_5_IRQHandler(void) {
    generic_handler(5, 9);
}

void EXTI15_10_IRQHandler(void) {
    generic_handler(10, 15);
}

void EXTI0_IRQHandler(void) {
    generic_handler(0, 0);
}

void EXTI3_IRQHandler(void) {
    generic_handler(3, 3);
}

void EXTI4_IRQHandler(void) {
    generic_handler(4, 4);
}
