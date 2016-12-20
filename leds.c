#include "leds.h"
#include <delay.h>
#include <gpio.h>
#include <stm32.h>

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA
#define BLUE_LED_GPIO GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7
#define BLUE_LED_PIN 0
#define GREEN2_LED_PIN 5

#define RedLEDon() \
	RED_LED_GPIO->BSRRH = 1 << RED_LED_PIN
#define RedLEDoff() \
	RED_LED_GPIO->BSRRL = 1 << RED_LED_PIN

#define GreenLEDon() \
	GREEN_LED_GPIO->BSRRH = 1 << GREEN_LED_PIN
#define GreenLEDoff() \
	GREEN_LED_GPIO->BSRRL = 1 << GREEN_LED_PIN

#define BlueLEDon() \
	BLUE_LED_GPIO->BSRRH = 1 << BLUE_LED_PIN
#define BlueLEDoff() \
	BLUE_LED_GPIO->BSRRL = 1 << BLUE_LED_PIN

#define Green2LEDon() \
	GREEN2_LED_GPIO->BSRRL = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
	GREEN2_LED_GPIO->BSRRH = 1 << GREEN2_LED_PIN

void init_tim3(void) {
    TIM3->PSC = 63999;
    TIM3->ARR = 749;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CCR1 = 249;
    TIM3->CCR2 = 449;
    TIM3->CCR3 = 649;

    TIM3->CCMR1 =
        TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
        TIM_CCMR1_OC1PE |
        TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |
        TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE;

    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
        TIM_CCER_CC2E | TIM_CCER_CC2P |
        TIM_CCER_CC3E | TIM_CCER_CC3P;

    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void init_leds(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
		RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	__NOP();

    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

	GPIOoutConfigure(RED_LED_GPIO,
			RED_LED_PIN,
			GPIO_OType_PP,
			GPIO_Low_Speed,
			GPIO_PuPd_NOPULL);

    GPIOafConfigure(RED_LED_GPIO,
            RED_LED_PIN,
            GPIO_OType_PP,
            GPIO_Low_Speed,
            GPIO_PuPd_NOPULL,
            GPIO_AF_TIM3);

	GPIOoutConfigure(GREEN_LED_GPIO,
			GREEN_LED_PIN,
			GPIO_OType_PP,
			GPIO_Low_Speed,
			GPIO_PuPd_NOPULL);

    GPIOafConfigure(GREEN_LED_GPIO,
            GREEN_LED_PIN,
            GPIO_OType_PP,
            GPIO_Low_Speed,
            GPIO_PuPd_NOPULL,
            GPIO_AF_TIM3);

	GPIOoutConfigure(BLUE_LED_GPIO,
			BLUE_LED_PIN,
			GPIO_OType_PP,
			GPIO_Low_Speed,
			GPIO_PuPd_NOPULL);

    GPIOafConfigure(BLUE_LED_GPIO,
            BLUE_LED_PIN,
            GPIO_OType_PP,
            GPIO_Low_Speed,
            GPIO_PuPd_NOPULL,
            GPIO_AF_TIM3);

	GPIOoutConfigure(GREEN2_LED_GPIO,
			GREEN2_LED_PIN,
			GPIO_OType_PP,
			GPIO_Low_Speed,
			GPIO_PuPd_NOPULL);

    init_tim3();
}

static char state[4];

char get_led(enum led led) {
    return state[led];
}

void toggle_led(enum led led) {
    set_led(led, !get_led(led));
}

void set_led(enum led led, char on) {
    state[led] = on;
    switch (led) {
        case LED_RED:
            if (on)
                RedLEDon();
            else
                RedLEDoff();
            break;
        case LED_BLUE:
            if (on)
                BlueLEDon();
            else
                BlueLEDoff();
            break;
        case LED_GREEN:
            if (on)
                GreenLEDon();
            else
                GreenLEDoff();
            break;
        case LED_GREEN2:
            if (on)
                Green2LEDon();
            else
                Green2LEDoff();
            break;
    }
}
