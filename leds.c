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

#define RedLEDon() RED_LED_GPIO->BSRRH = 1 << RED_LED_PIN
#define RedLEDoff() RED_LED_GPIO->BSRRL = 1 << RED_LED_PIN

#define GreenLEDon() GREEN_LED_GPIO->BSRRH = 1 << GREEN_LED_PIN
#define GreenLEDoff() GREEN_LED_GPIO->BSRRL = 1 << GREEN_LED_PIN

#define BlueLEDon() BLUE_LED_GPIO->BSRRH = 1 << BLUE_LED_PIN
#define BlueLEDoff() BLUE_LED_GPIO->BSRRL = 1 << BLUE_LED_PIN

#define Green2LEDon() GREEN2_LED_GPIO->BSRRL = 1 << GREEN2_LED_PIN
#define Green2LEDoff() GREEN2_LED_GPIO->BSRRH = 1 << GREEN2_LED_PIN

/* Initialises LEDS */
void init_leds(void) {
    RCC->AHB1ENR |=
        RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    __NOP();

    RedLEDoff();
    GreenLEDoff();
    BlueLEDoff();
    Green2LEDoff();

    GPIOoutConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    GPIOoutConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP,
                     GPIO_Low_Speed, GPIO_PuPd_NOPULL);

    GPIOoutConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP, GPIO_Low_Speed,
                     GPIO_PuPd_NOPULL);

    GPIOoutConfigure(GREEN2_LED_GPIO, GREEN2_LED_PIN, GPIO_OType_PP,
                     GPIO_Low_Speed, GPIO_PuPd_NOPULL);
}

static bool state[4];

bool get_led(enum led led) { return state[led]; }

void toggle_led(enum led led) { set_led(led, !get_led(led)); }

void set_led(enum led led, bool on) {
    state[led] = on;
    switch (led) {
    case LED_RED:
        if (on) {
            RedLEDon();
        } else {
            RedLEDoff();
        }
        break;
    case LED_BLUE:
        if (on) {
            BlueLEDon();
        } else {
            BlueLEDoff();
        }
        break;
    case LED_GREEN:
        if (on) {
            GreenLEDon();
        } else {
            GreenLEDoff();
        }
        break;
    case LED_GREEN2:
        if (on) {
            Green2LEDon();
        } else {
            Green2LEDoff();
        }
        break;
    }
}
