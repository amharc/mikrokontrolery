#pragma once

#include <stdbool.h>

enum led { LED_RED, LED_BLUE, LED_GREEN, LED_GREEN2 };

/* Initialises leds */
extern void init_leds(void);

/* Gets the state of a led (true - on, false - off) */
extern bool get_led(enum led led);

/* Sets the state of a led */
extern void set_led(enum led led, bool on);

/* Toggles the state of a led */
extern void toggle_led(enum led led);
