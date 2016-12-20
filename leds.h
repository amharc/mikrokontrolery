#pragma once

enum led { LED_RED, LED_BLUE, LED_GREEN, LED_GREEN2 };
extern void init_leds(void);
extern char get_led(enum led led);
extern void set_led(enum led led, char on);
extern void toggle_led(enum led led);
