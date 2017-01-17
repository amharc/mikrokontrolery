#pragma once

#define TIMERS_IRQ_LEVEL 2

enum timer_select_t { TIMER_RED, TIMER_GREEN };

/* Intiializes timers */
void init_timer(void);

/* Resets the timer controlling a led, and turn the led on */
void reset_timer(enum timer_select_t timer);
