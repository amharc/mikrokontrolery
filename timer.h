#pragma once

enum timer_select_t { TIMER_RED, TIMER_GREEN };

void init_timer(void);
void reset_timer(enum timer_select_t timer);
