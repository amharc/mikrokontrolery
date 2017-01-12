#pragma once

#define USART_IRQ_LEVEL 1

/* Initialises USART */
extern void init_usart(void);

/** Enqueues the given string for transmission. Interrupt-safe */
extern void output(const char *str);

/** Enqueues the given integer for transmission (as a decimal number).
 * Interrupt-safe */
extern void output_int(int number);
