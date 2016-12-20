#pragma once

extern void init_usart(void);
extern void maybe_in(void);
extern void maybe_out(void);
extern void output(const char *str);
extern int input(char *dest, int length);
void output_int(int number);
