#ifndef UART_H
#define UART_H

#include <avr/io.h>

void uart_init(void);
void uart_char(char c);
void uart_str(const char* s);
void uart_num(uint16_t n);

#endif