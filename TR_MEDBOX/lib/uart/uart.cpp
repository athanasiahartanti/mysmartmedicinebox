#include "uart.h"

void uart_init(void) {
    UBRR0H = 0; 
    UBRR0L = 103; // 9600 Baud @ 16MHz
    UCSR0B = (1 << TXEN0); 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void uart_char(char c) { 
    while (!(UCSR0A & (1 << UDRE0))); 
    UDR0 = c; 
}

void uart_str(const char* s) { 
    while (*s) uart_char(*s++); 
}

void uart_num(uint16_t n) {
    char buf[6]; 
    int8_t i = 0; 
    if (n == 0) { uart_char('0'); return; }
    while (n > 0) { buf[i++] = (n % 10) + '0'; n /= 10; }
    while (--i >= 0) uart_char(buf[i]);
}