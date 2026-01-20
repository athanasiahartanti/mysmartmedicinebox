#ifndef LCD_H
#define LCD_H

#include <avr/io.h>
#include <util/delay.h>

#define LCD_ADDR 0x4E 

void lcd_init(void);
void lcd_print(const char* s);
void lcd_goto(uint8_t x, uint8_t y);

#endif