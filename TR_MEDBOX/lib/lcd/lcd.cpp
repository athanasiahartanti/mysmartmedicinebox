#include "lcd.h"
#include "../i2c/i2c.h" // Depends on I2C

#define RS 0x01
#define EN 0x04
#define BK 0x08

void lcd_send(uint8_t val, uint8_t mode) {
    uint8_t h = (val & 0xF0) | mode | BK;
    uint8_t l = ((val << 4) & 0xF0) | mode | BK;
    i2c_start(); 
    i2c_write(LCD_ADDR);
    i2c_write(h | EN); i2c_write(h); 
    i2c_write(l | EN); i2c_write(l);
    i2c_stop(); 
    _delay_us(50);
}

void lcd_init(void) {
    _delay_ms(50); 
    lcd_send(0x33, 0); lcd_send(0x32, 0); 
    lcd_send(0x28, 0); lcd_send(0x0C, 0); 
    lcd_send(0x01, 0); 
    _delay_ms(2);
}

void lcd_print(const char* s) { 
    while (*s) lcd_send(*s++, RS); 
}

void lcd_goto(uint8_t x, uint8_t y) { 
    lcd_send((y == 0 ? 0x80 : 0xC0) + x, 0); 
}