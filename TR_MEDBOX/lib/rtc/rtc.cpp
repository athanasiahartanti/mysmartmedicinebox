#include "rtc.h"
#include "../i2c/i2c.h"

uint8_t bcd_to_dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

void read_rtc(uint8_t* h, uint8_t* m, uint8_t* s) {
    i2c_start(); i2c_write(RTC_ADDR); i2c_write(0x00);
    i2c_start(); i2c_write(RTC_ADDR | 0x01);
    *s = bcd_to_dec(i2c_read(1)); 
    *m = bcd_to_dec(i2c_read(1)); 
    *h = bcd_to_dec(i2c_read(0));
    i2c_stop();
}