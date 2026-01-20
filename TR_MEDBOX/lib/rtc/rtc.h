#ifndef RTC_H
#define RTC_H

#include <avr/io.h>

#define RTC_ADDR 0xD0

void read_rtc(uint8_t* h, uint8_t* m, uint8_t* s);

#endif