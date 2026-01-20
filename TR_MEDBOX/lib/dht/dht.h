#ifndef DHT_H
#define DHT_H

#include <avr/io.h>

// PIN CONFIGURATION (PB1 = Digital 9)
#define DHT_PORT PORTB
#define DHT_DDR  DDRB
#define DHT_PIN_REG PINB
#define DHT_PIN  1  

uint8_t dht_update(uint16_t *hum_out, uint16_t *temp_out);
void dht_to_str(uint16_t v, char* b);

#endif