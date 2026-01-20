#include "dht.h"
#include <util/delay.h>

// Internal Helper: Wait for High (1 = Success, 0 = Timeout)
static inline uint8_t wait_high() {
    uint16_t timeout = 0;
    while (!(DHT_PIN_REG & (1 << DHT_PIN))) {
        if (++timeout > 10000) return 0;
    }
    return 1;
}

// Internal Helper: Wait for Low (1 = Success, 0 = Timeout)
static inline uint8_t wait_low() {
    uint16_t timeout = 0;
    while (DHT_PIN_REG & (1 << DHT_PIN)) {
        if (++timeout > 10000) return 0;
    }
    return 1;
}

void dht_start_signal() {
    DHT_DDR |= (1 << DHT_PIN);   // Output
    DHT_PORT &= ~(1 << DHT_PIN); // Low
    _delay_ms(18);
    DHT_PORT |= (1 << DHT_PIN);  // High
    _delay_us(30);
    DHT_DDR &= ~(1 << DHT_PIN);  // Input
}

uint8_t dht_read_bit() {
    if (!wait_high()) return 0;
    _delay_us(35);
    uint8_t bit = (DHT_PIN_REG & (1 << DHT_PIN)) ? 1 : 0;
    wait_low();
    return bit;
}

uint8_t dht_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= dht_read_bit();
    }
    return byte;
}

uint8_t dht_update(uint16_t *hum_out, uint16_t *temp_out) {
    uint8_t data[5] = {0};

    dht_start_signal();

    if (!wait_low()) return 1;  // ERR 1
    _delay_us(80);
    if (!wait_high()) return 2; // ERR 2
    _delay_us(80);

    for (int i = 0; i < 5; i++) data[i] = dht_read_byte();

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != sum) return 4; // ERR 4

    *hum_out = (data[0] << 8) | data[1];
    uint16_t t_raw = (data[2] << 8) | data[3];
    if (t_raw & 0x8000) t_raw &= 0x7FFF; 
    *temp_out = t_raw;

    return 0; // Success
}

void dht_to_str(uint16_t v, char* b) {
    uint16_t i = v/10; 
    b[0]=(i/10)+'0'; b[1]=(i%10)+'0'; b[2]='.'; b[3]=(v%10)+'0'; b[4]='\0';
}