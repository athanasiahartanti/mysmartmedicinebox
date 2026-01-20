#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Definisi pin (PB0 = digital pin 8 di Arduino Uno)
#define DHT_PIN PB0
#define DHT_PORT PORTB
#define DHT_DDR DDRB
#define DHT_PIN_REG PINB

// Fungsi untuk mengirim sinyal start ke DHT22
void dht_start_signal() {
    DHT_DDR |= (1 << DHT_PIN);  // Set pin sebagai output
    DHT_PORT &= ~(1 << DHT_PIN); // Pull low
    _delay_ms(18);              // Tunggu 18ms
    DHT_PORT |= (1 << DHT_PIN);  // Pull high
    _delay_us(40);              // Tunggu 40us
    DHT_DDR &= ~(1 << DHT_PIN); // Set pin sebagai input
}

// Fungsi untuk membaca bit dari DHT22
uint8_t dht_read_bit() {
    while ((DHT_PIN_REG & (1 << DHT_PIN)) == 0); // Tunggu high
    _delay_us(30); // Tunggu 30us
    if (DHT_PIN_REG & (1 << DHT_PIN)) {
        while ((DHT_PIN_REG & (1 << DHT_PIN))); // Tunggu low
        return 1;
    } else {
        return 0;
    }
}

// Fungsi untuk membaca byte (8 bit)
uint8_t dht_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= dht_read_bit();
    }
    return byte;
}

// Fungsi utama untuk membaca DHT22
int dht_read(float *humidity, float *temperature) {
    uint8_t data[5] = {0};
    
    dht_start_signal();
    
    // Tunggu respons dari DHT22
    _delay_us(40);
    if ((DHT_PIN_REG & (1 << DHT_PIN)) == 0) {
        _delay_us(80);
        if ((DHT_PIN_REG & (1 << DHT_PIN))) {
            _delay_us(40); // Respons OK
        } else {
            return -1; // Error
        }
    } else {
        return -1; // Error
    }
    
    // Baca 40 bit data
    for (int i = 0; i < 5; i++) {
        data[i] = dht_read_byte();
    }
    
    // Verifikasi checksum
    if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
        return -1; // Checksum error
    }
    
    // Hitung kelembaban dan suhu
    *humidity = ((data[0] << 8) | data[1]) / 10.0;
    *temperature = ((data[2] << 8) | data[3]) / 10.0;
    
    return 0; // Sukses
}

// Fungsi UART untuk output (seperti Serial Monitor)
void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud at 16MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void uart_print_float(float val) {
    char buffer[10];
    dtostrf(val, 5, 2, buffer); // Konversi float ke string
    uart_print(buffer);
}

int main() {
    float humidity, temperature;
    
    uart_init(); // Inisialisasi UART
    
    while (1) {
        if (dht_read(&humidity, &temperature) == 0) {
            uart_print("Kelembaban: ");
            uart_print_float(humidity);
            uart_print(" %  Suhu: ");
            uart_print_float(temperature);
            uart_print(" C\n");
        } else {
            uart_print("Error membaca DHT22\n");
        }
        _delay_ms(2000); // Tunggu 2 detik
    }
    
    return 0;
}
