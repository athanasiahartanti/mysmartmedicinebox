// #include <avr/io.h> 
// #include <avr/interrupt.h> 
// #include <uart.h>
// #include <stdio.h>

// /// Debugging pins
// #define LED_PIN (PD2) // Arduino D2 pin / Onboard LED 
// #define Buzzer_PIN (PD3) // Arduino D3 pin / Buzzer

// #define DHT_PIN (PB0) // Arduino D8 pin / DHT22 data pin
// #define DHT_PORT (PORTB) 
// #define DHT_DDR DDRB
// #define DHT_PIN_REG PINB

// // Fungsi untuk mengirim sinyal start ke DHT22
// void dht_start_signal() {
//     DHT_DDR |= (1 << DHT_PIN);  // Set pin sebagai output
//     DHT_PORT &= ~(1 << DHT_PIN); // Pull low
//     _delay_ms(18);              // Tunggu 18ms
//     DHT_PORT |= (1 << DHT_PIN);  // Pull high
//     _delay_us(40);              // Tunggu 40us
//     DHT_DDR &= ~(1 << DHT_PIN); // Set pin sebagai input
// }

// // Fungsi untuk membaca bit dari DHT22
// uint8_t dht_read_bit() {
//     while ((DHT_PIN_REG & (1 << DHT_PIN)) == 0); // Tunggu high
//     _delay_us(30); // Tunggu 30us
//     if (DHT_PIN_REG & (1 << DHT_PIN)) {
//         while ((DHT_PIN_REG & (1 << DHT_PIN))); // Tunggu low
//         return 1;
//     } else {
//         return 0;
//     }
// }

// // Fungsi untuk membaca byte (8 bit)
// uint8_t dht_read_byte() {
//     uint8_t byte = 0;
//     for (int i = 0; i < 8; i++) {
//         byte <<= 1;
//         byte |= dht_read_bit();
//     }
//     return byte;
// }

// // Fungsi utama untuk membaca DHT22
// int dht_read(float *humidity, float *temperature) {
//     uint8_t data[5] = {0};
    
//     dht_start_signal();
    
//     // Tunggu respons dari DHT22 (idk what this means)
//     _delay_us(40);
//     if ((DHT_PIN_REG & (1 << DHT_PIN)) == 0) {
//         _delay_us(80);
//         if ((DHT_PIN_REG & (1 << DHT_PIN))) {
//             _delay_us(40); // Respons OK
//         } else {
//             return -1; // Error
//         }
//     } else {
//         return -1; // Error
//     }
    
//     // Baca 40 bit data
//     for (int i = 0; i < 5; i++) {
//         data[i] = dht_read_byte();
//     }
    
//     // Verifikasi checksum
//     if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
//         return -1; // Checksum error
//     }
    
//     // Hitung kelembaban dan suhu
//     *humidity = ((data[0] << 8) | data[1]) / 10.0;
//     *temperature = ((data[2] << 8) | data[3]) / 10.0;
    
//     return 0; // Sukses
// }

// // Fungsi UART untuk output (seperti Serial Monitor)
// void uart_init() {
//     UBRR0H = 0;
//     UBRR0L = 103; // 9600 baud at 16MHz
//     UCSR0B = (1 << TXEN0);
//     UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
// }

// void uart_transmit(char data) {
//     while (!(UCSR0A & (1 << UDRE0)));
//     UDR0 = data;
// }

// void uart_print(const char *str) {
//     while (*str) {
//         uart_transmit(*str++);
//     }
// }

// void uart_print_float(float val) {
//     char buffer[10];
//     dtostrf(val, 5, 2, buffer); // Konversi float ke string
//     uart_print(buffer);
// }

// int main() {
//     float humidity, temperature;
    
//     uart_init(); // Inisialisasi UART
    
//     while (1) {
//         if (dht_read(&humidity, &temperature) == 0) {
//             uart_print("Kelembaban: ");
//             uart_print_float(humidity);
//             uart_print(" %  Suhu: ");
//             uart_print_float(temperature);
//             uart_print(" C\n");
//         } else {
//             uart_print("Error membaca DHT22\n");
//         }
//         _delay_ms(2000); // Tunggu 2 detik
//     }
    
//     return 0;
// }

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h> // For snprintf

// --- Hardware Definitions ---

// I2C (TWI) Pins: SDA = PC4 (A4), SCL = PC5 (A5)
#define TWI_FREQ_KHZ 100 // TWI operating frequency (100kHz standard)
#define F_CPU 16000000UL // 16MHz Clock Speed

// Buzzer Pin: PD2 (Digital Pin D2)
#define BUZZER_PORT PORTD
#define BUZZER_DDR DDRD
#define BUZZER_PIN PD2

#define DHT_DDR DDRB
#define DHT_PORT PORTB
#define DHT_PIN PB0

// IR Receiver Array Pins: PC0, PC1, PC2
#define IR_ARRAY_DDR DDRC
#define IR_ARRAY_PORT PORTC
#define IR0_RX_PIN PC0 
#define IR1_RX_PIN PC1 
#define IR2_RX_PIN PC2 


// I2C Addresses
#define RTC_ADDR_W 0xD0 // DS3231 Write Address (0x68 << 1)
#define RTC_ADDR_R 0xD1 // DS3231 Read Address (0x68 << 1 | 1)
#define LCD_ADDR_W 0x4E // PCF8574 Write Address (0x27 << 1, common address)

// --- Time Structure ---
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} DateTime;

// --- Alarm Times (24-hour format) ---
const uint8_t ALARM_HOURS[] = {7, 14, 17}; // 7:00, 12:00, 17:00 (5:00 PM)
const uint8_t NUM_ALARMS = 3;

// --- Global State ---
volatile uint8_t isBuzzing = 0; // State flag

// --- Function Prototypes ---
void twi_init(void);
uint8_t twi_start(uint8_t address);
void twi_stop(void);
uint8_t twi_write(uint8_t data);
uint8_t twi_read_ack(void);
uint8_t twi_read_nack(void);

uint8_t bcdToDec(uint8_t bcd);
uint8_t decToBcd(uint8_t dec);

void rtc_getTime(DateTime *dt);
void rtc_setTime(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo, uint16_t y);

void lcd_send(uint8_t data, uint8_t mode);
void lcd_pulse_en(uint8_t data);
void lcd_write4bits(uint8_t value, uint8_t mode);
void lcd_init();
void lcd_clear();
void lcd_goto_xy(uint8_t col, uint8_t row);
void lcd_print(char *str);
void lcd_print_char(char c);

void checkAlarms(const DateTime *dt);
void buzz(uint16_t duration_ms);

// =================================================================
// MAIN
// =================================================================

int main(void) {
    // 1. Buzzer Port Initialization
    BUZZER_DDR |= (1 << BUZZER_PIN); // Set D0 as output
    BUZZER_PORT &= ~(1 << BUZZER_PIN); // Ensure D0 is LOW (OFF)

    // 2. TWI/I2C Initialization
    twi_init();

    // 3. LCD Initialization
    lcd_init();

    // 4. Set Time (Optional: Uncomment once to set time, then comment out)
    // rtc_setTime(11, 27, 0, 12, 12, 2025); // Set to Dec 12, 2025, 11:27:00
    
    // Initial display
    lcd_print("AVR RTC/LCD");
    lcd_goto_xy(0, 1);
    lcd_print("Ready...");
    _delay_ms(2000);
    lcd_clear();

    DateTime now;
    char buffer[17];

    while(1) {
        rtc_getTime(&now);

        // Display Date (Row 0: DD/MM/YYYY)
        snprintf(buffer, 17, "%02d/%02d/%04d", now.day, now.month, now.year);
        lcd_goto_xy(0, 0);
        lcd_print(buffer);

        // Display Time (Row 1: HH:MM:SS)
        snprintf(buffer, 17, "%02d:%02d:%02d", now.hour, now.min, now.sec);
        lcd_goto_xy(0, 1);
        lcd_print(buffer);

        // Check Alarms
        checkAlarms(&now);

        _delay_ms(100); // 100ms delay, allowing 10 loop cycles per second
    }
}

// =================================================================
// TWI (I2C) LOW-LEVEL IMPLEMENTATION
// =================================================================

/**
 * @brief Initializes the TWI (I2C) hardware.
 */
void twi_init(void) {
    // TWI Status Register (TWSR) - Clear prescaler bits to 00
    TWSR = 0; 
    
    // TWI Bit Rate Register (TWBR) calculation for 100kHz:
    // TWBR = ( (F_CPU / TWI_FREQ) - 16 ) / (2 * PrescalerValue)
    // For 16MHz, 100kHz, Prescaler=1 (TWSR=0): TWBR = (16000000 / 100000) - 16) / 2 = 72
    TWBR = 72; 
}

/**
 * @brief Transmits the START condition and device address.
 * @param address The 8-bit device address (including R/W bit).
 * @return TWI Status Register value.
 */
uint8_t twi_start(uint8_t address) {
    // Send START condition, clear TWI interrupt flag, enable TWI
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // Wait for TWI interrupt flag to be set
    while (!(TWCR & (1 << TWINT)));
    
    // Load address and R/W bit into TWI Data Register (TWDR)
    TWDR = address;
    // Clear TWI interrupt flag, enable TWI, clear START condition bit
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // Wait for TWI interrupt flag to be set (Address transmitted, ACK/NACK received)
    while (!(TWCR & (1 << TWINT)));
    
    return TWSR & 0xF8; // Return Status (masked to 5 bits)
}

/**
 * @brief Transmits the STOP condition.
 */
void twi_stop(void) {
    // Transmit STOP condition, clear TWI interrupt flag, enable TWI
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

/**
 * @brief Writes a byte of data.
 * @param data The byte to write.
 * @return TWI Status Register value.
 */
uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWSR & 0xF8;
}

/**
 * @brief Reads a byte of data and sends an ACK.
 * @return The received data byte.
 */
uint8_t twi_read_ack(void) {
    // Read with ACK
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

/**
 * @brief Reads a byte of data and sends a NACK (for the last byte).
 * @return The received data byte.
 */
uint8_t twi_read_nack(void) {
    // Read with NACK (last byte)
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// =================================================================
// BCD / DECIMAL CONVERSION
// =================================================================

/**
 * @brief Converts Binary Coded Decimal (BCD) to Decimal.
 */
uint8_t bcdToDec(uint8_t bcd) {
    return (bcd / 16 * 10) + (bcd % 16);
}

/**
 * @brief Converts Decimal to Binary Coded Decimal (BCD).
 */
uint8_t decToBcd(uint8_t dec) {
    return (dec / 10 * 16) + (dec % 10);
}

// =================================================================
// RTC (DS3231) IMPLEMENTATION
// =================================================================

/**
 * @brief Reads the current time from the DS3231 RTC.
 * @param dt Pointer to the DateTime structure to store the data.
 */
void rtc_getTime(DateTime *dt) {
    uint8_t data[7];

    // 1. Send START, DS3231 Write Address, and Register Address (0x00)
    twi_start(RTC_ADDR_W);
    twi_write(0x00); // Start reading from register 0x00 (Seconds)

    // 2. Send REPEATED START and DS3231 Read Address
    twi_start(RTC_ADDR_R);

    // 3. Read 7 bytes: Sec, Min, Hour, DayOfWeek, Day, Month, Year
    for (int i = 0; i < 6; i++) {
        data[i] = twi_read_ack();
    }
    data[6] = twi_read_nack(); // Read last byte (Year) with NACK

    twi_stop();

    // 4. Convert BCD to Decimal and store
    dt->sec = bcdToDec(data[0] & 0x7F); // Mask out CH (Clock Halt) bit
    dt->min = bcdToDec(data[1] & 0x7F);
    dt->hour = bcdToDec(data[2] & 0x3F); // Mask out 12/24 bit (assuming 24h mode)
    // data[3] is Day of Week - ignored
    dt->day = bcdToDec(data[4]);
    dt->month = bcdToDec(data[5] & 0x7F); // Mask out century bit
    dt->year = 2000 + bcdToDec(data[6]); // DS3231 stores 00-99 year
}

/**
 * @brief Sets the time on the DS3231 RTC.
 */
void rtc_setTime(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo, uint16_t y) {
    twi_start(RTC_ADDR_W);
    twi_write(0x00); // Start writing from register 0x00 (Seconds)
    
    // Write 7 BCD values: Sec, Min, Hour(24h), DayOfWeek(1), Day, Month, Year
    twi_write(decToBcd(s));
    twi_write(decToBcd(m));
    twi_write(decToBcd(h)); // Hour in 24-hour format
    twi_write(decToBcd(1)); // Day of Week (Monday=1, arbitrary here)
    twi_write(decToBcd(d));
    twi_write(decToBcd(mo));
    twi_write(decToBcd(y % 100)); // Year (last two digits)
    
    twi_stop();
}

// =================================================================
// LCD (PCF8574) IMPLEMENTATION
// =================================================================

// LCD Controller Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
#define LCD_SETDDRAMADDR 0x80

// PCF8574 Pin Mapping: RS | R/W | EN | Backlight | D4 | D5 | D6 | D7
#define PCF_RS_CMD 0x00
#define PCF_RS_DATA 0x01
#define PCF_EN 0x04
#define PCF_LIGHT 0x08

/**
 * @brief Pulses the Enable (EN) pin to latch data into the LCD controller.
 */
void lcd_pulse_en(uint8_t data) {
    // HIGH state
    twi_start(LCD_ADDR_W);
    twi_write(data | PCF_EN);
    twi_stop();
    _delay_us(1); // Wait for pulse

    // LOW state
    twi_start(LCD_ADDR_W);
    twi_write(data & ~PCF_EN);
    twi_stop();
    _delay_us(50); // Standard execution time for most commands
}

/**
 * @brief Writes 4 bits (nibble) of data/command to the PCF8574.
 */
void lcd_write4bits(uint8_t value, uint8_t mode) {
    uint8_t data = (value << 4) | mode | PCF_LIGHT;
    
    twi_start(LCD_ADDR_W);
    twi_write(data);
    twi_stop();

    lcd_pulse_en(data & ~PCF_EN); // Only pulse EN, R/W must be 0
}

/**
 * @brief Sends a full 8-bit command or data byte to the LCD.
 */
void lcd_send(uint8_t data, uint8_t mode) {
    // Send high nibble
    lcd_write4bits(data >> 4, mode);
    // Send low nibble
    lcd_write4bits(data & 0x0F, mode);
}

/**
 * @brief Initializes the 16x2 LCD in 4-bit mode.
 */
void lcd_init() {
    _delay_ms(50); // Power-up delay

    // Reset sequence (mandatory for 4-bit mode setup)
    lcd_write4bits(0x03, PCF_RS_CMD);
    _delay_ms(5);
    lcd_write4bits(0x03, PCF_RS_CMD);
    _delay_ms(5);
    lcd_write4bits(0x03, PCF_RS_CMD);
    _delay_ms(1);
    
    // Set 4-bit mode
    lcd_write4bits(0x02, PCF_RS_CMD); 
    
    // Function Set: DL=0 (4-bit mode), N=1 (2 line), F=0 (5x8 dots)
    lcd_send(LCD_FUNCTIONSET | 0x08, PCF_RS_CMD); 
    
    // Display Control: D=1 (Display ON), C=0 (Cursor OFF), B=0 (Blink OFF)
    lcd_send(LCD_DISPLAYCONTROL | 0x04, PCF_RS_CMD);
    
    lcd_clear();
    
    // Entry Mode Set: I/D=1 (Increment cursor), SH=0 (No shift)
    lcd_send(LCD_ENTRYMODESET | 0x02, PCF_RS_CMD);
}

/**
 * @brief Clears the display and returns cursor to Home (0,0).
 */
void lcd_clear() {
    lcd_send(LCD_CLEARDISPLAY, PCF_RS_CMD);
    _delay_ms(2); // Clear display is a slow command
}

/**
 * @brief Sets the cursor position.
 * @param col Column (0-15).
 * @param row Row (0 or 1).
 */
void lcd_goto_xy(uint8_t col, uint8_t row) {
    // Line addresses: Row 0 = 0x00-0x0F, Row 1 = 0x40-0x4F
    uint8_t address = (row == 0) ? col : (col + 0x40);
    lcd_send(LCD_SETDDRAMADDR | address, PCF_RS_CMD);
}

/**
 * @brief Prints a single character.
 */
void lcd_print_char(char c) {
    lcd_send(c, PCF_RS_DATA);
}

/**
 * @brief Prints a null-terminated string.
 */
void lcd_print(char *str) {
    while (*str) {
        lcd_print_char(*str++);
    }
}

// =================================================================
// ALARM LOGIC
// =================================================================

/**
 * @brief Checks the current time against the predefined alarm times.
 */
void checkAlarms(const DateTime *dt) {
    // If the buzzer is currently active, prevent re-triggering this second
    if (isBuzzing) return;

    for (uint8_t i = 0; i < NUM_ALARMS; i++) {
        // Match Hour, Minute, and Second (must be 00 for exact match)
        if (dt->hour == ALARM_HOURS[i] && dt->min == 0 && dt->sec == 0) {
            
            buzz(500); // Trigger buzzer for 500ms

            // Display ALARM message
            lcd_goto_xy(10, 1);
            lcd_print("ALARM!");
            
            // Clear the alarm message after the buzzer stops
            _delay_ms(50); // Short delay before clearing
            lcd_goto_xy(10, 1);
            lcd_print("      "); // Print spaces to erase
            
            break; 
        }
    }
}

/**
 * @brief Turns the buzzer ON for a specified duration and then OFF.
 * @param duration_ms The duration in milliseconds to buzz.
 */
void buzz(uint16_t duration_ms) {
    isBuzzing = 1; // Set state to buzzing

    // Turn ON the buzzer (Set PD0 HIGH)
    BUZZER_PORT |= (1 << BUZZER_PIN); 
    
    // Use the optimized _delay_ms from util/delay.h
    _delay_ms(duration_ms); 
    
    // Turn OFF the buzzer (Set PD0 LOW)
    BUZZER_PORT &= ~(1 << BUZZER_PIN);
    
    isBuzzing = 0; // Reset state
}