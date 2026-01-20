#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h> 
#include <stdlib.h>

// --- Hardware Definitions ---
#define F_CPU 16000000UL // 16MHz Clock Speed
#define TWI_FREQ_KHZ 100 

// Buzzer Pin: PD2 (Digital Pin 2)
#define BUZZER_PORT PORTD
#define BUZZER_DDR DDRD
#define BUZZER_PIN PD2

// IR Sensor Pins (Analog Channels)
#define IR_1_CHANNEL 0 // ADC0/PC0/A0
#define IR_2_CHANNEL 1 // ADC1/PC1/A1
#define IR_3_CHANNEL 2 // ADC2/PC2/A2
#define NUM_IR_SENSORS 3

// I2C Addresses
#define RTC_ADDR_W 0xD0 
#define RTC_ADDR_R 0xD1 
#define LCD_ADDR_W 0x4E 

// --- DHT22 Sensor Definitions ---
#define DHT_PIN PB0     // Connected to Digital Pin 8 (Port B, Pin 0)
#define DHT_PORT PORTB
#define DHT_DDR DDRB
#define DHT_PIN_REG PINB

// --- Data Structure for Temp/Humidity ---
typedef struct {
    float temperature; // XX.xx
    float humidity;  // XX
} TempHumidity;

// --- Calibration Settings (READ THESE FROM A REAL CIRCUIT TEST!) ---
// These are the baseline values when the object is PRESENT (obstructed).
// The alarm will be cancelled when the ADC reading is *greater than* the calibration value.
// Example values (replace with your measured 10-bit ADC readings, 0-1023):
const uint16_t IR_CALIBRATION_VALUES[NUM_IR_SENSORS] = {
    100, // IR_1 (Obstructed baseline ADC value)
    100, // IR_2 (Obstructed baseline ADC value)
    100  // IR_3 (Obstructed baseline ADC value)
};

// --- Time Structure & Alarms (24-hour format) ---
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} DateTime;

const uint8_t ALARM_HOURS[] = {7, 12, 17}; // 7:00, 12:00, 17:00 (5:00 PM)
const uint8_t NUM_ALARMS = 3;

// --- Global State ---
#define NUM_IR_SENSORS 3
volatile uint8_t isBuzzing = 0; // State flag
volatile uint8_t sensorState[NUM_IR_SENSORS] = {0, 0, 0}; // 0=Empty, 1=Object Present
volatile uint8_t requiredCancelSensor = 0;
volatile uint8_t objectCount[NUM_IR_SENSORS] = {0, 0, 0};
// Alarm State and Counters
volatile uint8_t totalObjectCount = 0; 
volatile uint8_t previousTotalObjectCount = 0; // NEW: Tracks count from the last cycle


// Alarm State Management
#define ALARM_DEBOUNCE_MS   300 // 300ms window to prevent instant cancellation

volatile uint32_t alarmSilenceDebounceTimer = 0; // Tracks when silencing is allowed

// --- Alarm State Machine ---
typedef enum {
    ALARM_OFF = 0,
    ALARM_ARMED,        // NEW: Alarm has timed out and is ready to sound/wait for cancel
    ALARM_ACTIVE,       // Alarm is buzzing and waiting for cancel
    ALARM_SILENCED
} AlarmState;

volatile AlarmState currentAlarmState = ALARM_OFF;

// --- Global Timer Variable (Replaces millis()) ---
volatile uint32_t ms_timer = 0;

// --- Global Timer and Status Variables ---
volatile uint32_t hold_timer = 0; // NEW: Needed globally for 2s silence hold

// --- Helper Macro (Replaces millis() call) ---
#define get_current_ms() (ms_timer)

// --- Helper Function (Replaces strcpy) ---
size_t string_length(const char *s) {
    size_t len = 0;
    while (s[len] != '\0') {
        len++;
    }
    return len;
}

void string_copy(char *dest, const char *src) {
    while (*src) {
        *dest++ = *src++;
    }
    *dest = '\0'; // Ensure null termination
}

TempHumidity currentTH;


// =================================================================
// Debugging Parameters
// =================================================================

#define DEBUG_ALARM_MINUTES 2 // For testing: trigger alarms every 2 minutes instead of hours

// --- UART/Serial Definitions ---
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD) - 1)

// =================================================================
// ADC IMPLEMENTATION
// =================================================================

/**
 * @brief Initializes the ADC hardware for single-ended conversion.
 */
void adc_init(void) {
    // 1. Set Vcc as reference (AVCC with external capacitor at AREF pin)
    // 2. ADLAR=0 (Right adjust)
    // 3. Select ADC Channel 0 (will be changed in adc_read)
    ADMUX = (1 << REFS0); 

    // 1. Enable ADC (ADEN)
    // 2. Set Prescaler to 128 (16MHz / 128 = 125kHz, within 50-200kHz optimal range)
    // 3. Start Conversion (ADSC) and Auto Trigger (ADATE) are initially off
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/**
 * @brief Reads a single conversion from the specified ADC channel.
 * @param channel The analog channel (0-5 for A0-A5).
 * @return The 10-bit ADC reading (0-1023).
 */
uint16_t adc_read(uint8_t channel) {
    // 1. Select the channel: Clear the MUX bits, then set the channel number
    ADMUX = (ADMUX & 0xF0) | channel;
    
    // 2. Start single conversion
    ADCSRA |= (1 << ADSC);
    
    // 3. Wait for conversion to complete (ADIF bit is set when complete)
    while (ADCSRA & (1 << ADSC)); 
    
    // 4. Return the 10-bit result (ADCL must be read first)
    return ADC; 
}

// =================================================================
// UART/SERIAL IMPLEMENTATION (FOR DEBUGGING)
// =================================================================

/**
 * @brief Initializes the UART for serial communication.
 */
void uart_init(void) {
    // Set Baud Rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    
    // Enable Receiver and Transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    
    // Set frame format: 8 data bits, no parity, 1 stop bit
    UCSR0C = (3 << UCSZ00); // 3 (0b011) sets 8 data bits
}

/**
 * @brief Sends a single character over UART.
 * @param data The character to send.
 */
void uart_transmit(char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    
    // Put data into buffer, sends the data
    UDR0 = data;
}

/**
 * @brief Sends a null-terminated string over UART.
 * @param str The string to send.
 */
void uart_print(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

/**
 * @brief Reads all IR ADC values and prints them to the serial monitor.
 * Format: "IR1:xxx | IR2:yyy | IR3:zzz\r\n"
 */
void print_ir_adc_values(void) {
    char buffer[40]; // Buffer for the full serial string

    // Read all three ADC channels
    uint16_t adc1 = adc_read(IR_1_CHANNEL);
    uint16_t adc2 = adc_read(IR_2_CHANNEL);
    uint16_t adc3 = adc_read(IR_3_CHANNEL);
    
    // Format the string using snprintf
    // Note: The ADC values are 10-bit (0-1023), requiring a %d format.
    snprintf(buffer, sizeof(buffer), 
             "IR1:%4d | IR2:%4d | IR3:%4d\r\n", 
             adc1, adc2, adc3);
             
    uart_print(buffer);
}

// =================================================================
// --- Function Prototypes (Previous TWI/RTC/LCD functions omitted for brevity, assumed integrated) ---
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
void lcd_print_char(char c);

void dht_set_pin_output(void);
void dht_set_pin_input(void);
void dht_pull_down(void);
void dht_pull_up(void);
static inline void wait_high(void); // Add 'void' for clarity
static inline void wait_low(void);  // Add 'void' for clarity
void dht_start_signal(void); 
uint8_t dht_read_bit(void);
uint8_t dht_read_byte(void);
// Must match the return type of your working function
int dht_read_data(TempHumidity *th); // <--- FIX: Changed return type to 'int'

// New Function Prototypes
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void checkAlarms(const DateTime *dt);
void checkIRCancel(void);
void buzz_on(void);
void buzz_off(void);

// --- Display State Machine ---
typedef enum {
    DISPLAY_NORMAL = 0,
    DISPLAY_ALARM_BLINK,  // Alarm, Temp, or Hum error (blinking message)
    DISPLAY_SILENCED_HOLD // Thanks message (2-second hold)
} DisplayState;

volatile DisplayState currentDisplayState = DISPLAY_NORMAL;
char alarmMessage[17] = ""; // The specific message to show on Row 1 when alarming

// --- DHT Error Tracking ---
volatile uint8_t dht_error_count = 0; // Tracks consecutive DHT errors

void buzz(uint16_t duration_ms);


// =================================================================
// SYSTEM INITIALIZATION: SETTING BASELINE CAPACITY
// =================================================================

// =================================================================
// TIMER 0: MILLISECOND TICK GENERATOR
// =================================================================

/**
 * @brief Initializes Timer0 for a 1ms interrupt tick.
 * 16MHz Clock / 64 Prescaler / 250 Output Compare Match = 1000 Hz (1ms)
 */
void timer0_init(void) {
    // Set up Timer 0 in CTC mode (Clear Timer on Compare Match)
    TCCR0A |= (1 << WGM01); 
    
    // Set Compare Match Register (OCR0A) to 249 (0 to 249 = 250 counts)
    OCR0A = 249; 
    
    // Enable Timer/Counter 0 Output Compare Match A interrupt
    TIMSK0 |= (1 << OCIE0A); 
    
    // Set Prescaler to 64 and start timer
    TCCR0B |= (1 << CS01) | (1 << CS00); 
    
    // Globally enable interrupts
    sei();
}

// Interrupt Service Routine for Timer0 Compare Match A
ISR(TIMER0_COMPA_vect) {
    ms_timer++;
}


// ... (twi_init, twi_start, rtc_getTime, lcd_init, lcd_print, etc. from previous code) ...
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
// CAPACITY/COUNT LOGIC (Update)
// =================================================================

/**
 * @brief Updates the total object count based on sensorState.
 * Must be called right after checkIRCancel() updates sensorState.
 */
void updateObjectCount(void) {
    totalObjectCount = 0;
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        if (sensorState[i] == 1) { // If sensor is marked as Present/Filled
            totalObjectCount++;
        }
    }
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

void lcd_print(const char *str) {
    while (*str) {
        lcd_print_char(*str++);
    }
}

// =================================================================
// LCD STATUS DISPLAY (MODIFIED FOR C1 C2 C3 INDICATORS)
// =================================================================

void updateLCDDisplay(void) {
    char buffer[17];
    static uint32_t blink_timer = 0;
    static uint32_t hold_timer = 0;
    static uint8_t blink_on = 1;

    // --- ROW 0 (Persistent) ---
    // Note: Time (hh:mm:ss) is updated in main loop's RTC check
    
    // Object indicator is updated every loop
    lcd_goto_xy(10, 0); 
    snprintf(buffer, 7, "OBJ:%02d", totalObjectCount); // Format: OBJ:XX
    lcd_print(buffer);

    // --- ROW 1 (Dynamic Display State) ---
    
    if (currentDisplayState == DISPLAY_NORMAL) {
        // Normal Display (T:XX.XX C, H:XX%)
        
        lcd_goto_xy(0, 1);
        // T:
        lcd_print("T:");
        char float_buffer[6];
        dtostrf(currentTH.temperature, 4, 2, float_buffer); // XX.XX
        lcd_print(float_buffer);
        lcd_print("C  "); // Include spaces to ensure old text (like ALARM!) is cleared
        
        // H:
        lcd_goto_xy(11, 1); 
        snprintf(buffer, 5, "H:%02d%%", (uint8_t)currentTH.humidity); // XX% (integer only)
        lcd_print(buffer);

    } else {
        // Alarm/Error Display (Blinking or Held Message)
        
        // 1. Blinking Logic
        if (currentDisplayState == DISPLAY_ALARM_BLINK) {
            // Check timer using the global millisecond counter
            if ((get_current_ms() - blink_timer) > 300) { // Blink rate: 300ms
                blink_on = !blink_on;
                blink_timer = get_current_ms();
            }
            
            // Only show message when blink is ON
            if (blink_on) {
                lcd_goto_xy(0, 1);
                lcd_print(alarmMessage);
                // Pad with spaces to clear the rest of the line
                for(size_t i = string_length(alarmMessage); i < 16; i++) { // FIX: Use string_length
                    lcd_print(" ");
                }
            } else {
                lcd_goto_xy(0, 1);
                lcd_print("                "); // Blank line
            }
        } 
        
        // 2. Silenced/Held Logic
        else if (currentDisplayState == DISPLAY_SILENCED_HOLD) {
            lcd_goto_xy(0, 1);
            string_copy(buffer, alarmMessage); // Use local buffer for printing
            lcd_print(buffer); // Display the held message
            
            // Check hold timer using the global millisecond counter
            if ((get_current_ms() - hold_timer) > 2000) { // 2 second hold
                currentDisplayState = DISPLAY_NORMAL;
            }
        }
    }
}

// =================================================================
// BUZZER CONTROL (Updated to continuous control)
// =================================================================

/**
 * @brief Turns the buzzer ON indefinitely.
 */
void buzz_on(void) {
    BUZZER_PORT |= (1 << BUZZER_PIN); 
    currentAlarmState = ALARM_ACTIVE;
}

/**
 * @brief Turns the buzzer OFF.
 */
void buzz_off(void) {
    BUZZER_PORT &= ~(1 << BUZZER_PIN);
}

// =================================================================
// ALARM LOGIC
// =================================================================

/**
 * @brief Checks if the current time matches any alarm time to trigger the alarm.
 * @param dt Pointer to the current DateTime structure. (NORMAL OPERATION/COMPARE AT HOUR)
 */
// void checkAlarms(const DateTime *dt) {
//     if (currentAlarmState == ALARM_OFF) {
//         for (uint8_t i = 0; i < NUM_ALARMS; i++) {
//             if (dt->hour == ALARM_HOURS[i] && dt->min == 0 && dt->sec == 0) {
//                 // 1. Trigger the continuous buzzer
//                 buzz_on(); 
//                 // 2. Set the required sensor for cancellation (next in sequence)
//                 requiredCancelSensor = (requiredCancelSensor + 1) % NUM_IR_SENSORS;
//                 // 3. Display ALARM message
//                 // Fix 2: String constant fix is applied via the lcd_print prototype change
//                 lcd_goto_xy(10, 1);
//                 lcd_print("ALARM!"); 
//                 break; 
//             }
//         }
//     }
// }

/**
 * @brief Checks if the current time matches any alarm time to trigger the alarm.
 * @param dt Pointer to the current DateTime structure. (DEBUGGING MODE/2 MINUTE INTERVAL)
 */
void checkAlarms(const DateTime *dt) {
    uint8_t time_to_trigger = 0;
    
    // Check if the current time matches the 2-minute trigger point (every 2 minutes at :00 second)
    if (dt->min % 2 == 0 && dt->sec == 0) {
        time_to_trigger = 1;
    }
    
    if (time_to_trigger) {
        
        // Reset SILENCED state
        if (currentAlarmState == ALARM_SILENCED) {
            currentAlarmState = ALARM_OFF;
        }
        
        // Trigger the alarm if it's currently OFF
        if (currentAlarmState == ALARM_OFF) {
            
            // Set the alarm state to ACTIVE
            currentAlarmState = ALARM_ACTIVE; 
            
            // This is the trigger point: turn on the buzzer and display the alert
            buzz_on(); 
            currentDisplayState = DISPLAY_ALARM_BLINK;
            string_copy(alarmMessage, "ALARM!!!"); 
            // --- NEW: Start the debounce timer ---
            alarmSilenceDebounceTimer = get_current_ms(); // Use your millisecond timer function
        }
    }
}
// =================================================================
// CANCELLATION AND COUNTING LOGIC (INVERTED DETECTION FIX)
// =================================================================

void checkIRCancel(void) {
    uint16_t readings[NUM_IR_SENSORS];
    uint8_t currentTotalObjectCount = 0; // Local variable for calculation
    
    // --- 1. Update individual sensor states (Calculates currentTotalObjectCount) ---
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
        readings[i] = adc_read(i);
        uint16_t cal_val = IR_CALIBRATION_VALUES[i];
        
        // Object PRESENT (ADC high: > 30)
        if (readings[i] > cal_val) { 
            if (sensorState[i] == 0) {
                sensorState[i] = 1; 
            }
        } 
        // Object REMOVED (ADC low: <= 30)
        else { // readings[i] <= cal_val
            if (sensorState[i] == 1) {
                objectCount[i]++; // Event Counter
                sensorState[i] = 0; 
            }
        }
        
        // Calculate the current object count based on the sensor state
        if (sensorState[i] == 1) {
            currentTotalObjectCount++;
        }
    }
    
    // --- 2. Alarm Cancellation Logic (Comparison) ---
    if (currentAlarmState == ALARM_ACTIVE) {
        
        // Check if the overall object count has decreased (an object was taken)
        if (currentTotalObjectCount < previousTotalObjectCount) {
            
            // 1. Cancel the buzzer
            buzz_off();
            
            // 2. Transition state to SILENCED
            currentAlarmState = ALARM_SILENCED;
            
            // 3. Set display state for 2s hold ("Thanks :)")
            currentDisplayState = DISPLAY_SILENCED_HOLD;
            string_copy(alarmMessage, "Thanks :)");
            hold_timer = get_current_ms(); 
        }
    }
    
    // --- 3. Update Global Variables ---
    // Update the global total for the LCD display
    totalObjectCount = currentTotalObjectCount; 
    
    // Update the previous total for the next cycle's comparison
    previousTotalObjectCount = currentTotalObjectCount; 
    
    // NOTE: The debounce timer and ALARM_DEBOUNCE_MS constant are no longer needed.
}

// =================================================================
// ERROR CHECKING AND ALARM GENERATION
// =================================================================

void checkErrorAlarms(void) {
    // Only check and set error alarms if the primary alarm is OFF
    if (currentAlarmState == ALARM_OFF) {
        
        if (currentTH.temperature > 70.0f) {
            currentDisplayState = DISPLAY_ALARM_BLINK;
            string_copy(alarmMessage, "MELTING!!!");
            buzz_on();
        } else if (currentTH.humidity > 90.0f) {
            currentDisplayState = DISPLAY_ALARM_BLINK;
            string_copy(alarmMessage, "TOO WET!!!");
            buzz_on();
        } else if (dht_error_count > 3) { // Use a threshold for persistent errors
            currentDisplayState = DISPLAY_ALARM_BLINK;
            string_copy(alarmMessage, "DHT... ( ;-;)");
            buzz_on();
        } else {
            // If no errors, ensure we are in the normal display state
            if (currentDisplayState != DISPLAY_SILENCED_HOLD) {
                 currentDisplayState = DISPLAY_NORMAL;
            }
            buzz_off(); // Ensure buzzer is off if no alarm is active
        }
    }
}
// =================================================================
// DHT22 LOW-LEVEL PIN MANIPULATION
// =================================================================

inline void dht_set_pin_output(void) { DHT_DDR |= (1 << DHT_PIN); }
inline void dht_set_pin_input(void) { DHT_DDR &= ~(1 << DHT_PIN); }
inline void dht_pull_down(void) { DHT_PORT &= ~(1 << DHT_PIN); } // Set LOW
inline void dht_pull_up(void) { DHT_PORT |= (1 << DHT_PIN); } // Set HIGH (with pull-up resistor/code)
inline uint8_t dht_read_pin(void) { return (PINB & (1 << DHT_PIN)); }

// =================================================================
// DHT22 CORE PROTOCOL
// =================================================================

// Delay stabil untuk SimulIDE
static inline void wait_high() {
    uint16_t timeout = 0;
    while (!(DHT_PIN_REG & (1 << DHT_PIN))) {
        if (++timeout > 10000) break;
    }
}

static inline void wait_low() {
    uint16_t timeout = 0;
    while (DHT_PIN_REG & (1 << DHT_PIN)) {
        if (++timeout > 10000) break;
    }
}


// Fungsi untuk mengirim sinyal start ke DHT22
void dht_start_signal() {
    DHT_DDR |= (1 << DHT_PIN);  // Set pin sebagai output
    DHT_PORT &= ~(1 << DHT_PIN); // Pull low
    _delay_ms(18);              // Tunggu 18ms
    DHT_PORT |= (1 << DHT_PIN);  // Pull high
    _delay_us(30);              // Tunggu 30us
    DHT_DDR &= ~(1 << DHT_PIN); // Set pin sebagai input
}

// Fungsi untuk membaca bit dari DHT22
uint8_t dht_read_bit() {
    wait_high();       // tunggu high
    _delay_us(35);     // sampling window

    uint8_t bit = (DHT_PIN_REG & (1 << DHT_PIN)) ? 1 : 0;

    wait_low();        // tunggu kembali ke low
    return bit;
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
    
    // Tunggu response low 80us
    wait_low();
    _delay_us(80);
    // Tunggu response high 80us
    wait_high();
    _delay_us(80);
    
    // Baca 40 bit data
    for (int i = 0; i < 5; i++) {
        data[i] = dht_read_byte();
    }
    
    // Verifikasi checksum
    if (data[4] != (uint8_t) (data[0] + data[1] + data[2] + data[3])) {
        return -1; // Checksum error
    }
    
    // Hitung kelembaban dan suhu
    *humidity = ((data[0] << 8) | data[1]) / 10.0;
    *temperature = ((data[2] << 8) | data[3]) / 10.0;
    
    return 0; // Sukses
}

uint8_t dht_wait_for_pin(uint8_t state) {
    uint8_t count = 0;
    
    // Use the optimized read pin macro
    if (state) {
        // Wait for pin to go HIGH
        while (!dht_read_pin()) {
            if (count++ > 250) return 0; // Timeout threshold (approx 250 cycles/15us)
            _delay_us(1); // Small delay to prevent infinite loop/busy wait
        }
    } else {
        // Wait for pin to go LOW
        while (dht_read_pin()) {
            if (count++ > 250) return 0;
            _delay_us(1);
        }
    }
    return count;
}

/**
 * @brief Reads data from DHT22 and populates the TempHumidity structure.
 * @param th Pointer to the TempHumidity structure.
 * @return 0 on success, -1 on checksum error.
 */
int dht_read_data(TempHumidity *th) {
    uint8_t data[5] = {0};
    
    dht_start_signal();
    
    // Tunggu response low 80us
    wait_low();
    _delay_us(80); // Wait out the 80us LOW response
    
    // Tunggu response high 80us
    wait_high();
    _delay_us(80); // Wait out the 80us HIGH response
    
    // Baca 40 bit data
    for (int i = 0; i < 5; i++) {
        data[i] = dht_read_byte();
    }
    
    // Verifikasi checksum
    if (data[4] != (uint8_t) (data[0] + data[1] + data[2] + data[3])) {
        return -1; // Checksum error
    }
    
    // Hitung kelembaban dan suhu
    th->humidity = ((data[0] << 8) | data[1]) / 10.0;
    th->temperature = ((data[2] << 8) | data[3]) / 10.0;
    
    return 0; // Sukses
}

// =================================================================
// TEMP/HUMIDITY OUTPUT
// =================================================================

/**
 * @brief Prints Temp/Humid data to LCD and Serial using float conversion.
 */
void updateTH_and_Output(void) {
    
    // Buffer for float to string conversion (XX.XX)
    char float_buffer[6]; 
    
    if (dht_read_data(&currentTH) == 0) {
        
        // --- 1. SERIAL OUTPUT ---
        
        // T:
        uart_print("T:");
        dtostrf(currentTH.temperature, 5, 2, float_buffer);
        uart_print(float_buffer);
        uart_print(" H:");
        
        // H:
        dtostrf(currentTH.humidity, 4, 2, float_buffer);
        uart_print(float_buffer);
        uart_print(" | ");

        // Call ADC debug output function to complete the line
        print_ir_adc_values(); 

        // --- 2. LCD OUTPUT (Row 0, Col 0) ---
        
        lcd_goto_xy(0, 0); 
        
        // T:XX.XX
        uart_print("T:");
        dtostrf(currentTH.temperature, 4, 2, float_buffer);
        lcd_print(float_buffer);
        
        // H:XX.XX% (using Row 0, Col 8)
        lcd_goto_xy(8, 0); 
        lcd_print("H:");
        dtostrf(currentTH.humidity, 4, 2, float_buffer);
        lcd_print(float_buffer);
        
    } else {
        // DHT Read Failed
        lcd_goto_xy(0, 0);
        lcd_print("T/H Error     ");
        uart_print("T/H Read Error\r\n");
    }
}

/**
 * @brief Checks the required IR sensor to cancel the currently active alarm.
 * * Logic: Alarm is cancelled when the required sensor's ADC value is *greater* * than its calibrated baseline (indicating the object was removed/unobstructed).
 */
// void checkIRCancel(void) {
//     if (currentAlarmState == ALARM_ACTIVE) {
//         uint16_t currentReading = adc_read(requiredCancelSensor);
//         uint16_t calibrationValue = IR_CALIBRATION_VALUES[requiredCancelSensor];
//         // Check if the sensor reading exceeds the baseline (Object removed)
//         if (currentReading > calibrationValue) {
//             // 1. Cancel the buzzer
//             buzz_off();
//             // 2. Transition state to SILENCED
//             currentAlarmState = ALARM_SILENCED;
//             // 3. Clear LCD message
//             lcd_goto_xy(10, 1);
//             lcd_print("SILENCED"); 
//             // Note: The requiredCancelSensor index is kept for the NEXT alarm,
//             // enforcing the sequence (e.g., if alarm 1 was cancelled by sensor 2,
//             // the next alarm must be cancelled by sensor 3).
//         }
//     }
//     // If ALARM_SILENCED, we wait until the next exact time match in checkAlarms
//     // resets the state to ALARM_ACTIVE.
// }

// /**
//  * @brief Checks all IR sensors for object detection and manages alarm cancellation.
//  * (DEBUGGING MODE WITH OBJECT COUNTING)
//  */
// void checkIRCancel(void) {
//     // Array to hold the current readings of all three sensors
//     uint16_t readings[NUM_IR_SENSORS];

//     // Read all sensor values first
//     for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
//         readings[i] = adc_read(i);
//         uint16_t cal_val = IR_CALIBRATION_VALUES[i];
        
//         // --- Object Detection and Count Reset Logic ---
        
//         // 1. Check for 'Object Present' (ADC low/obstructed)
//         if (readings[i] < cal_val) {
            
//             // If the state just changed from Empty to Present, reset the count
//             if (sensorState[i] == 0) {
//                 objectCount[i] = 0; // Reset count when object is 'reloaded'
//                 sensorState[i] = 1; // Mark state as Object Present
//             }
//         } 
        
//         // 2. Check for 'Object Removed' (ADC high/unobstructed)
//         else { // readings[i] >= cal_val
            
//             // If the state just changed from Present to Empty, increment count
//             if (sensorState[i] == 1) {
//                 objectCount[i]++; // Increment count
//                 sensorState[i] = 0; // Mark state as Empty
//             }
//         }
//     }

//     // --- Alarm Cancellation Logic (Only runs if ALARM_ACTIVE) ---
//     if (currentAlarmState == ALARM_ACTIVE) {
        
//         uint16_t required_reading = readings[requiredCancelSensor];
//         uint16_t required_cal_val = IR_CALIBRATION_VALUES[requiredCancelSensor];

//         // Alarm is cancelled when the required sensor detects 'Object Removed'
//         if (required_reading > required_cal_val) {
            
//             // Cancel the buzzer and update state
//             buzz_off();
//             currentAlarmState = ALARM_SILENCED;
            
//             // Display SILENCED
//             lcd_goto_xy(10, 1);
//             lcd_print("SILENCED"); 
//         }
//     }
// }

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

// =================================================================
// MAIN LOOP (Normal Operation)
// =================================================================

// int main() {
//     // 1. Hardware Initialization
//     BUZZER_DDR |= (1 << BUZZER_PIN); 
//     BUZZER_PORT &= ~(1 << BUZZER_PIN); 
//     twi_init();
//     adc_init(); // New ADC initialization
//     lcd_init();
//     // Initial display...
//     // ... (omitted for brevity) ...
//     DateTime now;
//     char buffer[17];
//     uint8_t last_sec = 99; // Initialize to a value that forces the first RTC read
//     while(1) {
//         // Only read RTC once per second to reduce I2C traffic
//         rtc_getTime(&now);
//         if (now.sec != last_sec) {
//             // A new second has passed
//             last_sec = now.sec;
//             // Display Time
//             snprintf(buffer, 17, "%02d:%02d:%02d", now.hour, now.min, now.sec);
//             lcd_goto_xy(0, 1);
//             lcd_print(buffer);
//             // Check for new alarm trigger
//             checkAlarms(&now);
//         }
//         // --- IR Cancellation Check (Runs continuously for fast response) ---
//         checkIRCancel(); 
//         // Wait a short time to save power and keep the loop cycle fast (10ms)
//         _delay_ms(10);
//     }
// }

// =================================================================
// MAIN LOOP (Debugging Mode with 2-Minute Alarms)
// =================================================================

int main(void) {
    // Buzzer Initialization
    BUZZER_DDR |= (1 << BUZZER_PIN); 
    BUZZER_PORT &= ~(1 << BUZZER_PIN); 
    timer0_init(); // NEW: Initialize the 1ms tick generator
    twi_init();
    adc_init(); // New ADC initialization
    lcd_init();
    uart_init(); // Initialize UART for debugging

    DateTime now;
    char buffer[17];
    uint8_t last_sec = 99; 
    uint8_t th_read_timer = 0; // Timer to manage DHT read frequency
    // RTC Initial Time Setup (for testing, set to 06:59:50)
    rtc_setTime(21, 30, 0, 15, 12, 2025); // Dec 14, 2025, 19:10:00

    while(1) {
        rtc_getTime(&now); 

        if (now.sec != last_sec) {
            last_sec = now.sec;
            
            // Display Time
            snprintf(buffer, 9, "%02d:%02d:%02d", now.hour, now.min, now.sec);
            lcd_goto_xy(0, 0);
            lcd_print(buffer);

            // 1. Check/Arm Alarm
            checkAlarms(&now);

            // --- DHT22 Read Timer ---
            th_read_timer++;
            if (th_read_timer >= 5) { // Read every 5 seconds
                if (dht_read_data(&currentTH) == 0) {
                    dht_error_count = 0;
                } else {
                    dht_error_count++;
                }
                th_read_timer = 0;
            }
        }

        // --- Continuous Logic (Runs every 10ms) ---
        checkIRCancel();      // Handles object removal/alarm silence
        checkErrorAlarms();   // Checks Temp/Hum/DHT failure alarms
        updateLCDDisplay();   // Draws LCD based on current states
        print_ir_adc_values(); // Debug: Print ADC values to Serial
        
        
        _delay_ms(10);  
    }
}