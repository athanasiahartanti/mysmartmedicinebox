#include <avr/io.h>
#include <util/delay.h>
#include <i2c.h>
#include <uart.h>
#include <lcd.h>
#include <rtc.h>
#include <dht.h>

#define BUZZER_PIN PD3
#define NUM_SENSORS 3

// --- Global State ---
uint16_t ir_readings[NUM_SENSORS];
uint16_t currentTemp = 0, currentHum = 0;
uint8_t totalCount = 0, prevTotalCount = 0;
uint8_t dht_error = 0;

enum AlarmState { OFF, ACTIVE, SILENCED } currentAlarm = OFF;
enum DisplayMode { NORMAL, BLINK, HOLD } currentMode = NORMAL;
uint16_t holdCounter = 0;

void u8_to_str2(uint8_t v, char* b) { 
    b[0]=(v/10)+'0'; b[1]=(v%10)+'0'; b[2]='\0'; 
}

void debug_serial(uint8_t h, uint8_t m, uint8_t s) {
    // rtc time
    uart_str("["); uart_num(h); uart_char(':'); uart_num(m); uart_char(':'); uart_num(s); uart_str("] ");
    // sensor readings {1,2,3}
    uart_str("IR:{");
    for(uint8_t i=0; i<3; i++) { uart_num(ir_readings[i]); if(i<2) uart_str(","); }
    // DHT22 readings + (close bracket of sensor readings)
    uart_str("} T:"); uart_num(currentTemp); uart_str(" H:"); uart_num(currentHum);

    // DHT22 error code (debugging)
    if(dht_error) { uart_str(" ERR:"); uart_num(dht_error); }
    uart_str("\r\n");
}

int main(void) {
    uart_init(); i2c_init(); lcd_init();
    
    // Setup Hardware
    DDRD |= (1 << BUZZER_PIN);
    ADMUX = (1 << REFS0); 
    ADCSRA = (1 << ADEN) | 0x07; // ADC Enable, Prescaler 128

    uint8_t h, m, s, last_s = 99;

    while (1) {
        read_rtc(&h, &m, &s);

        // --- 1-Second Logic Loop ---
        if (s != last_s) {
            last_s = s;
            dht_error = dht_update(&currentHum, &currentTemp);
            debug_serial(h, m, s);

            // Alarm Trigger (Even minutes, 00 seconds)
            if (m % 2 == 0 && s == 0 && currentAlarm == OFF) {
                PORTD |= (1 << BUZZER_PIN); 
                currentAlarm = ACTIVE; 
                currentMode = BLINK;
            } else if (m % 2 != 0 && currentAlarm == SILENCED) {
                currentAlarm = OFF; 
                currentMode = NORMAL;
            }
        }

        // --- IR Sensor Logic ---
        uint8_t currentTotal = 0;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            ADMUX = (ADMUX & 0xF0) | (i & 0x0F);
            ADCSRA |= (1 << ADSC); 
            while (ADCSRA & (1 << ADSC));
            ir_readings[i] = ADC;
            if (ADC > 30) currentTotal++;
        }

        // Alarm Silence Logic
        if (currentAlarm == ACTIVE && currentTotal < prevTotalCount) {
            PORTD &= ~(1 << BUZZER_PIN); 
            currentAlarm = SILENCED; 
            currentMode = HOLD; 
            holdCounter = 0;
        }
        totalCount = currentTotal; 
        prevTotalCount = currentTotal;

        // --- LCD Display Logic ---
        char buf[8];
        lcd_goto(0, 0); 
        u8_to_str2(h, buf); lcd_print(buf); lcd_print(":");
        u8_to_str2(m, buf); lcd_print(buf); lcd_print(":");
        u8_to_str2(s, buf); lcd_print(buf);
        
        lcd_goto(10, 0); 
        lcd_print("OBJ:"); u8_to_str2(totalCount, buf); lcd_print(buf);

        lcd_goto(0, 1);
        if (currentMode == BLINK) {
            lcd_print(s % 2 == 0 ? "ALARM!!!        " : "                ");
        } else if (currentMode == HOLD) {
            lcd_print("Thanks :)       "); 
            if (++holdCounter > 40) currentMode = NORMAL;
        } else {
            lcd_print("T:"); dht_to_str(currentTemp, buf); lcd_print(buf);
            lcd_print("C H:"); dht_to_str(currentHum, buf); lcd_print(buf); lcd_print("%");
        }
        
        _delay_ms(50);
    }
}