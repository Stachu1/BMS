#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"
#include "lcd.h"
#include "i2cmaster.h"


#define ADC_TO_VOLTAGE(adc_value) ((float)adc_value * 0.0048875855f)    // 5 / 1023
#define VOLTAGE_DIVIDER_RATIO 3.0f                                      // 1:3 voltage divider
#define VOLTAGE_OFFSET -0.23f                                             // 0.2 V
#define CURRENT_SENSOR_GAIN 18.2                                  // 2.7027028 A/V
#define CURRENT_SENSOR_OFFSET -1.69f                                    // -1.12 A
#define UNDERVOLTAGE_THRESHOLD 8.0f                                     // 8 V
#define OVERCURRENT_THRESHOLD 20.0f                                     // 10.0 A
#define IR_LED_PIN PB5                                                  // PD 13
#define INTERRUPT_PIN PD2                                               // PD2(INT0)


volatile uint16_t adc0_value = 0;
volatile uint16_t adc7_value = 0;
volatile uint8_t current_channel = 0;
volatile uint32_t millis = 0;
volatile uint32_t last_rotation = 0;
volatile uint16_t rps_led = 0;
int16_t rps_esc = 0;


void adc_init(void) {
    // AREF = AVcc
    ADMUX = (1 << REFS0);
    // ADC Enable and prescaler of 128
    // 16 MHz / 128 = 125 KHz
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);
}

ISR(TIMER1_COMPA_vect) {
    // Start ADC conversion on the current channel
    ADMUX = (ADMUX & 0xF8) | current_channel;
    ADCSRA |= (1 << ADSC);
    millis++;
}

ISR(ADC_vect) {
    uint16_t adc_value = ADC;
    if (current_channel == 0) {
        adc0_value = adc_value;
        current_channel = 7;
    } else {
        adc7_value = adc_value;
        current_channel = 0;
    }
}

int main(void) {
  // Initialize the UART
  uart_init();
  io_redirect();
  i2c_init();
  LCD_init();

  // Set PD3 as output
  DDRD |= (1 << PD3);

  // Set IR LED pin as output
  DDRB |= (1 << IR_LED_PIN);
  PORTB |= (1 << IR_LED_PIN);

  // Initialize the ADC
  adc_init();

  // Enable global interrupts
  sei();

  _delay_ms(100);
  LCD_clear();

  while (1) {
    float voltage = ADC_TO_VOLTAGE(adc0_value) * VOLTAGE_DIVIDER_RATIO + VOLTAGE_OFFSET;
    float current = ADC_TO_VOLTAGE(adc7_value) * CURRENT_SENSOR_GAIN + CURRENT_SENSOR_OFFSET;
    float power = voltage * current;

    if (millis % 500 == 0) {
      LCD_set_cursor(0,0);
      printf("Vol:% 5.2f V    ", voltage);
      LCD_set_cursor(0,1);
      printf("Amp:% 5.2f A    ", current);
      LCD_set_cursor(0,3);
      printf("Pow:% 5.2f W    ", power);
    }
    
    if (current > OVERCURRENT_THRESHOLD || voltage < UNDERVOLTAGE_THRESHOLD) {
      PORTD |= (1 << PD3);
      LCD_clear();
      LCD_set_cursor(0,0);
      printf("Battery protection");
      LCD_set_cursor(0,1);
      printf("Vol:% 5.2f V    ", voltage);
      LCD_set_cursor(0,2);
      printf("Amp:% 5.2f A    ", current);
      LCD_set_cursor(0,3);
      printf("Pow:% 5.2f W    ", power);
      _delay_ms(10e3);
      LCD_clear();
    }
  }
}