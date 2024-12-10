#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"


#define ADC_TO_VOLTAGE(adc_value) ((float)adc_value * 0.0048875855f)    // 5 / 1023
#define VOLTAGE_DIVIDER_RATIO 3.0f                                      // 1:3 voltage divider
#define CURRENT_SENSOR_GAIN 2.7027028f                                  // 2.7027028 A/V
#define CURRENT_SENSOR_OFFSET -1.12f                                    // -1.12 A
#define UNDERVOLTAGE_THRESHOLD -9.6f                                     // 9.6 V
#define OVERCURRENT_THRESHOLD 10.0f                                     // 10.0 A


volatile uint16_t adc0_value = 0;
volatile uint16_t adc7_value = 0;
volatile uint8_t current_channel = 0;
volatile uint32_t millis = 0;
float rps = 0;

void adc_init(void) {
    // AREF = AVcc
    ADMUX = (1 << REFS0);
    // ADC Enable and prescaler of 128
    // 16 MHz / 128 = 125 KHz
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);
}


void timer1_init(void) {
    // Set CTC mode (Clear Timer on Compare Match)
    TCCR1B |= (1 << WGM12);
    // Set compare value for 1ms interrupt
    OCR1A = 249;
    // Enable Timer1 compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    // Start Timer1 with prescaler 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
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


// Read the target speed from the UART and update rps_target
int8_t read_current_speed(void) {
  static char buff[4];                    // Buffer to hold the incoming string (max 3 digits + null terminator)
  static uint8_t index = 0;

  if (UCSR0A & (1 << RXC0)) {             // Check if data is available in the UART buffer
    char c = uart_getchar(NULL);
    if (c == '\n') {
      buff[index] = '\0';                 // Null-terminate the string
      rps = (float)atoi(buff);         // Convert string to uint16_t
      index = 0;                          // Reset buff index
      return 1;                           // rps_target updated
    } else if (index < sizeof(buff) - 1) {
      buff[index] = c;                    // Add character to buff
      index++;                            // Increment buff index
    }
  }
  return 0;                               // rps_target not updated
}


int main(void) {
  // Initialize the UART
  uart_init();
  io_redirect();

  // Set PD3 as output
  DDRD |= (1 << PD3);

  // Initialize the ADC
  adc_init();

  // Initialize Timer1
  timer1_init();

  // Enable global interrupts
  sei();

  _delay_ms(100);
  while (1) {
    read_current_speed();

    float voltage = ADC_TO_VOLTAGE(adc0_value) * VOLTAGE_DIVIDER_RATIO;
    float current = ADC_TO_VOLTAGE(adc7_value) * CURRENT_SENSOR_GAIN + CURRENT_SENSOR_OFFSET;
    float power = voltage * current;

    if (millis % 500 == 0) {
      printf("%.2f V, %.2f A, %.2f W, RPS: %.2f\n", voltage, current, power, rps);
    }
    
    if (current > OVERCURRENT_THRESHOLD || voltage < UNDERVOLTAGE_THRESHOLD) {
      PORTD |= (1 << PD3);
      printf("Battery protection triggered\n");
      printf("%.2f V, %.2f A, %.2f W\n", voltage, current, power);
      while(1);
    }
  }
}