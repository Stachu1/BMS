#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

volatile uint16_t adc0_value = 0;
volatile uint16_t adc7_value = 0;
volatile uint8_t current_channel = 0;

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

    // Initialize the ADC
    adc_init();

    // Initialize Timer1
    timer1_init();

    // Enable global interrupts
    sei();

    while (1) {
        // Main loop
        // You can use adc0_value and adc7_value here
    }
}