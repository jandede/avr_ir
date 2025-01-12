

#define F_CPU 8000000UL
#define TRANSMISSION_MAX_LEN 32
#define SIG_LONG_INTRVL 5

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "USART.h"
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#define CFREQ_US 15         // sampling frequency in us
#define GAPTIME_US 20000    // gaptime in us (min. high time after IR command)

/*
    SIG GND VCC
            5V
    38 kHz
    1/f = 26,32 us
    SIGNAL GAP TIME = 20ms
*/

/*
WAIT FOR SIGNAL    sign.start        signal         signal ended (gap time before next signal
———————————————————___________—————_—_——_——_—_——_——_————————...
low time seems to be 600us, 1,667 kHz
high time is either <600us or 1600us
sample with 1024 prescaler, should be safe enough - that's 7,8125 kHz
if sign. switches to low after long hightime => 0
if sign. switches to low after short hightime => 1

// 25 ms (+gap time) = 45ms duration of signal
45ms / 2^16 = 0,68 us samples

*/

// 

int irTransmissionInProgress = 0;
int transLength = 0;
// Constantly write to this variable, and validate against known pattern (long low+longish high ... data ... long high)
uint64_t transmission = 0;
uint32_t lastTransmission = 0;
uint16_t lastRisingEdgeTimeStep = 0;
uint16_t TCNT11 = 0;
uint64_t SHIFTMASK64 = (1 << 63);

uint64_t test = 0;

void print64Byte(int64_t num){
    for (int i = 64-1; i >= 0; i--){
        if ((num >> i) % 2 == 1){
            printString("1");
        } else {
            printString("0");
        }
    }
    printString("\r\n");
}


// Pin change interrupt (PORTD)
ISR(PCINT2_vect){
    //PORTB ^= (1 << PB0);
    if (!(PIND & (1 << PD2))){ // falling edge
        if (!irTransmissionInProgress){
            irTransmissionInProgress = 1;
        } else {
            transmission = ((transmission & ~(SHIFTMASK64)) << 1);
            if(TCNT11 - lastRisingEdgeTimeStep > 100){
                transmission |= 1;
                printString("1");
            } else {
                // else LSB remains 0
                printString("0");
            } 
        }
    } else {
        if (irTransmissionInProgress){
            lastRisingEdgeTimeStep = TCNT11;
        }
    }
}

// debug button PC5
ISR(PCINT1_vect){
    if (PINC & (1 << PC5)){
        printString("\r\n");

        print64Byte(lastTransmission);
        printString("\r\n");
        printByte(lastTransmission);
        printString("\r\n");
        
    }
}

// Timer 1 has overflown, check if transmission ready (or not started at all)
ISR(TIMER1_COMPA_vect){
    PORTB ^= (1 << PB0);
    if (irTransmissionInProgress){
        TCNT11++;
        if (TCNT11 - lastRisingEdgeTimeStep > GAPTIME_US/CFREQ_US - 10){
            //printString("Gap time exceeded\r\n");
            //printString("Transmission 8LSB: ");
            //printBinaryByte((int8_t) (transmission & 0xFF));
            //printBinaryByte((int8_t) ((transmission)));
            lastRisingEdgeTimeStep = 0;
            TCNT11 = 0;
            // Will transition into a 32bit var...first bit will disappear (always 1)
            // Maybe just use 32bit everywhere...first (33th) bit will disappear anyway when shifting 
            lastTransmission = transmission;
            transmission = 0;
            irTransmissionInProgress = 0;
        }
    }
}

// Setup and enable timer
void initTimer(){
    TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10) ; // Start timer at Fcpu/256 = 31,25 kHz
    TIMSK1 |= (1 << OCIE1A); // overflow interrupt enable
    OCR1A = 1;
    //OCR1A = 334;
}

void initPinChangeIntr(){
    // Set falling edge interrupt on INT0 (PD2)
    DDRD &= ~(1 << PD2);  // INT0 as input
    PCICR |= (1<<PCIE2);  // Enable Port D interrupts (PC2)
    PCMSK2 |= (1<<PCINT18); 
}

void initDebugButton(){
    DDRC &= ~(1 << PC5); // debug button
    PORTC |= (1 << PC5);
    PCICR |= (1<<PCIE1);
    PCMSK1 |= (1<<PCINT13); 
}

int main(void) {
    initDebugButton();
    clock_prescale_set(clock_div_1);
    initUSART();
    printString("\r\n\r\n");
    DDRB |= (1 << PB0);
    initTimer();
    initPinChangeIntr();
    
    sei();  // Set the global interrupt enable bit
    
    while(1){
    }                                                  
  return 0;                           
}