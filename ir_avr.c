
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "USART.h"
#include <stdint.h>
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

// This flag is set when IR signal is detected (transition to LOW)
// and will be unset when signal stop condition triggered (20+ ms HIGH)
int irTransmissionInProgress = 0;
uint32_t transmission = 0;            // Build a bitstream from IR Recv. digital input
uint32_t lastTransmission = 0;        // Save IR command into clean variable
uint16_t lastRisingEdgeTimeStep = 0;  // Used to calculate pulse high-time
// Increment this pseudo-timer everytime TCNT1 ticks (different condition for overflow!)
uint16_t TCNT11 = 0;    
uint32_t SHIFTMASK32 = (1 << 31);  // Used to unset 32-bit MSB


// Used to debug 64 bit ints
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
    if (!(PIND & (1 << PD2))){ // falling edge
        if (!irTransmissionInProgress){
            // First falling edge signaling start of IR code transmission
            irTransmissionInProgress = 1;
        } else {
            // Shift left so that new bit can be stored in LSB
            // Need to clear MSB before shifting so it doesn't overflow potentially
            // Hence SHIFTMASK32 here
            transmission = ((transmission & ~(SHIFTMASK32)) << 1);
            // If signal has stayed high for a certain interval, it's a 1
            if(TCNT11 - lastRisingEdgeTimeStep > 100){
                transmission |= 1;
                //printString("1");
            } else {
                // else LSB remains 0
                //printString("0");
            } 
        }
    } else {
        if (irTransmissionInProgress){
            // Rising edge, just save this timestep so next time a falling edge is detected, pulse time can be calculated
            // (current_time - lastRisingEdgeTimeStep) == pulse_time
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
    //PORTB ^= (1 << PB0); // test timer frequency
    if (irTransmissionInProgress){
        // Increment pseudo-timer when IR transmission is in progress
        TCNT11++;
        // Check if IR signal transmission is finished (if HIGH for at least GAPTIME) 
        if (TCNT11 - lastRisingEdgeTimeStep > GAPTIME_US/CFREQ_US - 10){
            // reset all vars but keep "transmission" in "lastTransmission"
            lastRisingEdgeTimeStep = 0;
            TCNT11 = 0;
            lastTransmission = transmission;
            transmission = 0;
            irTransmissionInProgress = 0;
        }
    }
}

// Setup and enable timer
void initTimer(){
    TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10) ; // Start timer at Fcpu/64
    TIMSK1 |= (1 << OCIE1A); // overflow interrupt enable
    OCR1A = 1;               // overflow every tick, another variable will track ticks...
}

void initPinChangeIntr(){
    // Set edge detection interrupt on INT0 (PD2)
    DDRD &= ~(1 << PD2);    // INT0 as input
    PCICR |= (1<<PCIE2);    // Enable Port D interrupts (PC2)
    PCMSK2 |= (1<<PCINT18); 
}

// Used to print stuff on command when debugging
void initDebugButton(){
    DDRC &= ~(1 << PC5);
    PORTC |= (1 << PC5);
    PCICR |= (1<<PCIE1);
    PCMSK1 |= (1<<PCINT13); 
}

int main(void) {
    initDebugButton();
    clock_prescale_set(clock_div_1);  // 8 MHz
    initUSART();
    DDRB |= (1 << PB0);
    initTimer();
    initPinChangeIntr();
    
    sei();  // Set the global interrupt enable bit
    
    while(1){
    }                                                  
  return 0;                           
}