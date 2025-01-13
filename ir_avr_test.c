
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "USART.h"
#include <stdint.h>
#include "ir_avr.h"

void print32Byte(int64_t num){
    for (int i = 32-1; i >= 0; i--){
        if ((num >> i) % 2 == 1){
            printString("1");
        } else {
            printString("0");
        }
    }
    printString("\r\n");
}

uint32_t code1 = 0b00000000111111110011101011000101;
uint32_t code2 = 0b00000000111111111011101001000101;
uint32_t code3 = 0b00000000111111111000001001111101;
uint32_t code4 = 0b00000000111111110000001011111101;
uint32_t code5 = 0b00000000111111110001101011100101;
uint32_t code6 = 0b00000000111111111001101001100101;
uint32_t code7 = 0b00000000111111111010001001011101;
uint32_t code8 = 0b00000000111111110010001011011101;

int main(void){
    uint32_t irrecv;
    extern uint32_t lastTransmission;
    clock_prescale_set(clock_div_1);  // 8 MHz
    int32_t receivedIR = 0;
    IRSetup();
    initUSART();
    printString("\r\n\r\n");
    while(1){
        //printString("Listen...");
        irrecv = IRListen(254);
        if (irrecv){
            //print32Byte(irrecv);
            if (irrecv == code1){
                printString("KEY1");
            }
            else if (irrecv == code2){
                printString("KEY2");
            }
            else if (irrecv == code3){
                printString("KEY3");
            }
            else if (irrecv == code4){
                printString("KEY4");
            }
            else if (irrecv == code5){
                printString("KEY5");
            }
            else if (irrecv == code6){
                printString("KEY6");
            }
            else if (irrecv == code7){
                printString("KEY7");
            }
            else if (irrecv == code8){
                printString("KEY8");
            }
        } else {
            irrecv = 0;
        }
        
    }
    return 0;
}