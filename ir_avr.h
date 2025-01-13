#ifndef IR_AVR_H_   /* Include guard */
#define IR_AVR_H_

extern uint32_t lastTransmission;

void print64Byte(int64_t num);
ISR(PCINT2_vect);
ISR(PCINT1_vect);
ISR(TIMER1_COMPA_vect);
void initTimer(void);
void stopTimer(void);
void initPinChangeIntr(void);
void stopPinChangeIntr(void);
void initDebugButton(void);
void IRSetup(void);
uint32_t IRListen(int8_t timeout);

#endif
