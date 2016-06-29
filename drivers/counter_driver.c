/*
 * counter.c
 *
 * Created: 20-6-2016 14:04:15
 *  Author: Efraim
 */ 

#include <avr/io.h>
#include "counter_driver.h"

void init_counter()
{
	TCC0.CTRLA = 0b00000101; // 0000 RESERVED | 0101 CLK/64
	TCC0.CTRLB = 0b00000000; // 0000 CCxEN for FRQ or PWM | 0 RESERVED | 000 NORMAL Wavefrom mode
	//	TCC0.CTRLC = 0b00000000; // 0000 RESERVED | xxxx Compare output value
	TCC0.CTRLD = 0b00000000; // 000 Event action OFF | 0 Timer delay for 32-bit | 0000 Timer event source OFF
	
	TCC0.CTRLE = 0b00000000; // 0000000 RESERVED | 00 BYTEMODE NORMAL
	
	TCC0.CNTH = 0x00; // set counter to 0
	TCC0.PERH = 1000000; // set counter top to 1.000.000
	
	TCC0.INTCTRLA = 0b00000010; // 0000 RESERVED | 00 Timer error interrupt level | Timer Overflow/Underflow interrupt level MEDIUM
	TCC0.INTFLAGS = 0x01; // Clear interrupt flags
	
	TCC1.CTRLA = 0b00000101; // 0000 RESERVED | 0101 CLK/64
	TCC1.CTRLB = 0b00000000; // 0000 CCxEN for FRQ or PWM | 0 RESERVED | 000 NORMAL Wavefrom mode
	//	TCC1.CTRLC = 0b00000000; // 0000 RESERVED | xxxx Compare output value
	TCC1.CTRLD = 0b00000000; // 000 Event action OFF | 0 Timer delay for 32-bit | 0000 Timer event source OFF
		
	TCC1.CTRLE = 0b00000000; // 0000000 RESERVED | 00 BYTEMODE NORMAL
		
/*	TCC1.CNTH = 0x00; // set counter to 0*/
	TCC1.PERL = 0xFF; // set counter top to 1.000.000
	TCC1.PERH = 0xFF;	
	TCC1.INTCTRLA = 0b00000010; // 0000 RESERVED | 00 Timer error interrupt level | Timer Overflow/Underflow interrupt level MEDIUM
	TCC1.INTFLAGS = 0x01; // Clear interrupt flags
	
}

void start_counter_TCC1(uint16_t COUNTER_TIME)
{
	// set timer time
//	COUNTER_TIME = COUNTER_TIME / 32; // TODO: Need to be placed somewhere else!
	
// 	TCC1.PERH = (uint8_t)((COUNTER_TIME & 0xFF00) >> 8);
// 	TCC1.PERL = (uint8_t)(COUNTER_TIME & 0x00FF);

	
	// force restart of the timer
	TCC1.CNTH = 0x00;
	TCC1.CNTL = 0x01;
	
// 	TCC1.CTRLFSET |= (1 << 4);
// 	TCC1.CTRLFSET &= ~(1 << 3); 
}
