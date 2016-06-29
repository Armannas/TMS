/*
 * clock_32MHz_driver.c
 *
 * Created: 26-6-2016 20:16:42
 *  Author: Efraim
 */ 


#include <drivers/clock_32MHz_driver.h>
#include <avr/io.h>



void init_clock(void)
{
	//CCP = 0xD8;
	OSC.CTRL |= (1 << 1); // 
	while(!(OSC.STATUS & (1 << 1)));
	CCP = 0xD8;
	CLK.CTRL = 0b00000001; // 00000 RESEVERD | 001 32MHz selected as clock
	
}