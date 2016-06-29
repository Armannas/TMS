/*
 * init.c
 *
 * Created: 5/12/2016 10:00:47 PM
 *  Author: Cookie Factory
 */ 
#include <avr/io.h>
#include "init.h"
#include <interrupt.h>

void init(void)
{
	// Set GPIO pin as output
	PORTE.DIRSET = 0xFF;
	PORTE.OUT = 0xFF;
	PORTB.DIRSET &= 0x00;

	//PORTB.OUT |= (1 << 1) ;
	//Set GPIO pin to low (LED's are energized low side)
	
	safety_signals.THYR1_MAXTEMP = 0b00111100; // 00111100 = 60 -> TODO:change!!
	safety_signals.THYR2_MAXTEMP = 0b00111100;
	safety_signals.THYR3_MAXTEMP = 0b00111100;
	safety_signals.COIL_MAXTEMP = 0b00111100;
	safety_signals.CAPBANK1_MAXTEMP = 0b00111100;
	safety_signals.CAPBANK2_MAXTEMP = 0b00111100;
	
	user_settings.mode = 0;
	user_settings.precharge = 0;
	user_settings.interstimulus_interval = 30; // in ms
	user_settings.voltage_amplitude = 0.5; // 50% of Vmax=325 V
	
}


void init_GPIO_interrupt(void)
{
	PORTD.DIR = 0x00; // set PORTD as output
	PORTD.INT0MASK = 0b01111111; // enable pin 0 - 5 as interrupts
	PORTD.INTCTRL = 0b00001010; // 0000 RESERVED | 10 = INT1 MEDIUM LEVEL | 10 = INT0 MEDIUM LEVEL
	PORTD.PIN0CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN1CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN2CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN3CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN4CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN5CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
	PORTD.PIN6CTRL = 0b00011010; // 0 SLEWRATE | 0 INVERTED MODE | 011 PULLUP | 001 RISING
}

void interrupt_init(void)
{
	// The global interrupt enable bit must be set for interrupts to be enabled. If the global interrupt enable register is cleared,
	// none of the interrupts are enabled independent of the individual interrupt enable settings
	sei();

	// Interrupt initialisation
	/* Enable low-level interrupts. */
	PMIC.CTRL |= (1 << 0);


	/* Disable low-level interrupts. */
	// PMIC.CTRL &= ~(1 << 0);


	/* Enable medium-level interrupts. */
	PMIC.CTRL |= (1 << 1);


	/* Disable medium-level interrupts. */
	// PMIC.CTRL &= ~(1 << 1);


	/* Enable high-level interrupts. */
	PMIC.CTRL |= (1 << 2);


	/* Disable high-level interrupts. */
	// PMIC.CTRL &= ~(1 << 2);


	// /* Enable round-robin scheduling for low-level interrupts. */
	// PMIC.CTRL |= (1 << 7);
	//
	//
	// /* Disable round-robin scheduling for low-level interrupts. */
	// PMIC.CTRL &= ~(1 << 7);
}
