#include <asf.h>
#include <avr/io.h>
#include <pulse.h>

#define F_CPU 32000000UL
#include <util/delay.h>
#include "init.h"
#include <drivers/counter_driver.h>
#define THYRISTOR1_ON	(PORTE.OUT |= (1<<7))
#define	THYRISTOR1_OFF	(PORTE.OUT &= ~(1 << 7))
#define THYRISTOR2_ON	(PORTE.OUT |= (1<<6))
#define	THYRISTOR2_OFF	(PORTE.OUT &= ~(1 << 6))
#define THYRISTOR3_ON	(PORTE.OUT |= (1<<5))
#define	THYRISTOR3_OFF	(PORTE.OUT &= ~(1 << 5))

void single_pulse(uint8_t capBank)
{
	if (capBank == 1)
	{
		// Disable all interrupts to focus on low trigger
		cli();
		
		// Close Thyristor 1 driver : Discharge cap and charge coil
		THYRISTOR1_ON;
		//_delay_ms(2000);
//		start_counter_TCC1(62500); // 2s
		
		while (ACA.STATUS & (1 << 4) ) 
		{
			// Thyristor watchdog timer, prevent thyristor driver from staying on indefinitely
			if (TCC1.INTFLAGS & (1 << 0) || flag.HARDWARE_ERROR_FLAG == 1 || flag.OVERTEMP_FLAG == 1)
			{
				PORTE.OUT = 0x07;
				//_delay_ms(2000);
				// Reset
				TCC1.INTFLAGS |= (1 << 0);
				
				state = DISCHARGE;
				break;
				
			}
			
		}
		
		// TODO: Open Thyristor 1 driver
		THYRISTOR1_OFF;
		//PORTC.OUT &= ~(1 << 0);
		
		// TODO: Close thyristor 3 driver : Flyback coil current
		THYRISTOR3_ON;
		PORTC.OUT |= (1 << 0);
		//_delay_ms(2000);
		// TODO: Open thyristor 3 driver after some short time
		_delay_us(50);
		THYRISTOR3_OFF;
		PORTC.OUT &= ~(1 << 0);
		//_delay_ms(2000);
		
		// Re-enable all interrupts
		sei();
	}
	
	else if(capBank == 2)
	{

		// Disable all interrupts to focus on low trigger
		cli();
		// Close Thyristor 2 driver : Discharge cap and charge coil
		THYRISTOR2_ON;
//		start_counter_TCC1(62500); // 2s
		//_delay_ms(3000);

		while (ACA.STATUS & (1 << 4) )
		{
			
			// Thyristor watchdog timer, prevent thyristor driver from staying on indefinitely
			if (TCC1.INTFLAGS & (1 << 0) || flag.HARDWARE_ERROR_FLAG == 1 || flag.OVERTEMP_FLAG == 1)
			{
				PORTE.OUT = 0x06;
				//_delay_ms(2000);
				
				// Reset
				TCC1.INTFLAGS |= (1 << 0);
				
				state = DISCHARGE;
				break;
				
			}
			
			
		}
		
		// TODO: Open Thyristor 2 driver
		THYRISTOR2_OFF;
		//PORTC.OUT &= ~(1 << 0);
		//_delay_ms(2000);
		// TODO: Close thyristor 3 driver : Flyback coil current
		THYRISTOR3_ON;
		PORTC.OUT |= (1 << 0);
		//_delay_ms(2000);
		
		// TODO: Open thyristor 3 driver after some short time
		_delay_us(50);
		THYRISTOR3_OFF;
		PORTC.OUT &= ~(1 << 0);
		//_delay_ms(2000);
		// Re-enable all interrupts
		sei();		
	}

}

// 		double x = user_settings.interstimulus_interval*20;
// 			for (double i = 0; i < x; i++)
// 			{
// 				_delay_us(50);
// 			}	
void delay_in_ms( double ms )
{
	for (double i = 0; i < ms; i++)
	{
		_delay_us(100);
	}
}
// 
// void paired_pulse(uint8_t ISI)
// {
// 	// Set PD1 as output
// 	PORTD.DIRSET = 0x02;
// 	
// 	while(1) {
// 		_delay_us(50);
// 		
// 		// First pulse using PD1
// 		PORTD.OUT = 0x02;
// 		_delay_us(50);
// 		PORTD.OUT = 0x00;
// 		
// 		//Interstimulus Interval 
// 		//TODO: Fix Error __builtin_avr_delay_cycles expects a compile time integer constant
// 		// Can't use variable as input to _delay functions
// 		//_delay_ms(ISI);
// 		
// 		// Second pulse using PD1
// 		PORTD.OUT = 0x02;
// 		_delay_us(50);
// 		PORTD.OUT = 0x00;
// 	}
// }
// 
// void repetitive_pulse(uint8_t stimulation_frequency)
// {
// 	// Set PD1 as output
// 	PORTD.DIRSET = 0x02;
// 	
// 	while(1) {
// 	_delay_us(50);
// 	
// 	//Set PD1 high
// 	PORTD.OUT = 0x02;
// 	_delay_us(50);
// 	//Set PD1 low
// 	PORTD.OUT = 0x00;
// 	}
// }
