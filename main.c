/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


// define cpu speed
#include <asf.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include <interrupt.h>
#include <pulse.h>
#include "init.h"
#include "trigger.h"
#include <drivers/adc_driver.h>
#include <drivers/uart_driver.h>
#include <drivers/dac_driver.h>
#include <drivers/ac_driver.h>
#include <drivers/counter_driver.h>
#include <drivers/clock_32MHz_driver.h>



#define THYRISTOR1_ON	(PORTE.OUT |= (1<<7))
#define	THYRISTOR1_OFF	(PORTE.OUT &= ~(1 << 7))
#define THYRISTOR2_ON	(PORTE.OUT |= (1<<6))
#define	THYRISTOR2_OFF	(PORTE.OUT &= ~(1 << 6))		
#define THYRISTOR3_ON	(PORTE.OUT |= (1<<5))
#define	THYRISTOR3_OFF	(PORTE.OUT &= ~(1 << 5))
#define PRECHARGE1_ON	(PORTE.OUT |= (1<<4))
#define	PRECHARGE1_OFF	(PORTE.OUT &= ~(1 << 4))
#define PRECHARGE2_ON	(PORTE.OUT |= (1<<4))
#define	PRECHARGE2_OFF	(PORTE.OUT &= ~(1 << 4))
#define DISCHARGE1_ON	(PORTE.OUT |= (1<<4))
#define	DISCHARGE1_OFF	(PORTE.OUT &= ~(1 << 4))
#define DISCHARGE2_ON	(PORTE.OUT |= (1<<4))
#define	DISCHARGE2_OFF	(PORTE.OUT &= ~(1 << 4))
#define CAPBANK1		1
#define CAPBANK2		2

#define V_MAX			325

void LED_KIT(void);


volatile uint8_t ADCA_CH_COUNTER;
volatile uint8_t ADCB_CH_COUNTER;
struct Buffer UART_buffer_write = {6, 0, {0} }; // package size is 6 bytes.
struct Buffer UART_buffer_send    = {4, 0, {0} };
	
int main(void) {
	PORTE.DIRSET |= 0xFF;
	PORTD.DIRSET = 0x00;
	PORTD.PIN1CTRL = 0b00011010;
	PORTC.DIRSET |= 0x01;

	
	
  while(1)
  {
    /* Read the next incoming event. Usually this is a blocking function. */
   // EVENTS event = readEventFromMessageQueue();
	
	if((flag.HARDWARE_ERROR_FLAG == 1 || flag.OVERTEMP_FLAG == 1) && flag.DISCHARGED_FLAG == 0)
	{
		state = DISCHARGE;
	}   
	
	/* Finite State Machine */
	switch(state)
    {
		
	case INIT:
		init_clock();
		init();
		interrupt_init();
		UART_init();
		AC_init();
		init_DAC();
		init_GPIO_interrupt();
//		init_counter();
		
		// Start by using capbank 1
		flag.CAPBANK_FLAG = 0;
		PORTE.OUT = 0x00;
		//_delay_ms(4000);
		
		// comparator reference voltages
		
		// Desired voltage reference level expressed in Capacitor voltage
		// TODO: high_trigger_cap is based on user_settings.voltage_amplitude which is in percent but should be converted to absolute.

		double low_trigger_cap = 0.3;
		
		// Voltage divided, digital version of capacitor voltage
		uint16_t high_trigger_b;
		uint16_t low_trigger_b;

//		high_trigger_b = voltage_to_DAC(voltage_to_vdac(high_trigger_cap));


		low_trigger_b = voltage_to_DAC(low_trigger_cap);
		
		state = STANDBY;
        break;

	case STANDBY:
		PORTE.OUT = 0x01;

		if (flag.PRECHARGE_FLAG)
		{
			state = PRECHARGE;
// 			PORTE.OUT = 0x04;
// 			_delay_ms(3000);
			break;
		}
		
		else if(flag.WRITE_FLAG)
		{
			state = WRITE;
			//delay_ms(3000);
			break;
		}
		
		else if(flag.READ_FLAG)
		{
			state = READ;
			// PORTE.OUT = 0x05;
			//_delay_ms(3000);
			break;
		}
		
		else if(!(PORTD.IN & (1 << 1)))
		{
			PORTE.OUT = 0x06;
			//_delay_ms(3000);
			if(user_settings.mode == 0)
			{
				state = SINGLE_PULSE;
				break;
			}
			else if(user_settings.mode == 1)
			{
				state = PAIRED_PULSE;
				break;
			}
		}
		
		else
		{
			state = STANDBY;
			break;
		}
		break;
		
	case PRECHARGE:
		// Clear precharge flag
		flag.PRECHARGE_FLAG = 0;
		flag.DISCHARGED_FLAG = 0;
		
		PORTE.OUT = 0x03;
		//_delay_ms(3000);
		// Set AC trigger high
//		write_DAC(high_trigger_b);
		write_DAC(voltage_to_DAC(voltage_to_vdac(user_settings.voltage_amplitude * V_MAX)));	
		
			// Go to pulse state, single or paired depending on user settings
			if (user_settings.mode == 0) // single pulse mode
			{
				// Capbank 1
				if (flag.CAPBANK_FLAG == 0)
				{
					// Close precharge switch Capbank 1
					PRECHARGE1_ON;
					PORTC.OUT |= (1 << 0);
					// check if precharge reference triggered
					if(ACA.STATUS & (1 << 4) )
					{
						// TODO: Open precharge switch
						PRECHARGE1_OFF;
						PORTC.OUT &= ~(1 << 0);
						PORTE.OUT = 0x04;
						//_delay_ms(3000);
						
						// Send precharge ready ACK
						USARTF0.DATA = 0x06;
						
						state = STANDBY;
						break;
					}
				}
				
				// Capbank 2
				else if (flag.CAPBANK_FLAG == 1)
				{
					// Close precharge switch Capbank 2
					PRECHARGE2_ON;
					//PORTC.OUT |= (1 << 0);
					// check if precharge reference triggered
//					if(ACA.STATUS & (1 << 5) )
					if(ACA.STATUS & (1 << 4) )
					{
						// TODO: Open precharge switch
						PRECHARGE2_OFF;
						//PORTC.OUT &= ~(1 << 0);
						PORTE.OUT = 0x04;
						//_delay_ms(3000);
						
						// Send precharge ready ACK
						USARTF0.DATA = 0x06;
						
						state = STANDBY;
					}
				}
			}
			
			else if (user_settings.mode == 1) // paired pulse mode
			{
				PRECHARGE1_ON;
				//PORTC.OUT |= (1 << 0);
				// check if precharge reference capbank 1 triggered
				if (ACA.STATUS & (1 << 4) )
				{
					// Open precharge switch capbank 1
					PRECHARGE1_OFF;
					//PORTC.OUT &= ~(1 << 0);
					PORTE.OUT = 0x04;
					//_delay_ms(1500);
					
					PRECHARGE2_ON;
					//PORTC.OUT |= (1 << 0);
					// check if precharge reference capbank 2 triggered
//					if (ACA.STATUS & (1 << 5) )
					if (ACA.STATUS & (1 << 4) )
					{
						// Open precharge switch capbank 2
						PRECHARGE2_OFF;
						//PORTC.OUT &= ~(1 << 0);
						PORTE.OUT = 0x05;
						//_delay_ms(1500);
						
						// Send precharge ready ACK
						USARTF0.DATA = 0x06;
						state = STANDBY;
					}	
				}
			}
		
		else
		{
			PORTE.OUT = 0x07;
			//_delay_ms(1500);
			// stay in precharge if trigger level not reached
			state = PRECHARGE;
		}

		break;

	case READ:
		//_delay_ms(3000);
		PORTE.OUT = 0x03;
		// read stuff here
		// TODO: read not possible yet
		
		state = STANDBY;
		break;
		
	case WRITE:
		PORTE.OUT = 0x02;
		// write stuff here
		bufferWrite(&UART_buffer_write, USARTF0.DATA);
		// Get  char from RX buffer
		flag.WRITE_FLAG = 0;
		//_delay_ms(1500);
		
		state = STANDBY;
		break;


		
	case SINGLE_PULSE:
		// set reference for AC with DAC
		write_DAC(low_trigger_b);
		
		// single pulse here
		PORTE.OUT = 0x05;
		//_delay_ms(3000);
		

		if (flag.CAPBANK_FLAG == 0)
		{
			// Pulse using capbank 1
			single_pulse(CAPBANK1);
			// TODO: Wait for pulse to damp out
			_delay_ms(1);
		}
		
		else
		{
			// Pulse using capbank 2
			single_pulse(CAPBANK2);
			// TODO: Wait for pulse to damp out
			_delay_ms(1);
		}
		
		
		// Toggle capbank used
		flag.CAPBANK_FLAG ^= (1 << 0);
		break;
		
	case PAIRED_PULSE:
		// paired pulse here
		PORTE.OUT = 0x05;
		//_delay_ms(3000);
		// set reference for AC with DAC
		write_DAC(low_trigger_b);
		state = STANDBY;
		// Pulse using capbank 1
		single_pulse(CAPBANK1);
		
// 		_delay_us(50);
		delay_in_ms(user_settings.interstimulus_interval);
		// Pulse using capbank 2
		single_pulse(CAPBANK2);
		_delay_us(50);
		// TODO: Wait for pulse to damp out
		//_delay_ms(1);
		// When finished, go to precharge state again to wait for new pulse
		state = STANDBY;
		break;
		
	case DISCHARGE:
	PORTE.OUT = 0x08;
	//_delay_ms(3000);
	// set reference for AC with DAC
	write_DAC(low_trigger_b);
	DISCHARGE1_ON;
	DISCHARGE2_ON;
	
	// Discharge complete when both capbank voltages below AC reference
	//		if ( (ACA.STATUS & (1 << 4)) && (ACA.STATUS & (1 << 5)) )
	if ( !(ACA.STATUS & (1 << 4)))

	{
		DISCHARGE1_OFF;
		DISCHARGE2_OFF;
		flag.DISCHARGED_FLAG = 1;
		flag.HARDWARE_ERROR_FLAG = 0;
		flag.OVERTEMP_FLAG = 0;
// 		_delay_ms(5000);
		state = STANDBY;
	}
	
	else
	{
		// Keep discharging
		state = DISCHARGE;
	}
	break;

    }
  } 
}

	/* ********UART SANDBOX******** */

// 	ring buffer initialization
		// struct ringBuffer Rbuffer = {89,54,65};
		
// 		// LEDs
/*		PORTE.DIRSET |= 0xFF;*/
// 		
// 		uint8_t sendData = 255;
// 		uint8_t sendArray[NUM_BYTES] = {'w', '1','2', '3', 'X', 'X', 'X', 'X'};
// 		/*! Array to put received data in. */
// 		// uint8_t receiveArray[NUM_BYTES];
//  		while(1)
//  		{
// 			 if (ACA.STATUS & (1 << 4))
// 			 {
// 				 PORTE.OUT = 0x01;
// 			 }
// 			 
// 			 else
// 			 {
// 				 PORTE.OUT = 0XFF;
// 			 }
//  		USARTF0.DATA = 0x03;
// 			uint8_t i;
// 			for (i =0; i < NUM_BYTES; i++)
// 			{
// 				// sendData--;
// 				// The DREIF flag is one when the transmit
// 				// buffer is empty and zero when the transmit buffer contains data to be transmitted that has not yet been moved into the
// 				// shift register
// 				while ( !( USARTF0.STATUS & (1 << 5)) );
// 				// Put char in TX buffer
// 				
// // 				USARTF0.DATA = sendArray[i];
 		//USARTF0.DATA = 0x03;
// 
// 
//  		}
// 		}


void LED_KIT(void)
{

	uint16_t LED_KIT_TIME = 500;
	PORTE.OUT = 0b11101110;
	_delay_ms(LED_KIT_TIME);
	PORTE.OUT = 0b11011101;
	_delay_ms(LED_KIT_TIME);
	PORTE.OUT = 0b10111011;
	_delay_ms(LED_KIT_TIME);
	PORTE.OUT = 0b01110111;
	_delay_ms(LED_KIT_TIME);
	PORTE.OUT = 0b10111011;
	_delay_ms(LED_KIT_TIME);
	PORTE.OUT = 0b11011101;
	_delay_ms(LED_KIT_TIME);


	
	// 			PORTE_OUT = 0xFF;
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 0);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 1);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 2);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 3);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 4);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 5);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 6);
	// 			_delay_ms(250);
	// 			PORTE.OUT &= ~(1 << 7);
	// 			_delay_ms(250);
}

// Receive complete interrupt service routine
ISR(USARTF0_RXC_vect)

{
	
	// Notify main that write request has been recieved
	flag.WRITE_FLAG = 1;
}


ISR(TCC0_OVF_vect)
{
	
	// after the clock has reached his maximum value it will start the ADC channels 0-2 for ADCA and channels 0-3 for ADCB
	
	if(ADCA_CH_COUNTER < 3)
	{
		start_ADC('A',ADCA_CH_COUNTER);
		ADCA_CH_COUNTER++;
	}
	else if((ADCA_CH_COUNTER == 3) & (ADCB_CH_COUNTER < 3))
	{
		start_ADC('B',ADCB_CH_COUNTER);
		ADCB_CH_COUNTER++;
	}
	else
	{
		ADCA_CH_COUNTER = 0;
		ADCB_CH_COUNTER = 0;
		
		start_ADC('A',ADCA_CH_COUNTER);
		ADCA_CH_COUNTER = 1;
	}
	
}

// The ADC ISRs measure the temperatures and store it in the safety_signals struct. It then compares if it exceeds the maximum value.
// If so, the overtemperature flag will go high.

ISR(ADCA_CH0_vect)
{
	safety_signals.COIL_TEMP = ADCA.CH0RES;
	if(safety_signals.COIL_TEMP >= safety_signals.COIL_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCA_CH1_vect)
{
	safety_signals.THYR3_TEMP = ADCA.CH1RES;
	if(safety_signals.THYR3_TEMP >= safety_signals.THYR3_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCA_CH2_vect)
{
	safety_signals.THYR2_TEMP = ADCA.CH2RES;
	if(safety_signals.THYR2_TEMP >= safety_signals.THYR2_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCA_CH3_vect)
{
	
}

ISR(ADCB_CH0_vect)
{
	safety_signals.CAPBANK1_TEMP = ADCB.CH0RES;
	if(safety_signals.CAPBANK1_TEMP >= safety_signals.CAPBANK1_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCB_CH1_vect)
{
	safety_signals.CAPBANK2_TEMP = ADCB.CH1RES;
	if(safety_signals.CAPBANK2_TEMP >= safety_signals.CAPBANK2_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCB_CH2_vect)
{
	safety_signals.THYR1_TEMP = ADCA.CH2RES;
	if(safety_signals.THYR1_TEMP >= safety_signals.THYR1_MAXTEMP)
	{
		flag.OVERTEMP_FLAG = 1;
	}
}

ISR(ADCB_CH3_vect)
{
	
}

ISR(PORTD_INT0_vect)
{

// 	if(PORTE.OUT == 0xFF)
// 	{
// 		PORTE.OUT = 0x00;
// 	}
// 	else
// 	{
// 		PORTE.OUT = 0xFF;
// 	}

// Check which safety signal has an error

if(!(PORTD.IN & (1 << 2)))
{
	safety_signals.THYR3_TEMP_ERROR = 1;
	flag.HARDWARE_ERROR_FLAG = 1;
}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.THYR2_TEMP_ERROR = 1;
// 	}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.THYR1_TEMP_ERROR = 1;
// 	}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.CAPBANK1_TEMP_ERROR = 1;
// 	}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.CAPBANK2_TEMP_ERROR = 1;
// 	}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.VCAP1_ERROR = 1;
// 	}
// 	else if(!(PORTD.IN & (1 << 2)))
// 	{
// 		safety_signals.VCAP2_ERROR = 1;
// 	}

// 	else
// 	{
// 		flag.HARDWARE_ERROR_FLAG = 1;
//
// 	}
}

// ISR(USARTF0_TXC_vect)
// 
// {
// 	 // Get  char from RX buffer
// 	 //PORTE.OUT = ~USARTF0.DATA;
// 	 _delay_ms(500);
// }

// 	single_pulse();
// 	init_ADC();
// 	uint8_t result;
// 	PORTE.OUT |= 0xFF;
// 	
// 	while(1)
// 	{
// 		result = read_ADC();
// 		
// 		PORTE.OUT &= ~result;
// 		_delay_ms(250);
// 		PORTE.OUT = 0xFF;
// 	}




