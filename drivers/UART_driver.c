/*
 * UART_driver.c
 *
 * Created: 5/12/2016 9:58:32 PM
 *  Author: Cookie Factory
 */ 

#include <avr/io.h>
#include <drivers/uart_driver.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include <init.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void UART_init(void)
{
	PORTF.DIRSET |= (1 << 3);   // TX pin as output
	PORTF.OUTSET   |= (1 << 3);   // TX pin high
	
	// PORTF.DIRSET &= ~0x02; // RX pin as input
	// PORTF.OUT    &= ~0x02; // RX pin initialized with low
	

	
	// Set modes
	USARTF0.CTRLA = 0;                // Geen usart interrupts.
	USARTF0.CTRLC = 0x03;            // async, no parity, 8 bit data, 1 stop bit.
	
	// Baudrate selection using http://www.avrcalc.elektronik-projekt.de/xmega/baud_rate_calculator
	// BSCALE = -4
	// BSEL = 3317
	USARTF0.BAUDCTRLB = 0b11001100;
	USARTF0.BAUDCTRLA = 0b11110101;
	
	// Enable Tx
	USARTF0.CTRLB |= (1 << 3);
	
	// Enable Rx
	USARTF0.CTRLB |= (1 << 4);
	
	// Enable the receive complete interrupt as medium priority
	USARTF0.CTRLA &= !(1 << 4);
	USARTF0.CTRLA |= (1 << 5);
}

void bufferWrite(struct Buffer *buffer, uint8_t writeData)
{
		if ( (buffer->bufferSize) - (buffer->counter+1) == 0) 
		{   
			// Message must end with newline character
			if (writeData == '\n')
			{
				buffer->bufferData[buffer->counter] = writeData;
				buffer->counter = 0;

				// Decode message
				messageDecode(buffer->bufferData);
			}
			
			else
			{
				// send NAK
				USARTF0.DATA = 0x15;
				buffer->counter = 0;
			}
			
		}
		
		else
		{
			buffer->bufferData[buffer->counter] = writeData;	//store data to buffer
			buffer->counter++;
			_delay_ms(20);
			USARTF0.DATA = 0x04;
			
		}
}

void bufferRead(struct Buffer *buffer, uint8_t readData)
{
	if ( (buffer->bufferSize) - (buffer->counter+1) == 0)
	{
		buffer->bufferData[buffer->counter] = readData;
		
		// last two elements (4 and 5) were already zero. Are going to be ignored
		buffer->counter = 0;

		// Encode message
		messageEncode(buffer->bufferData);
	}
	
	else
	{
		buffer->bufferData[buffer->counter] = readData;	//store data to buffer
		buffer->counter++;
	}
}

void messageEncode(char message[])
{
	if( (message[1] == 'M') & (message[2] =='D') )
	{
		// Send ACK
		USARTF0.DATA = 0x06;
		USARTF0.DATA = user_settings.mode;

	}
	
	// Interstimulus Interval
	else if( (message[1] == 'I') & (message[2] =='I') )
	{
		// Send ACK
		USARTF0.DATA = 0x06;
		USARTF0.DATA = user_settings.interstimulus_interval;

		
	}
	
	// Voltage amplitude
	else if( (message[1] == 'V') & (message[2] =='A') )
	{
		// Send ACK
		USARTF0.DATA = 0x06;
		USARTF0.DATA = user_settings.voltage_amplitude;

	}
	
	else
	{
		//Invalid command
		USARTF0.DATA = 0x15;
		// send value to user

	}
	return;
}


void messageDecode(char message[])
{
	// W VA 20 \n
	// First  character denotes write
	// Second and third charater denote parameter
	// Fourth and fifth character denote value
	// Sixth denotes end of line, represented by newline character '\n'
	// stringparam is the value part of the message as char's
	// valueparam is the value part of the message as int
	
	char message_param[2];
	char message_value_string[2];
	uint8_t message_value_int;
	// uint8_t valueparam1; // First 8 bits of valueparam
	// uint8_t valueparam2; // Second 8 bits of valueparam
	
	// parameter array
	message_param[0] = message[1];
	message_param[1] = message[2];
	
	// value array
	message_value_string[0] = message[3];
	message_value_string[1] = message[4];

	
	// Convert value string to value int i.e the string "15" becomes the int 15
	message_value_int = atoi(message_value_string);

	// Split 16 bit number in two 8 bits to validate result over UART
// 	valueparam2 = (uint8_t)((valueparam & 0xFF00) >> 8);
// 	valueparam1 = (uint8_t)(valueparam & 0x00FF);
	
// 	USARTF0.DATA = valueparam2;
// 	_delay_ms(1000);
// 	USARTF0.DATA = valueparam1;
	
	// USARTF0.DATA = message_value_int;
	
	messageValidate(message_param, message_value_int);
	return;
}

// 
void messageValidate(char message_param[], uint8_t message_value_int)
{
	// Mode (Single, Paired, Repetitive)
	if( (message_param[0] == 'M') && (message_param[1] =='D') )
	{
		// Check if value is within accepted range
		if ( message_value_int >= 0 && message_value_int <=2 )
 		{
 			user_settings.mode = message_value_int;
			
			// Send ACK
			USARTF0.DATA = 0x06;
			
 		}
		 
		else
		{
			// send NAK
			USARTF0.DATA = 0x15;
			
			// send value
			// USARTF0.DATA = message_value_int;
		}
	}
	
	// Interstimulus Interval
	else if( (message_param[0] == 'I') && (message_param[1] =='I') )
	{
		// Check if value is within accepted range (1.5 - 6.0 ms)
 		if ( message_value_int >=15 && message_value_int <= 60 )
		{
 			user_settings.interstimulus_interval = (double) (message_value_int);
			 
			// Send ACK
			USARTF0.DATA = 0x06;
 		}
		
		else
		{
			// send NAK + invalid parameter
			USARTF0.DATA = 0x15;
			_delay_ms(100);
			USARTF0.DATA = message_param[0];
			_delay_ms(100);
			USARTF0.DATA = message_param[1];
		}
	}
	
	// Voltage amplitude
	else if( (message_param[0] == 'V') && (message_param[1] =='A') )
	{
		// Check if value is within accepted range ( 0% - 99%)
 		if ( message_value_int >= 0 && message_value_int <= 99 )
 		{
			user_settings.voltage_amplitude = message_value_int / 100.0;
			
			// Send ACK
			USARTF0.DATA = 0x06;
		}
	
		else
		{
			// send NAK
			USARTF0.DATA = 0x15;
			
			// send value
			// USARTF0.DATA = message_value_int;
		}
	}
	
	else if( (message_param[0] == 'P') && (message_param[1] =='R') )
	{
		// Check if value is within accepted range
		if ( message_value_int == 0 || message_value_int == 1 )
		{
			user_settings.precharge = message_value_int;
			
			// Just for the consistency of using the flags struct
			flag.PRECHARGE_FLAG = user_settings.precharge;
			// Send ACK
			USARTF0.DATA = 0x06;
		}
	
		else
		{
		// send NAK
		USARTF0.DATA = 0x15;
		_delay_ms(100);
		// send value
		USARTF0.DATA = message_value_int;
		}
	}
	
	else
	{	
		// send NAK + invalid parameter
		USARTF0.DATA = 0x15;
		_delay_ms(100);
		USARTF0.DATA = message_param[0];
		_delay_ms(100);
		USARTF0.DATA = message_param[1];
	}
}