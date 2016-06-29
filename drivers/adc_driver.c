/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      XMEGA ADC driver source file.
 *
 *      This file contains the function implementations the XMEGA ADC driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the XMEGA ADC module.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *      Several functions use the following construct:
 *          "some_register = ... | (some_parameter ? SOME_BIT_bm : 0) | ..."
 *      Although the use of the ternary operator ( if ? then : else ) is discouraged,
 *      in some occasions the operator makes it possible to write pretty clean and
 *      neat code. In this driver, the construct is used to set or not set a
 *      configuration bit based on a boolean input parameter, such as
 *      the "some_parameter" in the example above.
 *
 * \par Application note:
 *      AVR1300: Using the XMEGA ADC
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 2564 $
 * $Date: 2009-07-06 17:45:56 +0200 (ma, 06 jul 2009) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "adc_driver.h"
#include <asf.h>
#include <avr/io.h>
void init_ADC(void)
{
	/* Move stored calibration values to ADC B and ADC A */
	ADC_CalibrationValues_Load(&ADCB);
	ADC_CalibrationValues_Load(&ADCA);
	ADCA.CTRLA = 0x01; // Enable ADCA
	ADCA.CTRLB = ADC_RESOLUTION_8BIT_gc; // Set up ADC to have signed conversion mode and 8 bit resolution.
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc; // Sample rate is CPUFREQ/512
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc; // REF - Internal 1V
	ADCA.INTFLAGS = 0x0F; // Clear interrupt flags. First 4 bits need to be 0, last 4 bits are 1 (reset)
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCA.CH2.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCA.CH3.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	
	uint8_t ADCA_pin_number_CH0 = 0x09;
	uint8_t ADCA_pin_number_CH1 = 0x09;
	uint8_t ADCA_pin_number_CH2 = 0x09;
	uint8_t ADCA_pin_number_CH3 = 0x09;
	
	ADCA.CH0.MUXCTRL = (ADCA_pin_number_CH0 << 3); // MUX the requested pin number
	ADCA.CH1.MUXCTRL = (ADCA_pin_number_CH1 << 3); // MUX the requested pin number
	ADCA.CH2.MUXCTRL = (ADCA_pin_number_CH2 << 3); // MUX the requested pin number
	ADCA.CH3.MUXCTRL = (ADCA_pin_number_CH3 << 3); // MUX the requested pin number
	
	
	ADCB.CTRLA = 0x01; // Enable ADCB
	ADCB.CTRLB = ADC_RESOLUTION_8BIT_gc; // Set up ADC to have signed conversion mode and 8 bit resolution.
	ADCB.PRESCALER = ADC_PRESCALER_DIV512_gc; // Sample rate is CPUFREQ/512
	ADCB.REFCTRL = ADC_REFSEL_INT1V_gc; // REF - Internal 1V
	ADCB.INTFLAGS = 0x0F; // Clear interrupt flags. First 4 bits need to be 0, last 4 bits are 1 (reset)
	ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCB.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCB.CH2.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	ADCB.CH3.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; 	// single ended
	
	uint8_t ADCB_pin_number_CH0 = 1;
	uint8_t ADCB_pin_number_CH1 = 1;
	uint8_t ADCB_pin_number_CH2 = 1;
	uint8_t ADCB_pin_number_CH3 = 1;
	
	ADCB.CH0.MUXCTRL = (ADCB_pin_number_CH0 << 3); // MUX the requested pin number
	ADCB.CH1.MUXCTRL = (ADCB_pin_number_CH1 << 3); // MUX the requested pin number
	ADCB.CH2.MUXCTRL = (ADCB_pin_number_CH2 << 3); // MUX the requested pin number
	ADCB.CH3.MUXCTRL = (ADCB_pin_number_CH3 << 3); // MUX the requested pin number
}


uint8_t read_ADC(char PORT_letter, uint8_t CH_number)
{
	uint8_t ch_data = 0x00;


	
	if(PORT_letter == 'A')
	{
		switch (CH_number)
		{
			case 0:
			ADCA.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
			while(ADCA.INTFLAGS == 0); // check if conversion is completed
			ADCA.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCA.CH0RES;
			break;
			case 1:
			ADCA.CH1.CTRL |= ADC_CH_START_bm; // start conversion on channel 1
			while(ADCA.INTFLAGS == 0); // check if conversion is completed
			ADCA.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCA.CH1RES;
			break;
			case 2:
			ADCA.CH2.CTRL |= ADC_CH_START_bm; // start conversion on channel 2
			while(ADCA.INTFLAGS == 0); // check if conversion is completed
			ADCA.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCA.CH2RES;
			break;
			case 3:
			ADCA.CH3.CTRL |= ADC_CH_START_bm; // start conversion on channel 3
			while(ADCA.INTFLAGS == 0); // check if conversion is completed
			ADCA.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCA.CH3RES;
			break;
		}
	}
	else if(PORT_letter == 'B')
	{
		switch (CH_number)
		{
			case 0:
			ADCB.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
			while(ADCB.INTFLAGS == 0); // check if conversion is completed
			ADCB.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCB.CH0RES;
			break;
			case 1:
			ADCB.CH1.CTRL |= ADC_CH_START_bm; // start conversion on channel 1
			while(ADCB.INTFLAGS == 0); // check if conversion is completed
			ADCB.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCB.CH1RES;
			break;
			case 2:
			ADCB.CH2.CTRL |= ADC_CH_START_bm; // start conversion on channel 2
			while(ADCB.INTFLAGS == 0); // check if conversion is completed
			ADCB.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCB.CH2RES;
			break;
			case 3:
			ADCB.CH3.CTRL |= ADC_CH_START_bm; // start conversion on channel 3
			while(ADCB.INTFLAGS == 0); // check if conversion is completed
			ADCB.INTFLAGS = 0x0F; // reset flags
			ch_data = ADCB.CH3RES;
			break;
		}
	}
	

	
	return(ch_data);
}

void start_ADC(char PORT_letter, uint8_t CH_number)
{
	
	if(PORT_letter == 'A')
	{
		switch (CH_number)
		{
			case 0:
			ADCA.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
			break;
			case 1:
			ADCA.CH1.CTRL |= ADC_CH_START_bm; // start conversion on channel 1
			break;
			case 2:
			ADCA.CH2.CTRL |= ADC_CH_START_bm; // start conversion on channel 2
			break;
			case 3:
			ADCA.CH3.CTRL |= ADC_CH_START_bm; // start conversion on channel 3
			break;
		}
	}
	else if(PORT_letter == 'B')
	{
		switch (CH_number)
		{
			case 0:
			ADCB.CH0.CTRL |= ADC_CH_START_bm; // start conversion on channel 0
			break;
			case 1:
			ADCB.CH1.CTRL |= ADC_CH_START_bm; // start conversion on channel 1
			break;
			case 2:
			ADCB.CH2.CTRL |= ADC_CH_START_bm; // start conversion on channel 2
			break;
			case 3:
			ADCB.CH3.CTRL |= ADC_CH_START_bm; // start conversion on channel 3
			break;
		}
	}
}

double ADC_to_voltage(uint16_t ADC_value)
{
	uint16_t ADC_OFFSET = 200; // delta_V = V_ref x 0.05 = V_ref x 200/4095
	uint16_t ADC_range = 4095; // 12-bits
	double V_ref = 2.048; // Volts
	double V_ADC = 0;
	
	// 0V = 200
	// V_ref = 4095
	if(ADC_value < (ADC_OFFSET - 1))
	{
		ADC_value = 0;
	}
	else
	{
		ADC_value = ADC_value - ADC_OFFSET;
	}
	
	V_ADC = (ADC_value / ADC_range) * V_ref;
	
	return(V_ADC);
	
}

/*! \brief This function get the calibration data from the production calibration.
 *
 *  The calibration data is loaded from flash and stored in the calibration
 *  register. The calibration data reduces the non-linearity error in the adc.
 *
 *  \param  adc          Pointer to ADC module register section.
 */
void ADC_CalibrationValues_Load(ADC_t * adc)
{
	if (&ADCA == adc) {
		/* Get ADCACAL0 from production signature . */
		adc->CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL0_offset );
		adc->CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCACAL1_offset );
	} else {
		/* Get ADCBCAL0 from production signature  */
		adc->CALL = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCBCAL0_offset );
		adc->CALH = SP_ReadCalibrationByte( PROD_SIGNATURES_START + ADCBCAL1_offset );
	}
}


uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}
