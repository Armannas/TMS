/*
 * dac.c
 *
 * Created: 30-5-2016 11:55:45
 *  Author: Efraim
 */ 

#include <avr/io.h>
#include "dac_driver.h"

void init_DAC(void)
{
	DACA.CTRLA = 0b00010101; // RESERVED 000 | IDOEN = 1 INPUT FOR AC | CH1EN = 0 | CH0EN = 1 | LowPower MODE = 0 | ENABLE = 1
	DACA.CTRLB = 0b00000000; // RESERVED 0 | CHSEL = 00 | RESERVED 000 | CH1TRIG = 0 | CH0TRIG 1
	DACA.CTRLC = 0b00000000; // RESERVED 000 | REFSEL 00 (1V internal) | RESERVED 00 | LEFTADJ = 0
}

void write_DAC(uint16_t DAC_data)
{
	uint8_t DAC_data_4MSB;
	uint8_t DAC_data_8LSB;
	
	DAC_data_4MSB = (uint8_t)((DAC_data & 0xFF00) >> 8);
	DAC_data_8LSB = (uint8_t)(DAC_data & 0x00FF);
	
	while(DACA.STATUS == 0);
	// 	DACA.STATUS = 0x00;
	// 	DACA.CH0DATAH = DAC_data_4MSB;
	// 	DACA.CH0DATAL = DAC_data_8LSB;

	DACA.CH0DATAH = DAC_data_4MSB;
	DACA.CH0DATAL = DAC_data_8LSB;
	
	DACA.EVCTRL = 0b00001000;

}

uint16_t voltage_to_DAC(double V_DAC)
{
	double V_ref = 1; // Volts
	double ERROR_GAIN = 0.989; // GAIN error
	double ERROR_OFFSET = -0.003; // OFFSET error in Volts
	uint16_t ADC_range = 4096; // 12-bits
	uint16_t DAC_value = 0;
	
	
	DAC_value = ADC_range * ((V_DAC + ERROR_OFFSET) / (V_ref * ERROR_GAIN));
	
	return(DAC_value);
	
}

double voltage_to_vdac(double VOLTAGE)
{
		double V_DAC; // the voltage on the input of the DAC on the microcontroller
		double R_divider = 200.1; // the voltage divider ratio in hardware
		
		// VOLTAGE is the voltage on the highvoltage side (before the voltage divider) on caps
		
		V_DAC = VOLTAGE / R_divider; // V_out = V_in * R1/(R1+R2)
		
		return(V_DAC);
}

// double x 100; // volt
// write_DAC(voltage_to_DAC(voltage_to_vdac(x)));
