/*
 * sensors.c
 *
 * Created: 30-5-2016 11:57:17
 *  Author: Efraim
 */ 

#include <avr/io.h>
#include "sensors.h"
#include <drivers/adc_driver.h>

double calc_voltage(char PORT_letter, uint8_t CH_number)
{
	double VOLTAGE;
	double ADC_data_V;
	uint16_t ADC_data;
	double R_divider = 200.1;

	
	ADC_data = read_ADC(PORT_letter, CH_number); // in bits
	ADC_data_V = ADC_to_voltage(ADC_data); // convert ADC data in bits to voltage
	
	VOLTAGE = ADC_data_V * R_divider; // V_out = V_in * R1/(R1+R2)
	
	return(VOLTAGE);
	
}

double calc_current(char PORT_letter, uint8_t CH_number)
{
	double CURRENT;
	double ADC_data_V;
	uint16_t ADC_data;
	uint16_t R_shunt = 1;
	
	
	ADC_data = read_ADC(PORT_letter, CH_number); // in bits
	ADC_data_V = ADC_to_voltage(ADC_data); // convert ADC data in bits to voltage
	
	CURRENT = ADC_data_V / R_shunt; // I = V/R
	
	return(CURRENT);
}

double calc_temp(char PORT_letter, uint8_t CH_number)
{
	double TEMPERATURE;
	double ADC_data_V;
	uint16_t ADC_data;
	
	ADC_data = read_ADC(PORT_letter, CH_number); // in 16bit
	ADC_data_V = ADC_to_voltage(ADC_data); // convert ADC data in bits to voltage
	
	TEMPERATURE = (ADC_data_V - 0.424) / 0.0065; // V_adc = T*6.5m + 0.424 ===> T = (V_adc - 0.424) / 6.5m
	
	return(TEMPERATURE);
}
