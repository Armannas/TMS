/*
 * trigger.c
 *
 * Created: 6/21/2016 10:30:34 PM
 *  Author: Cookie Factory
 */ 

#include <asf.h>
#include <drivers/ac_driver.h>
#include <drivers/dac_driver.h>
#include "trigger.h"

void setTriggerHigh(double high_trigger_AC)
{
	// Divide desired voltage reference level expressed in capacitor voltage based on voltage divider, convert to binary and write to DAC.
	// write_DAC(voltage_to_DAC(voltage_to_vdac(high_trigger_cap)));

}

void setTriggerLow(double low_trigger_AC)
{
	// Divide desired voltage reference level expressed in capacitor voltage based on voltage divider, convert to binary and write to DAC.
	// write_DAC(voltage_to_DAC(voltage_to_vdac(low_trigger_cap)));

}