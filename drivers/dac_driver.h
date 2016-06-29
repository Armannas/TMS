/*
 * dac.h
 *
 * Created: 30-5-2016 11:55:56
 *  Author: Efraim
 */ 


#ifndef DAC_H_
#define DAC_H_

void init_DAC(void);
void write_DAC(uint16_t);
uint16_t voltage_to_DAC(double);
double voltage_to_vdac(double);


#endif /* DAC_H_ */