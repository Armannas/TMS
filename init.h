/*
 * init.h
 *
 * Created: 5/12/2016 10:01:22 PM
 *  Author: Cookie Factory
 */ 


#ifndef INIT_H_
#define INIT_H_

void init(void);
void interrupt_init(void);
void init_GPIO_interrupt(void);

enum State
{
	INIT,
	STANDBY,
	READ,
	WRITE,
	PRECHARGE,
	SINGLE_PULSE,
	PAIRED_PULSE,
	DISCHARGE,
} state;

struct Flags
{
	uint8_t READ_FLAG;
	uint8_t WRITE_FLAG;
	uint8_t PRECHARGE_FLAG;
	uint8_t OVERTEMP_FLAG;
	uint8_t HARDWARE_ERROR_FLAG;
	uint8_t DISCHARGED_FLAG;
	uint8_t CAPBANK_FLAG;
	} flag;

struct UserSettings
{
	uint8_t mode;
	uint8_t precharge;
	double interstimulus_interval;	
	double voltage_amplitude;
} user_settings;

struct safety_signals_struct
{
	uint16_t THYR1_TEMP; // in degree C
	uint16_t THYR2_TEMP; // in degree C
	uint16_t THYR3_TEMP; // in degree C
	uint16_t COIL_TEMP; // in degree C
	uint16_t CAPBANK1_TEMP; // in degree C
	uint16_t CAPBANK2_TEMP; // in degree C
	
	uint16_t THYR1_MAXTEMP;
	uint16_t THYR2_MAXTEMP;
	uint16_t THYR3_MAXTEMP;
	uint16_t COIL_MAXTEMP;
	uint16_t CAPBANK1_MAXTEMP;
	uint16_t CAPBANK2_MAXTEMP;
	
	uint8_t INTERLOCK_ERROR;
	uint8_t THYR1_TEMP_ERROR;
	uint8_t THYR2_TEMP_ERROR;
	uint8_t THYR3_TEMP_ERROR;
	uint8_t COIL_TEMP_ERROR;
	uint8_t CAPBANK1_TEMP_ERROR;
	uint8_t CAPBANK2_TEMP_ERROR;
	uint8_t VCAP1_ERROR;
	uint8_t VCAP2_ERROR;
	

	
} safety_signals;

#endif /* INIT_H_ */