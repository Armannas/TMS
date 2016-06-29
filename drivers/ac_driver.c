/*
 * ac_driver.c
 *
 * Created: 6/21/2016 11:35:57 AM
 *  Author: Cookie Factory
 */ 
#include <asf.h>
#include "ac_driver.h"
void AC_init(void)
{
	/* Enable Analog comparator A0 and A1. */
	ACA.AC0CTRL = (1 << 3) |(1 << 2) | (1 << 0); // HSMODE = 1 | HYSMODE = 10 (HIGH) | ENABLE = 1
	ACA.AC1CTRL = (1 << 3) |(1 << 2) | (1 << 0); // HSMODE = 1 | HYSMODE = 10 (HIGH) | ENABLE = 1
	ACA.CTRLA &= 0x00;
	/* Set up MUXes to sense pin 4(A0) and 5(A1). */
//	ACA.AC0MUXCTRL = (1 << 5) | (1 << 2) |(1 << 0);  // MUXPOS[2:0] = 100 (PIN 4) | MUXNEG[2:0] = 101 (DAC)
	ACA.AC0MUXCTRL = (1 << 3) | (1 << 2) |(1 << 0);  // MUXPOS[2:0] = 100 (PIN 4) | MUXNEG[2:0] = 101 (DAC)

	ACA.AC1MUXCTRL = (1 << 5) | (1 << 3) | (1 << 2) |(1 << 0);  // MUXPOS[2:0] = 101 (PIN 5) | MUXNEG[2:0] = 101 (DAC)


	
// 	/* Sense for 4 changes on comparator output. */
// 	for(uint8_t i=0; i < 4; i++){
// 		/* Wait for Comparator to change value. */
// 		AC_WaitForComparator_Blocking(&AC, ANALOG_COMPARATOR0);
// 	}

}