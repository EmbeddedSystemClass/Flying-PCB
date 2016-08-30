/*
 * Main.c
 *
 *  Created on: 05.10.2015
 *      Author: maan
 */


#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)
#include "BlockCommutation.h"
#include "ADC.h"
#include "DaisyChain.h"

int main(void)
{
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)
	DAVE_Init();			// Initialization of DAVE Apps
	InitBlockCommutation();
	InitADC();
	InitDaisyWatchDog();
	InitDaisyReset();
	//erase buffer during startup
	uint32_t daisy_chain_delay = 0;
	while(daisy_chain_delay < 100000)
	{
		USIC_FlushRxFIFO(UART001_Handle0.UartRegs);
		daisy_chain_delay++;
	}
	//SetReferenceCurrent(130);
	while(1)
	{
		DaisyChain();
	}
	return 0;
}
