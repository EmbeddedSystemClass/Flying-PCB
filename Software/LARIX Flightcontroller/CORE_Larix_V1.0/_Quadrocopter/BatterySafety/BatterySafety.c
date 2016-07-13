/*
 * BatterySafety.c
 *
 *  Created on: 16.03.2016
 *      Author: Tschinder
 */

#include "BatterySafety.h"

extern float VBat;

ADC001_ResultHandleType ADC_Result;
const int R1 = 10;		//in kOhm
const int R2 = 30;		//in kOhm

void BAT_Safety_Meas() //Triggered with 1Hz
{
	ADC001_GenerateLoadEvent (&ADC001_Handle0 );
}

void ADC_Result_ISR()
{
	ADC001_GetResult (&ADC001_Handle0 , &ADC_Result );
	VBat=(3.3/4095)*ADC_Result.Result;		//Umrechnung Bit in Volt
	VBat=VBat*(R1+R2)/R1;					//Umrechnung Spannungsteiler

	//LED Anzeige
	if(VBat > 12.0)
	{
		IO004_SetPin(VBat_LED_Green);
		IO004_SetPin(VBat_LED_Yellow);
		IO004_SetPin(VBat_LED_Red);
	}
	else if(VBat > 11.0)
	{
		IO004_ResetPin(VBat_LED_Green);
		IO004_SetPin(VBat_LED_Yellow);
		IO004_SetPin(VBat_LED_Red);
	}
	else if(VBat > 10.5)
	{
		IO004_ResetPin(VBat_LED_Green);
		IO004_ResetPin(VBat_LED_Yellow);
		IO004_SetPin(VBat_LED_Red);
	}
	else
	{
		IO004_ResetPin(VBat_LED_Green);
		IO004_ResetPin(VBat_LED_Yellow);
		IO004_TogglePin(VBat_LED_Red);
	}
}
