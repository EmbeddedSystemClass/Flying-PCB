
#include "Altitude_Control.h"

//# from AP_Baro::get_altitude_difference()
float Calc_Alt()
{
	float ret;
	float temp = ground_temperature + 273.15f;
	float scaling = pressure / ground_pressure;
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
	return ret;
}


void Altitude_Controller()
{
	return;
}

void Alt_Controller_Int_Handler()
{
	alt = Calc_Alt();
	alt_ctrl_cnt++;
}
