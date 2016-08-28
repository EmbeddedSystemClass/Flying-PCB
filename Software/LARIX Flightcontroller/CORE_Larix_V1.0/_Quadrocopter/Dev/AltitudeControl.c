
#include "AltitudeControl.h"
#include "imu.h"

uint32_t baro_alt_update_time;

//# from AP_Baro::get_altitude_difference()
float CalcBaroAlt()
{
	float ret;
	float temp = ground_temperature + 273.15f;
	float scaling = pressure / ground_pressure;
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
	baro_alt_update_time = millis();
	return ret;
}

void AltitudeController()
{
	return;
}

void Alt_Controller_Int_Handler()
{
	alt = CalcBaroAlt();

	//CalcEstimatedAlt();
	alt_ctrl_cnt++;
}
