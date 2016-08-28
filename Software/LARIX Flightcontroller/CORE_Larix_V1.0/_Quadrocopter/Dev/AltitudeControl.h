
#ifndef ALTITUDECONTROL_H_
#define ALTITUDECONTROL_H_

//#include "def.h"
#include <stdint.h>		// declare uint8_t
#include <math.h>
#include "../Sensors/DPS310/DPS310.h"
#include "../_HAL/Delay/util.h"

extern float ground_pressure;
extern float ground_temperature;
extern float alt;
extern uint32_t baro_alt_update_time;

extern float vel;

extern int alt_ctrl_cnt;

float CalcBaroAlt(void);
void AltitudeController(void);
void Alt_Controller_Int_Handler(void);

#endif /* ALTITUDECONTROL_H_ */
