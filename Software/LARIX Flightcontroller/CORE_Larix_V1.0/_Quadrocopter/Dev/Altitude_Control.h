
#ifndef ALTITUDECONTROL_H_
#define ALTITUDECONTROL_H_

//#include "def.h"
#include <math.h>
#include "../Sensors/DPS310/DPS310.h"

extern float ground_pressure;
extern float ground_temperature;
extern float alt;

extern int alt_ctrl_cnt;

float Calc_Alt(void);
void Altitude_Controller(void);
void Alt_Controller_Int_Handler(void);

#endif /* ALTITUDECONTROL_H_ */
