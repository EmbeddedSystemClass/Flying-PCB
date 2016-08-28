#ifndef __IMU_H__
#define __IMU_H__

#include "../Sensors/MPU9X50/MPU9150.h"
#include "maths.h"
#include <math.h>
#include <stdint.h>		// declare uint8_t

#define GRAVITY 9.80665f
#define M_PIf 3.14159265358979323846f

typedef struct{
	float x;
	float y;
	float z;
} Vector3f;

extern float YPR[3];
extern float heading;
extern float eulers[3];

extern float acc[3];
extern Vector3f accel_ef;
extern Vector3f _velocity;
extern Vector3f _position;

//void redefineEulers(float eulers[3], float euler_dot[3]);
void imuComputeRotationMatrix(void);
void imuTransformVectorBodyToEarth(float vb[3], float ve[3]);

void InertialNavUpdate(float dt);
void check_baro(void);
void correct_with_baro(float baro_alt, float dt);
void set_altitude( float new_altitude);

void check_gps(void);

#endif
