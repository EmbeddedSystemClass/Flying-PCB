#ifndef __MATHS_H__
#define __MATHS_H__

#define ABS(x) ((x) > 0 ? (x) : (-x))

float applyDeadband(float value, float deadband);
float constrain(float amt, float low, float high);

#endif
