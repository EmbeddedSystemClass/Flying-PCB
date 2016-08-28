#include "maths.h"

float applyDeadband(float value, float deadband)
{
	if(ABS(value) < deadband) {
		value = 0;
	} else if (value > 0) {
		value -= deadband;
	} else if (value < 0) {
		value += deadband;
	}
	return value;
}

float constrain(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
