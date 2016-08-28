
#include "imu.h"
#include "AltitudeControl.h"
#include "buffer.h"

static float rMat[3][3];
Vector3f accel_ef;
Vector3f accel_correction_ef;
Vector3f _velocity;
Vector3f _position_error = {0,0,0};
Vector3f _position_correction;
Vector3f _position_base;
Vector3f _position;


//void redefineEulers(float eulers[3], float euler_dot[3])
//{
//	eulers[0] = YPR[2]; // roll
//	eulers[1] = -YPR[1]; // pitch
//	eulers[2] = -heading; // yaw
//
//	float mag[3];
//	GetMagData(mag);
//	euler_dot[0] = mag[0]; // roll_dot
//	euler_dot[1] = -mag[1]; // pitch_dot
//	euler_dot[2] = -mag[2]; // yaw_dot
//}


//# bf->ef rotation matrix
void imuComputeRotationMatrix()
{
//	float roll = eulers[0] * DEG_TO_RAD;
//	float pitch = eulers[1] * DEG_TO_RAD;
//	float yaw = eulers[2] * DEG_TO_RAD;
	float roll = YPR[2] * DEG_TO_RAD;
	float pitch = YPR[1] * DEG_TO_RAD;
	float yaw = heading * DEG_TO_RAD;
	//# q[3] is changing rapidly, so we cannot use quaternion to calculate rotation matrix.
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	rMat[0][0] = cp * cy;
	rMat[0][1] = (sr * sp * cy) - (cr * sy);
	rMat[0][2] = (cr * sp * cy) + (sr * sy);
	rMat[1][0] = cp * sy;
	rMat[1][1] = (sr * sp * sy) + (cr * cy);
	rMat[1][2] = (cr * sp * sy) - (sr * cy);
	rMat[2][0] = -sp;
	rMat[2][1] = sr * cp;
	rMat[2][2] = cr * cp;

	//# debug: Recalculate euler angles with rotation matrix and compare with YPR and heading.
	//# If the difference is little, then the rotation matrix is correct.
	eulers[0] = atan2f(rMat[2][1],rMat[2][2]) * RAD_TO_DEG;
	eulers[1] = -asinf(rMat[2][0]) * RAD_TO_DEG;
	eulers[2] = atan2f(rMat[1][0],rMat[0][0]) * RAD_TO_DEG;
}

void imuTransformVectorBodyToEarth(float vb[3], float ve[3])	// 前左上 => 北西天
{
	float x,y,z;
	x = rMat[0][0] * vb[0] + rMat[0][1] * vb[1] + rMat[0][2] * vb[2];
	y = rMat[1][0] * vb[0] + rMat[1][1] * vb[1] + rMat[1][2] * vb[2];
	z = rMat[2][0] * vb[0] + rMat[2][1] * vb[1] + rMat[2][2] * vb[2];

	ve[0] = x;
	ve[1] = y;
	ve[2] = z;
}


static bool InertialNavInited = false;
static float _k1_xy, _k2_xy, _k3_xy;
static float _k1_z, _k2_z, _k3_z;
static float _time_constant_xy = 2.5f;
static float _time_constant_z = 0.5f;	// 5.0f

Buffer _hist_position_estimate_x;
Buffer _hist_position_estimate_y;
Buffer _hist_position_estimate_z;

void InertialNavUpdate(float dt)	//# 50hz
{
	// update gains from time constant(given in seconds)
	if(!InertialNavInited) {
		_k1_xy = 3 / _time_constant_xy;
		_k2_xy = 3 / (_time_constant_xy * _time_constant_xy);
		_k3_xy = 1 / (_time_constant_xy * _time_constant_xy * _time_constant_xy);

		_k1_z = 3 / _time_constant_z;
		_k2_z = 3 / (_time_constant_z * _time_constant_z);
		_k3_z = 1 / (_time_constant_z * _time_constant_z * _time_constant_z);

		InertialNavInited = true;
	}

    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
	check_baro();

    // check if new gps readings have arrived and use them to correct position estimates
//	check_gps();

	// TODO: smooth acc to accSmooth
	float acc_ef_temp[3];
	imuComputeRotationMatrix();
	imuTransformVectorBodyToEarth(acc,acc_ef_temp);
	accel_ef.x = acc_ef_temp[0] * GRAVITY;
	accel_ef.y = acc_ef_temp[1] * GRAVITY;
	accel_ef.z = acc_ef_temp[2] * GRAVITY;

	// remove influence of gravity
	accel_ef.z -= GRAVITY;
//    accel_ef.x *= 100.0f;
//    accel_ef.y *= 100.0f;
//    accel_ef.z *= 100.0f;

    // convert North-West-Up to North-East-Up
    accel_ef.y = -accel_ef.y;

    float tmp = _k3_xy * dt;
    accel_correction_ef.x += _position_error.x * tmp;
    accel_correction_ef.y += _position_error.y * tmp;
    accel_correction_ef.z += _position_error.z * _k3_z  * dt;

    tmp = _k2_xy * dt;
    _velocity.x += _position_error.x * tmp;
    _velocity.y += _position_error.y * tmp;
    _velocity.z += _position_error.z * _k2_z  * dt;

    tmp = _k1_xy * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    // calculate velocity increase adding new acceleration from accelerometers
    Vector3f velocity_increase;
    velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
    velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
    velocity_increase.z = (accel_ef.z + accel_correction_ef.z) * dt;

    // calculate new estimate of position
    _position_base.x += (_velocity.x + velocity_increase.x * 0.5) * dt;
    _position_base.y += (_velocity.y + velocity_increase.y * 0.5) * dt;
    _position_base.z += (_velocity.z + velocity_increase.z * 0.5) * dt;

    // update the corrected position estimate
    _position.x = _position_base.x + _position_correction.x;
    _position.y = _position_base.y + _position_correction.y;
    _position.z = _position_base.z + _position_correction.z;

    // calculate new velocity
    _velocity.x += velocity_increase.x;
    _velocity.y += velocity_increase.y;
    _velocity.z += velocity_increase.z;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    push_back(_hist_position_estimate_z, _position_base.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
//    _historic_xy_counter++;
//    if( _historic_xy_counter >= 10 ) {
//        _historic_xy_counter = 0;
//        push_back(_hist_position_estimate_x, _position_base.x);
//        push_back(_hist_position_estimate_y, _position_base.y);
//    }
}


uint32_t _baro_last_update = 0;
// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void check_baro()
{
	// calculate time since last baro reading(in ms)
	if(baro_alt_update_time != _baro_last_update) {
		const float dt = (float)(baro_alt_update_time - _baro_last_update) * 0.001f; // in seconds
		// call correction method
		correct_with_baro(alt, dt);
		_baro_last_update = baro_alt_update_time;
	}
}

// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void correct_with_baro(float baro_alt, float dt)
{
	static uint8_t first_reads = 0;
    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    float hist_position_base_z;
    if (is_full(_hist_position_estimate_z)) {
    	hist_position_base_z = front(_hist_position_estimate_z);
    } else {
    	hist_position_base_z = _position_base.z;
    }

    // calculate error in position from baro with our estimate
    _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
}

// set_altitude - set base altitude estimate in cm
void set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
    _position.z = new_altitude; // _position = _position_base + _position_correction
}



void check_gps()
{
	// empty. TBD
}







