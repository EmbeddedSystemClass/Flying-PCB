/*
 * Main.c
 *
 *  Created on: 25.12.2014
 *      Author: Andreas
 */

//----------------------------------------------------------------------------
//
//					LARIX Flight Controller Software with DPS310
//				(compatible with Flight Controller App V2.0 or higher)
//
//----------------------------------------------------------------------------
//
//		Default Mode: RC controlled with switch for Bluetooth-App Control
//		(for Bluetooth only Version "#define BT_ONLY" in the RCReceive.h)
//
//----------------------------------------------------------------------------

//#>>>>>>>>>>
//#include "_Quadrocopter/Dev/def.h"	//# user defined variables
//#<<<<<<<<<<

#include <DAVE3.h>			//Declarations from DAVE3 Code Generation (includes SFR declaration)
#include <limits.h>
#include "_Quadrocopter/_HAL/Delay/util.h"
#include "_Quadrocopter/_HAL/I2C/I2Cdev.h"
#include "_Quadrocopter/Sensors/MPU9X50/MPU9150.h"
#include "_Quadrocopter/Sensors/DPS310/DPS310.h"

#include "_Quadrocopter/RemoteControl/RCReceive.h"
#include "_Quadrocopter/Attitude_Control/Attitudecontroller.h"
#include "_Quadrocopter/DaisyChain/DaisyChain.h"

#include "_Quadrocopter/_FIR_Filter/FIR_Filter.h"

#include "_Quadrocopter/BatterySafety/BatterySafety.h"

//Define which Frame is used:
//++++++++++++++++++++++++++++++++++++++++
//#define Carbon
#define Plexi
//++++++++++++++++++++++++++++++++++++++++


#define CONTROL_ORDER 2
//Actor values
float PWM_percent[] = {0.0, 0.0, 0.0, 0.0};

float x_pitch[CONTROL_ORDER];
float x_roll[CONTROL_ORDER];

float u_yaw_dot = 0.0;
float u_pitch = 0.0;
float u_roll = 0.0;
//------------------------------------------------------------------------
//Controllerparamter for Carbon
#ifdef Carbon
const float T=0.002;
const float P_roll = 125.0;		//for Carbon: 125.0;
const float I_roll = 0.0;		//for Carbon: 0.2;
const float D_roll =  4.0;		//for Carbon: 4.0;
const float N_roll = 400.0;		//for Carbon: 400.0;
const float P_pitch = 125.0;	//for Carbon: 125.0;
const float I_pitch = 0.0;		//for Carbon: 0.2;
const float D_pitch = 4.0;		//for Carbon: 4.0;
const float N_pitch = 400.0;	//for Carbon: 400.0;
const float P_yaw = 200.0;		//for Carbon: 40.0;
#endif
//------------------------------------------------------------------------
//Controllerparamter for Plexi
#ifdef Plexi
const float T=0.002;
const float P_roll = 70.0;		//for Plexi: 70.0
const float I_roll = 0.2;		//for Plexi: 0.2
const float D_roll =  1.7;		//for Plexi: 1.7
const float N_roll = 400.0;		//for Plexi: 400.0
const float P_pitch = 70.0;		//for Plexi: 70.0
const float I_pitch = 0.2;		//for Plexi: 0.2
const float D_pitch = 1.7;		//for Plexi: 1.7
const float N_pitch = 400.0;	//for Plexi: 400.0
const float P_yaw = 200.0;		//for Plexi: 200.0
#endif
//------------------------------------------------------------------------
//Controllerpolynomials
float a_roll[CONTROL_ORDER];
float a_pitch[CONTROL_ORDER];
float b_roll[CONTROL_ORDER+1];
float b_pitch[CONTROL_ORDER+1];

//Remote Control
float powerD = 0.0;
uint8_t height_control = 0;		// 0xFF when activated
float yawD_dot = 0.0;
float pitchD = 0.0;
float rollD = 0.0;

float YPR[3];
float sensorData[3];

//DPS3100 Pressure-Sensor
float pressure = 0;
float temperature = 0;
struct structFIR PressureFIR;

int8_t TxBuffer[100] = { 0 };
char c[100]; //for displaying data

//Battery Safety
float VBat = 0;

bool sendMag = FALSE;

uint16_t counter_main=0;

int8_t MonitorBuffer[14] = {0};
int8_t MotorRunning=0;

//#>>>>>>>>>>
//# user defined variables
float ground_pressure = 0;
float ground_temperature = 0;
float alt = 0;
int alt_ctrl_cnt = 0;
//#<<<<<<<<<<

void Monitoring_Int_Handler();

void Controller_CompareMatch_Int_Handler(void)
{
	GetAngles(YPR);
	GetRCData(&powerD, &height_control, &yawD_dot, &pitchD, &rollD);
	//yaw control
	AngleRateController(&yawD_dot, &YPR[0], &P_yaw, &u_yaw_dot);
	//pitch control
	AngleController(&pitchD, &YPR[1], CONTROL_ORDER, a_pitch, b_pitch, x_pitch, &u_pitch);
	//roll control
	AngleController(&rollD, &YPR[2], CONTROL_ORDER, a_roll, b_roll, x_roll, &u_roll);

	//generate actuator values
	CreatePulseWidth(&u_roll, &u_pitch, &u_yaw_dot, &powerD, PWM_percent);

	if (powerD > 5.0)
		SendDaisyData(SET_REF_CURRENT,
			PWM_percent[2]/100.0*1279,
			PWM_percent[0]/100.0*1279,
			PWM_percent[1]/100.0*1279,
			PWM_percent[3]/100.0*1279);
	else
		SendDaisyData(STOP_MOTOR,0,0,0,0);

	counter_main++;
}

void Initialize()
{

	initBluetoothStorage();
	delay(1000);
    // initialize device
	MPU9150_Setup();
	delay(1000);

    // initilize controller polynomials
	b_roll[0]=P_roll-I_roll*T-P_roll*N_roll*T+N_roll*I_roll*T*T+D_roll*N_roll;
	b_roll[1]=I_roll*T-2*P_roll+P_roll*N_roll*T-2*D_roll*N_roll;
	b_roll[2]=P_roll+D_roll*N_roll;
	a_roll[0]=1-N_roll*T;
	a_roll[1]=N_roll*T-2;

	b_pitch[0]=P_pitch-I_pitch*T-P_pitch*N_pitch*T+N_pitch*I_pitch*T*T+D_pitch*N_pitch;
	b_pitch[1]=I_pitch*T-2*P_pitch+P_pitch*N_pitch*T-2*D_pitch*N_pitch;
	b_pitch[2]=P_pitch+D_pitch*N_pitch;
	a_pitch[0]=1-N_pitch*T;
	a_pitch[1]=N_pitch*T-2;

	WatchRC_Init(); //Initialize RC watchdog

	//initialize DPS310
	setupDPS310();
	getCoefficients();

	//initialize FIR Filter
	PressureFIR = Initialize_FIR_Filter(PressureFIR, MOVING_AVERAGE);
	//#>>>>>>>>>>
	//# record ground pressure and temperature
	/*
	int gnd_cnt = 0;
	do {
		ground_pressure = pressure;
		ground_temperature = temperature;
		delay(100);
		//if(gnd_cnt++ > 10) break;
	} while(!(pressure > 90000 && pressure < 100000) || !(temperature > 0 && temperature < 50));
	*/
	//#<<<<<<<<<<

	delay(2000);
	//#>>>>>>>>>>
	//ground_pressure = pressure;
	//ground_temperature = temperature;
	//PWMSP001_Start(&PWMSP001_Handle4);
	//#<<<<<<<<<<

	PWMSP001_Start(&PWMSP001_Handle0);
}

int main(void)
{
	uint16_t Bytes = 0;
	uint16_t nByte;
//	status_t status;		// Declaration of return variable for DAVE3 APIs (toggle comment if required)

	DAVE_Init();			// Initialization of DAVE Apps
	USBVC001_Init();		//Initialize the USB core in Device mode

	Initialize();

	while(1)
	{

	    //------------------------------------------------------------------------------------------------------------------------------
		//		for serial communication (USB)

		// Check number of bytes received
	    Bytes = USBVC001_BytesReceived();

	    if(Bytes != 0)
	    {
	    	for(nByte = 0; nByte < Bytes; nByte++)
	    	{
	    		// Receive Byte
	    		if(USBVC001_ReceiveByte(&TxBuffer[0]) != DAVEApp_SUCCESS)
	    		{
	    			//Error
	    		}
	    	}
			switch(TxBuffer[0])
			{
				case '1':
					sprintf(c, "Throttle: %f   Rudder: %f   Elevator: %f   Aileron: %f\n", powerD, yawD_dot, pitchD, rollD);
					break;
				case '2':
					PWMSP001_Start(&PWMSP001_Handle2);
					break;
				case '3':
					sprintf(c, "Y:%1.2f P:%1.2f R:%1.2f\n", YPR[0], YPR[1], YPR[2]);
					break;
				case '4':
					sprintf(c, "PWM1:%f PWM2:%f PWM3:%f PWM4:%f\n", PWM_percent[0], PWM_percent[1], PWM_percent[2], PWM_percent[3]);
					break;
				case '5':
					sprintf(c, "eY:%f eP:%f eR:%f\n", yawD_dot-YPR[0], pitchD-YPR[1], rollD-YPR[2]);
					break;
				case '6':
					sprintf(c, "TimerSensor:%d TimerMain:%d TimerRC:%d\n", (int)GetSensorCount(), (int)counter_main, (int)GetRCCount());
					break;
				case '7':
					GetGyroData(sensorData);
					sprintf(c, "GyroX:%3.2f GyroY:%3.2f GyroZ:%3.2f\n", sensorData[0], sensorData[1], sensorData[2]);
					break;
				case '8':
					GetAccData(sensorData);
					sprintf(c, "AccX:%f AccY:%f AccZ:%f\n", sensorData[0], sensorData[1], sensorData[2]);
					break;
				case '9':
					sprintf(c, "Pressure:%f Temperature:%f\n", pressure, temperature);
					break;
				case '0':
					sprintf(c, "VBat:%0.2f\n", VBat);
					break;
				//#>>>>>>>>>>
				case 'a':
					sprintf(c,"Ground Pressure:%0.2f, Ground Temperature:%0.2f\n",ground_pressure,ground_temperature);
					break;
				case 'b':
					sprintf(c,"Altitude:%0.2f, alt_ctrl_cnt:%d\n",alt,alt_ctrl_cnt);
					break;
				//#<<<<<<<<<<
			}
			USBVC001_SendString((const char *)c);
	    }
	    if (sendMag)
	    {
	    	sendMag = FALSE;
	    	USBVC001_SendString((const char *)c);
	    }
	    // Call continuous to handle class specific control request
	    USB_USBTask();
	    //------------------------------------------------------------------------------------------------------------------------------
	}
	return 0;
}

void Mag_Calibration_Int_Handler()
{
    GetMagData(sensorData);
    sprintf(c, "%f,%f,%f\r\n", sensorData[0], sensorData[1], sensorData[2]);
    sendMag = TRUE;
}
