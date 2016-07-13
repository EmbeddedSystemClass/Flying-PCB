/*
 * RCReceive.c
 *
 *  Created on: 22.12.2014
 *      Author: maan
 */

#include "RCReceive.h"
#include "../_HAL/GPIO.h"

////////////////////Variablen Elias///////////////////////////////
extern uint8_t* ReadBufBT;
extern ControlValue control_value;
extern DataPacket dpacket;
///////////////////////////////////////////////////////////////////

//Joystick values (range between -1 and 1, 0 in middle position)
float throttle = 0.0;
float rudder = 0.0;
float elevator = 0.0;
float aileron = 0.0;

//Bluetooth values
uint16_t throttleBT = 0;
uint8_t heightControlBT = 0;
float rudderBT = 0;
float aileronBT = 0;
float elevatorBT = 0;

uint8_t modeBT = 0;

uint8_t flightmode = 0;

uint8_t RCTimeOut = 1; //Timeout remote control 0:OK 1:TimeOut
uint8_t RCCount = 0; //Counter for RC read (used for TimeOut)

uint8_t BTTimeOut = 1;
uint8_t BTCount = 0;
_Bool conn = false;

// Software timer
handle_t TimerWatchRC; //Timer for RC watchdog

/*
 * Merge two bytes to one integer
 *
 * Parameters:
 * uint8_t a: high-byte
 * uint8_t b: low-byte
 *
 * Returnvalue:
 * int Merged bytes
 */
int mergeBytes(uint8_t a, uint8_t b)
{
	int c = a;
	return (c << 8) | b;
}

/**
 *Bluetooth Keep-Alive Messages
 * for Android-App to check connection
 */
void Bluetooth_Keep_Alive()
{
	uint8_t keep_alive = 0xFF;
	UART001_WriteDataBytes(&BT_UART_Handle,(uint8_t*)&keep_alive,1);
}
/**
 * Interrupt routine from UART FIFO in buffer
 * Buffer is full, new data is ready to read
 */
void RC_RECEIVE_ISR()
{
	//Raw values from receiver
	int throttleRaw;
	int rudderRaw;
	int elevatorRaw;
	int aileronRaw;
	int flightmodeRaw;

	uint8_t ReadBufRC[32]; //Readbuffer
	int start = 0; //Index of start byte (contains 0x30)

	//Check if receive buffer interrupt is set
	if (UART001_GetFlagStatus(&RC_UART_Handle, UART001_FIFO_STD_RECV_BUF_FLAG)
			== UART001_SET)
	{
		//Read data from UART buffer
		UART001_ReadDataBytes(&RC_UART_Handle, ReadBufRC, 32);
		//Clear receive buffer interrupt flag
		UART001_ClearFlag(&RC_UART_Handle, UART001_FIFO_STD_RECV_BUF_FLAG);
		//Search for start byte and check static values
		while (ReadBufRC[start] != 0x30 || ReadBufRC[start + 1] != 0x00
				|| ReadBufRC[start + 5] != 0xA2 || ReadBufRC[start + 8] != 0x2B
				|| ReadBufRC[start + 9] != 0xFE)
		{
			if (start++ > 16)
			{
				//Communication check bytes not in buffer
				return;
			}
		}
		//Get data from stream
		throttleRaw = mergeBytes(ReadBufRC[start + 2], ReadBufRC[start + 3]);
		throttle = map(throttleRaw, THROTTLE_MIN, THROTTLE_MAX, 0, 60000)
				/ 60000.0;
		aileronRaw = mergeBytes(ReadBufRC[start + 6], ReadBufRC[start + 7]);
		aileron = map(aileronRaw, AILERON_MIN, AILERON_MAX, -30000, 30000)
				/ 30000.0;
		elevatorRaw = mergeBytes(ReadBufRC[start + 10], ReadBufRC[start + 11]);
		elevator = map(elevatorRaw, ELEVATOR_MIN, ELEVATOR_MAX, -30000, 30000)
				/ 30000.0;
		rudderRaw = mergeBytes(ReadBufRC[start + 14], ReadBufRC[start + 15]);
		rudder = map(rudderRaw, RUDDER_MIN, RUDDER_MAX, -30000, 30000)
				/ 30000.0;
		flightmodeRaw = mergeBytes(ReadBufRC[start + 12],
				ReadBufRC[start + 13]);
		if (flightmodeRaw == FLIGHTMODE0)
			flightmode = 0;
		if (flightmodeRaw == FLIGHTMODE1)
			flightmode = 1;
		//Set values for RC Timeout check
		RCTimeOut = 0;
		RCCount++;
	}
}
void BT_RECEIVE_ISR()
{
	if (UART001_GetFlagStatus(&BT_UART_Handle, UART001_FIFO_STD_RECV_BUF_FLAG)
			== UART001_SET)
	{
		UART001_ReadDataBytes(&BT_UART_Handle,ReadBufBT,PACKET_SIZE);
		//Clear receive buffer interrupt flag
		UART001_ClearFlag(&BT_UART_Handle, UART001_FIFO_STD_RECV_BUF_FLAG);
		status_t rec_mode = maintainBluetoothInputBuffer(ReadBufBT,
				&control_value, &dpacket);
		switch (rec_mode)
		{
		case RECEIVED_CONTROL_PACKET:
			throttleBT = (uint16_t)*control_value.speed;
			heightControlBT = *control_value.height_control;
			aileronBT = *control_value.x_pitch*0.7;
			elevatorBT = -*control_value.y_roll*0.7;
			rudderBT = *control_value.z_rotate;
			BTTimeOut = 0;
			BTCount++;
			break;
		case RECEIVED_DATA_PACKET:
			break;
		case CHECKSUM_ERROR:
			throttleBT = 0;
			aileronBT = 0;
			elevatorBT = 0;
			rudderBT = 0;
			//Todo: Add Error-Handling
			break;
		case UNDEFINED_ERROR:

			throttleBT = 0;
			aileronBT = 0;
			elevatorBT = 0;
			rudderBT = 0;
			//undef_error++;
			//Todo: Add Error-Handling
			break;
		}
	}
}

/*
 * Check if new data has been arrived since last function call
 * This function is called from software Timer "TimerWatchRC"
 */
void WatchRC(void* Temp)
{
	static uint8_t lastCount;
	static uint8_t lastBTCount;

	if (lastCount == RCCount)
		RCTimeOut = 1;
	lastCount = RCCount;

	if (lastBTCount == BTCount)
		BTTimeOut = 1;
	lastBTCount = BTCount;

	//keep_alive packet for connection state_check
	//uint8_t keep_alive = 0xFF;
	//UART001_WriteDataBytes(&BT_UART_Handle, (uint8_t*) &keep_alive, 1);
}

/*
 * Initialize RC watchdog
 */
void WatchRC_Init()
{
	//Set timer to check every 500ms, if new data has arrived
	TimerWatchRC = SYSTM001_CreateTimer(200, SYSTM001_PERIODIC, WatchRC, NULL);
	if (TimerWatchRC != 0)
	{
		//Timer is created successfully
		if (SYSTM001_StartTimer(TimerWatchRC) == DAVEApp_SUCCESS)
		{
			//Timer started
		}
	}
}

uint8_t GetRCCount()
{
	return RCCount;
}

void GetRCData(float* power,uint8_t* height_control, float* yaw_dot, float* pitch, float* roll)
{
#ifdef BT_ONLY		//BT only controlled (no RC needed)
		if (BTTimeOut)
		{
			*power = 0.0;
			*yaw_dot = 0.0;
			*pitch = 0.0;
			*roll = 0.0;
		}
		else
		{
			*power = throttleBT;
			*height_control = heightControlBT;
			*pitch = elevatorBT;
			*roll = -aileronBT;
			*yaw_dot = rudderBT;
			if (*pitch > 30.0)
			{
				*pitch = 30.0;
			}
			else if (*pitch < -30.0)
			{
				*pitch = -30.0;
			}

			if (*roll > 30.0)
			{
				*roll = 30.0;
			}
			else if (*roll < -30.0)
			{
				*roll = -30.0;
			}
		}

#else		//for RC Control with switch to BT Control
	if (flightmode == 0)
		{
			if (RCTimeOut)
			{
				*power = 0;
				*yaw_dot = 0;
				*pitch = 0;
				*roll = 0;
			}
			else
			{
				*power = SCALE_POWER * throttle;
				if (rudder > 0.01 || rudder < -0.01)
					*yaw_dot = rudder * SCALE_YAW;
				else
					*yaw_dot = 0;
					*pitch=elevator*SCALE_PITCH;
					*roll=-aileron*SCALE_ROLL;
			}
		}
		else
		{
			if (BTTimeOut || RCTimeOut)
			{
				*power = 0.0;
				*yaw_dot = 0.0;
				*pitch = 0.0;
				*roll = 0.0;
			}
			else
			{
				*power = throttleBT;
				*height_control = heightControlBT;
				*pitch = elevatorBT;
				*roll = -aileronBT;
				*yaw_dot = rudderBT;
				if (*pitch > 30.0)
				{
					*pitch = 30.0;
				}
				else if (*pitch < -30.0)
				{
					*pitch = -30.0;
				}

				if (*roll > 30.0)
				{
					*roll = 30.0;
				}
				else if (*roll < -30.0)
				{
					*roll = -30.0;
				}
			}
		}
#endif
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
