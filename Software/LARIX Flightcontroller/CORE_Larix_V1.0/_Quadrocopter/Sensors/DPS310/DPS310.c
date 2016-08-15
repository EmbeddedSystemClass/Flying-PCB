/*
 * DPS310.c
 *
 *  Created on: 01.09.2015
 *      Author: SteurerE
 */

#include "DPS310.h"



int32_t Praw = 0;
int32_t Traw = 0;

// variables for calculation
float Praw_sc = 0;
float Traw_sc = 0;
float Pcomp = 0;
float Tcomp = 0;

// coefficients to calculate physical values
int32_t c00 = 0;
int32_t c10 = 0;
int32_t c20 = 0;
int32_t c30 = 0;
int32_t c01 = 0;
int32_t c11 = 0;
int32_t c21 = 0;
int32_t c0 = 0;
int32_t c1 = 0;
// compensation scale factors (data sheet page 15, table 9)
uint32_t kP = 0;
uint32_t kT = 0;

void DPS_EXT_INT_ISR(void)
{
	updateValues(&pressure, &temperature);
    I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x0A);
    PressureFIR = FIR_FILTER(PressureFIR, pressure);
    //#>>>>>>>>>>
    //# due to the unstable performance of DPS310, I assign ground pressure and temperature here
    //# rather than in Initialize() to make sure they have legal values.
    //# These sentences will be executed only once.
    if(!(ground_pressure > 90000 && ground_pressure < 100000)) ground_pressure = pressure;
    if(!(ground_temperature > 0 && ground_temperature < 50)) ground_temperature = temperature;
    DPS_ISR_cnt++;
    //#<<<<<<<<<<
}

SensorError setupDPS310()
{
//*****************************************************************
//		Attention: Measurement Time of the DPS310 must be <1s!
//	  Measurement Time = PM_Rate*Time_Meas + TMP_RATE*Time_Meas
//*****************************************************************
//	Table: Time for 1 Measurement with Oversampling (PRC)
//	_____________________________________________________________________________
// |             |       |       |       |       |       |       |       |       |
// |     PRC     |   1   |   2   |   4   |   8   |   16  |   32  |   64  |  128  |
// |_____________|_______|_______|_______|_______|_______|_______|_______|_______|
// |             |       |       |       |       |       |       |       |       |
// |  Time_Meas  |  3.6  |  5.2  |  8.4  | 14.8  | 27.6  | 53.2  | 104.4 | 206.8 |
// |     [ms]    |       |       |       |       |       |       |       |       |
// |_____________|_______|_______|_______|_______|_______|_______|_______|_______|
//
// --------------------------Measurement Modes (MEAS_config)--------------------------
//		Standby Mode:		0x00	Idle/Stop background measurement
//		Command Mode:		0x01	Single Pressure Measurement
//							0x02	Single Temperature Measurement
//		Background Mode:	0x05	Continuous Pressure Measurement (change in updateValues, if (x08>>4 == 13))
//							0x06	Continuous Temperature Measurement
//							0x07 	Continuous Pressure and Temperature Measurement (change in updateValues, if (x08>>4 == 15))
//
//--------------------Interrupt an FIFO Configuration(CFG_config)--------------------
//						(check DPS310 Datasheet for Details)
//					0x01	No Interrupts, FIFO disabled, 3-Wire Interface
//					0x11	Temperature Interrupt, FIFO disabled, 3-Wire Interface
//					0x21	Pressure Interrupt, FIFO disabled, 3-Wire Interface
//
//------------------------------configure sensor start------------------------------
	uint8_t P_config = PM_RATE_16 | PM_PRC_32;	// values/sec | oversampling rate
	uint8_t T_config = TMP_RATE_16 | TMP_PRC_4;	// values/sec | oversampling rate
	int8_t	MEAS_config = 0x07;					// continuous pressure measurement
	uint8_t CFG_config = 0x21;					// Pressure Interrupt, FIFO disabled, 3-Wire-SPI/I2C
//-------------------------------configure sensor end-------------------------------

	int t = SYSTM001_GetTime();				// wait 12ms for sensor to be ready
	while (SYSTM001_GetTime() - t < 12);
	SensorError result = I2Cdev_writeByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,RESET_REGISTER,SOFT_RESET);	// Software Reset
	t = SYSTM001_GetTime();					// wait 40ms for coefficients to be ready
	while (SYSTM001_GetTime() - t < 40);

	result = setRegister(DPS310_Address,PRS_CFG,P_config);
	if(result)return result;

	// check which temperature sensor is used (bit7 == 0 => ASIC, bit7 == 1 => MEMS)
	uint8_t x28 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle, DPS310_Address,COEF_SRC)>>7;
	if (x28 == 1)	result = setRegister(DPS310_Address,TMP_CFG,1<<7 | T_config);								// use external sensor (MEMS)
	else			result = setRegister(DPS310_Address,TMP_CFG,T_config);										// use internal sensor (ASIC)
	if(result)return result;

	result = setRegister(DPS310_Address,MEAS_CFG,0xc0 | MEAS_config);
	if(result)return result;

	// check if P-Shift and/or T-Shift is necessary (PM_PRC > 8 P-shift necessary; TMP_PRC > 8 -> T-shift necessary)
	uint8_t x06 = P_config<<4;
	uint8_t x07 = T_config<<4;
	if (x06>>4 > 3 && x07>>4 > 3)			result = setRegister(DPS310_Address,CFG_REG,3<<2 | CFG_config);		// P-Shift and T-Shift
	else if (x06>>4 <= 3 && x07>>4 > 3)		result = setRegister(DPS310_Address,CFG_REG,2<<2 | CFG_config);		// T-Shift
	else if (x06>>4 > 3 && x07>>4 <= 3)		result = setRegister(DPS310_Address,CFG_REG,1<<2 | CFG_config);		// P-Shift
	else									result = setRegister(DPS310_Address,CFG_REG,CFG_config);			// no P-Shift or T-Shift
	if(result)return result;

	I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,INT_STS);
	return result;
}

void updateValues(float *Pcomp,float *Tcomp)
{
	uint8_t x08 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x08);
	if (x08>>4 == 15)						// read pressure register only if values are ready (==13 if pressure measurement; ==15 if pressure & temperature measurement
	{
		// generate Praw
		Praw = getPressure();				// 24bit two�s complement value out of reg 0-2
		if (Praw > (pow(2, 23) - 1))		// convert to signed int
		{
			Praw = Praw - pow(2, 24);
		}

		// generate Traw
		Traw = getTemperature();			// 24bit two�s complement value out of reg 3-5
		if (Traw > (pow(2, 23) - 1))		// convert to signed int
		{
			Traw = Traw - pow(2, 24);
		}

		// calculate physical temperature Tcomp [�C]
		Traw_sc = (float)Traw/kT;
		*Tcomp = c0*0.5 + c1*Traw_sc;
		// calculate physical pressure Pcomp [Pa]
		Praw_sc = (float)Praw/kP;
		*Pcomp = c00 + Praw_sc*(c10 + Praw_sc *(c20+ Praw_sc *c30)) + Traw_sc*c01 + Traw_sc *Praw_sc *(c11+Praw_sc*c21);
	}
}

void getCoefficients()
{
	// get fused coefficients from a calibrated sensor
	c0 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x10)<<4 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x11)>>4;
	if (c0 > (pow(2, 11) - 1))	c0 = c0 - pow(2, 12);
	uint8_t c1_ = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x11)<<4;	// "delete" bit7 - bit4
	c1 = c1_<<4 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x12);
	if (c1 > (pow(2, 11) - 1))	c1 = c1 - pow(2, 12);
	c00 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x13)<<12 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x14)<<4 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x15)>>4;
	if (c00 > (pow(2, 19) - 1))	c00 = c00 - pow(2, 20);
	uint8_t c10_ = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x15)<<4;	// "delete" bit7 - bit4
	c10 = c10_<<12 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x16)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x17);
	if (c10 > (pow(2, 19) - 1))	c10 = c10 - pow(2, 20);
	c01 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x18)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x19);
	if (c01 > (pow(2, 15) - 1))	c01 = c01 - pow(2, 16);
	c11 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1a)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1b);
	if (c11 > (pow(2, 15) - 1))	c11 = c11 - pow(2, 16);
	c20 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1c)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1d);
	if (c20 > (pow(2, 15) - 1))	c20 = c20 - pow(2, 16);
	c21 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1e)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x1f);
	if (c21 > (pow(2, 15) - 1))	c21 = c21 - pow(2, 16);
	c30 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x20)<<8 | I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x21);
	if (c30 > (pow(2, 15) - 1))	c30 = c30 - pow(2, 16);

	//use some default values if coefficient registers are "empty"
	if (c0 == 0 || c1 == 0 || c00 == 0 || c10 == 0 || c01 == 0 || c11 == 0 || c20 == 0 || c21 == 0 || c30 == 0)
	{
		c00 = 81507;
		c10 = -66011;
		c20 = -13579;
		c30 = -2154;
		c01 = -2115;
		c11 = 1585;
		c21 = 117;
		c0 = 205;
		c1 = -258;
	}

	// select compensation scale factors
	uint8_t P_sampling = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,PRS_CFG)<<4;
	switch (P_sampling>>4)
	{
		case 0: kP = 524288;
		break;
		case 1: kP = 1572864;
		break;
		case 2: kP = 3670016;
		break;
		case 3: kP = 7864320;
		break;
		case 4: kP = 253952;
		break;
		case 5: kP = 516096;
		break;
		case 6: kP = 1040384;
		break;
		case 7: kP = 2088960;
		break;
	}
	uint8_t T_sampling = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,TMP_CFG)<<4;
	switch (T_sampling>>4)
	{
		case 0: kT = 524288;
		break;
		case 1: kT = 1572864;
		break;
		case 2: kT = 3670016;
		break;
		case 3: kT = 7864320;
		break;
		case 4: kT = 253952;
		break;
		case 5: kT = 516096;
		break;
		case 6: kT = 1040384;
		break;
		case 7: kT = 2088960;
		break;
	}
}

int32_t getPressure()
{
	//Read Pressure start------------------------------------------------------------------------------------------------------------------------------
	uint32_t register0 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x00);
	uint32_t register1 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x01);
	uint32_t register2 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x02);
	//Read Pressure end------------------------------------------------------------------------------------------------------------------------------
	return (uint32_t)(register0<<16 | register1<<8 | register2);
}

int32_t getTemperature()
{
	//Read Temp start------------------------------------------------------------------------------------------------------------------------------
	uint32_t register3 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x03);
	uint32_t register4 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x04);
	uint32_t register5 = I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310_Address,0x05);
	//Read Temp end------------------------------------------------------------------------------------------------------------------------------
	return (uint32_t)(register3<<16 | register4<<8 | register5);
}

SensorError setRegister(uint8_t DPS310Address, uint8_t registerAdress, uint8_t setValue)
{

SensorError	worked = NO_ERROR;
I2Cdev_writeByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310Address,registerAdress,setValue); // write variable in register
//if(setValue != I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310Address,registerAdress))
uint8_t reg=I2Cdev_readByte((const I2C001Handle_type*)&DPS310_I2C_Handle,DPS310Address,registerAdress);
if(setValue != reg)
{
	switch(registerAdress)
	{
	case 0x06:
	worked = PRESSURE_CONFIG;
	break;

	case 0x07:
	worked = TEMPERATURE_CONFIG;
	break;

	case 0x08:
	worked = SET_CONTINOUS;
	break;

	case 0x09:
	worked = IR_CONFIG;
	break;
	}
}
return worked;
}
