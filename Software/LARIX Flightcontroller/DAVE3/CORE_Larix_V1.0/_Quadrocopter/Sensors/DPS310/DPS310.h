/*
 * DPS310.h
 *
 *  Created on: 01.09.2015
 *      Author: SteurerE
 */

#ifndef DPS310_H_
#define DPS310_H_

#include <Dave3.h>
#include "../../_HAL/Delay/util.h"
#include "../../_HAL/I2C/I2Cdev.h"
#include "../../_FIR_Filter/FIR_Filter.h"

#define DPS310_Address (0x77<<1)


#define PRS_CFG 0x06 //pressure config register
#define TMP_CFG 0x07 //temperature config register
#define MEAS_CFG 0x08 //Sensor Operating mode register
#define CFG_REG 0x09//Interrupt and FIFO-config
#define INT_STS 0x0A//Interrupt status register
#define RESET_REGISTER 0x0C//reset register of sensor
#define COEF_SRC 0x28//Temperature coefficient source

//reset
#define SOFT_RESET 0x09//sequence to perform soft-reset

//sensor interrupt and fifo configuration
#define GENERATE_IR_T_SHIFT_P_SHIFT 0x2C//Generate interrupt if new pressure-value is available + shift values right(see datasheet for info)
#define GENERATE_IR_PR 0x20 //Generate Interrupt if new pressure-value is available, without T_SHIFT and P_SHIFT

//sensor operating mode and status register-config
#define MEAS_CTRL_CONTINUES_TEMP_PRESSURE 0x07
#define MEAS_CTRL_CONTINUES_TEMP 0x06
#define MEAS_CTRL_CONTINUES_PRESSURE 0x05

//pressure-config
#define PM_RATE_128 0x70 //128 messungen/sec
#define PM_PRC_128 0x07 //pressure oversampling rate -128 times

#define PM_RATE_64 0x60 //64 messungen/sec
#define PM_PRC_64 0x06 //pressure oversampling rate -64 times

#define PM_RATE_32 0x50 //32 messungen/sec
#define PM_PRC_32 0x05 //pressure oversampling rate -32 times

#define PM_RATE_16 0x40 //16 messungen/sec
#define PM_PRC_16 0x04 //pressure oversampling rate -16 times

#define PM_RATE_8 0x30 //8 messungen/sec
#define PM_PRC_8 0x03 //pressure oversampling rate -8 times

#define PM_RATE_4 0x20 //4 messungen/sec
#define PM_PRC_4 0x02 //pressure oversampling rate -4 times

#define PM_RATE_2 0x10 //2 messungen/sec
#define PM_PRC_2 0x01 //pressure oversampling rate -2 times

#define PM_RATE_1 0x00 //1 messungen/sec
#define PM_PRC_1 0x00 //pressure oversampling rate -1 time

//temp-config
#define TMP_RATE_128 0x70 //128 messungen/sec
#define TMP_PRC_128 0x07 //temp oversampling rate -128 times

#define TMP_RATE_64 0x60 //64 messungen/sec
#define TMP_PRC_64 0x06 //temp oversampling rate -64 times

#define TMP_RATE_32 0x50 //32 messungen/sec
#define TMP_PRC_32 0x05 //temp oversampling rate -32 times

#define TMP_RATE_16 0x40 //16 messungen/sec
#define TMP_PRC_16 0x04 //temp oversampling rate -16 times

#define TMP_RATE_8 0x30 //8 messungen/sec
#define TMP_PRC_8 0x03 //temp oversampling rate -8 times

#define TMP_RATE_4 0x20 //4 messungen/sec
#define TMP_PRC_4 0x02 //temp oversampling rate -4 times

#define TMP_RATE_2 0x10 //2 messungen/sec
#define TMP_PRC_2 0x01 //temp oversampling rate -2 times

#define TMP_RATE_1 0x00 //1 messungen/sec
#define TMP_PRC_1 0x00 //temp oversampling rate -1 time


typedef enum _SENSOR_ERROR_
{
	NO_ERROR,
	PRESSURE_CONFIG,
	TEMPERATURE_CONFIG,
	SET_CONTINOUS,
	IR_CONFIG
}SensorError;

extern float pressure;
extern float temperature;
extern struct structFIR PressureFIR;

SensorError setupDPS310();
SensorError setRegister(uint8_t DPS310Address, uint8_t registerAdress, uint8_t setValue);
int32_t getTemperature();
int32_t getPressure();
void updateValues(float *Pcomp,float *Tcomp);
void getCoefficients();

#endif /* DPS310_H_ */
