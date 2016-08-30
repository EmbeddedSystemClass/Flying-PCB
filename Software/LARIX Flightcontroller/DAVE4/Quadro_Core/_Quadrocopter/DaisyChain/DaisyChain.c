/*******************************************************************************
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** MAAN         					                                          **
**                                                                            **
**                                                                            **
**                                                                            **
*******************************************************************************/
 
 /*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/*
 * V0.0: 15-09-2015, MAAN:  Initial Version
 */
 /*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "DaisyChain.h"

/*******************************************************************************
**                      Private Constant Definitions to be changed            **
*******************************************************************************/

/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/

/*******************************************************************************
**                      Global Type Definitions                               **
*******************************************************************************/

/*******************************************************************************
**                      Private Type Definitions                              **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/

/*******************************************************************************
**                      Global Constant Definitions                           **
*******************************************************************************/

/*******************************************************************************
**                      Private Constant Definitions                          **
*******************************************************************************/

/*******************************************************************************
**                      Global Variable Definitions                           **
*******************************************************************************/

/*******************************************************************************
**                      Private Variable Definitions                          **
*******************************************************************************/
uint8_t DaisyTransmit[DAISY_BUFFER_SIZE]; /**< intern buffer for DaisyChain transmit data */

uint16_t speedval1 = 0;/**< Motor 1 speed parameter Range: 0-1279	1279->Motorspeed=100% 0->Motorspeed=0% */
uint16_t speedval2 = 0;/**< Motor 2 speed parameter Range: 0-1279   1279->Motorspeed=100% 0->Motorspeed=0% */
uint16_t speedval3 = 0;/**< Motor 3 speed parameter Range: 0-1279   1279->Motorspeed=100% 0->Motorspeed=0% */
uint16_t speedval4 = 0;/**< Motor 4 speed parameter Range: 0-1279   1279->Motorspeed=100% 0->Motorspeed=0% */

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/



/**
 *  \brief Sends command plus data via DaisyChain to ESCs
 *  
 *  \param [in] command Command to Transmit
 *  \param [in] data1 Motor 1 Data
 *  \param [in] data2 Motor 2 Data
 *  \param [in] data3 Motor 3 Data
 *  \param [in] data4 Motor 4 Data
 *  
 *  \details Fills the internal buffer DaisyTransmit and sends the data inside the buffer via UART to the ESCs.
 */
void SendDaisyData(uint8_t command, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)//sends command plus variable data for each motorcontroller
{
	//Motor 1
	DaisyTransmit[0]=command;
	DaisyTransmit[1]=(uint8_t)(data1>>8);
	DaisyTransmit[2]=(uint8_t)data1;

	//Motor 2
	DaisyTransmit[3]=command;
	DaisyTransmit[4]=(uint8_t)(data2>>8);
	DaisyTransmit[5]=(uint8_t) data2;

	//Motor 3
	DaisyTransmit[6]=command;
	DaisyTransmit[7]=(uint8_t)(data3>>8);
	DaisyTransmit[8]=(uint8_t) data3;

	//Motor 4
	DaisyTransmit[9]=command;
	DaisyTransmit[10]=(uint8_t)(data4>>8);
	DaisyTransmit[11]=(uint8_t) data4;

	DaisyTransmit[12]=DAISY_STOP_BYTE;

	UART_WriteDataBuffer(&DaisyChain_Handle, DaisyTransmit, DAISY_BUFFER_SIZE);
}
/**
 *  \brief DaisyChain Stop command
 * 
 *  \details Sends Stop Motor Command plus Data=0 to each Motor
 */
void SendDaisyStopCommand() //stops all motors
{
	SendDaisyData(STOP_MOTOR,0,0,0,0);
}
/**
 *  \brief DaisyChain sends motor speeds
 *  
 *  \param [in] PWM_percentages pointer to an Array including the desired motor-speed in percent
 *  
 *  \details Calculates the values that need to be transmitted to the ESC out of the desired motorspeed stored in PWM percentages array.\n
 *  Starts: SendDaisyData function with set speed command.
 */
void SendDaisyPWMPercentages(float *PWM_percentages) //calculates the values and sends command for setting motorspeed to each controller
{
	speedval1=(uint16_t)(*(PWM_percentages+2)/100.0f*1279.0f);
	speedval2=(uint16_t)(*(PWM_percentages)/100.0f*1279.0f);
	speedval3=(uint16_t)(*(PWM_percentages+1)/100.0f*1279.0f);
	speedval4=(uint16_t)(*(PWM_percentages+3)/100.0f*1279.0f);

	SendDaisyData(SET_REF_SPEED,speedval1,speedval2,speedval3,speedval4);
}
