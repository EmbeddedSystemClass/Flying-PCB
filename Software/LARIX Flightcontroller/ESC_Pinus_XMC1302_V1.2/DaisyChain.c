/*
 * DaisyChain.c
 *
 *  Created on: 02.05.2015
 *      Author: Andreas
 */

#include <DAVE3.h>
#include "DaisyChain.h"
#include <stdarg.h>
#include <stdbool.h>

uint8_t FifoRecBuffer[DAISY_BUFFER_SIZE] = {0};
uint8_t FifoTransBuffer[DAISY_BUFFER_SIZE] = {0};

uint8_t DaisyTimeOut = 1;
uint8_t DaisyCount = 0;
uint8_t send_ready = 0;

static uint8_t CharCNT = 0;

extern volatile enum MotorState motorState;

void DaisyChain(void)
{
	uint8_t i=0;
	status_t status=0;
	uint32_t data=0;
	
	static enum MotorState LastmotorState = Stopped;

	if ( motorState == StartUp )
	{
		CharCNT = 0;
		USIC_FlushRxFIFO(UART001_Handle0.UartRegs);
	}

	if ( (LastmotorState == StartUp) && (motorState != Running) )//(motorState == Running)StartUp
	{
		CCU40_CC42->TCCLR |= 0x02;	//定时器清零
		//Start slicesCCU4定时器运行
		CCU40_CC42->TCSET |= 0x01UL;
		
		CharCNT++;
		if ( CharCNT >= 13 )	//能收到连续的13字节完整包
		{
			CharCNT = 0;
			LastmotorState = motorState;
			CCU40_CC42->TCCLR |= 3UL;	//定时器停止运行
			USIC_FlushRxFIFO(UART001_Handle0.UartRegs);///added
		}
		else
		{
			return;
		}
	}
	else
		LastmotorState = motorState;


	//	if (DaisyTimeOut)
	//		StopMotor();

	if(USIC_GetRxFIFOFillingLevel(UART001_Handle0.UartRegs) >= DAISY_BUFFER_SIZE)
	{
		CCU40_CC42->TCCLR |= 3UL;	//定时器停止运行

		//Read data from UART buffer
		UART001_ReadDataBytes(&UART001_Handle0,FifoRecBuffer,DAISY_BUFFER_SIZE);
		//Assumption that communication is lost --> emtpy Receive Buffer
		if (FifoRecBuffer[DAISY_BUFFER_SIZE-1] != DAISY_STOP_BYTE)
		{
			//IO004_TogglePin(IO004_Handle1);
			USIC_FlushRxFIFO(UART001_Handle0.UartRegs);
			return;
		}

		uint8_t cmd = FifoRecBuffer[0];
		uint16_t params =  (FifoRecBuffer[1] << 8 | FifoRecBuffer[2]);

		switch (cmd)
		{
			case START_MOTOR:
				StartMotor();
				break;
			case STOP_MOTOR:
				StopMotor();
				break;
			case SET_REF_CURRENT:
				SetReferenceCurrent(params);
				break;
		}

		for(i=DAISY_MESSAGE_LENGTH; i<DAISY_BUFFER_SIZE-1; i++)
			FifoTransBuffer[i-DAISY_MESSAGE_LENGTH]=FifoRecBuffer[i];

		//Status-Code
		FifoTransBuffer[i-DAISY_MESSAGE_LENGTH]=status;
		i++;
		//Data
		FifoTransBuffer[i-DAISY_MESSAGE_LENGTH]=(uint8_t)(data >> 8);
		i++;
		FifoTransBuffer[i-DAISY_MESSAGE_LENGTH]=(uint8_t)data;
		i++;
		FifoTransBuffer[i-DAISY_MESSAGE_LENGTH]=DAISY_STOP_BYTE;
		DaisyTimeOut = 0;
		DaisyCount++;

		UART001_WriteDataBytes(&UART001_Handle0, FifoTransBuffer, DAISY_BUFFER_SIZE);
	}
}

void InitDaisyWatchDog()
{
	//Watchdog Z鋒ler f黵 start der daysi chain
	CCU40_CC41->PSC |= 0x03;	//预分频器值,(Fccu4)/8
	CCU40_CC41->PRS = 0xFFFF;	//定时器映射周期值,(32mhz/8)/65536=16.384ms
	CCU40_CC41->CRS = 0;		//定时器映射比较值，产生占空比
	CCU40->GCSS |= (0x01UL << 4);	//定时器片1映射传送置位使能

	//Interrupt Compare Match Slice 1
	CCU40_CC41->INTE |= 0x04UL;		//向上计数时比较匹配中断使能
	CCU40_CC41->SRS |= 0x04UL;		//比较匹配服务请求转发到SR1服务请求线
	NVIC_SetPriority((IRQn_Type)22, 1);	//CCU40_1_IRQn = 22; CCU40 SR1 Interrupt
	NVIC_EnableIRQ((IRQn_Type)22);

	//Enable slice
	CCU40->GIDLC |= 0x01UL << 1;	//CC41退出空闲模式
	CCU40_CC41->TCSET |= 0x01UL;	//CC41定时器运行位置位
}

void InitDaisyReset()
{
	CCU40_CC42->PSC &= 0x00;	//预分频器值,(Fccu4)
	CCU40_CC42->PRS = 0xFFFF;	//定时器映射周期值,(32mhz)/65536=2.048ms
	CCU40_CC42->CRS = 0xFFFF;		//定时器映射比较值，产生占空比
	CCU40->GCSS |= (0x01UL << 8);	//定时器片2映射传送置位使能

	//Interrupt Compare Match Slice 1
	CCU40_CC42->INTE |= 0x04UL;		//向上计数时比较匹配中断使能
	CCU40_CC42->SRS |= 0x08UL;		//比较匹配服务请求转发到SR2服务请求线
	NVIC_SetPriority((IRQn_Type)23, 1);	//CCU40_2_IRQn = 23; CCU40 SR2 Interrupt
	NVIC_EnableIRQ((IRQn_Type)23);

	//Enable slice
	CCU40->GIDLC |= 0x01UL << 2;	//CC42退出空闲模式
	//CCU40_CC42->TCSET |= 0x01UL;	//CC42定时器运行位置位
}

void DaisyrReset_ISR()	//丢掉不完整数据包
{
	CharCNT = 0;
	USIC_FlushRxFIFO(UART001_Handle0.UartRegs);
}

extern volatile bool OpenLoopFinish;

void DaisyWatchDog_ISR()
{
	static uint8_t lastCount = 0;
	static uint16_t t_counter = 0;
	
	if (lastCount == DaisyCount)
	{
		DaisyTimeOut = 1;
	}
	else
	{
		DaisyTimeOut=0;
	}
	lastCount = DaisyCount;
/*	
	//if (OpenLoopFinish != false)
	{
		t_counter++;
		switch ( t_counter )
		{
			case 200:	//16.384ms * 200 = 3.2768s
				//SetReferenceCurrent(128*3);
				StopMotor();
			break;
			case 400:
				//SetReferenceCurrent(128*4);
				StartMotor();
			break;
			case 600:
				//SetReferenceCurrent(128*5);
				StopMotor();
			break;
			case 800:
				//SetReferenceCurrent(128*6);
				StartMotor();
			break;
			case 1000:
				//SetReferenceCurrent(128*7);
				StopMotor();
			break;
			case 1200:
				//SetReferenceCurrent(128*8);
				StartMotor();
			break;
			case 1400:
				//SetReferenceCurrent(128*9);
				StopMotor();
			break;
			case 1600:
				//SetReferenceCurrent(128*10);
				StartMotor();
			break;
			case 1800:
				//SetReferenceCurrent(128*1);
				StopMotor();
			break;
			case 2000:
				//SetReferenceCurrent(128*2);
				StartMotor();
				t_counter = 0;
			break;
		}
	}
	//else
	//{
	//	t_counter = 0;
	//}
*/
}

void uart_write_printf(uint8_t * data,uint32_t len)
{
	uint32_t index;

	for (index = 0; index < len; index++)
	{
		//U0C1_UART_vSendData(data[index]);
		//UART001_WriteDataBytes(&UART001_Handle0, &data[index], 1);
	}
}

static const char *const g_pcHex = "0123456789abcdef";
void log(const char *pcString, ...)
{
    unsigned long ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg;
    char *pcStr, pcBuf[16], cFill;
    va_list vaArgP;

    va_start(vaArgP, pcString);

    while (*pcString)
    {
        for (ulIdx = 0; (pcString[ulIdx] != '%') && (pcString[ulIdx] != '\0'); ulIdx++)
        {
        }

        uart_write_printf((uint8_t *) pcString, ulIdx);

        pcString += ulIdx;

        if (*pcString == '%')
        {
            pcString++;

            ulCount = 0;
            cFill = ' ';

again:
            switch (*pcString++)
            {
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					if ((pcString[-1] == '0') && (ulCount == 0))
					{
						cFill = '0';
					}

					ulCount *= 10;
					ulCount += pcString[-1] - '0';

					goto again;
				}

				case 'c':
				{
					ulValue = va_arg(vaArgP, unsigned long);
					uart_write_printf((uint8_t *) &ulValue, 1);

					break;
				}

				case 'd':
				case 'i':
				{
					ulValue = va_arg(vaArgP, unsigned long);
					ulPos = 0;

					if ((long) ulValue < 0)
					{
						ulValue = -(long) ulValue;
						ulNeg = 1;
					}
					else
					{
						ulNeg = 0;
					}

					ulBase = 10;

					goto convert;
				}

				case 's':
				{
					pcStr = va_arg(vaArgP, char *);

					for (ulIdx = 0; pcStr[ulIdx] != '\0'; ulIdx++)
					{
					}

					uart_write_printf((uint8_t *) pcStr, ulIdx);

					if (ulCount > ulIdx)
					{
						ulCount -= ulIdx;
						while (ulCount--)
						{
							uart_write_printf((uint8_t *) " ", 1);
						}
					}
					break;
				}

				case 'u':
				{
					ulValue = va_arg(vaArgP, unsigned long);
					ulPos = 0;
					ulBase = 10;
					ulNeg = 0;
					goto convert;
				}

				case 'x':
				case 'X':
				case 'p':
				{
					ulValue = va_arg(vaArgP, unsigned long);
					ulPos = 0;
					ulBase = 16;
					ulNeg = 0;

convert:
					for (ulIdx = 1;
							(((ulIdx * ulBase) <= ulValue)
							 && (((ulIdx * ulBase) / ulBase) == ulIdx));
							ulIdx *= ulBase, ulCount--)
					{
					}

					if (ulNeg)
					{
						ulCount--;
					}

					if (ulNeg && (cFill == '0'))
					{
						pcBuf[ulPos++] = '-';

						ulNeg = 0;
					}

					if ((ulCount > 1) && (ulCount < 16))
					{
						for (ulCount--; ulCount; ulCount--)
						{
							pcBuf[ulPos++] = cFill;
						}
					}

					if (ulNeg)
					{
						pcBuf[ulPos++] = '-';
					}

					for (; ulIdx; ulIdx /= ulBase)
					{
						pcBuf[ulPos++] = g_pcHex[(ulValue / ulIdx) % ulBase];
					}

					uart_write_printf((uint8_t *) pcBuf, ulPos);
					break;
				}

				case '%':
				{
					uart_write_printf((uint8_t *) pcString - 1, 1);

					break;
				}

				default:
				{
					uart_write_printf((uint8_t *) "ERROR", 5);
					break;
				}
            }
        }
    }

    va_end(vaArgP);
}



