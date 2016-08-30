/*
 * DaisyChain.h
 *
 *  Created on: 02.05.2015
 *      Author: Andreas
 */

#ifndef DAISYCHAIN_H_
#define DAISYCHAIN_H_

#include "DaisyCodes.h"
#include "BlockCommutation.h"

#define HARDWARE_BUFFER_SIZE 32

#define DaisyWatchDog_ISR	IRQ_Hdlr_22
#define DaisyrReset_ISR		IRQ_Hdlr_23

void DaisyChain();
void InitDaisyWatchDog();
void InitDaisyReset();

#endif /* DAISYCHAIN_H_ */
