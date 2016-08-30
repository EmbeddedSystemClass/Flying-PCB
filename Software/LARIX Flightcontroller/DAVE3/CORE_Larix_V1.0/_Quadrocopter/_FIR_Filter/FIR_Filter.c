/*
 * FIR_Filter.c
 *
 *  Created on: 30.01.2016
 *      Author: Heinz-Peter Liechtenecker
 */

#include "FIR_Filter.h"


struct structFIR FIR_FILTER(struct structFIR temp, float NewValue)
{
	//Neuen Wert in den Ringbuffer schreiben
	temp.CIRC_BUFF[temp.POS] = NewValue;

	//Neuen Zeiger auf das Array berrechnen
	temp.POS = ((temp.POS+1)%FIR_SIZE);

	//Rückgabewert erstmal resetten
	temp.VALUE = 0;

	//Eigentliches Werteberechnung durch Faltungsintegral
	for(int i = 0; i < FIR_SIZE; i++)
	{
		temp.VALUE += (temp.FIR_COEFF[i] * temp.CIRC_BUFF[(temp.POS+i)%FIR_SIZE]);
	}

	//Rückgabewert des neuen Buffers
	return temp;
}


struct structFIR Initialize_FIR_Filter(struct structFIR temp, int type)
{
	//Alle Werte im Ringbuffer auf 0 setzen
	for(int i = 0; i < FIR_SIZE; i++)
		temp.CIRC_BUFF[i] = 0.0;

	//Pointer im Circ-Buffer auf 0 setzen
	temp.POS = 0;

	//Filter-Koeffizienten erstellen und im Array befüllen
	switch(type)
	{
	case MOVING_AVERAGE:
	default:
		for(int i = 0; i < FIR_SIZE; i++)
			temp.FIR_COEFF[i] = 1.0/FIR_SIZE;
		break;
	}

	return temp;
}
