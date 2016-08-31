/*
 * FIR_Filter.h
 *
 *  Created on: 30.01.2016
 *      Author: Heinz-Peter Liechtenecker
 */

#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#define FIR_SIZE           6
#define MOVING_AVERAGE     0
//
struct structFIR {
	float             CIRC_BUFF[FIR_SIZE];
	float           FIR_COEFF[FIR_SIZE];
	float           VALUE;
	int             POS;
};

struct structFIR FIR_FILTER(struct structFIR temp, float NewValue);
struct structFIR Initialize_FIR_Filter(struct structFIR temp, int type);
#endif /* FIR_FILTER_H_ */
