/*
 * funcaux.h
 *
 *  Created on: 3 lut 2020
 *      Author: Tomasz
 */

#ifndef INC_FUNCAUX_H_
#define INC_FUNCAUX_H_


#include "tim.h"
#include "BH1750.h"

typedef enum {
	AUX_OK		= 0,
	AUX_ERROR	= 1
} AUX_STATUS;

void UstawPulse(int pulse);
AUX_STATUS UstawGranice(float *dol, float *gora);
#endif /* INC_FUNCAUX_H_ */
