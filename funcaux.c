/*
 * funcaux.c
 *
 *  Created on: 3 lut 2020
 *      Author: Tomasz
 */
#include "funcaux.h"


void UstawPulse(int pulse){
	  TIM3->CCR1=pulse;
	  TIM3->CCR2=pulse;
	  TIM3->CCR3=pulse;
}


AUX_STATUS UstawGranice(float *dol, float *gora){
	float BH1750_data;
	int margines;

	UstawPulse(0);
	HAL_Delay(300);
	if(BH1750_OK == BH1750_read(&BH1750_data))
	{
		*dol = BH1750_data;
	}
	else
		return AUX_ERROR;

	UstawPulse(1000);
	HAL_Delay(300);
	if(BH1750_OK == BH1750_read(&BH1750_data))
	{
		*gora = BH1750_data-margines;
		if(gora<dol)
		{
			*gora=BH1750_data;
	  	}
	}
	else
		return AUX_ERROR;
	return AUX_OK;

}
