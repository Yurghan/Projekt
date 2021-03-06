/**
  * @file           funcaux.c
  * @brief          Funkcje pomocnicze
  * @authors		Miłosz Topolewski
  * 				Tomasz Łukomski
  */

#include "funcaux.h"
#include "main.h"
#include "stdarg.h"

void AUX_UstawPulse(int pulse){
	  TIM3->CCR1=pulse;
	  TIM3->CCR2=pulse;
	  TIM3->CCR3=pulse;
}


AUX_STATUS AUX_UstawGranice(float *dol, float *gora){
	float BH1750_data;
	int margines=100;

	AUX_UstawPulse(0);
	HAL_Delay(300);
	if(BH1750_OK == BH1750_read(&BH1750_data))
	{
		if(BH1750_data>66000)
			return AUX_ERROR;
		*dol = BH1750_data+margines;
	}
	else
		return AUX_ERROR;

	AUX_UstawPulse(1000);
	HAL_Delay(300);
	if(BH1750_OK == BH1750_read(&BH1750_data))
	{
		*gora = BH1750_data-margines;
		if(gora<dol)
		{
		if(BH1750_data>66000){
			return AUX_ERROR;
		}
		*gora=BH1750_data;
	  	}
	}
	else
		return AUX_ERROR;

	return AUX_OK;

}

int AUX_Potega(int baza, unsigned int wykladnik)
{
	int result = 1;
	for(int i=0;i<wykladnik;i++)
	{
		result*=baza;
	}
	return result;
}

