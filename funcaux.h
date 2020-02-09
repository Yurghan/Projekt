/**
  * @file           funcaux.h
  * @brief          Ten plik zawiera definicje funkcji pomocniczych
  * @authors		Miłosz Topolewski
  * 				Tomasz Łukomski
  */

#ifndef INC_FUNCAUX_H_
#define INC_FUNCAUX_H_


#include "tim.h"
#include "BH1750.h"
#include "usart.h"

typedef enum {
	AUX_OK		= 0,
	AUX_ERROR	= 1
} AUX_STATUS;


void AUX_UstawPulse(int pulse);
/*! \fn void AUX_UstawPulse(int pulse)
 *  \brief Ustawiane wypełnienia PWM wszystkich kolorów diody
 *  \param pulse Zadana wartość TIM3->CCR(1-3)
 *  \warning Ta funkcja zmienia tylko wypełnienie PWM timera 3
 *  \return None
 */
AUX_STATUS AUX_UstawGranice(float *dol, float *gora);
/*! \fn AUX_STATUS AUX_UstawGranice(float *dol, float *gora);
 *  \brief Ustawia granice które może odczytać czujnik
 *  \param dol dolna granica
 *  \param gora gorna granica
 *  \return AUX_STATUS
 */
int AUX_Potega(int baza, unsigned int wykladnik);
/*! \fn int AUX_Potega(int baza, unsigned int wykladnik)
 *  \brief Potęgowanie na liczbach całkowitych
 *  \param baza podstawa
 *  \param wykladnik nieujemny wykładnik
 *  \return int
 */
#endif /* INC_FUNCAUX_H_ */
