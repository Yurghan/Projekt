/**
  * @file           BH1750.h
  * @brief          Ten plik zawiera definicje funkcji obsługi czujnika BH1750
  * @author			Nieznany
  */

#ifndef INC_BM1750_H_
#define INC_BM1750_H_


#define BH1750_ADDRESS			(0x23<<1)

#define	BH1750_POWER_DOWN		0x00
#define	BH1750_POWER_ON			0x01
#define	BH1750_RESET			0x07
#define	BH1750_DEFAULT_MTREG	69

#define BH1750_CONVERSION_FACTOR	1.2

typedef enum {
BH1750_OK		= 0,
BH1750_ERROR	= 1
} BH1750_STATUS;

typedef enum
{
    CONTINUOUS_HIGH_RES_MODE  	= 0x10, /*!< wysoka rozdzielczość w trybie ciągłym */
    CONTINUOUS_HIGH_RES_MODE_2 	= 0x11, /*!< wysoka rozdzielczość w trybie ciągłym 2 */
    CONTINUOUS_LOW_RES_MODE 	= 0x13, /*!< niska rozdzielczość w trybie ciągłym */
    ONETIME_HIGH_RES_MODE 		= 0x20, /*!< wysoka rozdzielczość w trybie pojedynczego odczytu */
    ONETIME_HIGH_RES_MODE_2 	= 0x21, /*!< wysoka rozdzielczość w trybie pojedynczego odczytu 2 */
    ONETIME_LOW_RES_MODE 		= 0x23 /*!< niska rozdzielczość w trybie pojedynczego odczytu */
}bh1750_mode;

/*! \enum bh1750_mode
 * Zawiera zdefiniowane wartości służące do wyboru trybu pracy czujnika
 */

BH1750_STATUS BH1750_init(I2C_HandleTypeDef *hi2c);
/*! \fn BH1750_STATUS BH1750_init(I2C_HandleTypeDef *hi2c)
 *  \brief Funkcja inicjująca pracę czujnika
 *  \param hi2c wybór oprawy HAL i2c
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_reset(void);
/*! \fn BH1750_STATUS BH1750_reset(void)
 *  \brief Funkcja resetująca czujnik
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_power_state(uint8_t power_on);
/*! \fn BH1750_STATUS BH1750_power_state(uint8_t power_on)
 *  \brief Włączenie/wyłączenie czujnika
 *  \param power_on 1 żeby uruchomić, 0 żeby wyłączyć
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_set_mtreg(uint8_t Mtreg);
/*! \fn BH1750_STATUS BH1750_set_mtreg(uint8_t Mtreg)
 *  \brief Ustawienie rejestru measurement time, służącego do zmiany okresu pomiaru
 *  \param Mtreg przyjmuje wartość od 0001_1111 do 1111_1110
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_set_mode(bh1750_mode mode);
/*! \fn BH1750_STATUS BH1750_set_mode(bh1750_mode mode)
 *  \brief Zmiana trybu pracy czujnika
 *  \param mode wybór trybu
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_trigger_manual_conversion(void);
/*! \fn BH1750_STATUS BH1750_trigger_manual_conversion(void)
 *  \brief Wyzwalanie ręcznej konwersji
 *  \return BH1750_STATUS
 */
BH1750_STATUS BH1750_read(float *result);
/*! \fn BH1750_STATUS BH1750_read(float *result)
 *  \brief Funkcja odczytywania wartości natężenia światła
 *  \param result zmiennoprzycinkowa wartość zawierająca wynik, przekazywana przez referencję
 *  \return BH1750_STATUS
 */


#endif /* INC_BM1750_H_ */
