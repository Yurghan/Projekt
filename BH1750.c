/**
  * @file           BH1750.c
  * @brief          Obsługa czujnika BH1750
  * @author			Nieznany
  */

#include "main.h"
#include "i2c.h"

#include "bh1750.h"

I2C_HandleTypeDef 	*bh1750_i2c;
bh1750_mode 		Bh1750_Mode;
uint8_t 			Bh1750_Mtreg;


BH1750_STATUS BH1750_init(I2C_HandleTypeDef *hi2c)
{
	bh1750_i2c = hi2c;
	if(BH1750_OK == BH1750_reset())
	{
		if(BH1750_OK == BH1750_set_mtreg(BH1750_DEFAULT_MTREG)) // Set default value;
			return BH1750_OK;
	}
	return BH1750_ERROR;
}


BH1750_STATUS BH1750_reset(void)
{
	uint8_t tmp = 0x07;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}


BH1750_STATUS BH1750_power_state(uint8_t power_on)
{
	power_on = (power_on? 1:0);
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &power_on, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}


BH1750_STATUS BH1750_set_mode(bh1750_mode mode)
{
	if(!((mode >> 4) || (mode >> 5))) return BH1750_ERROR;
	if((mode & 0x0F) > 3) return BH1750_ERROR;

	Bh1750_Mode = mode;
	if(HAL_OK == HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &mode, 1, 10))
		return BH1750_OK;

	return BH1750_ERROR;
}


BH1750_STATUS BH1750_set_mtreg(uint8_t Mtreg)
{
	HAL_StatusTypeDef retCode;
	if (Mtreg < 31 || Mtreg > 254) {
		return BH1750_ERROR;
	}

	Bh1750_Mtreg = Mtreg;

	uint8_t tmp[2];

	tmp[0] = (0x40 | (Mtreg >> 5));
	tmp[1] = (0x60 | (Mtreg & 0x1F));

	retCode = HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp[0], 1, 10);
	if (HAL_OK != retCode) {
		return BH1750_ERROR;
	}

	retCode = HAL_I2C_Master_Transmit(bh1750_i2c, BH1750_ADDRESS, &tmp[1], 1, 10);
	if (HAL_OK == retCode) {
		return BH1750_OK;
	}

	return BH1750_ERROR;
}


BH1750_STATUS BH1750_trigger_manual_conversion(void)
{
	if(BH1750_OK == BH1750_set_mode(Bh1750_Mode))
	{
		return BH1750_OK;
	}
	return BH1750_ERROR;
}


BH1750_STATUS BH1750_read(float *Result)
{
	float result;
	uint8_t tmp[2];

	if(HAL_OK == HAL_I2C_Master_Receive(bh1750_i2c, BH1750_ADDRESS, tmp, 2, 10))
	{
		result = (tmp[0] << 8) | (tmp[1]);

		if(Bh1750_Mtreg != BH1750_DEFAULT_MTREG)
		{
			result *= (float)((uint8_t)BH1750_DEFAULT_MTREG/(float)Bh1750_Mtreg);
		}

		if(Bh1750_Mode == ONETIME_HIGH_RES_MODE_2 || Bh1750_Mode == CONTINUOUS_HIGH_RES_MODE_2)
		{
			result /= 2.0;
		}

		*Result = result / (float)BH1750_CONVERSION_FACTOR;
		return BH1750_OK;
	}
	return BH1750_ERROR;
}
