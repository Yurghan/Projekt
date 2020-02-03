/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BH1750.h"
#include "stdio.h"
#include "string.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float BH1750_data, PID_ERROR, zakres_dolny, zakres_gorny;
int WAR_ZADANA, BH1750_data_int, Duty;
char buffer[80], buffer2[5];
uint8_t rozmiar, size;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BH1750_init(&hi2c1);


  arm_pid_instance_f32 PID;

  PID.Kp =0;
  PID.Ki =0.025;
  PID.Kd =0;



  arm_pid_init_f32(&PID, 0);

  if(BH1750_OK == BH1750_set_mode(CONTINUOUS_HIGH_RES_MODE_2))
  {
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }
  TIM3->CCR3=0;
  TIM3->CCR1=0;
  TIM3->CCR2=0;
  HAL_Delay(300);
  if(BH1750_OK == BH1750_read(&BH1750_data))
  	{
  	 zakres_dolny = BH1750_data;
  	}
  TIM3->CCR3=1000;
  TIM3->CCR1=1000;
  TIM3->CCR2=1000;
  HAL_Delay(300);
  if(BH1750_OK == BH1750_read(&BH1750_data))
  {
	  zakres_gorny = BH1750_data-200;
	  if(zakres_gorny<zakres_dolny)
	  {
		  zakres_gorny=BH1750_data;
	  }
  }
  /* USER CODE END 2 */
 

  size = sprintf(buffer, "Podaj wartosc referencyjna w granicach od: %d do %d lux. \n\r", (int)zakres_dolny, (int)zakres_gorny);
  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
  HAL_Delay(100);
  size = sprintf(buffer, "Wartosc podac w formacie \"00xxx\" (w luksach), np. 00500 lub 54000.\n\r", (int)zakres_dolny, (int)zakres_gorny);
  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(WAR_ZADANA>zakres_gorny)
	  {
		  WAR_ZADANA=-1;
		  size = sprintf(buffer, "Podano wartosc powyzej zakresu. Wprowadz mniejsza wartosc. \n\r", (int)zakres_dolny, (int)zakres_gorny);
		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
		  HAL_Delay(100);
	  }

	  if(WAR_ZADANA<zakres_dolny && WAR_ZADANA>0)
	  {
		  WAR_ZADANA=-1;
		  size = sprintf(buffer, "Podano wartosc ponizej zakresu. Wprowadz wieksza wartosc. \n\r", (int)zakres_dolny, (int)zakres_gorny);
		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
		  HAL_Delay(100);
	  }
	  if(BH1750_OK == BH1750_read(&BH1750_data) && WAR_ZADANA>0)
	  {
			BH1750_data_int = BH1750_data;
			size = sprintf(buffer, "BH1750 Lux: %d, Duty: %d, wartosc zadana: %d lux \n\r", BH1750_data_int, Duty,WAR_ZADANA);
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
	  }



	HAL_Delay(120);
	PID_ERROR =  WAR_ZADANA- BH1750_data;
	Duty = arm_pid_f32(&PID, PID_ERROR);

	if(Duty<0)
		Duty=0;
	if(Duty>1000)
		Duty=1000;



	TIM3->CCR3=Duty;
	TIM3->CCR1=Duty;
	TIM3->CCR2=Duty;


	HAL_UART_Receive_IT(&huart3,(uint8_t*)buffer2,5);


	//
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3)
	{
		if(buffer2[0]!='\0')
		{
			rozmiar = sscanf(buffer2,"%d",&WAR_ZADANA);
			memset(buffer2,'\0',sizeof(buffer2));
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
