/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           main.c
  * @brief          Main program body
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

/*! \mainpage Projekt zaliczeniowy - systemy mikroprocesorowe
 *
 * \section intro_sec Wstęp
 * To jest dokumentacja do projektu zaliczeniowego
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
#include "funcaux.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER1_SIZE 40
#define BUFFER2_SIZE 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/// zmienna przechowująca odczytaną wartość natężenia światła
float BH1750_data;
float PID_ERROR;
float zakres_dolny;
float zakres_gorny;

/// zmienna do przechowywania zadanej wartości
int WAR_ZADANA;
int BH1750_data_int;
int Duty;
int war_zadana_temp;
/// Decyduje o wyborze ustawianej cyfry w trybie ręcznym
int decyzja = 1;
int cyfra = 0;

/// Tablica do przechowywania wysyłanych komunikatów
char buffer[BUFFER1_SIZE];
/// Tablica do przechowywania odbieranych komunikatów
char buffer2[BUFFER2_SIZE];
char zgoda_tx;

uint8_t size;

arm_pid_instance_f32 PID;




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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Inicjalizacja timera 4, ustawienie wartości prescalera oraz rejestru Auto Reload */
  HAL_TIM_Base_Start_IT(&htim4);

  TIM4->ARR=4999;
  TIM4->PSC=7199;

  TIM2->ARR=999;
  TIM2->PSC=7199;

  /* Inicjalizacja czujnika*/
  BH1750_init(&hi2c1);

  /* Ustawienie wartości członów PID oraz inicjalizja instancji PID*/
  PID.Kp =0;
  PID.Ki =0.025;
  PID.Kd =0;
  arm_pid_init_f32(&PID, 0);

  /* Uruchomienie PWM*/
  if(BH1750_OK == BH1750_set_mode(CONTINUOUS_HIGH_RES_MODE_2))
  {
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }

  /* Ustawienie zakresu pomiaru oraz wyświetlenie informacji do użytkownika*/
  if(AUX_OK==AUX_UstawGranice( &zakres_dolny, &zakres_gorny))
  {
	  size = sprintf(buffer, "min %d, max %d \n\r", (int)zakres_dolny, (int)zakres_gorny);
	 		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
	 		  HAL_Delay(100);
  }

  /* Uruchomienie timera 2*/
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Sprawdzenie czy wartość referencyjna podana przez użytkownika znajduje się w zakresie pomiarowym, jeśli nie,
	     wystawiany jest komunikat z prośbą o ponowne wpisanie wartości*/
	if(WAR_ZADANA>zakres_gorny)
	{
		size = sprintf(buffer, "Podano wartosc %d. Wprowadz wartosc miedzy %d, a %d \n\r",
			WAR_ZADANA, (int)zakres_dolny, (int)zakres_gorny);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
		WAR_ZADANA=-1;
	}

	if(WAR_ZADANA<zakres_dolny && WAR_ZADANA>0)
	{
		size = sprintf(buffer, "Podano wartosc %d. Wprowadz wartosc miedzy %d, a %d \n\r",
			WAR_ZADANA, (int)zakres_dolny, (int)zakres_gorny);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
		WAR_ZADANA=-1;
	}
	/* Po zadaniu wartości referncyjnej z zakresu pomiarowego, następuje wyświetlenie komunikatu o aktualnie odczytywanej wartości natężenia,
	   wypełnieniu PWM w % oraz aktualnie zadanej wartości*/
	if(BH1750_OK == BH1750_read(&BH1750_data) && WAR_ZADANA>0)
	{
		if(zgoda_tx)
		{
			zgoda_tx=0;
			size = sprintf(buffer, "BH1750 Lux: %d, Duty: %d, wartosc zadana: %d lux \n\r",
			(int)BH1750_data, Duty/10,WAR_ZADANA);
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, size);
		}
	}


	/* Odczytywanie wartości referencyjnej. Wysyłana ramka musi zakończyć się terminatorem w postaci EOF '\n'*/
	HAL_UART_Receive_IT(&huart3,(uint8_t*)buffer2,BUFFER2_SIZE);
	if(buffer2[0]!='\0')
	{
		for(int i=0;i<BUFFER2_SIZE;i++)
		{
			if(buffer2[i]=='\n')
			{
				sscanf(buffer2,"%d",&WAR_ZADANA);
				memset(buffer2,'\0',sizeof(buffer2));
				huart3.pRxBuffPtr  = (uint8_t*)buffer2;
			}
		}
	}

	/* Krótki delay w celu wyeliminowania zwielokrotnienia odczytywanych sygnałów wejść zewnętrznych*/
	HAL_Delay(40);
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

/**
  * @brief  Odwołania upłyniętego okresu w trybie nie blokującym
  *
  *	@detail Instancja TIM4 odpowiada za udzielanie zgody na wysyłanie komunikatów maksymalnie 2 razy na sekundę. Instancja TIM2 odpowiada za ustawianie wypełnienia PWM.
  * @param  htim oprawa TIM
  * @retval None
  *
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4)
	{
		if(zgoda_tx == 0)
		{
			zgoda_tx = 1;
		}
	}
	if(htim->Instance==TIM2)
	{
		PID_ERROR = WAR_ZADANA - BH1750_data;
		Duty = arm_pid_f32(&PID, PID_ERROR);
		if(Duty<0)
		{
			Duty = 0;
		}
		if(Duty > 1000)
		{
			Duty = 1000;
		}
		AUX_UstawPulse(Duty);
	}
}

/**
  * @brief  Wykrywanie odwołań lini EXTI
  *
  * Jeden przycisk zmienia miejsce ustawianej cyfry na kolejno: jedności, dziesiątek, setek,
  * tysięcy i dziesiątek tysięcy. Dwa przyciski do zmiany wartości ustawianej cyfry.
  * Po wciśnięciu przycisku zmiany miejsca pięć razy następuje zmiana wartości referncyjnej.
  * @param  GPIO_Pin Określa piny podłączone do lini EXTI
  *
  *
  * @retval None
  *
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_0)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
		war_zadana_temp += AUX_Potega(10, decyzja-1) * cyfra;
		if (decyzja<5)
		{
			decyzja++;
			cyfra=0;
		}
		else
		{
			decyzja=1;
			WAR_ZADANA = war_zadana_temp;
			HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
			war_zadana_temp=0;
			cyfra=0;
		}
	}

	if(GPIO_Pin == GPIO_PIN_3)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
		if(cyfra>0)
			cyfra--;
		else
			cyfra=9;
	}

	if(GPIO_Pin == GPIO_PIN_4)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
			if(cyfra<9)
				cyfra++;
			else
				cyfra=0;
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
