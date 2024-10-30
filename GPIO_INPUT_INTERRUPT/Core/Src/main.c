/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Key_1_Handler(uint32_t TimePress, GPIO_TypeDef* listport[], uint16_t listpin[]);
void Toggle_Leds(GPIO_TypeDef* ListPort[], uint16_t ListPin[]);
void CountUp(GPIO_TypeDef* listport[], uint16_t listpin[]);
//void Control_All_Leds(bool state);
void Off_All(void);
void CountDown(GPIO_TypeDef* listport[], uint16_t listpin[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile bool Inter_Key_1 = 0;
volatile bool Inter_Key_2 = 0;
volatile uint32_t Last_Interrupt_Time = 0;
volatile bool counter = 0;
volatile bool Key_State = 0;
volatile bool Key_Stable = 0;
volatile uint32_t Time_Press_Start = 0;
volatile uint32_t Time_Press_Duration = 0;
volatile uint32_t Time_Press_Duration_Key_2 = 0;

GPIO_TypeDef* led_port[] = {led_1_GPIO_Port, led_2_GPIO_Port, led_3_GPIO_Port, led_4_GPIO_Port,
		  led_5_GPIO_Port,led_6_GPIO_Port,led_7_GPIO_Port,led_8_GPIO_Port};

uint16_t led_pin[] = {led_1_Pin, led_2_Pin, led_3_Pin, led_4_Pin, led_5_Pin, led_6_Pin,
		  led_7_Pin,led_8_Pin};
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
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(Inter_Key_1 == 1)
	{

		if(Key_Stable == 1)
		{
			Key_Stable = 0;
			Inter_Key_1 = 0;
			Key_1_Handler(Time_Press_Duration, led_port, led_pin);
		}
	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_1_Pin|led_2_Pin|led_3_Pin|led_4_Pin
                          |led_5_Pin|led_6_Pin|led_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_8_GPIO_Port, led_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_1_Pin led_2_Pin led_3_Pin led_4_Pin
                           led_5_Pin led_6_Pin led_7_Pin */
  GPIO_InitStruct.Pin = led_1_Pin|led_2_Pin|led_3_Pin|led_4_Pin
                          |led_5_Pin|led_6_Pin|led_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led_8_Pin */
  GPIO_InitStruct.Pin = led_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : key_1_Pin key_2_Pin */
  GPIO_InitStruct.Pin = key_1_Pin|key_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Key_1_Handler(uint32_t TimePress, GPIO_TypeDef* listport[], uint16_t listpin[])
{

	if(TimePress > 0 && TimePress < 1000)
	{

		Toggle_Leds(listport, listpin);
	}else if(TimePress > 1000 && TimePress < 5000 )
	{

		CountUp(listport, listpin);
	}else if(TimePress > 5000)
	{

		CountDown(listport, listpin);
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == key_1_Pin )
	{
		uint32_t CurrentTime = HAL_GetTick();

		if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == 0)
				{
				    if ((CurrentTime - Last_Interrupt_Time > 50) && (Key_State == 0))
				    {
				        Last_Interrupt_Time = CurrentTime;
				        Inter_Key_1 = 1;
				        Time_Press_Start = CurrentTime;
				        Key_State = 1;
				        Key_Stable = 0;
				    }
				}
				else
				{
				    if ((CurrentTime - Last_Interrupt_Time > 50) && (Key_State == 1))
				    {
				        Last_Interrupt_Time = CurrentTime;
				        Time_Press_Duration = CurrentTime - Time_Press_Start;
				        Key_State = 0;
				        Key_Stable = 1;
				        counter = 0;
				    }
				}

	}else if(GPIO_Pin == key_2_Pin)
	{
		counter = 0;
		Time_Press_Duration = 0;
		Off_All();
	}

}

//void Control_All_Leds(bool state)
//{
//	for(int led = 0; led < 8; led++)
//	{
//		HAL_GPIO_WritePin(, led_pin[led], state);
//	}
//}

void Off_All(void)
{
	HAL_GPIO_WritePin(led_1_GPIO_Port,led_1_Pin, 0);
	HAL_GPIO_WritePin(led_2_GPIO_Port,led_2_Pin, 0);
	HAL_GPIO_WritePin(led_3_GPIO_Port,led_3_Pin, 0);
	HAL_GPIO_WritePin(led_4_GPIO_Port,led_4_Pin, 0);
	HAL_GPIO_WritePin(led_5_GPIO_Port,led_5_Pin, 0);
	HAL_GPIO_WritePin(led_6_GPIO_Port,led_6_Pin, 0);
	HAL_GPIO_WritePin(led_7_GPIO_Port,led_7_Pin, 0);
	HAL_GPIO_WritePin(led_8_GPIO_Port,led_8_Pin, 0);

}
void Toggle_Leds(GPIO_TypeDef* ListPort[], uint16_t ListPin[])
{

	for (int led = 0; led < 8; led++)
	{
		HAL_GPIO_TogglePin(ListPort[led], ListPin[led]);
	}
}

void CountUp(GPIO_TypeDef* listport[], uint16_t listpin[])
{
	counter = 1;
	for(uint8_t count = 0; count < 256; count++)
	{
		if(counter == 0)
		{

			break;
		}
		for(int led = 0 ; led < 8; led++)
		{
			if(count & (1 << led))
			{
				HAL_GPIO_WritePin(listport[led], listpin[led],1);
			}else
			{
				HAL_GPIO_WritePin(listport[led], listpin[led],0);
			}
		}
		HAL_Delay(1000);
	}
	counter = 0;
}

void CountDown(GPIO_TypeDef* listport[], uint16_t listpin[])
{
	counter = 1;
	for(uint8_t count = 255; count  >= 0; count--)
	{
		if(counter == 0)
		{

			break;
		}
		for(int led = 0 ; led < 8; led++)
		{
			if(count & (1 << led))
			{
				HAL_GPIO_WritePin(listport[led], listpin[led],1);
			}else
			{
				HAL_GPIO_WritePin(listport[led], listpin[led],0);
			}
		}
		HAL_Delay(1000);
	}
	counter = 0;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
