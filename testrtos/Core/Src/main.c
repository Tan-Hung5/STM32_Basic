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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osMutexId myMutex01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool adc_flag = 0;
uint32_t adcValue;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) 
    {
		adc_flag = 1;
        adcValue = HAL_ADC_GetValue(hadc);
		HAL_ADC_Start_IT(&hadc1);
	}
}


const uint8_t DS1307_ADDRESS_WRITE =  0xD0; 
const uint8_t DS1307_ADDRESS_READ =  0xD1;
const uint8_t LCD_ADDRESS = 0x4E;
uint8_t res[50];

uint8_t data_buffer[50];
uint8_t data_receive;

uint32_t last_time_IT = 0;


GPIO_TypeDef* led_port[] = {led_1_GPIO_Port, led_2_GPIO_Port, led_3_GPIO_Port, led_4_GPIO_Port,
		  led_5_GPIO_Port,led_6_GPIO_Port,led_7_GPIO_Port,led_8_GPIO_Port};

uint16_t led_pin[] = {led_1_Pin, led_2_Pin, led_3_Pin, led_4_Pin, led_5_Pin, led_6_Pin,
		  led_7_Pin,led_8_Pin};


void Binary_Count_Up(GPIO_TypeDef* listport[], uint16_t listpin[])
{
	
	for(uint8_t count = 0; count < 256; count++)
	{	
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
		osDelay(1000);
	}
}


void get_adc_value(uint16_t adc_val[]) {
  int chanel = 0;
  while (1) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_val[chanel] = HAL_ADC_GetValue(&hadc1);
    chanel++;
    if (chanel == 3) {
      return;
    }
    osDelay(10);
  }
}


uint8_t Dec_to_BCD(uint8_t dec) 
{
	dec = ((dec / 10) << 4) | (dec % 10);

    return dec; 
}

uint8_t BCD_to_Dec(uint8_t bcd) 
{
	bcd = ((bcd >> 4) * 10) + (bcd & 0x0F);

    return bcd;
}
void DS1307_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds) 
{
    uint8_t data[4];

	data[0] = 0x00;
    data[1] = Dec_to_BCD(seconds);
	data[2] = Dec_to_BCD(minutes);
    data[3] = Dec_to_BCD(hours);
   
	
	HAL_I2C_Master_Transmit(&hi2c1,DS1307_ADDRESS_WRITE , &data[0], 4, 100);
    
}

void DS1307_SetDate(uint8_t day, uint8_t month, uint16_t year) 
{
    uint8_t data[4];
	
	data[0] = 0x04;
    data[1] = Dec_to_BCD(day);
    data[2] = Dec_to_BCD(month);  
	data[3] = Dec_to_BCD(year);      

	HAL_I2C_Master_Transmit(&hi2c1, DS1307_ADDRESS_WRITE , &data[0], 4, 100);
}

void DS1307_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) 
{
    uint8_t data[3];
	uint8_t reg = 0x00;

	HAL_I2C_Master_Transmit(&hi2c1, DS1307_ADDRESS_WRITE , &reg, 1, 10); 
	HAL_I2C_Master_Receive(&hi2c1, DS1307_ADDRESS_READ, &data[0], 3, 50);     
    *seconds = BCD_to_Dec(data[0]);
	*minutes = BCD_to_Dec(data[1]);
    *hours   = BCD_to_Dec(data[2]);
}

void DS1307_GetDate(uint8_t *day, uint8_t *month, uint16_t *year) 
{
    uint8_t data[3];
	uint8_t reg = 0x04;

	HAL_I2C_Master_Transmit(&hi2c1, DS1307_ADDRESS_WRITE,&reg, 1, 100); 
	HAL_I2C_Master_Receive(&hi2c1, DS1307_ADDRESS_READ, &data[0], 3, 100);   
    *day    = BCD_to_Dec(data[0]);
    *month  = BCD_to_Dec(data[1]);
    *year   = 2000 + BCD_to_Dec(data[2]);
}


void lcd_write_nibble(uint8_t rs, uint8_t data)
{
	uint8_t dat_lcd;

	dat_lcd = ((data << 4) & 0xF0) | 0x0C | rs;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, &dat_lcd, 1, 20);

	dat_lcd = ((data << 4) & 0xF0) | 0x08 | rs;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, &dat_lcd, 1, 20);


}


void lcd_send_cmd(uint8_t cmd)
{
	lcd_write_nibble(0, (cmd>>4)&0x0F);
	lcd_write_nibble(0, cmd&0x0F);

	HAL_Delay(2);
}


void lcd_send_data(uint8_t data)
{
	lcd_write_nibble(1, (data>>4)&0x0F);
	lcd_write_nibble(1, data & 0x0F);

	HAL_Delay(2);
}


void lcd_init(void)
{
	HAL_Delay(20);

	lcd_write_nibble(0, 0x03); HAL_Delay(5);
	lcd_write_nibble(0, 0x03); HAL_Delay(1);
	lcd_write_nibble(0, 0x03); HAL_Delay(1);

	lcd_write_nibble(0, 0x02); HAL_Delay(1);

	lcd_send_cmd(0x28);
	lcd_send_cmd(0x0C);
	lcd_send_cmd(0x01);
	lcd_send_cmd(0x06);
}


void lcd_send_string(char *data)
{
	while(*data)
	{

		lcd_send_data(*data++);
	}
}


void lcd_gotoxy(uint8_t row, uint8_t col)
{
	uint8_t coordinates = 0;

	switch(row)
	{
	case 0:
		coordinates = 0x80 | col;
		break;
	case 1:
		coordinates = 0xC0 | col;
		break;
	}
	lcd_send_cmd(coordinates);
}



void lcd_clear(void)
{
    lcd_send_cmd(0x01);  
	HAL_Delay(2);
}

osMutexId lcdMutexHandle;
void LCD_Write(int row, int col, char* data)
{
	if (osMutexWait(lcdMutexHandle, osWaitForever) == osOK)
	{
		lcd_gotoxy(row, col);
		lcd_send_string(data);
			osMutexRelease(lcdMutexHandle);
	}
}


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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_clear();
	
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	lcdMutexHandle = osMutexCreate(osMutex(myMutex01));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityRealtime, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, led_1_Pin|led_2_Pin|led_3_Pin|led_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_5_Pin|led_6_Pin|led_7_Pin|led_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_1_Pin led_2_Pin led_3_Pin led_4_Pin */
  GPIO_InitStruct.Pin = led_1_Pin|led_2_Pin|led_3_Pin|led_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : led_5_Pin led_6_Pin led_7_Pin led_8_Pin */
  GPIO_InitStruct.Pin = led_5_Pin|led_6_Pin|led_7_Pin|led_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	
  /* Infinite loop */
  for(;;)
  {

	//Binary_Count_Up(led_port, led_pin);	
   
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t data_trans[50];
	
	uint16_t t;
  /* Infinite loop */
  for(;;)
  {
	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
	 t = HAL_ADC_GetValue(&hadc1);
	sprintf((char*)data_trans, "temp: %d",(int)t);
	HAL_UART_Transmit(&huart2, &data_trans[0], strlen((char*)data_trans), 100);
	
	
	osDelay(1000);
    
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

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
