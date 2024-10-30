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
#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_write_nibble(uint8_t rs, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, rs);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);




	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (data>>3)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (data>>2)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (data>>1)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (data>>0)&0x01);

	for(uint8_t i = 0; i < 72; i++) __asm("NOP");

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	for(uint8_t i = 0; i < 72; i++) __asm("NOP");
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


void lcd_display(char *data)
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
}

uint8_t data_receive[1];
uint8_t data_buffer[20];
uint8_t response[50];
int package_size = 0;
int rx_index = 0;
volatile uint32_t last_receive_time = 0;




bool check_char(uint8_t data_buffer[])
{

	if(data_buffer[0] != '*' || data_buffer[package_size - 1] != '#')
	{
		return 0;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "char ok\r\n", 11,100);
	return 1;
}

bool check_len(uint8_t data_buffer[],int data_size,int data_size_num )
{

	if(data_size < 1 || data_size > 16)
	{
		return 1;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "1 ok\r\n", 8,100);
	if(data_size_num < 1 || data_size_num > 16)
	{
		return 0;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "2 ok\r\n", 8,100);
	if(data_size != data_size_num)
	{
		return 0;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "3 ok\r\n", 8,100);
	return 1;
}

void display_data(char* data)
{
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_display(data);
}

char* getdata_from_buffer(uint8_t data_buffer[],int size)
{
	uint8_t data[size+1];
	for(int index = 0; index <= size; index++)
	{
		data[index] = data_buffer[index+2];
	}

	static char lcd_string[20];
	memcpy(lcd_string, data, size);
	lcd_string[size] = '\0';
	return lcd_string;
}

void check_package(uint8_t data_buffer[])
{
	int data_size = package_size - 3;
	int data_size_num = data_buffer[1];
	if(check_char(data_buffer) == 0 || check_len(data_buffer, data_size, data_size_num) == 0)
	{
		strcpy((char*)response,"Error\r\n");
	}else
	{
		char* data = getdata_from_buffer(data_buffer, data_size);
		display_data(data);
		strcpy((char*)response, "Ok\r\n");
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  HAL_UART_Receive_IT(&huart1, &data_receive[0],1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(rx_index > 0 && HAL_GetTick() - last_receive_time >= 100)
	  {
		  package_size = rx_index;
		  check_package(data_buffer);
		  HAL_UART_Transmit(&huart1, response, strlen((char*)response), 100);
		  rx_index = 0;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|RW_Pin|E_Pin|D4_Pin
                          |D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin RW_Pin E_Pin D4_Pin
                           D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|E_Pin|D4_Pin
                          |D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
