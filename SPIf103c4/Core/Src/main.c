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
#include "stdio.h"
#include "string.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd_read_id = 0x9F;
uint8_t data_read[9];
char data_buffer[50];

void enable_write(void)
{
	uint8_t code = 0x06;
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &code, 1, 10);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 1);
}

void disable_write(void)
{
	uint8_t code = 0x04;
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &code, 1, 10);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 1);
}

uint8_t read_status(void)
{
    uint8_t code = 0x05;  
    uint8_t status = 0;

    HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 0);  
    HAL_SPI_Transmit(&hspi1, &code, 1, 10);          
    HAL_SPI_Receive(&hspi1, &status, 1, 10);         
    HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 1);  

    return status;
}

void wait_until_not_busy(void)
{
    uint8_t status;
    do {
        status = read_status();
    } while (status == 1);  	
}

void write_data(uint16_t add, uint8_t* data, uint16_t size)
{

	uint8_t add_h = (add >> 8) & 0xFF;
	uint8_t add_l = add & 0xFF;
	uint8_t data_trans[3];

	data_trans[0] = 0x02;
	data_trans[1] = add_h;
	data_trans[2] = add_l;

	/*for (int i = 0;i < size; i++)
	{
		data_trans[i+3] = data[i];	
	}*/

	enable_write();

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &data_trans[0], 3, 100);
	//wait_until_not_busy();
	HAL_SPI_Transmit(&hspi1, &data[0], size, HAL_MAX_DELAY);
	wait_until_not_busy();
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 1);

	disable_write();
}

void read_data(uint16_t add, uint8_t* data, uint16_t size)
{
	uint8_t code = 0x03;
	uint8_t add_h = (add >> 8) & 0xFF;
	uint8_t add_l = add & 0xFF;
	uint8_t add_trans[3] = {code, add_h, add_l};

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &add_trans[0], 3, 10);
	HAL_SPI_Receive(&hspi1, &data[0], size, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, 1);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t data_write[256] = " hello\r\n";
	uint8_t poem[] = "Toi yeu dat nuoc toi. Tu bao gio khong biet nua. Co the tu khi toi sinh ra, dat nuoc da co roi.\n Dat nuoc len m	uon thuong noi buon doi lan mat.\nToi yeu dan toc toi. Tu bao gio khong biet nua. Co the tu khi toi biet noi cuoi buoi so sinh.aasdfjasdfkjasldfjasldjflk";
	uint16_t size = strlen((char*)poem);
	uint8_t data_size[2];
	data_size[0] = (size >> 8) & 0xFF;
	data_size[1] = size & 0xFF;
	uint8_t data_trans[100];
	uint8_t data_read[1024] = {0};

	sprintf((char*)data_trans,"do dai data: %03d%03d",data_size[0],data_size[1]);
	write_data(0x0000,&poem[0], size);
	read_data(0x0000, &data_read[0], size);
	HAL_UART_Transmit(&huart1, &data_trans[0], strlen((char*)data_trans), 100);
	//HAL_UART_Transmit(&huart1, &data_read[0], size, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	/*write_data(0x0000, &data_write[0], size);
	read_data(0x0000, &data_read[0], size);
	HAL_UART_Transmit(&huart1, &data_read[0], size, 100);

	HAL_Delay(1000);*/
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : F_CS_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(F_CS_GPIO_Port, &GPIO_InitStruct);

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
