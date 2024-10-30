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
#include<stdio.h>
#include"string.h"
#include"stdbool.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t DS1307_ADDRESS_WRITE =  0xD0; 
const uint8_t DS1307_ADDRESS_READ =  0xD1;
const uint8_t F8574_ADDRESS = 0x4E;
uint8_t res[50];
int rx_index = 0;
uint8_t data_buffer[9];
uint8_t data_receive;
bool IT_flag = 0;
uint32_t last_time_IT = 0;

uint8_t Dec_to_BCD(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

uint8_t BCD_to_Dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
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

void lcd_send_cmd(uint8_t cmd)
{
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    
    data_u = (cmd & 0xF0);  
	data_l = (cmd << 4);      
    data_t[0] = data_u | 0x0C;  
	data_t[1] = data_u | 0x08;             
   
	data_t[2] = data_l | 0x0C;  
	data_t[3] = data_l | 0x08;        

    HAL_I2C_Master_Transmit(&hi2c1,F8574_ADDRESS , data_t, 4, 100);  
}



void lcd_send_data(uint8_t data)
{
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    
    data_u = (data & 0xF0);  
	data_l = (data << 4);  

    
	data_t[0] = data_u | 0x0D;  
	data_t[1] = data_u | 0x09;  

    
	data_t[2] = data_l | 0x0D;  
	data_t[3] = data_l | 0x09;  

    HAL_I2C_Master_Transmit(&hi2c1, F8574_ADDRESS , data_t, 4, 100);  
}



void lcd_init(void) {
    
    HAL_Delay(50);

    
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);

    lcd_send_cmd(0x20);  
    HAL_Delay(10);

    
    lcd_send_cmd(0x28);  
    HAL_Delay(1);
    lcd_send_cmd(0x08);  
    HAL_Delay(1);
    lcd_send_cmd(0x01);  
    HAL_Delay(1);
    lcd_send_cmd(0x06);  
    HAL_Delay(1);
    lcd_send_cmd(0x0C);  
}



void lcd_clear(void)
{
    lcd_send_cmd(0x01);  
	HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);  
	lcd_send_cmd(addr);
}

void lcd_send_string(char *str)
{
    while (*str)
    {
        lcd_send_data(*str++);  
	}
}

bool check_len_data(int rx_index) 
{
	int buffer_len = rx_index;
	if(buffer_len  < 8)
	{
		strcpy((char*)res,"Error: Thieu du lieu\r\n");
		return 0;
	}else if(buffer_len > 8)
	{
		strcpy((char*)res,"Error: Thua du lieur\r\n");
		return 0;
	}
	return 1;
}

bool check_form_data(uint8_t buffer[])
{
	if(buffer[0] == '*' && buffer[strlen((char*)buffer) - 1] == '#')
	{
		return 1;
	}
	
	strcpy((char*)res,"Error: Sai cu phap\r\n");
	return 0;		
}

bool is_leap_year(uint16_t year)
{
	if(year % 4 == 0 && year % 100 != 0)
	{
		return 1;
	}

	if(year % 400 == 0)
	{
		return 1;
	}
	return 0;
}

uint8_t days_in_feb(uint16_t year)
{
	if(is_leap_year(year) == 1)
	{
		return 29;
	}
	
	return 28;
	
}

bool is_valid_date(uint8_t day, uint8_t month, uint16_t year)
{
	if(year < 0)
	{
		strcpy((char*)res,"Error: Nam khong hop le\r\n"); 
		return 0;
	}

	if(month < 1 || month > 12)
	{
		strcpy((char*)res,"Error: Thang khong hop le\r\n");
		return 0;
	}
	
	uint8_t days_in_year[12] = {31,days_in_feb(year), 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	if(day < 1 || day > days_in_year[month - 1])
	{
		strcpy((char*)res,"Error: Ngay khong hop le\r\n");
		return 0;
	}
	
	return 1;
}

bool is_valid_time(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	if(hours < 0 || hours > 23)
	{
		strcpy((char*)res,"Error: Gio khong hop le\r\n");
		return 0;
	}

	if(minutes < 0 || minutes > 59)
	{
		strcpy((char*)res,"Phut khong hop le\r\n");
		return 0;
	}

	if(seconds < 0 || seconds > 59)
	{
		strcpy((char*)res,"Giay khong hop le\r\n");
		return 0;
	}

	return 1;

}


void check_valid_data(uint8_t buffer[])
{
	uint8_t day = buffer[1];
	uint8_t month = buffer[2];
	uint8_t year = buffer[3];
	uint8_t hours = buffer[4];
	uint8_t minutes = buffer[5];
	uint8_t seconds = buffer[6];

	if(check_len_data(rx_index) == 1 && check_form_data(buffer) == 1 && is_valid_date(day,month,year) == 1 && 
			is_valid_time(hours,minutes,seconds) == 1)
	{
		DS1307_SetTime(hours, minutes, seconds);
		DS1307_SetDate(day, month, year);
	}else
	{
		HAL_UART_Transmit(&huart1, res, strlen((char*)res), 100);
	}
}


void get_seconds(uint8_t* seconds)
{
	uint8_t buffer;
	uint8_t sec = 0x02;

	//HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS_READ,0x00,1,&buffer,1,10);
	HAL_I2C_Master_Transmit(&hi2c1,DS1307_ADDRESS_WRITE,&sec,1,10);
	HAL_I2C_Master_Receive(&hi2c1,DS1307_ADDRESS_READ,&buffer,1,10);
	*seconds = BCD_to_Dec(buffer & 0x7F);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t month;
	uint16_t year;

	char date_to_display_lcd[100];
	char time_to_display_lcd[100];
	lcd_init();
	lcd_put_cur(0,0);
	lcd_send_string("hello");
	uint8_t time[3] = {4,0,0};
	uint8_t date[3] = {15,10,24};
	
	DS1307_SetTime(time[0],time[1],time[2]);
	DS1307_SetDate(date[0],date[1],date[2]);

	HAL_UART_Receive_IT(&huart1,&data_receive,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(IT_flag == 1 && (HAL_GetTick() - last_time_IT) > 50)
	{
		IT_flag = 0;
		check_valid_data(data_buffer);
		rx_index = 0;
	}

	DS1307_GetTime(&hours, &minutes, &seconds);
	DS1307_GetDate(&day, &month, &year);

	sprintf(date_to_display_lcd, "date: %02d/%02d/%d",day,month,year);
	sprintf(time_to_display_lcd, "time: %02d:%02d:%02d",hours,minutes,seconds);

	lcd_clear();
	lcd_put_cur(0,0);
	lcd_send_string(date_to_display_lcd);
	lcd_put_cur(1,0);
	lcd_send_string(time_to_display_lcd);
	
	HAL_Delay(1000);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
