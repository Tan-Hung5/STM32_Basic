void lcd_write_nibble(uint8_t rs, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, rs);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);




	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (data>>3)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (data>>2)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (data>>1)&0x01);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (data>>0)&0x01);

	for(uint8_t i = 0; i < 72; i++) asm("NOP");

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	for(uint8_t i = 0; i < 72; i++) asm("NOP");
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