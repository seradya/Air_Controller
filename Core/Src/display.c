#include <display.h>
#include <stdio.h>
#include <string.h>

extern uint32_t pressure[4];
//extern uint8_t buf1[5] = {0};
 uint16_t i=1,retr_cnt_full=0;

void Display_Init(void) {
	//HAL_GPIO_WritePin(SPI_Led_GPIO_Port, SPI_Led_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(I2C_Led_GPIO_Port, I2C_Led_Pin, GPIO_PIN_RESET);
	ssd1306_Init();

	//ssd1306_UpdateScreen();
}

void Main_Screen(void) {
	uint8_t x = 1, y = 10;

	char ps1[32];
	char ps2[5];
	char ps3[32];
	char ps4[32];
	//char ps4[64];
	//for(int i = 0; i < 4; i++)
	//float press1 = (pressure[0] / 101312000);
	float press2 = (float)(pressure[1] / 133.32);
	//sprintf(ps1, "%1.3f", press1);
	sprintf(ps2, "%1.3f", press2);
	sprintf(ps1, "%d", pressure[1]);
	sprintf(ps3, "%d", i);
	sprintf(ps4, "%d", retr_cnt_full);
	//sprintf(ps2, "%d", pressure[1]);
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString("             ", Font_11x18, White);
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(ps1, Font_11x18, White);
	x += SSD1306_WIDTH/2 + 3;
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString("     ", Font_11x18, White);
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(ps2, Font_11x18, White);
	y += SSD1306_HEIGHT/2;
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString("     ", Font_11x18, White);
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(ps4, Font_11x18, White);
	x -= SSD1306_WIDTH/2 + 3;
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString("     ", Font_11x18, White);
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(ps3, Font_11x18, White);

	ssd1306_Line(SSD1306_WIDTH/2,0,SSD1306_WIDTH/2,SSD1306_HEIGHT,White);
	ssd1306_Line(0,SSD1306_HEIGHT/2,SSD1306_WIDTH,SSD1306_HEIGHT/2,White);

	ssd1306_UpdateScreen();
}
