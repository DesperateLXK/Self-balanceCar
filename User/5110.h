#ifndef  __NOKIA5110_H__
#define  __NOKIA5110_H__

#include "stm32f10x.h"
#include "bsp_systick.h"

#define WIDTH    6
#define HEIGHT   8

//枚举D/C的模式选择
typedef enum {DC_CMD = 0, DC_DATA = 1}DCType;  //enum是枚举变量


//定义NOKIA5110的各个引脚
#define NOKIA_RCCCLK  RCC_APB2Periph_GPIOA
#define NOKIA_PORT    GPIOA
#define NOKIA_CLK     GPIO_Pin_0
#define NOKIA_DIN     GPIO_Pin_1
#define NOKIA_DC      GPIO_Pin_2
#define NOKIA_CE      GPIO_Pin_3
#define NOKIA_RST     GPIO_Pin_4

#define LCD_RST_H     GPIO_SetBits(NOKIA_PORT, NOKIA_RST)
#define LCD_RST_L     GPIO_ResetBits(NOKIA_PORT, NOKIA_RST)

#define LCD_SCE_H     GPIO_SetBits(NOKIA_PORT, NOKIA_CE)
#define LCD_SCE_L			GPIO_ResetBits(NOKIA_PORT, NOKIA_CE)	
										
#define LCD_SCLK_H    GPIO_SetBits(NOKIA_PORT, NOKIA_CLK)
#define LCD_SCLK_L    GPIO_ResetBits(NOKIA_PORT, NOKIA_CLK)
										
#define LCD_DC_CMD    GPIO_ResetBits(NOKIA_PORT, NOKIA_DC) //写控制字
#define LCD_DC_DATA	  GPIO_SetBits(NOKIA_PORT, NOKIA_DC)   //写数据
								     								
#define LCD_MOSI_H    GPIO_SetBits(NOKIA_PORT, NOKIA_DIN)								
#define LCD_MOSI_L    GPIO_ResetBits(NOKIA_PORT, NOKIA_DIN)								


void LCD_GPIO_Init(void);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Write_Byte(uint8_t data, DCType command);

void LCD_Set_XY(uint8_t x, uint8_t y);
void LCD_ENChar(uint8_t x, uint8_t y, uint8_t c);
void LCD_ENStr(uint8_t x, uint8_t y, uint8_t *c);
void LCD_ShownNum(int16_t x, int16_t y, int32_t num);
void LCD_CHStr(u8, u8 y, u8 n);

#endif
