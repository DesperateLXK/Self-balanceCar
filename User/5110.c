#include "5110.h"

#include "English.h"
//#include "chinese.h"

//LCD引脚初始化
void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	RCC_APB2PeriphClockCmd(NOKIA_RCCCLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = NOKIA_CLK | NOKIA_DIN | NOKIA_DC | NOKIA_CE | NOKIA_RST;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(NOKIA_PORT, &GPIO_InitStructure);
	LCD_Init();
}

//向LCD发送命令 写数据
void LCD_Write_Byte(uint8_t data, DCType command)
{
	uint8_t i;
	
	if(command == DC_CMD)
		LCD_DC_CMD;   //发送控制字
	else
		LCD_DC_DATA;  //发送数据

	//片选信号拉低有效
	LCD_SCE_L;
	
	for(i=0; i<8; i++)
	{
		//时钟线拉低，等待数据传输
		LCD_SCLK_L;
		if(data & 0x80)  //高位在前
			LCD_MOSI_H;
		else
			LCD_MOSI_L;		
		//拉高时钟线，进行数据传输
		LCD_SCLK_H;
		
		data = data<<1; //去掉最高位
	}
	
	//片选信号拉高，数据传输完成
	LCD_SCE_H;
}
//液晶清屏操作
void LCD_Clear(void)
{
	uint16_t i;	
	LCD_Set_XY(0, 0);
	for(i=0; i<504; i++)
		LCD_Write_Byte(0x00, DC_DATA);
}

//LCD初始化
void LCD_Init(void)
{
	//复位操作，低电平有效
	LCD_RST_L;
	Delay_ms(1);
	LCD_RST_H;
	
	LCD_SCE_L; //关闭LCD
	Delay_ms(1);
	LCD_SCE_H; //打开LCD
	Delay_ms(1);
	
	//设置LCD的命令操作
  LCD_Write_Byte(0x21, DC_CMD);	// 使用扩展命令设置LCD模式
  LCD_Write_Byte(0x9D, DC_CMD);	// 设置偏置电压(相当于文字的亮度，适当调节)
  LCD_Write_Byte(0x07, DC_CMD);	// 温度校正
  LCD_Write_Byte(0x17, DC_CMD);	// 1:48
  LCD_Write_Byte(0x20, DC_CMD);	// 使用基本命令
  LCD_Clear();	        // 清屏
  LCD_Write_Byte(0x0c, DC_CMD);	// 设定显示模式，正常显示
	
	LCD_SCE_L;
}
//设置液晶屏的X,Y的坐标（水平寻址）
void LCD_Set_XY(uint8_t x, uint8_t y)
{
	LCD_Write_Byte(0x40 | y, DC_CMD);  //行设置0——5
	LCD_Write_Byte(0x80 | x, DC_CMD);  //列设置0——83
}
//显示英文字符
void LCD_ENChar(uint8_t x, uint8_t y, uint8_t c)
{
  uint8_t i;
  c -= 32;  //ASCL码表前面32字符没有
	LCD_Set_XY(x, y);	
  for (i=0; i<6; i++)
    LCD_Write_Byte(font6x8[c][i], DC_DATA);
}
////显示多个英文字符
//void LCD_ENStr(uint8_t x, uint8_t y, uint8_t *c)
//{
//	uint8_t i;
//	while(*c != '\0')
//	{
//		i = 84 - x;		
//		if(i >= 6)    //防止一行最后面一行剩下的像素点不够
//			LCD_ENChar(x, y, *c++);
//		
//			x = x + 6; //每个字母占据6个像素，至少六个
//		
//		if(x > 80)  
//		{
//			x = 0;
//			y = y + 1; //下一行
//			if(y > 5)
//			{
//				x = 0;
//				y = 0;
//			}
//		}
//  }
//}
//显示任意的数字
static int32_t mypes(int16_t m, int16_t n)
{
	int32_t result=1;
	while(n--)
		result = result*m;
	return result;
}
void LCD_ShownNum(int16_t x, int16_t y, int32_t num)
{
	int16_t t=0, t1=0;
	int32_t res=0;
	
	if(!num)
		LCD_ENChar(x, y, '0');  //如果是0，就直接显示0
	
	if(num > 0)
	{
		//计算数据的长度
		res = num;   //此处记得把num进行赋值，以备以后再用
		while(res)
		{
			res = res /10;
			t++;
		}
		t1 = t;
		//逐个显示数字
		while(t)
		{
			res = mypes(10, t-1);
			LCD_ENChar(x+(t1-t)*6, y, (num/res)%10+'0');  //此语句就是显示数字并且使显示的坐标发生变化
			t--;
		}
	}
	else
	{
		//计算数据的长度
		res = -num;   //此处记得把num进行赋值，以备以后再用
		while(res)
		{
			res = res /10;
			t++;
		}
		t1 = t;
		
		//显示负号
		LCD_ENChar(x, y, 45);
		
		//逐个显示数字
		while(t)
		{
			res = mypes(10, t-1);
			LCD_ENChar(x+6+(t1-t)*6, y, (-num/res)%10+'0');  //此语句就是显示数字并且使显示的坐标发生变化
			t--;
		}
		
	}
}
//显示中文汉字(汉字的显示分为两个部分，上半部分和下半部分)
//void LCD_CHStr(u8 x, u8 y, u8 n)
//{
//	uint8_t i;
//	
//	LCD_Set_XY(x, y);
//	//显示上半部分
//	for(i=0; i<12; i++)
//	{
//		LCD_Write_Byte(CHIN[n][i], DC_DATA);
//	}
//	LCD_Set_XY(x, y+1);
//	//显示下半部分
//	for(i=12; i<24; i++)
//	{
//		LCD_Write_Byte(CHIN[n][i], DC_DATA);
//	}


//}
