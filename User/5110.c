#include "5110.h"

#include "English.h"
//#include "chinese.h"

//LCD���ų�ʼ��
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

//��LCD�������� д����
void LCD_Write_Byte(uint8_t data, DCType command)
{
	uint8_t i;
	
	if(command == DC_CMD)
		LCD_DC_CMD;   //���Ϳ�����
	else
		LCD_DC_DATA;  //��������

	//Ƭѡ�ź�������Ч
	LCD_SCE_L;
	
	for(i=0; i<8; i++)
	{
		//ʱ�������ͣ��ȴ����ݴ���
		LCD_SCLK_L;
		if(data & 0x80)  //��λ��ǰ
			LCD_MOSI_H;
		else
			LCD_MOSI_L;		
		//����ʱ���ߣ��������ݴ���
		LCD_SCLK_H;
		
		data = data<<1; //ȥ�����λ
	}
	
	//Ƭѡ�ź����ߣ����ݴ������
	LCD_SCE_H;
}
//Һ����������
void LCD_Clear(void)
{
	uint16_t i;	
	LCD_Set_XY(0, 0);
	for(i=0; i<504; i++)
		LCD_Write_Byte(0x00, DC_DATA);
}

//LCD��ʼ��
void LCD_Init(void)
{
	//��λ�������͵�ƽ��Ч
	LCD_RST_L;
	Delay_ms(1);
	LCD_RST_H;
	
	LCD_SCE_L; //�ر�LCD
	Delay_ms(1);
	LCD_SCE_H; //��LCD
	Delay_ms(1);
	
	//����LCD���������
  LCD_Write_Byte(0x21, DC_CMD);	// ʹ����չ��������LCDģʽ
  LCD_Write_Byte(0x9D, DC_CMD);	// ����ƫ�õ�ѹ(�൱�����ֵ����ȣ��ʵ�����)
  LCD_Write_Byte(0x07, DC_CMD);	// �¶�У��
  LCD_Write_Byte(0x17, DC_CMD);	// 1:48
  LCD_Write_Byte(0x20, DC_CMD);	// ʹ�û�������
  LCD_Clear();	        // ����
  LCD_Write_Byte(0x0c, DC_CMD);	// �趨��ʾģʽ��������ʾ
	
	LCD_SCE_L;
}
//����Һ������X,Y�����꣨ˮƽѰַ��
void LCD_Set_XY(uint8_t x, uint8_t y)
{
	LCD_Write_Byte(0x40 | y, DC_CMD);  //������0����5
	LCD_Write_Byte(0x80 | x, DC_CMD);  //������0����83
}
//��ʾӢ���ַ�
void LCD_ENChar(uint8_t x, uint8_t y, uint8_t c)
{
  uint8_t i;
  c -= 32;  //ASCL���ǰ��32�ַ�û��
	LCD_Set_XY(x, y);	
  for (i=0; i<6; i++)
    LCD_Write_Byte(font6x8[c][i], DC_DATA);
}
////��ʾ���Ӣ���ַ�
//void LCD_ENStr(uint8_t x, uint8_t y, uint8_t *c)
//{
//	uint8_t i;
//	while(*c != '\0')
//	{
//		i = 84 - x;		
//		if(i >= 6)    //��ֹһ�������һ��ʣ�µ����ص㲻��
//			LCD_ENChar(x, y, *c++);
//		
//			x = x + 6; //ÿ����ĸռ��6�����أ���������
//		
//		if(x > 80)  
//		{
//			x = 0;
//			y = y + 1; //��һ��
//			if(y > 5)
//			{
//				x = 0;
//				y = 0;
//			}
//		}
//  }
//}
//��ʾ���������
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
		LCD_ENChar(x, y, '0');  //�����0����ֱ����ʾ0
	
	if(num > 0)
	{
		//�������ݵĳ���
		res = num;   //�˴��ǵð�num���и�ֵ���Ա��Ժ�����
		while(res)
		{
			res = res /10;
			t++;
		}
		t1 = t;
		//�����ʾ����
		while(t)
		{
			res = mypes(10, t-1);
			LCD_ENChar(x+(t1-t)*6, y, (num/res)%10+'0');  //����������ʾ���ֲ���ʹ��ʾ�����귢���仯
			t--;
		}
	}
	else
	{
		//�������ݵĳ���
		res = -num;   //�˴��ǵð�num���и�ֵ���Ա��Ժ�����
		while(res)
		{
			res = res /10;
			t++;
		}
		t1 = t;
		
		//��ʾ����
		LCD_ENChar(x, y, 45);
		
		//�����ʾ����
		while(t)
		{
			res = mypes(10, t-1);
			LCD_ENChar(x+6+(t1-t)*6, y, (-num/res)%10+'0');  //����������ʾ���ֲ���ʹ��ʾ�����귢���仯
			t--;
		}
		
	}
}
//��ʾ���ĺ���(���ֵ���ʾ��Ϊ�������֣��ϰ벿�ֺ��°벿��)
//void LCD_CHStr(u8 x, u8 y, u8 n)
//{
//	uint8_t i;
//	
//	LCD_Set_XY(x, y);
//	//��ʾ�ϰ벿��
//	for(i=0; i<12; i++)
//	{
//		LCD_Write_Byte(CHIN[n][i], DC_DATA);
//	}
//	LCD_Set_XY(x, y+1);
//	//��ʾ�°벿��
//	for(i=12; i<24; i++)
//	{
//		LCD_Write_Byte(CHIN[n][i], DC_DATA);
//	}


//}
