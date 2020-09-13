#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "mpu6050.h"
#include "bsp_i2c.h"
#include "5110.h"
#include "output.h"
#include "filter.h"
#include "Timer2.h"
#include "encoder.h"
#include "motor.h"
#include "bsp_usart1.h"
#include "control.h"
#include "bule_tooth.h"

int main(void)
{
	SysTick_Init();        //=====��ʼ��systick
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;	
	
	USART1_Config();      //======����1��ʼ�������ڵ����豸
	
	PWM_Init();           //======��ʼ��PWM�����Ƶ��
	
	USART3_Config();      //======����3��ʼ�����������������ݴ���
	
	EXTI_PA0_Config();    //======���������жϳ�ʼ�� 
	EXTI_PB12_Config();
	encoder_GPIO_Config();
	
	i2c_GPIO_Config();    //======I2C��ʼ��
  
	MPU6050_Init();       //======MPU6050��ʼ��

	TIM2_Configuration(); //======MPU6050��5ms�Ķ�ʱ�ж�
	TIM2_NVIC_Configuration();
	
	while(1)
	{	
//		printf("Z%d:%d:%fL$", speed_left, speed_right, Pitch);  //���������Ĵ�����������������ֻ��Ͻ�����ʾ
	}
				
}

/*********************************************END OF FILE**********************/
