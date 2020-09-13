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
	SysTick_Init();        //=====初始化systick
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;	
	
	USART1_Config();      //======串口1初始化，用于调试设备
	
	PWM_Init();           //======初始化PWM，控制电机
	
	USART3_Config();      //======串口3初始化，用于蓝牙的数据传输
	
	EXTI_PA0_Config();    //======编码器的中断初始化 
	EXTI_PB12_Config();
	encoder_GPIO_Config();
	
	i2c_GPIO_Config();    //======I2C初始化
  
	MPU6050_Init();       //======MPU6050初始化

	TIM2_Configuration(); //======MPU6050的5ms的定时中断
	TIM2_NVIC_Configuration();
	
	while(1)
	{	
//		printf("Z%d:%d:%fL$", speed_left, speed_right, Pitch);  //利用蓝牙的串口输出三个参数在手机上进行显示
	}
				
}

/*********************************************END OF FILE**********************/
