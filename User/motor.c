#include "motor.h"

//四个I/O口控制电机的正反转
void PWM_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE); //使能PB和PA的端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据设定参数初始化GPIOA
	
	//把所有的控制端I/O口置低
	GPIO_ResetBits(GPIOA, GPIO_Pin_2 |GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

//设置PWM的频率是10KHz
void PWM_Init(void)
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
  PWM_Motor_Init();	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设时钟使能
                                                         //设置该引脚为复用输出功能,输出TIM1 CH1 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	//通过分频设置PWM的输出为10KHz
	TIM_TimeBaseInitStructure.TIM_Period = 7199;                     
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                     
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	//设置TIM3的四个通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           //设置TIM的输出模式即为PWM1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //设置有效电平的极性是高电平
	
	//通道1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //配置输出模式的状态，使能输出
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置跳变值，当计数器到这个值得时候，电平发生跳变
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                      //使能通道1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);             //把TIM3的通道1的TIM_CCR预装载寄存器进行使能
	
	//通道2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//通道3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//通道4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//使能定时器3的通道1，把重载寄存器TIM3_ARR使能
	TIM_ARRPreloadConfig(TIM3, ENABLE);   
	
	//使能定时器3
	TIM_Cmd(TIM3, ENABLE);
} 
