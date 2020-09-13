#include "Timer2.h"

/* TIM2中断优先级配置 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占式优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      //副优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


//定时周期是5ms,十毫秒今日一次中断
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;     //定义一个定时器结构体变量

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2
  
  TIM_TimeBaseStructure.TIM_Period = (5000 - 1);	    //计数5000次，因为从0开始，所以减1
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);	    //时钟72分频，因为0不分频，所以减1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	      // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	    //初始化定时器2

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		      //清除定时器2中断标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		      //打开定时器2中断

  TIM_Cmd(TIM2, ENABLE);  //计数器使能，开始计数
}



