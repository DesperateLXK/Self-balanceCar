#include "Timer2.h"

/* TIM2�ж����ȼ����� */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռʽ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


//��ʱ������5ms,ʮ�������һ���ж�
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;     //����һ����ʱ���ṹ�����

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2
  
  TIM_TimeBaseStructure.TIM_Period = (5000 - 1);	    //����5000�Σ���Ϊ��0��ʼ�����Լ�1
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);	    //ʱ��72��Ƶ����Ϊ0����Ƶ�����Լ�1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	      // ʹ�õĲ���Ƶ��֮��ķ�Ƶ����
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ���
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	    //��ʼ����ʱ��2

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		      //�����ʱ��2�жϱ�־λ
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		      //�򿪶�ʱ��2�ж�

  TIM_Cmd(TIM2, ENABLE);  //������ʹ�ܣ���ʼ����
}



