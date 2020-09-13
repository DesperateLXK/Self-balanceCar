#include "motor.h"

//�ĸ�I/O�ڿ��Ƶ��������ת
void PWM_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PB��PA�Ķ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //�����趨������ʼ��GPIOA
	
	//�����еĿ��ƶ�I/O���õ�
	GPIO_ResetBits(GPIOA, GPIO_Pin_2 |GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

//����PWM��Ƶ����10KHz
void PWM_Init(void)
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
  PWM_Motor_Init();	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO����ʱ��ʹ��
                                                         //���ø�����Ϊ�����������,���TIM1 CH1 CH4��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	//ͨ����Ƶ����PWM�����Ϊ10KHz
	TIM_TimeBaseInitStructure.TIM_Period = 7199;                     
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                     
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	//����TIM3���ĸ�ͨ��
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           //����TIM�����ģʽ��ΪPWM1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //������Ч��ƽ�ļ����Ǹߵ�ƽ
	
	//ͨ��1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�������ģʽ��״̬��ʹ�����
	TIM_OCInitStructure.TIM_Pulse = 0;                            //��������ֵ���������������ֵ��ʱ�򣬵�ƽ��������
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                      //ʹ��ͨ��1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);             //��TIM3��ͨ��1��TIM_CCRԤװ�ؼĴ�������ʹ��
	
	//ͨ��2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//ͨ��3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//ͨ��4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//ʹ�ܶ�ʱ��3��ͨ��1�������ؼĴ���TIM3_ARRʹ��
	TIM_ARRPreloadConfig(TIM3, ENABLE);   
	
	//ʹ�ܶ�ʱ��3
	TIM_Cmd(TIM3, ENABLE);
} 
