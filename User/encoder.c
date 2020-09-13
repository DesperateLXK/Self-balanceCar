#include "encoder.h"
#include "bsp_usart1.h"


int16_t count_left, count_right;

//�ⲿ�ж�PA0���ж����ų�ʼ��
void EXTI_PA0_Config(void)
{
	GPIO_InitTypeDef GPIO_Initstructure; 
	
	EXTI_InitTypeDef EXTI_Initstructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	
	NVIC_Configuration();
	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Mode =  GPIO_Mode_IPU; 
	GPIO_Init(GPIOA,&GPIO_Initstructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 
	
	EXTI_Initstructure.EXTI_Line = EXTI_Line0;  						
	EXTI_Initstructure.EXTI_Mode = EXTI_Mode_Interrupt;      
	EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Falling;   //�����ش����ж�
	EXTI_Initstructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_Initstructure);
}

static void NVIC_Configuration(void) 
{ 

	NVIC_InitTypeDef NVIC_Initstructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_Initstructure.NVIC_IRQChannel =  EXTI0_IRQn;
  NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_Initstructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_Initstructure);	
}

//�ⲿ�ж�PB12���ж����ų�ʼ��
static void NVIC1_Configuration(void) 
{ 

	NVIC_InitTypeDef NVIC_Initstructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_Initstructure.NVIC_IRQChannel =  EXTI15_10_IRQn;
  NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_Initstructure.NVIC_IRQChannelSubPriority = 3;  
	NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_Initstructure);	
}

void EXTI_PB12_Config(void)
{
	GPIO_InitTypeDef GPIO_Initstructure; 
	
	EXTI_InitTypeDef EXTI_Initstructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	
	NVIC1_Configuration();
	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Initstructure.GPIO_Mode =  GPIO_Mode_IPU; 
	GPIO_Init(GPIOB,&GPIO_Initstructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12); 
	
	EXTI_Initstructure.EXTI_Line = EXTI_Line12;  						
	EXTI_Initstructure.EXTI_Mode = EXTI_Mode_Interrupt;      
	EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Falling;   //xiajiang�ش����ж�
	EXTI_Initstructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_Initstructure);
}

//��ȡ����������������B��ĵ�ƽ�ź�
void encoder_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_Initstructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;  //���ö˿�Ϊ��������ģʽ	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA,&GPIO_Initstructure);          //��ʼ���˿�
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;  //���ö˿�Ϊ��������ģʽ	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB,&GPIO_Initstructure);          //��ʼ���˿�	
}


//PA0�������жϺ���
void EXTI0_IRQHandler(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==1)   //���B��λ�ߵ�ƽ����ת
	{
		count_left++;
	}
	else  //��ת
	{
		count_left--;
	}
	//����жϱ�־λ
	EXTI_ClearITPendingBit(EXTI_Line0);

}

//PB12�������жϺ���
void EXTI15_10_IRQHandler(void)
{

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)   //���B��λ�ߵ�ƽ����ת
	{
		count_right++;
	}
	else  //��ת
	{
		count_right--;
	}	
	//����жϱ�־λ
	EXTI_ClearITPendingBit(EXTI_Line12);
}

/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
//	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
//		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}
///**************************************************************************
//�������ܣ�TIM2�жϷ�����
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void TIM2_IRQHandler(void)
//{ 		    		  			    
//	if(TIM2->SR&0X0001)//����ж�
//	{    				   				     	    	
//	}				   
//	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
//}







