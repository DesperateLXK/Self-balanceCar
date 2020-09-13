#include "encoder.h"
#include "bsp_usart1.h"


int16_t count_left, count_right;

//外部中断PA0的中断引脚初始化
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
	EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Falling;   //上升沿触发中断
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

//外部中断PB12的中断引脚初始化
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
	EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Falling;   //xiajiang沿触发中断
	EXTI_Initstructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_Initstructure);
}

//读取左右两个编码器的B相的电平信号
void encoder_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_Initstructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;  //设置端口为上拉输入模式	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA,&GPIO_Initstructure);          //初始化端口
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;  //设置端口为上拉输入模式	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB,&GPIO_Initstructure);          //初始化端口	
}


//PA0的引脚中断函数
void EXTI0_IRQHandler(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==1)   //检测B相位高电平，正转
	{
		count_left++;
	}
	else  //反转
	{
		count_left--;
	}
	//清除中断标志位
	EXTI_ClearITPendingBit(EXTI_Line0);

}

//PB12的引脚中断函数
void EXTI15_10_IRQHandler(void)
{

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==1)   //检测B相位高电平，正转
	{
		count_right++;
	}
	else  //反转
	{
		count_right--;
	}	
	//清除中断标志位
	EXTI_ClearITPendingBit(EXTI_Line12);
}

/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
///**************************************************************************
//函数功能：TIM2中断服务函数
//入口参数：无
//返回  值：无
//**************************************************************************/
//void TIM2_IRQHandler(void)
//{ 		    		  			    
//	if(TIM2->SR&0X0001)//溢出中断
//	{    				   				     	    	
//	}				   
//	TIM2->SR&=~(1<<0);//清除中断标志位 	    
//}







