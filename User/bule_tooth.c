#include "bule_tooth.h"
#include "math.h"
#include "stdio.h"

void USART3_Config(void)
{
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = 9600;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
	
}

//串口3的中断函数
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	  
	 static	int uart_receive=0;//蓝牙接收相关变量
	 uart_receive=USART_ReceiveData(USART3); 
		
	 if(uart_receive==0x59)  Flag_sudu=0;  //低速挡
	 if(uart_receive==0x58)  Flag_sudu=1;  //高速档
		
	 if(uart_receive == 0x5A)	      //===============刹车  字符(Z)  十六进制就是0x5A 
	 {
		 Flag_Qian = 0;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }		 
	 else if(uart_receive == 0x41)  //===============前    字符(A)  十六进制就是0x41
	 {		 
		 Flag_Qian = 1;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }
	 else if(uart_receive == 0x45)
	 {
		 Flag_Qian = 0; Flag_Hou = 1;
		 Flag_Left = 0;  Flag_Right = 0;		 	 
	 }                             //===============右    字符(B, C, D)  十六进制就是0x42  0x43  0x44
	 else if(uart_receive == 0x42 || uart_receive == 0x43 || uart_receive == 0x44)
	 {
		 Flag_Qian = 0; Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 1;		 
	 }                             //===============左    字符(F, G, H)  十六进制就是0x46  0x47  0x48
	 else if(uart_receive == 0x46 || uart_receive == 0x47 || uart_receive == 0x48)
	 {
	 	 Flag_Qian = 0; Flag_Hou = 0;
		 Flag_Left = 1;  Flag_Right = 0;		 
	 }
	 else                         //================刹车
	 {
		 Flag_Qian = 0;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }	 
	}  											 
} 

/// 重定向c库函数printf到USART3
//int fputc(int ch, FILE *f)
//{
//		/* 发送一个字节数据到USART3 */
//		USART_SendData(USART3, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		
//	
//		return (ch);
//}

///// 重定向c库函数scanf到USART1
//int fgetc(FILE *f)
//{
//		/* 等待串口3输入数据 */
//		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(USART3);
//}



