#include "bule_tooth.h"
#include "math.h"
#include "stdio.h"

void USART3_Config(void)
{
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��UGPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = 9600;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
	
}

//����3���жϺ���
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  
	 static	int uart_receive=0;//����������ر���
	 uart_receive=USART_ReceiveData(USART3); 
		
	 if(uart_receive==0x59)  Flag_sudu=0;  //���ٵ�
	 if(uart_receive==0x58)  Flag_sudu=1;  //���ٵ�
		
	 if(uart_receive == 0x5A)	      //===============ɲ��  �ַ�(Z)  ʮ�����ƾ���0x5A 
	 {
		 Flag_Qian = 0;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }		 
	 else if(uart_receive == 0x41)  //===============ǰ    �ַ�(A)  ʮ�����ƾ���0x41
	 {		 
		 Flag_Qian = 1;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }
	 else if(uart_receive == 0x45)
	 {
		 Flag_Qian = 0; Flag_Hou = 1;
		 Flag_Left = 0;  Flag_Right = 0;		 	 
	 }                             //===============��    �ַ�(B, C, D)  ʮ�����ƾ���0x42  0x43  0x44
	 else if(uart_receive == 0x42 || uart_receive == 0x43 || uart_receive == 0x44)
	 {
		 Flag_Qian = 0; Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 1;		 
	 }                             //===============��    �ַ�(F, G, H)  ʮ�����ƾ���0x46  0x47  0x48
	 else if(uart_receive == 0x46 || uart_receive == 0x47 || uart_receive == 0x48)
	 {
	 	 Flag_Qian = 0; Flag_Hou = 0;
		 Flag_Left = 1;  Flag_Right = 0;		 
	 }
	 else                         //================ɲ��
	 {
		 Flag_Qian = 0;  Flag_Hou = 0;
		 Flag_Left = 0;  Flag_Right = 0;
	 }	 
	}  											 
} 

/// �ض���c�⺯��printf��USART3
//int fputc(int ch, FILE *f)
//{
//		/* ����һ���ֽ����ݵ�USART3 */
//		USART_SendData(USART3, (uint8_t) ch);
//		
//		/* �ȴ�������� */
//		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		
//	
//		return (ch);
//}

///// �ض���c�⺯��scanf��USART1
//int fgetc(FILE *f)
//{
//		/* �ȴ�����3�������� */
//		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(USART3);
//}



