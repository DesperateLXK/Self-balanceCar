#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

//四个端口控制电机正反转
#define  IN1(a)  if(a) \
													GPIO_SetBits(GPIOA, GPIO_Pin_2);\
								else \
									        GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define  IN2(a)  if(a) \
													GPIO_SetBits(GPIOA, GPIO_Pin_3);\
								else \
									        GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define  IN3(a)  if(a) \
													GPIO_SetBits(GPIOA, GPIO_Pin_4);\
								else \
									        GPIO_ResetBits(GPIOA, GPIO_Pin_4)								
#define  IN4(a)  if(a) \
													GPIO_SetBits(GPIOA, GPIO_Pin_5);\
								else \
									        GPIO_ResetBits(GPIOA, GPIO_Pin_5)								


void PWM_Motor_Init(void);
void PWM_Init(void);	
								
#endif

