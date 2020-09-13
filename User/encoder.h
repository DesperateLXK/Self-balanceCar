#ifndef _ENCODER_H
#define _ENCODER_H

#include "stm32f10x.h"

//全局变量10ms一次编码器的值
extern int16_t count_left;
extern int16_t count_right;

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

//void TIM4_NVIC_Configuration(void);
void EXTI_PA0_Config(void);
void EXTI_PB12_Config(void);
//void TIM4_Configuration(void);
void encoder_GPIO_Config(void);

void Encoder_Init_TIM4(void);

int Read_Encoder(u8 TIMX);
#endif

