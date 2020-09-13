#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

void Delay_us(__IO u32 ncount);

#define Delay_ms(x)  Delay_us(1000*x)   //1msµÄÑÓÊ±

void TimingDelay_Decrement(void);

void SysTick_Init(void);

#endif /* __SYSTICK_H */
