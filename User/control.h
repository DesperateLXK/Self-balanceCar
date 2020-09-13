#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"

extern u8 Flag_Qian, Flag_Hou;
extern u8 Flag_Left, Flag_Right;
extern u8 Flag_sudu;
extern int16_t speed_left, speed_right;

void Xianfu_PWM(void);

int balance(float Angle, float Gyro1);
int velocity(int16_t Speed_left, int16_t Speed_right);
int Turn(int16_t Speed_left, int16_t Speed_right, int16_t gz);
void Set_PWM(int moto1, int moto2);

int myabs(int a);
int Stop(float Angle);


#endif




