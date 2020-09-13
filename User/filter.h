#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"
#include "bsp_SysTick.h"

//常见滤波函数的数据
extern float Angle;   			//融合之后的角度
extern float Gyro;    			//陀螺仪得出的角速度
extern double Gyro_angle;   //陀螺仪检测出来的角度
extern double ax_angle;     //加速度计测出来的角度

//四元数的数据
extern float Yaw,Yaw_360out, Pitch, Roll;
extern float Accel_ax, Accel_ay, Accel_az;     //保存加速度原始数据转化后的数据
extern float Gyro_gx, Gyro_gy, Gyro_gz;        //保存陀螺仪原始数据转化后的数据
extern float aa_x, aa_y, aa_z;


//加速度计和陀螺仪的原始数据
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

//四元数
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float init_mx, float init_my, float init_mz); 
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az); 
float invSqrt(float x);
void Quaternion_Calculate(void);

//常见的滤波算法
void Kalman_Filter(float Accel,float Gyro);
void Angle_Calculate(void);
void Yijielvbo(float angle_m, float gyro_m);

#endif
