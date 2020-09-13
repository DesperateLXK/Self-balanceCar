#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"
#include "bsp_SysTick.h"

//�����˲�����������
extern float Angle;   			//�ں�֮��ĽǶ�
extern float Gyro;    			//�����ǵó��Ľ��ٶ�
extern double Gyro_angle;   //�����Ǽ������ĽǶ�
extern double ax_angle;     //���ٶȼƲ�����ĽǶ�

//��Ԫ��������
extern float Yaw,Yaw_360out, Pitch, Roll;
extern float Accel_ax, Accel_ay, Accel_az;     //������ٶ�ԭʼ����ת���������
extern float Gyro_gx, Gyro_gy, Gyro_gz;        //����������ԭʼ����ת���������
extern float aa_x, aa_y, aa_z;


//���ٶȼƺ������ǵ�ԭʼ����
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

//��Ԫ��
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float init_mx, float init_my, float init_mz); 
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az); 
float invSqrt(float x);
void Quaternion_Calculate(void);

//�������˲��㷨
void Kalman_Filter(float Accel,float Gyro);
void Angle_Calculate(void);
void Yijielvbo(float angle_m, float gyro_m);

#endif
