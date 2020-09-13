#include "control.h"
#include "output.h"
#include "motor.h"
#include "bsp_usart1.h"
#include "filter.h"
#include "encoder.h"  //�����������ֵ
#include "MPU6050.h"
#include "math.h"
#include "bule_tooth.h"

int Balance_PWM, Velocity_PWM, Turn_PWM;
int Motor1, Motor2;
int Encoder_left, Encoder_right;
int16_t speed_left, speed_right;

//�ٶȻ��Ĳ���
int PWM = 0;
float Speed_r_l = 0, Speed = 0, Position = 0;
int16_t Movement = 0;   //��ʾС����Ŀ���ٶ�


//����ң�صı�־λ
u8 Flag_Qian = 0, Flag_Hou = 0;
u8 Flag_Left = 0, Flag_Right = 0;
u8 Flag_sudu = 0;
u8 Flag_Stop=1;

/*****************************************************************
 *   ���еĿ��Ƶ���Ĵ��붼�ڴ˴�
 *   ���ö�ʱ��2����5ms���ж϶�MPU6050�����ݽ��д���
 *   �Ѵ����������ݽ���PID�㷨���ƣ�����PWM
 *   ���ﵽ���Ƶ����Ŀ��
 *****************************************************************/

//��ʱ��2���жϺ���  �����жϵ�ʱ����5ms
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		static uint16_t ms = 0;
		ms++;
		
		//Angle_Calculate();                      //5�������һ���жϣ�����һ����̬
    Quaternion_Calculate();
		
		
		if(ms == 18)   //90ms�ɼ�һ�α�����������
		{
	    speed_left = -count_left;
		  speed_right = count_right;
//			printf("\r\n �������:   %d", speed_left);
//			printf("     �ұ�����:   %d", speed_right);
//			printf("Z%d:%d:%fL$", speed_left, speed_right, Pitch);
		  count_left = 0;
		  count_right = 0;
      ms = 0;			
		}
		
		Balance_PWM = balance(Pitch, gy);
		Velocity_PWM = velocity(speed_left, speed_right);
		Turn_PWM = Turn(speed_left, speed_right, gz);
		
//		printf("\r\n ftftftft: %d", Velocity_PWM);
		
		Motor1 = Balance_PWM + Velocity_PWM + Turn_PWM;
		Motor2 = Balance_PWM + Velocity_PWM - Turn_PWM;
		
		if(Motor1 < 0 || Motor2 <0)   //ȥ�����������
		{
			Motor1 = Motor1 - 4000;
			Motor2 = Motor2 - 4000;
		}
		else
		{
			Motor1 = Motor1 + 4000;
			Motor2 = Motor2 + 4000;		
		}
		
		Xianfu_PWM();

    if(Stop(Pitch) == 0)                           //===����������������
			Set_PWM(Motor1, Motor2);	
		
//		Set_PWM(1800, 5400);
// 		OutData[0] = Pitch*100;
// 		OutData[1] = -aa_x*100;
// 		OutData[2] = aa_y*100;		
// 	  OutPut_Data();

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //����жϱ�־λ
	}	
}

/***************************************************************************
 *   ֱ����PD����(����΢��)       ���ڸ�����ҪѸ��������Ӧ��ҪD(΢�ֿ���)
 *   �Ƕȣ����ٶ�                 �ǶȻ�����С��ֱ��
 ***************************************************************************/
int balance(float Angle, float gy)
{
	float Bias, Kp = 150, Kd = 0.95;       //�����kp��kd��ֵ����ʵ��Ӧ������Ҫ���ϵĵ���
	int balance;                           //ֱ��PWM�ķ���ֵ
  Bias = Angle - 0;                      //ֱ����ƫ��
	balance = Kp * Bias + gy * Kd;        //����ֱ����PWM

	return balance;
}

/***************************************************************************
 *   �ٶ�PI����(��������)      PI ��������һ�����Կ������������ݸ���ֵ��ʵ�����ֵ���ɿ���ƫ
                               ���ƫ��ı�����P���ͻ��֣�I��ͨ��������Ϲ��ɿ��������Ա���
                               ������п��ơ�
 *   ���ұ�������ֵ            
 ***************************************************************************/
int velocity(int16_t Speed_left, int16_t Speed_right)             //ƽ��С���Ŀ���ϵͳ��������--��С����һ�����ٶ�
{                                                         //���е�ʱ������Ҫ��С��ͣ������С����Ҫ��ʻ������ٶ�ȥ ��׷��
  float Kp = 20, Ki = 0.011;
	
	//�ж�С���Ǹ��ٻ��ǵ���
	if(Flag_sudu == 0)          
	{
		//ң�����ݱ�ʾ����ǰ���ͺ���
		if(Flag_Qian == 1)  
			Movement =  45; 
		else if(Flag_Hou == 1)
			Movement = -45;
		else
			Movement = 0;
  }
	else
	{
		if(Flag_Qian == 1)  
			Movement =  60; 
		else if(Flag_Hou == 1)
			Movement = -60;
		else
			Movement = 0;
  }	
	
	Speed_r_l =  Speed_left + Speed_right;       
	
	Speed =  Speed * 0.8;                               //��ͨ�˲���Ŀ����Ϊ�˼����ٶ�ֵ�ı仯����ֹ�ٶȿ��ƶ�ֱ����ɸ��ţ�
  Speed = Speed + Speed_r_l *0.2;                     //��Ϊƽ��С��ϵͳ���棬ֱ����������Ҫ�ģ��������ƶ���ֱ����˵����һ�ָ���
	
	
	Position = Position + Speed;                        //ƫ����ֵõ�С����λ��
	
	Position = Position - Movement;
	
	if(Position > 100000)  	
		Position = 100000;               //===�����޷�
  if(Position < -100000)	
		Position = -100000;              //===�����޷�	
	

	
	PWM = Kp * Speed + Ki * Position;                   //�ٶȺ�λ��
	
	if(Stop(Pitch)==1) //�������ر�,�Ͱѻ����������
		Position  = 0;

	return PWM;
}

/***************************************************************************
 *   ת��P����        ��ת��ң�صĲ������ӵ�ת����ƺ���֮��

 *   ���ұ�������ֵ   Z�������ǵ�ԭʼ����         
 ***************************************************************************/
int Turn(int16_t Speed_left, int16_t Speed_right, int16_t gz)
{
	float Turn;
//	float Kp = 0.61, Bias;
	int16_t Turn_Amplitude = 0;
//	int16_t Turn;
	
	if(Flag_Left == 1)
		Turn_Amplitude = 700;
  else if(Flag_Right == 1)
		Turn_Amplitude = -700;
	else
		Turn_Amplitude = 0;
	
//	Bias = gz - 0;   //Ŀ��ֵ��0�� Bias��ƫ��ֵ
//	Turn = Bias * Kp;
	
	Turn = Turn_Amplitude;
	
	return Turn;
}

/***************************************************************************
 *   
		��ֵ��PWM�Ĵ���,�ı�PWM��ռ�ձȵĴ�С
 *         
 ***************************************************************************/
void Set_PWM(int moto1, int moto2)
{
	if(moto1 < 0)         
	{	
		IN1(1); 
    IN2(0);  									
  }          
	else
	{	
    IN1(0);
		IN2(1);										
	}            
	
	TIM_SetCompare1(TIM3, myabs(moto1));       //�ı�ͨ��1��ռ�ձ�

	if(moto2 < 0)         
	{	
		IN3(1); 
    IN4(0);  									
  }          
	else
	{	
    IN3(0);
		IN4(1);										
	}            
		
	TIM_SetCompare2(TIM3, myabs(moto2));	     //�ı�ͨ��4��ռ�ձ�

}
/***************************************************************************
 *   
		�������ܣ�����PWM��ֵ 
 *         
 ***************************************************************************/
void Xianfu_PWM(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(Motor1<-Amplitude) Motor1=-Amplitude;	
		if(Motor1>Amplitude)  Motor1=Amplitude;	
	  if(Motor2<-Amplitude) Motor2=-Amplitude;	
		if(Motor2>Amplitude)  Motor2=Amplitude;		
	
}

/***************************************************************************
 *   
		����ֵ����
 *         
 ***************************************************************************/
int myabs(int a)
{
	int temp;
	if(a < 0)
		temp = -a;
	else
		temp = a;
	
	return temp;
}

/***************************************************************************
 *   
		С���ڽǶȹ涨�ķ�Χ���������У������ֹͣ
 *         
 ***************************************************************************/
int Stop(float Angle)
{
	u8 temp;
	if(Angle > 25 || Angle < -25)
	{
    IN1(0);
		IN2(0);		
	  IN3(0);
		IN4(0);	
		temp = 1;
	}
	else 
		temp = 0;
	
	return temp;
}


