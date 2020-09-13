#include "control.h"
#include "output.h"
#include "motor.h"
#include "bsp_usart1.h"
#include "filter.h"
#include "encoder.h"  //两组编码器的值
#include "MPU6050.h"
#include "math.h"
#include "bule_tooth.h"

int Balance_PWM, Velocity_PWM, Turn_PWM;
int Motor1, Motor2;
int Encoder_left, Encoder_right;
int16_t speed_left, speed_right;

//速度环的参数
int PWM = 0;
float Speed_r_l = 0, Speed = 0, Position = 0;
int16_t Movement = 0;   //表示小车的目标速度


//蓝牙遥控的标志位
u8 Flag_Qian = 0, Flag_Hou = 0;
u8 Flag_Left = 0, Flag_Right = 0;
u8 Flag_sudu = 0;
u8 Flag_Stop=1;

/*****************************************************************
 *   所有的控制电机的代码都在此处
 *   利用定时器2进行5ms的中断对MPU6050的数据进行处理
 *   把处理过后的数据进行PID算法控制，返回PWM
 *   最后达到控制电机的目的
 *****************************************************************/

//定时器2的中断函数  进入中断的时间是5ms
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		static uint16_t ms = 0;
		ms++;
		
		//Angle_Calculate();                      //5毫秒进入一次中断，更新一次姿态
    Quaternion_Calculate();
		
		
		if(ms == 18)   //90ms采集一次编码器的数据
		{
	    speed_left = -count_left;
		  speed_right = count_right;
//			printf("\r\n 左编码器:   %d", speed_left);
//			printf("     右编码器:   %d", speed_right);
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
		
		if(Motor1 < 0 || Motor2 <0)   //去除电机的死区
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

    if(Stop(Pitch) == 0)                           //===情况正常就启动电机
			Set_PWM(Motor1, Motor2);	
		
//		Set_PWM(1800, 5400);
// 		OutData[0] = Pitch*100;
// 		OutData[1] = -aa_x*100;
// 		OutData[2] = aa_y*100;		
// 	  OutPut_Data();

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除中断标志位
	}	
}

/***************************************************************************
 *   直立的PD控制(比例微分)       对于干扰需要迅速做出反应需要D(微分控制)
 *   角度，角速度                 角度环控制小车直立
 ***************************************************************************/
int balance(float Angle, float gy)
{
	float Bias, Kp = 150, Kd = 0.95;       //具体的kp和kd的值，在实际应用中需要不断的调试
	int balance;                           //直立PWM的返回值
  Bias = Angle - 0;                      //直立的偏差
	balance = Kp * Bias + gy * Kd;        //计算直立的PWM

	return balance;
}

/***************************************************************************
 *   速度PI控制(比例积分)      PI 调节器是一种线性控制器，它根据给定值与实际输出值构成控制偏
                               差，将偏差的比例（P）和积分（I）通过线性组合构成控制量，对被控
                               对象进行控制。
 *   左右编码器的值            
 ***************************************************************************/
int velocity(int16_t Speed_left, int16_t Speed_right)             //平衡小车的控制系统是正反馈--当小车以一定的速度
{                                                         //运行的时候，我们要让小车停下来，小车需要行驶更快的速度去 “追”
  float Kp = 20, Ki = 0.011;
	
	//判断小车是高速还是低速
	if(Flag_sudu == 0)          
	{
		//遥控数据表示车的前进和后退
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
	
	Speed =  Speed * 0.8;                               //低通滤波，目的是为了减缓速度值的变化，防止速度控制对直立造成干扰，
  Speed = Speed + Speed_r_l *0.2;                     //因为平衡小车系统里面，直立控制是主要的，其他控制对于直立来说都是一种干扰
	
	
	Position = Position + Speed;                        //偏差积分得到小车的位移
	
	Position = Position - Movement;
	
	if(Position > 100000)  	
		Position = 100000;               //===积分限幅
  if(Position < -100000)	
		Position = -100000;              //===积分限幅	
	

	
	PWM = Kp * Speed + Ki * Position;                   //速度和位置
	
	if(Stop(Pitch)==1) //如果电机关闭,就把积分清零操作
		Position  = 0;

	return PWM;
}

/***************************************************************************
 *   转向P控制        把转向遥控的参数叠加到转向控制函数之中

 *   左右编码器的值   Z轴陀螺仪的原始数据         
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
	
//	Bias = gz - 0;   //目标值是0， Bias是偏差值
//	Turn = Bias * Kp;
	
	Turn = Turn_Amplitude;
	
	return Turn;
}

/***************************************************************************
 *   
		赋值给PWM寄存器,改变PWM中占空比的大小
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
	
	TIM_SetCompare1(TIM3, myabs(moto1));       //改变通道1的占空比

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
		
	TIM_SetCompare2(TIM3, myabs(moto2));	     //改变通道4的占空比

}
/***************************************************************************
 *   
		函数功能：限制PWM赋值 
 *         
 ***************************************************************************/
void Xianfu_PWM(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(Motor1<-Amplitude) Motor1=-Amplitude;	
		if(Motor1>Amplitude)  Motor1=Amplitude;	
	  if(Motor2<-Amplitude) Motor2=-Amplitude;	
		if(Motor2>Amplitude)  Motor2=Amplitude;		
	
}

/***************************************************************************
 *   
		绝对值函数
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
		小车在角度规定的范围内正常运行，否则就停止
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


