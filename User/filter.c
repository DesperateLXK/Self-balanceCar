#include "filter.h"
#include "MPU6050.h"
#include "math.h"

int16_t ax=0, ay=0, az=0;
int16_t gx, gy, gz;

float K1 = 0.003;

float Gyro;       //陀螺仪检测出来的的角速度
float Angle;       //进行融合之后的人的角度
double Gyro_angle=0;   //陀螺仪检测出来的角度
double ax_angle=0.0;    //加速度计测出来的角度

//加速度计和陀螺仪的0偏修正值
#define AX_ZERO 1326       //把MPU6050放置水平位置，取2000次的数据取平均数 
#define GX_ZERO -35 

float Angle_gy=0.0; 

float Q_angle=0.19;  
float Q_gyro=0.83;
float R_angle=0.5;

float dt=0.001;	    //滤波器的采样周期
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0=0.0, PCt_1=0.0, E=0.0;
float K_0=0.0, K_1=0.0, t_0=0.0, t_1=0.0;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


/***********************************************************************

			                   		四元数

************************************************************************/
#define Kp 2.0//2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases
#define halfT 0.00259//0.0023//0.27//0.00233//0.00653(上位机状态比较好)//0.006f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度


#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

#define devAddr  0xD0		

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 * 陀螺仪的旋转矩阵，用于旋转陀螺仪的轴向，注意，轴向要符合右手定理，
 * 即4个手指指向x轴并向y轴握拳，此时竖起的拇指方向为z轴方向
 */
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

float q0=1.0,q1,q2,q3;
float Yaw,Yaw_360out, Pitch, Roll;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;

float Accel_ax, Accel_ay, Accel_az;     //保存加速度原始数据转化后的数据
float Gyro_gx, Gyro_gy, Gyro_gz;        //保存陀螺仪原始数据转化后的数据
float aa_x, aa_y, aa_z;

/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//四元素表示姿态时，是很不直观的，所以我们需要转成欧拉角，*57.3  (180.0/3.14)是将弧度转为更直观的角度
void toEuler(void)
{
	/* STANDARD ZYX
	 y=atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1);
	 p=-asin(2 * q1 * q3 + 2 * q0 * q2);
	 r=atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1);
	 */
	Yaw=(atan2(2*q1*q2+2*q0*q3,2*q0*q0+2*q1*q1-1))*57.3;
	Pitch=(-asin(2 * q1 * q3 - 2 * q0 * q2))*57.3;
	Roll=(atan2(2 * q2 * q3 + 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1))*57.3;
}

void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;//halfT;
	float vx, vy, vz;
	float ex, ey, ez;

	// normalise the measurements
	norm = invSqrt(ax*ax + ay*ay + az*az);
			
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	// normalise quaternion
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	toEuler();
}

/*******************************************************************************
* Function Name  : IMUupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick.
* Input          : None
* Output         : None
* Return         : None

// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度
*******************************************************************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float init_mx, float init_my, float init_mz) 
{
	float norm; //halfT;
	float vx, vy, vz;
	float ex, ey, ez;
	 
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
 
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//四元数更新算法，一阶龙格-库塔法
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;       //w
	q1 = q1 * norm;       //x
	q2 = q2 * norm;       //y
	q3 = q3 * norm;       //z

	/*======================================
		由四元数计算出Pitch  Roll  Yaw
		Roll=arctan2(2wx+2yz, 1-2xx-2yy);
		Pitch=arcsin(2wy-2zx);
		Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
		1=q0*q0+q1*q1+q2*q2+q3*q3;
		乘以57.3是为了将弧度转化为角度
	========================================*/
	Yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动
	Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);                                         //俯仰角，绕y轴转动	 
  Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);          //滚动角，绕x轴转动

  //0.9和0.1是修正系数，其中5.73=0.1*57.3，乘以57.3是为了将弧度转化为角度，该公式意思是将磁力计的长期准确度和
  //陀螺仪的高灵敏度进行互补滤波，即对陀螺仪的数据进行高通滤波，对磁力计的数据进行低通滤波，再相加
	Yaw   = -(0.9 * (-Yaw + init_gz*2*halfT) + 5.73 * atan2(init_mx*cos(Roll) + init_my*sin(Roll)*sin(Pitch) + init_mz*sin(Roll)*cos(Pitch), init_my*cos(Pitch) - init_mz*sin(Pitch)));
	Pitch = Pitch * 57.3+6;//6度偏差
	Roll = Roll * 57.3;
	
	if(Yaw <= -172) Yaw = -172;
	if(Yaw >=  172) Yaw =  172;
	
	Yaw_360out = Yaw + 172;
	
}

void Quaternion_Calculate(void)
{
	
	MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	Accel_ax = ax/8192.0;
	Accel_ay = ay/8192.0;
	Accel_az = az/8192.0;
	Gyro_gx = gx*3.14159165 / (180.0 * 16.4);
	Gyro_gy = gy*3.14159165 / (180.0 * 16.4);
	Gyro_gz = gz*3.14159165 / (180.0 * 16.4);
	
	updateIMU(Gyro_gx, Gyro_gy, Gyro_gz, Accel_ax, Accel_ay, Accel_az);
	
	//根据加速度计计算出的角度
	aa_x = atan(Accel_ax*invSqrt(Accel_ay*Accel_ay+Accel_az*Accel_az))*180/3.14; 
	aa_y = atan(Accel_ay*invSqrt(Accel_ax*Accel_ax+Accel_az*Accel_az))*180/3.14;
	aa_z = atan(Accel_az*invSqrt(Accel_ax*Accel_ax+Accel_ay*Accel_ay))*180/3.14;
	
}




/**************************************************************************
														
										卡尔曼滤波

**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; 

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; 

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   
	PP[0][1] += Pdot[1] * dt;   
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
  if(E!=0)
	{
    K_0 = PCt_0 / E;
	  K_1 = PCt_1 / E;
  }
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;
}

void Angle_Calculate(void) 
{
    MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

/* 加速度计的量程范围 2g 16384 LSB/g
 * 
 * 利用反三角函数进行输出 
 * sinx = 0.92*3.14*x/180 = y/16384
 * x=180*y/(0.92*3.14*16384)
 */
    ax -= AX_ZERO;
	  ax_angle=ax/262;
//    ax_angle=atan2((double)ax, (double)az) * 180 /3.1415926;
    
/* 陀螺仪的量程范围是 1000 32.8 LSB/s
 * 陀螺仪的角度计算公式
 * 求解陀螺仪的角度曲线就是不停的对角速度不停的积分
 * 最后不停的进行累加得出曲线
 */
    gy -= GX_ZERO;

    Gyro= -(double)gy/16.4;     //得出角速度
    Gyro_angle += Gyro*0.004;   //角速度对时间的积分
    
    Kalman_Filter(ax_angle,Gyro);
// 		Yijielvbo(ax_angle, Gyro);
}

/**************************************************************************

										一阶互补滤波

**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   Angle = K1 * angle_m+ (1-K1) * (Angle + gyro_m * 0.005);
}

