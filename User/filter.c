#include "filter.h"
#include "MPU6050.h"
#include "math.h"

int16_t ax=0, ay=0, az=0;
int16_t gx, gy, gz;

float K1 = 0.003;

float Gyro;       //�����Ǽ������ĵĽ��ٶ�
float Angle;       //�����ں�֮����˵ĽǶ�
double Gyro_angle=0;   //�����Ǽ������ĽǶ�
double ax_angle=0.0;    //���ٶȼƲ�����ĽǶ�

//���ٶȼƺ������ǵ�0ƫ����ֵ
#define AX_ZERO 1326       //��MPU6050����ˮƽλ�ã�ȡ2000�ε�����ȡƽ���� 
#define GX_ZERO -35 

float Angle_gy=0.0; 

float Q_angle=0.19;  
float Q_gyro=0.83;
float R_angle=0.5;

float dt=0.001;	    //�˲����Ĳ�������
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0=0.0, PCt_1=0.0, E=0.0;
float K_0=0.0, K_1=0.0, t_0=0.0, t_1=0.0;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


/***********************************************************************

			                   		��Ԫ��

************************************************************************/
#define Kp 2.0//2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases
#define halfT 0.00259//0.0023//0.27//0.00233//0.00653(��λ��״̬�ȽϺ�)//0.006f  //half the sample period,halfT 0.5f��Ҫ���ݾ�����̬����������������T����̬�������ڣ�T*���ٶ�=΢�ֽǶ�


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
 * �����ǵ���ת����������ת�����ǵ�����ע�⣬����Ҫ�������ֶ���
 * ��4����ָָ��x�Ტ��y����ȭ����ʱ�����Ĵָ����Ϊz�᷽��
 */
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

float q0=1.0,q1,q2,q3;
float Yaw,Yaw_360out, Pitch, Roll;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;

float Accel_ax, Accel_ay, Accel_az;     //������ٶ�ԭʼ����ת���������
float Gyro_gx, Gyro_gy, Gyro_gz;        //����������ԭʼ����ת���������
float aa_x, aa_y, aa_z;

/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
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

//��Ԫ�ر�ʾ��̬ʱ���Ǻܲ�ֱ�۵ģ�����������Ҫת��ŷ���ǣ�*57.3  (180.0/3.14)�ǽ�����תΪ��ֱ�۵ĽǶ�
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
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick.
* Input          : None
* Output         : None
* Return         : None

// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�
*******************************************************************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float init_mx, float init_my, float init_mz) 
{
	float norm; //halfT;
	float vx, vy, vz;
	float ex, ey, ez;
	 
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
	norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
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

	//��Ԫ�������㷨��һ������-������
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
		����Ԫ�������Pitch  Roll  Yaw
		Roll=arctan2(2wx+2yz, 1-2xx-2yy);
		Pitch=arcsin(2wy-2zx);
		Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
		1=q0*q0+q1*q1+q2*q2+q3*q3;
		����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�
	========================================*/
	Yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��
	Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);                                         //�����ǣ���y��ת��	 
  Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);          //�����ǣ���x��ת��

  //0.9��0.1������ϵ��������5.73=0.1*57.3������57.3��Ϊ�˽�����ת��Ϊ�Ƕȣ��ù�ʽ��˼�ǽ������Ƶĳ���׼ȷ�Ⱥ�
  //�����ǵĸ������Ƚ��л����˲������������ǵ����ݽ��и�ͨ�˲����Դ����Ƶ����ݽ��е�ͨ�˲��������
	Yaw   = -(0.9 * (-Yaw + init_gz*2*halfT) + 5.73 * atan2(init_mx*cos(Roll) + init_my*sin(Roll)*sin(Pitch) + init_mz*sin(Roll)*cos(Pitch), init_my*cos(Pitch) - init_mz*sin(Pitch)));
	Pitch = Pitch * 57.3+6;//6��ƫ��
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
	
	//���ݼ��ٶȼƼ�����ĽǶ�
	aa_x = atan(Accel_ax*invSqrt(Accel_ay*Accel_ay+Accel_az*Accel_az))*180/3.14; 
	aa_y = atan(Accel_ay*invSqrt(Accel_ax*Accel_ax+Accel_az*Accel_az))*180/3.14;
	aa_z = atan(Accel_az*invSqrt(Accel_ax*Accel_ax+Accel_ay*Accel_ay))*180/3.14;
	
}




/**************************************************************************
														
										�������˲�

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

/* ���ٶȼƵ����̷�Χ 2g 16384 LSB/g
 * 
 * ���÷����Ǻ���������� 
 * sinx = 0.92*3.14*x/180 = y/16384
 * x=180*y/(0.92*3.14*16384)
 */
    ax -= AX_ZERO;
	  ax_angle=ax/262;
//    ax_angle=atan2((double)ax, (double)az) * 180 /3.1415926;
    
/* �����ǵ����̷�Χ�� 1000 32.8 LSB/s
 * �����ǵĽǶȼ��㹫ʽ
 * ��������ǵĽǶ����߾��ǲ�ͣ�ĶԽ��ٶȲ�ͣ�Ļ���
 * ���ͣ�Ľ����ۼӵó�����
 */
    gy -= GX_ZERO;

    Gyro= -(double)gy/16.4;     //�ó����ٶ�
    Gyro_angle += Gyro*0.004;   //���ٶȶ�ʱ��Ļ���
    
    Kalman_Filter(ax_angle,Gyro);
// 		Yijielvbo(ax_angle, Gyro);
}

/**************************************************************************

										һ�׻����˲�

**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   Angle = K1 * angle_m+ (1-K1) * (Angle + gyro_m * 0.005);
}

