#include "mpu6050.h"
#include "bsp_usart1.h"
#include "bsp_i2c.h"
#include "math.h"

/*
 I2C的写时序

Master
主机     开始(S)  地址(AD)+写(W)             传送寄存器地址(RA)              传送寄存器数据(DATA)              停止(P) 数据传送完毕

SLAVE
从机                              应答(Ack)                       应答(Ack)                         应答(Ack)  

*/
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);  //发送从机的地址
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	i2c_SendByte(reg_dat);
	i2c_WaitAck();
	i2c_Stop();                          //停止,数据传送完毕
}

/*
 I2C的读时序

Master
主机    S  AD+W         RA         S    AD+R                  NAck(主设备拒绝应答信号)  P

SLAVE
从机             Ack         Ack                Ack    DATA  

*/
void MPU6050_ReadData(u8 reg_add, unsigned char *Read, u8 num)
{
	unsigned char i;
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);
	i2c_WaitAck();
	
	//从机产生应答信号之后进行寄存器数据的传输
	for(i=0;i<(num-1);i++)
	{
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);  //数据传送完毕，产生一个拒绝应答信号
	i2c_Stop();
}

//MPU6050传感器读取一个字节数据
uint8_t Single_ReadIIC(uint8_t REG_Address)
{
	uint8_t REG_data=0;
	i2c_Start();                          //起始信号
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);    //发送设备地址+写信号
	i2c_WaitAck();
	i2c_SendByte(REG_Address);;     //发送存储单元地址，从0开始
	i2c_WaitAck();	
	
	i2c_Start();                   //起始信号
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1); //发送设备地址+读信号
	i2c_WaitAck();
	REG_data=i2c_ReadByte(0);      //读出寄存器数据
	i2c_Stop();                    //停止信号
	
	
	return REG_data;
}


void MPU6050_Init(void)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x04);	    //陀螺仪采样率，1KHz 
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //低通滤波器的设置，截止频率是1K，带宽是5K  
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x08);	  //配置加速度传感器工作在4G模式，不自检                  4.5 加速度计配置
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) 4.4 陀螺仪配置
}

//读取MPU6050的ID，为了检验数据是否匹配
uint8_t MPU6050_ReadID(void)
{
	unsigned char Re = 0;
  MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	if(Re != 0x68)
	{
		printf(" MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return 0;
	}
	else
	{
		printf(" MPU6050 ID = %d\r\n",Re);
		return 1;
	}
}

//同时读取陀螺仪和加速度计的六个参数
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	u8 buf[12];
	
	//读取数据，每次读取8位数据，前两次是X轴，中间两次是Y轴，最后两次是Z轴
	MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6); //从加速度计的存储的起始地址开始

	//左移八位，与第二次读取的数据组成一个16位的加速度计的值
	*ax = (buf[0] << 8) | buf[1];  //X
	*ay = (buf[2] << 8) | buf[3];  //Y
	*az = (buf[4] << 8) | buf[5];  //Z

	MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);

	*gx = (buf[0] << 8) | buf[1];  //X
	*gy = (buf[2] << 8) | buf[3];  //Y
	*gz = (buf[4] << 8) | buf[5];  //Z

}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];   
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}


