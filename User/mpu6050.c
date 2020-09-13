#include "mpu6050.h"
#include "bsp_usart1.h"
#include "bsp_i2c.h"
#include "math.h"

/*
 I2C��дʱ��

Master
����     ��ʼ(S)  ��ַ(AD)+д(W)             ���ͼĴ�����ַ(RA)              ���ͼĴ�������(DATA)              ֹͣ(P) ���ݴ������

SLAVE
�ӻ�                              Ӧ��(Ack)                       Ӧ��(Ack)                         Ӧ��(Ack)  

*/
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);  //���ʹӻ��ĵ�ַ
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	i2c_SendByte(reg_dat);
	i2c_WaitAck();
	i2c_Stop();                          //ֹͣ,���ݴ������
}

/*
 I2C�Ķ�ʱ��

Master
����    S  AD+W         RA         S    AD+R                  NAck(���豸�ܾ�Ӧ���ź�)  P

SLAVE
�ӻ�             Ack         Ack                Ack    DATA  

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
	
	//�ӻ�����Ӧ���ź�֮����мĴ������ݵĴ���
	for(i=0;i<(num-1);i++)
	{
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);  //���ݴ�����ϣ�����һ���ܾ�Ӧ���ź�
	i2c_Stop();
}

//MPU6050��������ȡһ���ֽ�����
uint8_t Single_ReadIIC(uint8_t REG_Address)
{
	uint8_t REG_data=0;
	i2c_Start();                          //��ʼ�ź�
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);    //�����豸��ַ+д�ź�
	i2c_WaitAck();
	i2c_SendByte(REG_Address);;     //���ʹ洢��Ԫ��ַ����0��ʼ
	i2c_WaitAck();	
	
	i2c_Start();                   //��ʼ�ź�
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1); //�����豸��ַ+���ź�
	i2c_WaitAck();
	REG_data=i2c_ReadByte(0);      //�����Ĵ�������
	i2c_Stop();                    //ֹͣ�ź�
	
	
	return REG_data;
}


void MPU6050_Init(void)
{
  int i=0,j=0;
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //�������״̬
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x04);	    //�����ǲ����ʣ�1KHz 
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //��ͨ�˲��������ã���ֹƵ����1K��������5K  
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x08);	  //���ü��ٶȴ�����������4Gģʽ�����Լ�                  4.5 ���ٶȼ�����
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s) 4.4 ����������
}

//��ȡMPU6050��ID��Ϊ�˼��������Ƿ�ƥ��
uint8_t MPU6050_ReadID(void)
{
	unsigned char Re = 0;
  MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
	if(Re != 0x68)
	{
		printf(" MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
		printf(" MPU6050 ID = %d\r\n",Re);
		return 1;
	}
}

//ͬʱ��ȡ�����Ǻͼ��ٶȼƵ���������
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	u8 buf[12];
	
	//��ȡ���ݣ�ÿ�ζ�ȡ8λ���ݣ�ǰ������X�ᣬ�м�������Y�ᣬ���������Z��
	MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6); //�Ӽ��ٶȼƵĴ洢����ʼ��ַ��ʼ

	//���ư�λ����ڶ��ζ�ȡ���������һ��16λ�ļ��ٶȼƵ�ֵ
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
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];   
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}


