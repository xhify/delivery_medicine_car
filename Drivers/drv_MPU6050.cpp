#include "Basic.hpp"
#include "drv_MPU6050.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"
#include "FreeRTOS.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"
#include "AC_Math.hpp"
#include "vector3.hpp"
#include "inv_mpu.h"
#include "filter.h"
#include "delay.h"
#include <limits>

using namespace std;

//陀螺仪变量
short gyro[3], accel[3], sensors;
//三轴角度 Z轴陀螺仪
float Pitch,Roll,Yaw,Gryo_Z; 
//加速度传感器原始数据
short aacx,aacy,aacz;	
//陀螺仪原始数据
short gyrox,gyroy,gyroz;	
//温度
short temp;								

int16_t accRaw[3], gyroRaw[3];
int16_t temperuteRaw;
static uint8_t _mpu_half_resolution;

/*IIC操作*/

	//IIC写一个字节 
	//reg:寄存器地址
	//data:数据
	//返回值:0,正常
	//其他,错误代码
	u8 Write_Byte(u8 reg,u8 data) 				 
	{ 
		if(!IIC_CONFIG)
		{ 
			IIC_Start(); 
			IIC_Delay();
			IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
			if(IIC_Wait_Ack())	//等待应答
			{
				IIC_Stop();		 
				return 1;		
			}
			IIC_Send_Byte(reg);	//写寄存器地址
			IIC_Wait_Ack();		//等待应答 
			IIC_Send_Byte(data);//发送数据
			if(IIC_Wait_Ack())	//等待ACK
			{
				IIC_Stop();	 
				return 1;		 
			}		 
			IIC_Stop();	 
			return 0;
		}
		else
		{
			if(ExtIIC_SendOneAddr7(MPU_ADDR,reg,data,-1)==true)
				return 0;
			else
				return 1;
		}
	}
	//IIC读一个字节 
	//reg:寄存器地址 
	//返回值:读到的数据
	u8 Read_Byte(u8 reg)
	{
		u8 res;
		if(!IIC_CONFIG)
		{ 
			IIC_Start(); 
			IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
			IIC_Wait_Ack();		//等待应答 
			IIC_Send_Byte(reg);	//写寄存器地址
			IIC_Wait_Ack();		//等待应答
			IIC_Start();
			IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
			IIC_Wait_Ack();		//等待应答 
			res=IIC_Read_Byte(0);//读取数据,发送nACK 
			IIC_Stop();			//产生一个停止条件 
		}
		else
			res = ExtIIC_ReceiveOneAddr7(MPU_ADDR,reg,res,-1);
		return res;		
	}

/*IIC操作*/

/*MPU6050操作*/
	//初始化MPU6050
	//返回值:0,成功
	//其他,错误代码
	u8 MPU_Init(void)
	{ 
		u8 res;
		//初始化IIC总线
		if(!IIC_CONFIG)
			IIC_Init();							
		else
		{
			init_drv_ExtIIC();
		}
		//复位MPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X80);	
		delay_ms(100);
		//唤醒MPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X00);	 
		//陀螺仪传感器,±2000dps 65536/4000=16.4 LSB
		MPU_Set_Gyro_Fsr(3);				
		//加速度传感器,±8g 65536/16=4096 LSB
		MPU_Set_Accel_Fsr(2);		
		//设置采样率500Hz		
		MPU_Set_Rate(500);						
		//关闭所有中断
		Write_Byte(MPU_INT_EN_REG,0X00);	
		//I2C主模式关闭
		Write_Byte(MPU_USER_CTRL_REG,0X00);	
		//关闭FIFO
		Write_Byte(MPU_FIFO_EN_REG,0X00);	
		//INT引脚低电平有效
		Write_Byte(MPU_INTBP_CFG_REG,0X80);	
		res=Read_Byte(MPU_DEVICE_ID_REG);
		//设置CLKSEL,PLL Z轴为参考
		Write_Byte(MPU_PWR_MGMT1_REG,0X03);	
		//加速度与陀螺仪都工作，不进入低功耗模式
		Write_Byte(MPU_PWR_MGMT2_REG,0X00);					
		return 0;
	}
	//设置MPU6050陀螺仪传感器满量程范围
	//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 MPU_Set_Gyro_Fsr(u8 fsr)
	{
		return Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
	}
	//设置MPU6050加速度传感器满量程范围
	//fsr:0,±2g;1,±4g;2,±8g;3,±16g
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 MPU_Set_Accel_Fsr(u8 fsr)
	{
		return Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
	}
	//设置MPU6050的数字低通滤波器
	//lpf:数字低通滤波频率(Hz)
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 MPU_Set_LPF(u16 lpf)
	{
		u8 data=0;
		if(lpf>=188)data=1;
		else if(lpf>=98)data=2;
		else if(lpf>=42)data=3;
		else if(lpf>=20)data=4;
		else if(lpf>=10)data=5;
		else data=6; 
		return Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
	}
	//设置MPU6050的采样率(假定Fs=1KHz)
	//rate:4~1000(Hz)
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 MPU_Set_Rate(u16 rate)
	{
		u8 data;
		if(rate>1000)rate=1000;
		if(rate<4)rate=4;
		data=1000/rate-1;
		data=Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
		return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
	}

	//得到温度值
	//返回值:温度值(扩大了100倍)
	short MPU_Get_Temperature(void)
	{
		u8 buf[2]; 
		short raw;
		float temp;
		Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
		raw=((u16)buf[0]<<8)|buf[1];  
		temp=36.53+((double)raw)/340;  
		return temp*100;;
	}
	//得到陀螺仪值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
	{
		u8 buf[6],res;  
		res=Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
		if(res==0)
		{
			*gx=((u16)buf[0]<<8)|buf[1];  
			*gy=((u16)buf[2]<<8)|buf[3];  
			*gz=((u16)buf[4]<<8)|buf[5];
		} 	
		return res;;
	}
	//得到加速度值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
	{
		u8 buf[6],res;  
		res=Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
		if(res==0)
		{
			*ax=((u16)buf[0]<<8)|buf[1];  
			*ay=((u16)buf[2]<<8)|buf[3];  
			*az=((u16)buf[4]<<8)|buf[5];
		} 	
		return res;;
	}

	/**************************************************************************
	函数功能：读取MPU6050内置DMP的姿态信息
	入口参数：无
	返回  值：无
	**************************************************************************/
	void Read_DMP(void)
	{	
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
	}
	
	/**************************************************************************
	函数功能：读取MPU6050版本信息
	入口参数：无
	返回  值：无
	**************************************************************************/
	static void mpu6050FindRevision(void)
	{
			uint8_t readBuffer[6];
			uint8_t ack = !Read_Len(MPU_ADDR, MPU_ACCEL_OFFS_REG, 6, readBuffer);
			uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
			if (ack && revision) 
			{
					if (revision == 1) 
					{
							_mpu_half_resolution = 1;
					} 
					else if (revision == 2) 
					{
							_mpu_half_resolution = 0;
					} 
					else if ((revision == 3) || (revision == 7)) 
					{
							_mpu_half_resolution = 0;
					}
					else 
					{     
						
					}
			} 
			else 
			{
					uint8_t productId;
					ack = !Read_Len(MPU_ADDR, MPU_PROD_ID_REG, 1, &productId);
					revision = productId & 0x0F;
					if (!ack || revision == 0) 
					{
						
					} 
					else if (revision == 4) 
					{
							_mpu_half_resolution = 1;
					} 
					else 
					{
							_mpu_half_resolution = 0;
					}
			}
	}
	
	/**************************************************************************
	函数功能：读取MPU6050原始信息
	入口参数：无
	返回  值：无
	**************************************************************************/
	uint8_t mpu6050_read(float* acc, float* gyro)
	{		
		float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
		
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		
		acc[0] = Accel_X;
		acc[1] = Accel_Y;
		acc[2] = Accel_Z;
		gyro[0] = Gyro_X;
		gyro[1] = Gyro_Y;
		gyro[2] = Gyro_Z;
		
		return 1; 
	}
	
	/**************************************************************************
	函数功能：获取角度	
	入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
	返回  值：无
	**************************************************************************/	
	void Get_Angle(uint8_t way)
	{ 
		float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
		if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
		{	
			//读取加速度、角速度、倾角
			if(mpu_dmp_get_data(&Pitch,&Roll,&Yaw)==0)
			{ 
				temp=MPU_Get_Temperature();								//得到温度值
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			}
		}			
		else
		{
			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
			Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
			Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
			Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
			if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  
			if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  
			if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
			if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
			if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
			if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
			Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
			Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
			Gyro_X=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
			Gyro_Y=Gyro_Y/16.4;                              //陀螺仪量程转换	
			if(way==2)		  	
			{
				 Pitch = -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波
				 Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
			}
			else if(way==3) 
			{  
				 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
				 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
			}
		}	
	}
/*MPU6050操作*/

//MPU6050任务
static void MPU_Server(void* pvParameters)
{
	while(1)
	{
		//获取函数
		Get_Angle(1);
	}
}
	
//初始化MPU6050
void init_drv_MPU6050()	
{
	u8 res;
	//MPU6050初始化
	res = MPU_Init();
	//器件失败则卡死
	while(res);
	//等待MPU6050初始化完成
	while(mpu_dmp_init());
	mpu6050FindRevision();
}