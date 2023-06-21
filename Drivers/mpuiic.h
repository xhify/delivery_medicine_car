#pragma once

#ifdef __cplusplus
	extern "C" {
#endif

#include "sys.h"
#include "stdbool.h"
#include "stdint.h"
		
//为1代表使用硬件IIC
#define IIC_CONFIG 0
		
//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)8<<12;}	//SDA输入模式
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)3<<12;}  //SDA输出模式
#define SCL_OUT() {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=(u32)3<<8;} 	//SCL输出模式

#define SCL_PORT GPIOB												//SCL端口
#define SDA_PORT GPIOB												//SDA端口
#define SCL_PIN GPIO_Pin_10										//SCL管脚
#define SDA_PIN GPIO_Pin_11										//SDA管脚

//IO操作函数	 
#define IIC_SCL_High()  GPIO_SetBits(SCL_PORT,SCL_PIN) 				//SCL_High
#define IIC_SCL_Low()   GPIO_ResetBits(SCL_PORT,SCL_PIN) 			//SCL_Low
#define IIC_SDA_High()  GPIO_SetBits(SDA_PORT,SDA_PIN) 				//SDA_High
#define IIC_SDA_Low()   GPIO_ResetBits(SDA_PORT,SDA_PIN) 			//SDA_Low

#define READ_SDA    PBin(11)  										//输入SDA 
#define IIC_SCL     PBout(10) 										//SCL
#define IIC_SDA     PBout(11) 										//SDA	

//软件IIC操作函数
void IIC_Delay(void);													//IIC延时函数
void IIC_Init(void);                					//初始化IIC的IO口				 
void IIC_Start(void);													//发送IIC开始信号
void IIC_Stop(void);	  											//发送IIC停止信号
void IIC_Send_Byte(u8 txd);										//IIC发送一个字节
void IIC_Ack(void);														//IIC发送ACK信号
void IIC_NAck(void);													//IIC不发送ACK信号
u8 IIC_Read_Byte(unsigned char ack);					//IIC读取一个字节
u8 IIC_Wait_Ack(void); 												//IIC等待ACK信号
u8 Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);	//IIC连续写
u8 Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); 	//IIC连续读 
u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr);



#ifdef __cplusplus
	}
#endif