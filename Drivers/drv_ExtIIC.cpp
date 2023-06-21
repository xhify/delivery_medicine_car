#include "Basic.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"


#include "event_groups.h"
#include "semphr.h"
#include "AC_Math.hpp"
#include "vector3.hpp"
#include "inv_mpu.h"
#include "filter.h"
#include "delay.h"
#include <limits>

//IIC传输配置
	#define IIC_NACK (0<<10)
	#define IIC_ACK (1<<10)
	#define IIC_STOP (1<<9)
	#define IIC_START (1<<8)

//IIC状态
	#define IIC_OUT (1<<14)
	#define IIC_PECERR (1<<12)
	#define IIC_OVR (1<<11)
	#define IIC_NACKF (1<<10)
	#define IIC_ARLO (1<<9)
	#define IIC_BERR (1<<8)
	#define IIC_TE (1<<7)
	#define IIC_RXNE (1<<6)
	#define IIC_STOPF (1<<4)
	#define IIC_BTF (1<<2)
	#define IIC_ADDR (1<<1)
	#define IIC_SB (1<<0)
	
//超时时间
#define IIC_TIMEOUT 1.0*configTICK_RATE_HZ

using namespace std;
//IIC互斥锁
static SemaphoreHandle_t IICMutex = xSemaphoreCreateRecursiveMutex();
//发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
//待发送数据
static uint16_t RMTxLength = 0;
static uint16_t RMRxLength = 0;

/*软件IIC操作*/
	//IIC 延时函数
	void IIC_Delay(void)
	{
		delay_us(2);
	}

	/**************************************************************************
	函数功能：读取指定设备指定寄存器的一个值
	入口参数：I2C_Addr：设备IIC地址；addr:寄存器地址
	返回  值：res：读取的数据
	**************************************************************************/ 
	unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
	{
		unsigned char res=0;
		IIC_Start();	
		//发送写命令
		IIC_Send_Byte(I2C_Addr);	   		
		res++;
		IIC_Wait_Ack();
		//发送地址
		IIC_Send_Byte(addr); res++;  		
		IIC_Wait_Ack();	  
		IIC_Start();
		//进入接收模式	
		IIC_Send_Byte(I2C_Addr+1); res++;   		   
		IIC_Wait_Ack();
		res=IIC_Read_Byte(0);	   
		//产生一个停止条件
		IIC_Stop();							
		return res;
	}

	//初始化软件IIC
	void IIC_Init(void)
	{					     
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;                            
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
		IIC_SCL=1;
		IIC_SDA=1;
	}

	//产生IIC起始信号
	void IIC_Start(void)
	{
		//sda线输出
		SDA_OUT();     	
		IIC_SDA=1;	  	  
		IIC_SCL=1;
		delay_us(4);
		//START:when CLK is high,DATA change form high to low 
		IIC_SDA=0;		
		delay_us(4);
		//钳住I2C总线，准备发送或接收数据 
		IIC_SCL=0;		
	}

		  
	//产生IIC停止信号
	void IIC_Stop(void)
	{
		//sda线输出
		SDA_OUT();
		IIC_SCL=0;
		//STOP:when CLK is high DATA change form low to high
		IIC_SDA=0;
		delay_us(4);
		IIC_SCL=1; 
		//发送I2C总线结束信号
		IIC_SDA=1;
		delay_us(4);							   	
	}

	//等待应答信号到来
	//返回值：1，接收应答失败
	//        0，接收应答成功
	u8 IIC_Wait_Ack(void)
	{
		u8 ucErrTime=0;
		//SDA设置为输入  
		SDA_IN();      
		IIC_SDA=1;delay_us(1);	   
		IIC_SCL=1;delay_us(1);	 
		while(READ_SDA)
		{
			ucErrTime++;
			if(ucErrTime>250)
			{
				IIC_Stop();
				return 1;
			}
		}
		//时钟输出0 
		IIC_SCL=0;	   
		return 0;  
	} 

	//产生ACK应答
	void IIC_Ack(void)
	{
		IIC_SCL=0;
		SDA_OUT();
		IIC_SDA=0;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
	}

	//不产生ACK应答		    
	void IIC_NAck(void)
	{
		IIC_SCL=0;
		SDA_OUT();
		IIC_SDA=1;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
	}	

	//IIC发送一个字节
	//返回从机有无应答
	//1，有应答
	//0，无应答			  
	void IIC_Send_Byte(u8 txd)
	{                        
		u8 t;   
		SDA_OUT(); 	    
		IIC_SCL=0;                                                                 
		for(t=0;t<8;t++)
		{              
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			delay_us(2);                                                          
			IIC_SCL=1;
			delay_us(2); 
			IIC_SCL=0;	
			delay_us(2);
		}	 
	} 	
	
	//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
	u8 IIC_Read_Byte(unsigned char ack)
	{
		unsigned char i,receive=0;
		SDA_IN();                                                                  
		for(i=0;i<8;i++ )
		{
				IIC_SCL=0; 
				delay_us(2);
		IIC_SCL=1;
				receive<<=1;
				if(READ_SDA)receive++;   
		delay_us(1); 
		}					 
		if (!ack)
				IIC_NAck();                                                           
		else
				IIC_Ack();                                                            
		return receive;
	}

	//IIC连续写
	//addr:器件地址 
	//reg:寄存器地址
	//len:写入长度
	//buf:数据区
	//返回值:0,正常
	//    其他,错误代码
	u8 Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
	{
		u8 i; 
		IIC_Start(); 
		//发送器件地址+写命令
		IIC_Send_Byte((addr<<1)|0);	
		//等待应答
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();		 
			return 1;		
		}
		//写寄存器地址
		IIC_Send_Byte(reg);	
		//等待应答
		IIC_Wait_Ack();		
		for(i=0;i<len;i++)
		{
			//发送数据
			IIC_Send_Byte(buf[i]);	
			//等待ACK
			if(IIC_Wait_Ack())		
			{
				IIC_Stop();	 
				return 1;		 
			}		
		}    
		IIC_Stop();	 
		return 0;	
	} 

	//IIC连续读
	//addr:器件地址
	//reg:要读取的寄存器地址
	//len:要读取的长度
	//buf:读取到的数据存储区
	//返回值:0,正常
	//    其他,错误代码
	u8 Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
	{ 
		IIC_Start(); 
		//发送器件地址+写命令	
		IIC_Send_Byte((addr<<1)|0);
		//等待应答
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();		 
			return 1;		
		}
		//写寄存器地址
		IIC_Send_Byte(reg);	
		//等待应答
		IIC_Wait_Ack();		
		IIC_Start();
		//发送器件地址+读命令	
		IIC_Send_Byte((addr<<1)|1);
		//等待应答 
		IIC_Wait_Ack();		
		while(len)
		{
			//读数据,发送nACK 
			if(len==1)*buf=IIC_Read_Byte(0);
			//读数据,发送ACK 
			else *buf=IIC_Read_Byte(1);		 
			len--;
			buf++; 
		}    
		//产生一个停止条件
		IIC_Stop(); 
		return 0;	
	}
	
//	// 写一字节数据到I2C
//	// 传参数据，返回ACK(收到ACK返回0，失败返回1)
//	uint8_t soft_i2c_write(uint8_t data) 
//	{
//		uint8_t i = 8, ack;
//		
//		while(i--)
//		{		
//			if(data & (uint8_t)0x80) 
//			{
//				IIC_SDA_High();
//			} 
//			else 
//			{
//				IIC_SDA_Low();
//			}
//			data <<= 1;
//			IIC_Delay();
//			IIC_SCL_High();		
//			IIC_Delay();
//			IIC_SCL_Low();
//		}
//		
//		//检测ACK(0表示应答)
//		SDA_IN();		//端口配置为输入
//		
//		IIC_Delay();
//		IIC_SCL_High();	
//		IIC_Delay();
//		ack = (uint8_t)READ_SDA;
//		IIC_SCL_Low();
//		
//		SDA_OUT();		//恢复端口方向(输出)	
//		
//		return ack;
//	}


//	u8 soft_i2c_buffer_read(u8 devAddr, u8 regAddr,u8 *pBuffer, u8 numByteToRead)
//	{
//		uint8_t ack;
//		
//		IIC_Start();
//		ack = soft_i2c_write(devAddr | (uint8_t)0x00);       
//		if(ack) 
//			goto I2CBUFREADBAD1;	 
//		ack = soft_i2c_write(regAddr);	
//		if(ack) 
//			goto I2CBUFREADBAD2;
//		
//		IIC_Start();
//		ack = soft_i2c_write(devAddr | (uint8_t)0x01);
//		if(ack) 
//			goto I2CBUFREADBAD2;                            
//			 
//		SDA_IN();	 
//		
//		while(numByteToRead)  
//		{
//			if(numByteToRead == 1)
//			{
//				*pBuffer = IIC_Read_Byte(0);		
//			}	
//			else 
//			{
//				*pBuffer = IIC_Read_Byte(1);
//			}
//			pBuffer++; 
//			
//			numByteToRead--;  
//		}	 
//		
//		SDA_OUT();
//		I2CBUFREADBAD2:		
//		IIC_Stop();	
//		I2CBUFREADBAD1:

//		return ack;
//	}
	
/*软件IIC操作*/
	
/*硬件IIC操作*/
	
	/*
	上锁保证通信连续性
	上锁之后必须解锁
	Sync_waitTime：超时时间
	*/
	bool Lock_ExtIIC( double Sync_waitTime )
	{
		uint32_t Sync_waitTicks;
		if( Sync_waitTime >= 0 )
			Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
		else
			Sync_waitTicks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive( IICMutex , Sync_waitTicks ) == pdTRUE )
			return true;
		return false;
	}
	//解锁
	void Unlock_ExtIIC()
	{
		xSemaphoreGiveRecursive( IICMutex );
	}
	
	/*
	7位地址发送单个数据
	devaddr：7位器件地址
	addr：7位器件寄存器地址
	datas：要发送的数据指针
	Sync_waitTime：超时时间
	*/
	bool ExtIIC_SendOneAddr7(uint8_t devaddr, uint8_t addr, const uint8_t tx_data, double Sync_waitTime )
	{
		if( Lock_ExtIIC(Sync_waitTime) )
		{
			//计算当次发送长度
			RMTxLength = 1;
			RMTxLength -= 1;
			
			//清空状态
			xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
			//禁止DMA请求
			I2C2->CR2 |= (0<<11);
			//使能IIC
			I2C2->CR1 |= (1<<0);
			//不使用DMA
			/*
			//配置DMA
				//关闭DMA
				DMA1_Channel4->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<15) | (1<<14)  | (1<<13)  | (1<<12);
				//设置DMA存储器地址
				DMA1_Channel4->CMAR = (uint32_t)tx_datas;
				//设置DMA传输数量
				DMA1_Channel4->CNDTR = RMTxLength + length;
				//使能发送DMA1
				DMA1_Channel4->CCR |= (1<<0);
				I2C2->CR2 |= (1<<11);
			//配置DMA
			*/
			//允许发送空中断和接受非空中断使能
			I2C2->CR2 |= (1<<10);
			//等待IIC总线空闲
			while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
			//开始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
			//发送设备写地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Transmitter);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
			//发送要操作设备内部的地址
			I2C_SendData(I2C2,addr);
			//检测EV8_1事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING )==ERROR);
			I2C_SendData(I2C2,tx_data);
			//检测EV8_2事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED )==ERROR);
			//发送停止信号
			I2C_GenerateSTOP(I2C2,ENABLE);
			
			//等待传输完成
			uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT );
			//关闭DMA请求
			//I2C2->CR2 |= (0<<11);
			//关闭发送空中断和接受非空中断
			I2C2->CR2 |= (0<<10);
			//关闭IIC
			I2C2->CR1 &= ~( 1<<0 );
			//解锁
			Unlock_ExtIIC();
			
			if( revents & (IIC_NACKF|IIC_ARLO) )
				return false;
			else if( revents & IIC_STOPF )
				return true;
			return false;	
		}
		return false;
	}
	
	/*
	7位地址发送多个数据
	devaddr：7位器件地址
	addr：7位器件寄存器地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
	*/
	bool ExtIIC_SendAddr7(uint8_t devaddr, uint8_t addr, uint16_t tx_length, const uint8_t* tx_datas)
	{
		if( tx_length == 0 )
			return false;
		if( Lock_ExtIIC(-1) )
		{
			//计算当次发送长度
			RMTxLength = tx_length;
			if( RMTxLength > 255 )
				tx_length = 255;
			else
				tx_length = RMTxLength;
			RMTxLength -= tx_length;
			
			//清空状态
			xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
			//禁止DMA请求
			I2C2->CR2 |= (0<<11);
			//使能IIC
			I2C2->CR1 |= (1<<0);
			//不使用DMA
			/*
			//配置DMA
				//关闭DMA
				DMA1_Channel4->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<15) | (1<<14)  | (1<<13)  | (1<<12);
				//设置DMA存储器地址
				DMA1_Channel4->CMAR = (uint32_t)tx_datas;
				//设置DMA传输数量
				DMA1_Channel4->CNDTR = RMTxLength + length;
				//使能发送DMA1
				DMA1_Channel4->CCR |= (1<<0);
				I2C2->CR2 |= (1<<11);
			//配置DMA
			*/
			//允许发送空中断和接受非空中断使能
			I2C2->CR2 |= (1<<10);
			//等待IIC总线空闲
			while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
			//开始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
			//发送设备写地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Transmitter);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
			//发送要操作设备内部的地址
			I2C_SendData(I2C2,addr);
			//检测EV8_1事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING )==ERROR);
			//发送数据
			while(tx_length)
			{
				I2C_SendData(I2C2,*tx_datas);
				while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING )==ERROR);
				tx_length--;
				tx_datas++;
			}
			//检测EV8_2事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED )==ERROR);
			//发送停止信号
			I2C_GenerateSTOP(I2C2,ENABLE);
			
			//等待传输完成
			uint32_t revents = xEventGroupWaitBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR, pdTRUE, pdFALSE, IIC_TIMEOUT );
			//关闭DMA请求
			//I2C2->CR2 |= (0<<11);
			//关闭发送空中断和接受非空中断
			I2C2->CR2 |= (0<<10);
			//关闭IIC
			I2C2->CR1 &= ~( 1<<0 );
			//解锁
			Unlock_ExtIIC();
			
			if( revents & (IIC_NACKF|IIC_ARLO) )
				return false;
			else if( revents & IIC_STOPF )
				return true;
			return false;	
		}
		return false;
	}
	
	/*
	7位地址连续接收数据
	devaddr：7位器件地址
	addr：7位器件寄存器地址
	datas：要存储的数据指针
	length：接受的数据长度
	Sync_waitTime：超时时间
	*/
	bool ExtIIC_ReceiveAddr7(uint8_t devaddr, uint8_t addr, uint16_t rx_length, uint8_t* rx_datas)
	{
		if(rx_length==0 )
			return false;
		
		if( Lock_ExtIIC(-1))
		{
			//计算当次接收长度
			RMRxLength = rx_length;
			
			//清空状态
			xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
			//禁止DMA请求
			I2C2->CR2 |= (0<<11);
			//使能IIC
			I2C2->CR1 |= (1<<0);	
			/*配置DMA*/
			/*
				//关闭发送DMA
				DMA1_Channel4->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<15) | (1<<14)  | (1<<13)  | (1<<12);
				//设置DMA存储器地址
				DMA1_Channel4->CMAR = (uint32_t)tx_datas;
				//设置DMA传输数量
				DMA1_Channel4->CNDTR = RMTxLength + tx_length;
				//关闭接受DMA
				DMA1_Channel5->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<19) | (1<<18)  | (1<<17)  | (1<<16);
				//设置DMA存储器地址
				DMA1_Channel5->CMAR = (uint32_t)rx_datas;
				//设置DMA传输数量
				DMA1_Channel5->CNDTR = RMRxLength;
				//使能接受DMA1
				DMA1_Channel5->CCR |= (1<<0);
			*/
			/*配置DMA*/				
			//允许DMA请求
			//I2C2->CR2 |= (1<<11);
			
			//等待总线
			while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));  
			//开始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
			//发送设备写地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Transmitter);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
			//发送要操作设备内部的地址
			I2C_SendData(I2C2,addr);
			//检测EV8_1事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING )==ERROR);
			//发送起始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT )==ERROR);
			//发送设备读地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Receiver);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED )==ERROR);
			//接收数据
			while(RMRxLength--)
			{
				//是否是最后一个字节，若是则发送非应答信号
				if( RMRxLength==0)
				{
					//发送非应答信号
					I2C_AcknowledgeConfig(I2C2,DISABLE);
					//发送停止信号
					I2C_GenerateSTOP(I2C2,ENABLE);
				}
				//检测EV7事件
				while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED )==ERROR);
				*rx_datas=I2C_ReceiveData(I2C2);
				rx_datas++; 
			}
			//重新开启应答信号
			I2C_AcknowledgeConfig(I2C2,ENABLE);
			
			//关闭DMA请求
			//I2C2->CR2 |= (0<<11);
			//关闭IIC
			I2C2->CR1 &= ~( 1<<0 );
			//解锁
			Unlock_ExtIIC();
			return true;
		}
		return false;
	}
	
	/*
	7位地址接收数据
	devaddr：7位器件地址
	addr：7位器件寄存器地址
	datas：要存储的数据指针
	length：接受的数据长度
	Sync_waitTime：超时时间
	*/
	bool ExtIIC_ReceiveOneAddr7(uint8_t devaddr, uint8_t addr, uint8_t rx_data, double Sync_waitTime )
	{
		
		if( Lock_ExtIIC(Sync_waitTime) )
		{
			//计算当次接收长度
			RMRxLength = 1;
			
			//清空状态
			xEventGroupClearBits( events, IIC_STOPF|IIC_NACKF|IIC_ARLO|IIC_BERR );
			//禁止DMA请求
			I2C2->CR2 |= (0<<11);
			//使能IIC
			I2C2->CR1 |= (1<<0);	
			/*配置DMA*/
			/*
				//关闭发送DMA
				DMA1_Channel4->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<15) | (1<<14)  | (1<<13)  | (1<<12);
				//设置DMA存储器地址
				DMA1_Channel4->CMAR = (uint32_t)tx_datas;
				//设置DMA传输数量
				DMA1_Channel4->CNDTR = RMTxLength + tx_length;
				//关闭接受DMA
				DMA1_Channel5->CCR &= ~( (1<<0) );
				//清空DMA状态
				DMA1->IFCR = (1<<19) | (1<<18)  | (1<<17)  | (1<<16);
				//设置DMA存储器地址
				DMA1_Channel5->CMAR = (uint32_t)rx_datas;
				//设置DMA传输数量
				DMA1_Channel5->CNDTR = RMRxLength;
				//使能接受DMA1
				DMA1_Channel5->CCR |= (1<<0);
			*/
			/*配置DMA*/				
			//允许DMA请求
			//I2C2->CR2 |= (1<<11);
			
			//等待总线
			while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));  
			//开始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT)==ERROR);
			//发送设备写地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Transmitter);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)==ERROR);
			//发送要操作设备内部的地址
			I2C_SendData(I2C2,addr);
			//检测EV8_1事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING )==ERROR);
			//发送起始信号
			I2C_GenerateSTART(I2C2,ENABLE);
			//检测EV5事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT )==ERROR);
			//发送设备读地址
			I2C_Send7bitAddress(I2C2,devaddr,I2C_Direction_Receiver);
			//检测EV6事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED )==ERROR);
			//接收数据
			//检测EV7事件
			while( I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED )==ERROR);
			rx_data=I2C_ReceiveData(I2C2);
			RMRxLength--;
			//发送非应答信号
			I2C_AcknowledgeConfig(I2C2,DISABLE);
			//发送停止信号
			I2C_GenerateSTOP(I2C2,ENABLE);
			//重新开启应答信号
			I2C_AcknowledgeConfig(I2C2,ENABLE);
			
			//关闭DMA请求
			//I2C2->CR2 |= (0<<11);
			//关闭IIC
			I2C2->CR1 &= ~( 1<<0 );
			//解锁
			Unlock_ExtIIC();
			return true;
		}
		return false;
	}
	
	//发送接收中断
	extern "C" void I2C2_EV_IRQHandler()
	{	
		BaseType_t HigherPriorityTaskWoken = pdFALSE;
		uint8_t SR1 = I2C2->SR1;
		if(!(I2C2->SR2 & (1<<1)))
			//置位完成标志	
			xEventGroupSetBitsFromISR( events, SR1, &HigherPriorityTaskWoken );
		//清空已记录的所有标志
		I2C2->SR1 = SR1 & ( (1<<2) | (1<<1) | (1<<0) );
		portYIELD_FROM_ISR(HigherPriorityTaskWoken);
	}
	
	//错误中断
	extern "C" void I2C1_ER_IRQHandler()
	{
		BaseType_t HigherPriorityTaskWoken = pdFALSE;
		uint32_t SR1 = I2C2->SR1;
		
		//清空已记录的错误标志
		I2C2->SR1 = SR1 & ( (1<<14) | (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8) );
		//置位错误标志
		xEventGroupSetBitsFromISR( events, SR1, &HigherPriorityTaskWoken );	
		
		portYIELD_FROM_ISR(HigherPriorityTaskWoken);
	}
	/*硬件IIC操作*/
	
//初始化硬件IIC
void init_drv_ExtIIC()
{
	/*配置IIC2*/
		uint16_t tmpreg = 0, freqrange = 0;
		uint16_t result = 0x04;
		uint32_t pclk1 = 8000000;
		RCC_ClocksTypeDef  rcc_clocks;
		//打开IIC2时钟
		RCC->APB1ENR |= (1<<22);
		os_delay(0.01);
		
		//复位IIC
		I2C2->CR1 = (1<<15);
		os_delay(0.005);
			
		//IIC时间配置
		//获取 pclk1 时钟频率
		RCC_GetClocksFreq(&rcc_clocks);
		pclk1 = rcc_clocks.PCLK1_Frequency;
		//设置 pclk1 
		freqrange = (uint16_t)(pclk1 / 1000000);
		tmpreg |= freqrange;
		//配置IIC 时钟频率
		I2C2->CR2 = (tmpreg);
		//不使能IIC
		I2C2->CR1 &= 0xFFFE;tmpreg = 0;
		//快速模式下时钟配置
		result = (uint16_t)(pclk1 / (400000 * 3));
		tmpreg |= (uint16_t)(result | 0x8000);
		//设置最大上升时间
		I2C2->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);
		//设置时钟控制分频系数，IIC选择快速模式，Tlow/Thigh = 2；
		I2C2->CCR = (1<<15) | (0<<14) | (tmpreg);
		//7位地址:0x6f
		I2C2->OAR1 = (0<<15) | (0x6f<<0);
		//IIC模式，使能IIC
		I2C2->CR1 |= (0<<1) | (1<<0);
		//允许DMA请求
		//I2C2->CR2 |= (1<<11);
		//允许出错中断使能
		I2C2->CR2 |= (1<<8);
		NVIC_SetPriority(I2C2_EV_IRQn,5);
		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_SetPriority(I2C2_ER_IRQn,5);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
	/*配置IIC2*/
	
	/*DMA初始化*/
	/*IIC2的DMA与串口1的DMA通道连接在一起,建议不配置DMA*/
	/*
		//打开DMA1时钟
		RCC->AHBENR |= (1<<0);
		os_delay(1e-5);
		
		//DMA1_Channel4 IIC2 TX
		//外设地址寄存器为串口2数据寄存器
		DMA1_Channel4->CPAR = (uint32_t)&I2C2->DR;
		//优先级为0，存储器数据8位，存储器地址指针递增，外设地址指针固定，存储器到外设
		DMA1_Channel4->CCR = (0<<14) | (0<<12) | (0<<10) | (1<<7) | (1<<4);
		
		//DMA1_Channel5 IIC2 RX
		//外设地址寄存器为串口2数据寄存器
		DMA1_Channel5->CPAR = (uint32_t)&I2C2->DR;
		//优先级为0，存储器数据8位，存储器地址指针递增，外设地址指针固定，外设到存储器
		DMA1_Channel5->CCR = (0<<14) | (0<<12) | (0<<10) | (1<<7) | (0<<4);
	*/
	/*DMA初始化*/
	
	/*配置GPIO PB10(IIC2 SCL) PB11(IIC2 SDA)*/
		//打开GPIOB电源
		RCC->APB2ENR |= (1<<3);
		os_delay(0.01);
				
		//引脚开漏上拉，中速
		set_register( GPIOB->ODR , 1 , 10 , 1 );		//PB10
		set_register( GPIOB->ODR , 1 , 11 , 1 );		//PB11
		set_register( GPIOB->CRH , 0b10 , 8 , 2 );	//PB10
		set_register( GPIOB->CRH , 0b10 , 12 , 2 );	//PB11
		//复用IIC2
		set_register( GPIOB->CRH, 0b11, 10, 2 );		//PB10
		set_register( GPIOB->CRH, 0b11, 14, 2 );		//PB11
	/*配置GPIO PB10(IIC2 SCL) PB11(IIC2 SDA)*/
}