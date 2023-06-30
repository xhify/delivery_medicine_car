#include "Basic.hpp"
#include "drv_Uart3.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include <stdlib.h>
#define DATA_COUNT 5 // 数据的数量



 int openmv_data_array[DATA_COUNT] = {0}; // 用于存储解析的整数数据
int openmv_data_index = 0; // 当前数据索引
int uart3_receive=0;
int start_index = 0; // 缓冲区索引
int start_flag=0;
int end_index=0;

/*
 * 函数功能：串口1初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart3(u32 bound)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	//使能USART3，GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//重映射使能
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
	//USART3_TX   GPIOD.8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //USART3_RX	  GPIOD.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	//Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  //USART 初始化设置 
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	//初始化串口3
  USART_Init(USART3, &USART_InitStructure); 
	//开启串口接受中断
  USART_ITConfig(USART3, USART_IT_RXNE,ENABLE);
	//使能串口3 
  USART_Cmd(USART3, ENABLE);                    
}

/*
 * 函数功能：串口1接收中断
 * 入口参数：无
 * 返回  值：无
 */
extern "C" void USART3_IRQHandler(void)
{	
	
	int value;
	
	//接收到数据
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{	  
		//蓝牙接收相关变量
	   
		
		
		
  	uart3_receive = USART_ReceiveData(USART3); 
		if(uart3_receive == 0x5b && start_index == 0 && start_flag == 0)//检查开始标志第一位
			start_index = 1;
		else if (uart3_receive == 0x5b && start_index == 1 && start_flag == 0)//检查开始标志第二位
			start_flag =1;
		else if (uart3_receive == 0x29 && end_index == 0  && start_flag ==1)//检查结束标志第一位
			end_index =1;
		else if (uart3_receive == 0x29 && end_index == 1  && start_flag == 1)//检车结束标志第二位
		{
			start_flag =0;
			end_index =0;
			start_index =0;
			openmv_data_index =0;
		}
		else if(start_flag ==1)
		{
			openmv_data_array[openmv_data_index] =uart3_receive;
			openmv_data_index +=1 ;
		}
	}
} 
