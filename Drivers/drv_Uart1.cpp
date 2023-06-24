#include "Basic.hpp"
#include "drv_Uart1.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include <stdlib.h>
#define DATA_COUNT 9 // 数据的数量
 u8 Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0,Flag_sudu=2;
int uart_receive=0;
 
 int turn90left=0,turn90right=0,turn180left=0,turn180right=0,turn360left=0,turn360right=0;
 int data_array[DATA_COUNT] = {0}; // 用于存储解析的整数数据
int data_index = 0; // 当前数据索引
bool inside_brackets = false; // 是否在大括号内
char buffer[7]; // 临时存储数据的字符缓冲区，最大6位数字加上字符串结尾
int blue_flag_stop=0;
int blue_flag_wait =0;
int buffer_index = 0; // 缓冲区索引
bool update_single_value = false; // 是否仅更新单个值
bool all_parameters = false; // 是否更新所有参数
bool read_index = false; // 是否读取索引
 char rec[80];
 extern int mode;
 int in=0;
/*
 * 函数功能：串口1初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart1(u32 bound)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能UGPIOB时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//使能USART3时钟
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口3 
}

/*
 * 函数功能：串口1接收中断
 * 入口参数：无
 * 返回  值：无
 */
extern "C" void USART1_IRQHandler(void)
{	
	
	int value;
	//接收到数据
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{	  
		//蓝牙接收相关变量
	  
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=USART_ReceiveData(USART1); 
	
		//低速挡（默认值）
		if(uart_receive==0x59)  
			Flag_sudu=2;  
		//高速档
		if(uart_receive==0x58)  
			Flag_sudu=1;  
		//默认使用
		
	
		if (uart_receive == '{') {
			inside_brackets = true;
			data_index = 0;
			update_single_value = true;
			all_parameters = false;
			read_index = true;
		}

		else if (uart_receive == '}') {
			inside_brackets = false;

			if (update_single_value) {
				buffer[buffer_index] = '\0'; // 添加字符串结尾
				value = atoi(buffer); // 解析缓冲区中的整数
				data_array[data_index] = value; // 更新单个参数值
				update_single_value = false; // 重置更新标志
			}
			else if (all_parameters) {
				data_array[data_index++] = value; // 存储解析到的整数到数组中
			}
			buffer_index = 0; // 重置缓冲区索引

		}
		else if (inside_brackets) {
			if (uart_receive == '#') {
				all_parameters = true;
				update_single_value = false;
				read_index = false;
			}
			else if (uart_receive == ':') {
				buffer[buffer_index] = '\0'; // 添加字符串结尾
				value = atoi(buffer); // 解析缓冲区中的整数

				if (read_index && update_single_value) {
					data_index = value; // 更新参数索引
					read_index = false; // 重置读取索引标志
				}
				else if (update_single_value) {
					data_array[data_index] = value; // 更新单个参数值
					update_single_value = false; // 重置更新标志
				}
				else if (all_parameters) {
					data_array[data_index++] = value; // 存储解析到的整数到数组中
				}

				buffer_index = 0; // 重置缓冲区索引
			}
			else if (uart_receive >= '0' && uart_receive <= '9') {
				if (buffer_index < 6) {
					buffer[buffer_index++] = uart_receive; // 将字符添加到缓冲区
				}
				else {
					// 错误处理：数据值过大
					buffer[6] = '\0';
				}
			}
		}
	
		if(inside_brackets )
			rec[in++]=uart_receive;
		else {
			rec[in++]=uart_receive;
			rec[in]='\0';
			

			in=0;
		}
		
	  if(uart_receive>10)  //默认使用app为：MiniBalanceV3.5 因为MiniBalanceV3.5的遥控指令为A~H 其HEX都大于10
    {			
			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //右
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else if(uart_receive == 0x61)//接受到 ’a'
			{
				for(int i=0;i < 10; i++)
					data_array[i]=0;				
			}
			else if(uart_receive ==0x62)//				'b'	
														mode = 0;
			else if(uart_receive ==0x63)//    c
														mode =1;
			else if(uart_receive ==0x64)//   d
														mode =2;
			else if(uart_receive ==0x65)//		e
														mode =3;
			else if(uart_receive ==0x66)//    f
														blue_flag_stop=3;
			else if(uart_receive ==0x67)//		g
														blue_flag_stop = 2;
			else if(uart_receive ==0x68)//		h
														blue_flag_stop = 1;
			else if(uart_receive ==0x69)//		i
														blue_flag_wait = 1;
			
			else {Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0,turn90left=0,turn90right=0;//////////////刹车
				turn180left=0,turn180right=0,turn360left=0,turn360right=0;
  	}
	}
		
	}  											 
} 
