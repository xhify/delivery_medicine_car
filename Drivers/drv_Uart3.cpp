#include "Basic.hpp"
#include "drv_Uart3.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include <stdlib.h>
#define DATA_COUNT 5 // ���ݵ�����


/**
 * @brief 
 *  openMV �����Ĳ���
 * 0 ���Ƕ�
 * 1 �Ҳ�Ƕ�
 * 2 ��ʱδ��ʹ��
 * 3 ͣ����־
 * 4 �ȴ�ͣ����־
 */
 int openmv_data_array[DATA_COUNT] = {0}; 
int openmv_data_index = 0; // ��ǰ��������
int uart3_receive=0;
int start_index = 0; // ����������
int start_flag=0;
int end_index=0;

/*
 * �������ܣ�����1��ʼ��
 * ��ڲ�����������
 * ����  ֵ����
 */
void init_drv_Uart3(u32 bound)
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ��USART3��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//��ӳ��ʹ��
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
	//Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  //USART ��ʼ������ 
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	//��ʼ������3
  USART_Init(USART3, &USART_InitStructure); 
	//�������ڽ����ж�
  USART_ITConfig(USART3, USART_IT_RXNE,ENABLE);
	//ʹ�ܴ���3 
  USART_Cmd(USART3, ENABLE);                    
}

/*
 * �������ܣ�����1�����ж�
 * ��ڲ�������
 * ����  ֵ����
 */
extern "C" void USART3_IRQHandler(void)
{	
	
	int value;
	
	//���յ�����
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{	  
		//����������ر���
	   
		
		
		
  	uart3_receive = USART_ReceiveData(USART3); 
		if(uart3_receive == 0x5b && start_index == 0 && start_flag == 0)//��鿪ʼ��־��һλ
			start_index = 1;
		else if (uart3_receive == 0x5b && start_index == 1 && start_flag == 0)//��鿪ʼ��־�ڶ�λ
			start_flag =1;
		else if (uart3_receive == 0x29 && end_index == 0  && start_flag ==1)//��������־��һλ
			end_index =1;
		else if (uart3_receive == 0x29 && end_index == 1  && start_flag == 1)//�쳵������־�ڶ�λ
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
