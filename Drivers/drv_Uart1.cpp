#include "Basic.hpp"
#include "drv_Uart1.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include <stdlib.h>
#define DATA_COUNT 9 // ���ݵ�����
 u8 Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0,Flag_sudu=2;
int uart_receive=0;
 
 int turn90left=0,turn90right=0,turn180left=0,turn180right=0,turn360left=0,turn360right=0;
 /**
  * @brief ���ڴ洢��������������
  * 
  */
 int data_array[DATA_COUNT] = {0}; 
int data_index = 0; // ��ǰ��������
bool inside_brackets = false; // �Ƿ��ڴ�������
char buffer[7]; // ��ʱ�洢���ݵ��ַ������������6λ���ּ����ַ�����β
int blue_flag_stop=0;
int blue_flag_wait =0;
int buffer_index = 0; // ����������
bool update_single_value = false; // �Ƿ�����µ���ֵ
bool all_parameters = false; // �Ƿ�������в���
bool read_index = false; // �Ƿ��ȡ����
 char rec[80];
 extern int mode;
 int in=0;
/*
 * �������ܣ�����1��ʼ��
 * ��ڲ�����������
 * ����  ֵ����
 */
void init_drv_Uart1(u32 bound)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��UGPIOBʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//ʹ��USART3ʱ��
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���3 
}

/*
 * �������ܣ�����1�����ж�
 * ��ڲ�������
 * ����  ֵ����
 */
extern "C" void USART1_IRQHandler(void)
{	
	
	int value;
	//���յ�����
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{	  
		//����������ر���
	  
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=USART_ReceiveData(USART1); 
	
		//���ٵ���Ĭ��ֵ��
		if(uart_receive==0x59)  
			Flag_sudu=2;  
		//���ٵ�
		if(uart_receive==0x58)  
			Flag_sudu=1;  
		//Ĭ��ʹ��
		
	
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
				buffer[buffer_index] = '\0'; // ����ַ�����β
				value = atoi(buffer); // �����������е�����
				data_array[data_index] = value; // ���µ�������ֵ
				update_single_value = false; // ���ø��±�־
			}
			else if (all_parameters) {
				data_array[data_index++] = value; // �洢��������������������
			}
			buffer_index = 0; // ���û���������

		}
		else if (inside_brackets) {
			if (uart_receive == '#') {
				all_parameters = true;
				update_single_value = false;
				read_index = false;
			}
			else if (uart_receive == ':') {
				buffer[buffer_index] = '\0'; // ����ַ�����β
				value = atoi(buffer); // �����������е�����

				if (read_index && update_single_value) {
					data_index = value; // ���²�������
					read_index = false; // ���ö�ȡ������־
				}
				else if (update_single_value) {
					data_array[data_index] = value; // ���µ�������ֵ
					update_single_value = false; // ���ø��±�־
				}
				else if (all_parameters) {
					data_array[data_index++] = value; // �洢��������������������
				}

				buffer_index = 0; // ���û���������
			}
			else if (uart_receive >= '0' && uart_receive <= '9') {
				if (buffer_index < 6) {
					buffer[buffer_index++] = uart_receive; // ���ַ���ӵ�������
				}
				else {
					// ����������ֵ����
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
		
	  if(uart_receive>10)  //Ĭ��ʹ��appΪ��MiniBalanceV3.5 ��ΪMiniBalanceV3.5��ң��ָ��ΪA~H ��HEX������10
    {			
			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //��
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else if(uart_receive == 0x61)//���ܵ� ��a'
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
			
			else {Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0,turn90left=0,turn90right=0;//////////////ɲ��
				turn180left=0,turn180right=0,turn360left=0,turn360right=0;
  	}
	}
		
	}  											 
} 
