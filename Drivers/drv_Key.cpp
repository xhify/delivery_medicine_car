#include "stm32f10x.h"
#include "drv_Key.hpp"
#include "Basic.hpp"
#include "delay.h"
#include "GUI.hpp"
//������ʶ
int tran = 0;
static u16 Press_count;
extern int mode,R_PWM;
extern int wait_time,stop_time;
extern float angle_z;
extern int start_time ;
#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)//��ȡ����0
/**************************************************************************
�������ܣ������л�
��ڲ�������
����  ֵ����
����Ӳ��ȥ������ʹ�ó���˫�����
**************************************************************************/
void Key(void)
{	
	u16 tmp,tmp2;
	#if( Is_Detrembling != 0 )
		tmp = click();
		//��������
		if(tmp==1)
		{
			if(GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_0) == 0)
				GPIO_SetBits(GPIOE,GPIO_Pin_0);
			else if(GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_0) == 1)
				GPIO_ResetBits(GPIOE,GPIO_Pin_0);
			return;
		}
		else{}
	#else
	tmp=click_N_Double(25);
	//����
	if(tmp==1)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_0);
			LCD_Fill(0,0,240,240,WHITE);
		return;
	}
	//˫��
	else if(tmp==2)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_0);
		return;
	} 
	tmp2=Long_Press();
	//����������
	if(tmp2==3)
	{
		tran = 1;
	}
	#endif
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫�� 
**************************************************************************/
u8 click_N_Double (u16 time)
{
	static	u16 flag_key=0,count_key=0,double_key=0;	
	static	u16 count_single=0,Forever_count=0;
	//KEYΪPC0�ܽ�״̬����ʱ��������
	if(KEY==0)  		
		//������־λδ��1		
		Forever_count++;   					
	else       
		Forever_count=0;
	//�������£��Ұ������±�־Ϊ0	
	if(KEY == 0 && flag_key == 0)			
		flag_key=1;
	//��һ��Ϊ0	
	if(count_key == 0)						
	{
		if(flag_key==1) 
		{
			//��������һ�Σ�double_key��һ��
			double_key++;				
			//�������£�count=1			
			count_key=1;					
		}
		if(double_key==2) 
		{
			double_key=0;
			count_single=0;
			return 2;//˫��ִ�е�ָ��
		}
	}
	if(KEY == 1)			
		flag_key=0,count_key=0;				
	if(double_key == 1)						//�����Ѿ�����һ��
	{
		//�����ȴ�ʱ��
		count_single++;			
		//����ִ�е�ָ��		
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	
			return 1;
		}
		if(Forever_count>time)
		{
			double_key=0;
			count_single=0;	
		}
	}	
	return 0;
}
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
	//�������ɿ���־
	static u8 flag_key=1;
	//��������
	if(flag_key&&KEY==0)
	{
		flag_key=0;
		return 1;	
	}
	//�ް�������
	else if(1==KEY)			
		flag_key=1;
	return 0;
}
/**************************************************************************
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������2s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;
	//������־λδ��1
	if(Long_Press==0&&KEY==0) 
	{		
		Press_count++;
		Long_Press_count++;   
	}
	else                       
		Long_Press_count=0; 
	if(Long_Press_count>200)		
	{
		Long_Press=1;
		Long_Press_count=0;
		return 3;
	}	
	else if(Long_Press_count==1)
	{
		Press_count=0;
		return 1;
	}		
	//������־λ��1
	if(Long_Press==1)     
	{
		Long_Press=0;
	}
	return 0;
}

//Keyִ�к���
void Key_Task()
{
	Key();
	delay_ms(15);
}

//������ʼ������
void init_drv_Key(void)
{
	//KEY  PC0
	//����GPIOC����ʱ��,�������ù���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	   
	//KEY��ʼ��
	delay_us(1);
	//�����������ģʽ
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;                       //��������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;                           //PC0 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	//KEY��ʼ��
	
	EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
 

//GPIOE.2 �ж����Լ��жϳ�ʼ������,�½��ش���
 GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);//��
 EXTI_InitStructure.EXTI_Line=EXTI_Line0;
 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 EXTI_Init(&EXTI_InitStructure); //�ܳ�ʼ���ж��߲���
 
 
 NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //ʹ�ܰ����ⲿ�ж�ͨ��
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //��ռ���ȼ� 2��
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04; //�����ȼ� 2
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//�ݳ�ʼ�� NVIC
	
}
extern "C" void EXTI0_IRQHandler(void)
{
delay_ms(50); //����
if(KEY0==0) //���� KEY2
{   
	
	if(mode <3)
		mode+=1;
	else
		mode =0;
	
	wait_time = 0;
	angle_z = 0;
	stop_time =0;
	start_time=0;
}
EXTI_ClearITPendingBit(EXTI_Line0); //��� LINE2 �ϵ��жϱ�־λ 
delay_us(1);
}