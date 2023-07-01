#include "drv_hcsr04.h"
#include "sys.h"
extern char value[]; // �洢ת�����ֵ
extern int Length;
int Val = 0;
int countnum = 0;
int time = 0;
float length = 0;
float lengths = 0;
float sum = 0;
int cnt = 0;
/**
 * @brief ��ʼ�����������
 * 
 */
void hcsr04Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(HCSR04_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(HCSR04_TIM_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = HCSR04_TRIG;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // �������
	GPIO_Init(HCSR04_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������
	GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO;
	GPIO_Init(HCSR04_PORT, &GPIO_InitStructure);

	// GPIOB.9 �ж����Լ��жϳ�ʼ������,�½��ش���
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); // ��
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // �Ͻ��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); // �ܳ�ʼ���ж��߲���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			 // ʹ�ܰ����ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // ��ռ���ȼ� 2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		 // �����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 // ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);								 // �ݳ�ʼ�� NVIC

	TIM_DeInit(TIM5);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); // ��ʼ��

	TIM_ClearFlag(TIM5, TIM_FLAG_Update);	   // ��������жϣ����һ���ж����������ж�
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // �򿪶�ʱ�������ж�
	TIM_Cmd(TIM5, DISABLE);
}


/// @brief ��������������ź�
/// @param  
void UltraSonic_valuetance(void)
{
	float lengths = 0, sum = 0;
	int i = 0;

	GPIO_SetBits(HCSR04_PORT, HCSR04_TRIG);	  // ���ߵ�ƽ�ź�
	delay_us(20);							  // �ߵ�ƽ����10uS
	GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG); // ���͵�ƽ�ź�
}

extern "C" void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

extern "C" void EXTI9_5_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line8) != RESET) // �ж�ĳ�����ϵ��ж��Ƿ���
	{

		if (GPIO_ReadInputDataBit(HCSR04_PORT, HCSR04_ECHO) == 1)
		{
			TIM_Cmd(TIM5, ENABLE);
			cnt++;
		}
		else
		{
			TIM_Cmd(TIM5, DISABLE);
			time = int(TIM_GetCounter(TIM5));
			lengths = (time)*17 / 1000.0;
			if (lengths <= 0)
			{
				lengths = 0;
			}
			if (cnt != 5)
			{
				sum += lengths;
			}
			else
			{
				length = sum / 4.0;
				sum = 0;
				cnt = 0;
			}
			TIM_SetCounter(TIM5, 0); // ȡ��TIM5��counter�Ĵ������ֵ
		}

		EXTI_ClearITPendingBit(EXTI_Line8); // ��� LINE12 �ϵ��жϱ�־λ
	}
}
