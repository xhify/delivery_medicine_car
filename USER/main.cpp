#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Key.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "drv_Encoder.hpp"
#include "drv_PWMOut.hpp"
#include "drv_Uart1.hpp"
#include "drv_Uart3.hpp"
#include "drv_beep.h"
#include <stdbool.h>
#include "drv_hcsr04.h"
#include "control.h"
extern "C"
{
#include "adc.h"
#include "MPU6050.h"
#include "mpuiic.h"
}

RCC_ClocksTypeDef RCC_CLK;

/// @brief ����С�����е�ģʽ
int mode = -1;
int is_leader = 0; // �Ƿ�Ϊ��ͷС��
extern int openmv_data_array[5];
/// @brief ����ͨ�Ŵ��ݵĲ���
extern int data_array[9];
extern char rec[80];
float VDDA; // ��ص�ѹ
/// @brief �����������
extern int L_code;
/// @brief �Ҳ���������
extern int R_code;
extern float L_speed, R_speed;
int L_count = 0;
int R_count = 0;
/// @brief ���ҵ��pwm���
extern int L_PWM, R_PWM;
char str[50];
extern struct PID follow_PID, speed_PID, distance_PID;
int L_angle, R_angle;
extern float length;
/// @brief  �������泬�������õ��ĸ����ٶ�
float distance_speed = 0;

int i = 0;
/// @brief ͣ����־
int flag_stop = 0;
/// @brief �ȴ���־
int flag_wait = 0;
int start_time = 0;
int wait_time = 0;

int stop_time = 0;
extern int blue_flag_stop, blue_flag_wait; // �������ݵ�ͣ����־
int flag_follow = 0;
// if((angle_z<10)||(angle_z>80&&angle_z<=100)||(angle_z>170&&angle_z<=190)||(angle_z>260&&angle_z<=280)||(angle_z>350&&angle_z<370)
//||(angle_z>440&&angle_z<=460)||(angle_z>530&&angle_z<=550)||(angle_z>620&&angle_z<=640)||(angle_z>710&&angle_z<730)
//||(angle_z>800&&angle_z<=820)||(angle_z>890&&angle_z<=910)||(angle_z>980&&angle_z<=1000)||(angle_z>1070))

extern float G_X, G_Z, G_Y;

float angle_z = 0;

/// @brief Ŀ���ٶ�
float targer_speed = 0.1;

void TIM2_Int_Init(u16 arr, u16 psc);

int main(void)
{
	// ��ʼ�����ֳ���
	{
		// ϵͳʱ�ӳ�ʼ��
		SystemInit();
		Stm32_Clock_Init(9);
		// ��ʱʱ�ӳ�ʼ��
		delay_init();
		// ��SWD�ӿ�,�رո��ù���
		JTAG_Set(SWD_ENABLE);
		// ��ȡʱ������
		RCC_GetClocksFreq(&RCC_CLK);
		// ����ϵͳ�ж����ȼ�����4
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		// LED��ʼ������
		init_drv_LED();

		BEEP_Init();

		// �ر���mpu6050�ĳ�ʼ��
		MPU_Init();
		while (mpu_dmp_init())
			;
		// Play();

		// LED��˸
		init_Commulink();
		// ��ʼ����Ļ
		init_drv_LCD();
		// ��ʼ��GUI����
		init_GUI();
		init_drv_Key();
		Adc_Init();
		init_drv_Uart1(9600);
		drv_Encoder_Init_TIM3();
		drv_Encoder_Init_TIM4();
		init_drv_PWMOut();
		init_drv_Motor();
		TIM2_Int_Init(49, 7199); // 72mhz     0.005s
		init_drv_LED();
		// ��ʼ����Ļ
		init_drv_LCD();
		// ��ʼ��GUI����

		// ��ʼ������
		init_drv_Key();
		// ��ʼ����ʱ��
		// ��ʼ�����������
		hcsr04Init();
		// ��ʼ������2
		init_drv_Uart3(9600);
	}
	// ��ʼ�����

	// ������ѭ��

	// while 1ִ��Ƶ��Ϊ0.1s
	while (1)
	{
		
	
		VDDA = Get_Adc_Average(12, 5);
		VDDA = VDDA * 3.3 * 11 * 1.1 / 4096;
		/**
		 * @brief ��Ļ��ʾ����
		 */
		{
			sprintf(str, "%d", mode);
			LCD_ShowString(40, 0, str, BLUE, WHITE, 16, 0);
			// pitch
			sprintf(str, "%d", is_leader);
			LCD_ShowString(80, 16, str, BLUE, WHITE, 16, 0);
			// yaw
			sprintf(str, "%3.1f", targer_speed);
			LCD_ShowString(48, 32, str, BLUE, WHITE, 16, 0);

			/*��ȡ��̬*/

			sprintf(str, "%4d", L_code);
			LCD_ShowString(48, 48, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4d", R_code);
			LCD_ShowString(48, 64, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.4f", L_speed);
			LCD_ShowString(120, 48, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.4f", R_speed);
			LCD_ShowString(120, 64, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%3d", L_angle);
			LCD_ShowString(64, 80, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%3d", R_angle);
			LCD_ShowString(64, 96, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%6d", L_PWM);
			LCD_ShowString(48, 112, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%6d", R_PWM);
			LCD_ShowString(48, 128, str, BLUE, WHITE, 16, 0);

			// ѭ������pid����
			sprintf(str, "%4.2f", follow_PID.kp);
			LCD_ShowString(0, 144, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", follow_PID.ki);
			LCD_ShowString(80, 144, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4d", i);
			LCD_ShowString(160, 144, str, BLUE, WHITE, 16, 0);
			// �ٶȿ���pid����
			sprintf(str, "%4.2f", speed_PID.kp);
			LCD_ShowString(0, 160, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", speed_PID.ki);
			LCD_ShowString(80, 160, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", speed_PID.kd);
			LCD_ShowString(160, 160, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", distance_PID.kp);
			LCD_ShowString(0, 176, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", distance_PID.ki);
			LCD_ShowString(80, 176, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", distance_PID.kd);
			LCD_ShowString(160, 176, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", angle_z);
			LCD_ShowString(160, 192, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", distance_speed);
			LCD_ShowString(80, 192, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", G_Z);
			LCD_ShowString(0, 192, str, BLUE, WHITE, 16, 0);

			sprintf(str, "%2.2f", VDDA);
			LCD_ShowString(160, 0, str, BLUE, WHITE, 16, 0);

			LCD_ShowString(0, 208, rec, BLUE, WHITE, 16, 0);

			sprintf(str, "%4.2f", length);
			LCD_ShowString(0, 224, str, BLUE, WHITE, 16, 0);
		}
		
	}
}

extern "C" void TIM2_IRQHandler(void)
{
	// Ƶ��Ϊ0.005s,200hz
	// ��� TIM2���� �жϷ������
	int index;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		// ��� TIM2���� �ж� ��־
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		int L_bias = L_angle - 90;
		int R_bias = R_angle - 90;

		UltraSonic_valuetance();
		i++;
		L_angle = openmv_data_array[0];
		R_angle = openmv_data_array[1];
	}
}

void TIM2_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// ʱ�� TIM2 ʹ��

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// �����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Period = arr;
	// ����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	// ����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// TIM���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// ��ʼ�� TIM2
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	// ��������ж�
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// �ж� ���ȼ� NVIC ����
	// TIM2 �ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	// ��ռ���ȼ� 0 ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// �����ȼ� 3 ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	// IRQ ͨ����ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// ��ʼ�� NVIC �Ĵ���
	NVIC_Init(&NVIC_InitStructure);
	// ʹ�� TIM 2
	TIM_Cmd(TIM2, ENABLE);
}
