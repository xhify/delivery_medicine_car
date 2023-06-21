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
#include "drv_beep.h"
#include <stdbool.h>
#include "drv_hcsr04.h"
extern"C"{
	#include "adc.h"
#include "MPU6050.h"
#include "mpuiic.h"
}

RCC_ClocksTypeDef RCC_CLK;
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; 
extern int data_array[9] ;
extern int  uart_receive,turn90left,turn90right,turn180left,turn180right,turn360left,turn360right;
extern char rec[80];
float VDDA;//电池电压
int L_code;
int R_code;
int L_count=0;
int R_count=0;;
int L_PWM=200;
int R_PWM=200;
char str[50];
int PWM;

//函数原型

void TIM2_Int_Init(u16 arr,u16 psc);
int Balance(float Angle,float Gyro);
int Velocity(int encoder_left,int encoder_right);
int turn(int encoder_left, int encoder_right, float gyro);
int turn_90_degrees(float angle_z);

int main(void)
{	
	//初始化部分程序
	{
	//系统时钟初始化
	SystemInit();
	Stm32_Clock_Init(9);
	//延时时钟初始化
	delay_init();	
	//打开SWD接口,关闭复用功能
	JTAG_Set(SWD_ENABLE);        
	//获取时钟总线
	RCC_GetClocksFreq(&RCC_CLK);	
	//设置系统中断优先级分组4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
		//LED初始化函数
	init_drv_LED();

	BEEP_Init();
	
	//关闭了mpu6050的初始化
	//MPU_Init();	
	//while(mpu_dmp_init());
		
	Play();

	//LED闪烁
	init_Commulink();
	//初始化屏幕
	init_drv_LCD();
	//初始化GUI界面
	init_GUI();
	init_drv_Key();
	Adc_Init();
	init_drv_Uart1(9600);
	drv_Encoder_Init_TIM3();
	drv_Encoder_Init_TIM4();
	init_drv_PWMOut();
	init_drv_Motor();
	TIM2_Int_Init(49,7199);//72mhz     0.005s
	init_drv_LED();
	//初始化屏幕
	init_drv_LCD();
	//初始化GUI界面
	
	//初始化按键
	init_drv_Key();
	//初始化定时器
	//初始化超声波测距
	hcsr04Init();
}
	//初始化完成


	//进入主循环
/*
	data_array[0]=Balance_Kp;
	data_array[1]=Balance_Kd;
	data_array[2]=Velocity_Kp;
	data_array[3]=Middle_angle*100+1000;
*/
	
	
	float length;
	
//while 1执行频率为0.1s
	while(1)
	{	/*
		Balance_Kp =data_array[0];
		Balance_Kd =data_array[1];
		Velocity_Kp =data_array[2];
		Middle_angle=(data_array[3]-1000)/100.0;
		Velocity_Ki =Velocity_Kp/200.0;
		*/

		VDDA =Get_Adc_Average(12,5);
		VDDA = VDDA *3.3*11*1.1/4096;
		sprintf( str, "%3.1f", 0);
			LCD_ShowString(40,0,str,BLUE,WHITE,16,0);
			//pitch
			sprintf( str, "%3.1f", 0);
			LCD_ShowString(40,16,str,BLUE,WHITE,16,0);
			//yaw
			sprintf( str, "%3.1f",0);
			LCD_ShowString(40,32,str,BLUE,WHITE,16,0);
		
		/*获取姿态*/
			
		sprintf( str, "%4d",L_code);	
			LCD_ShowString(48,48,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%4d",-R_code);	
			LCD_ShowString(48,64,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%6d",L_count);	
		LCD_ShowString(48,80,str,BLUE,WHITE,16,0);
		
		
		sprintf( str, "%6d",R_count);	
		LCD_ShowString(48,96,str,BLUE,WHITE,16,0);
		sprintf( str, "%6f",0);	
		LCD_ShowString(0,144,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%6d",0);	
		LCD_ShowString(0,160,str,BLUE,WHITE,16,0);	

		sprintf( str, "%6d",0);	
		LCD_ShowString(120,160,str,BLUE,WHITE,16,0);	
		
	sprintf( str, "%6d",0);	
		LCD_ShowString(0,176,str,BLUE,WHITE,16,0);	

		sprintf( str, "%6.2f",0);	
		LCD_ShowString(120,176,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%6d",L_PWM);	
		LCD_ShowString(48,112,str,BLUE,WHITE,16,0);

		sprintf( str, "%6d",R_PWM);	
		LCD_ShowString(48,128,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%2.2f",VDDA);	
		LCD_ShowString(160,0,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%d",uart_receive);	
		LCD_ShowString(0,208,str,BLUE,WHITE,16,0);
		
		LCD_ShowString(0,208,rec,BLUE,WHITE,16,0);

		length=UltraSonic_valuetance();
		sprintf( str, "%4.2f",length );
		LCD_ShowString(0,224,str,BLUE,WHITE,16,0);
	}
}




extern "C" void TIM2_IRQHandler(void) 
{
	//检查 TIM2更新 中断发生与否
	int index;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		//清除 TIM2更新 中断 标志
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); 			
		
		R_PWMset(R_PWM);
		L_PWMset(L_PWM);
	
		L_code =Read_Encoder(4);
		L_count+=L_code;
		R_code =Read_Encoder(3);
		R_count +=R_code;
		
	}
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//时钟 TIM2 使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	//设置自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//设置时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//设置时钟分割
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//TIM向上计数	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//初始化 TIM2
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	//允许更新中断
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 
	//中断 优先级 NVIC 设置
	//TIM2 中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	//先占优先级 0 级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	//从优先级 3 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	//IRQ 通道被使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//初始化 NVIC 寄存器
	NVIC_Init(&NVIC_InitStructure); 
	//使能 TIM 2
	TIM_Cmd(TIM2, ENABLE); 
}


