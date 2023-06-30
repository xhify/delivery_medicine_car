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
extern"C"{
	#include "adc.h"
#include "MPU6050.h"
#include "mpuiic.h"
}

RCC_ClocksTypeDef RCC_CLK;
//决定程序正在运行的模式，也即题目
int mode=0;
int is_leader=0;//是否为领头小车
//蓝牙通信传递的参数
extern int openmv_data_array[5];
extern int data_array[9] ;
extern char rec[80];
float VDDA;//电池电压
extern int L_code;
extern int R_code;
extern float L_speed, R_speed;
int L_count=0;
int R_count=0;
extern int L_PWM ,R_PWM;
char str[50];
extern struct PID position_PID,speed_PID,distance_PID;
extern int pwml,pwmr;
int L_angle,R_angle;
extern float length;
float distance_speed=0;
int enter_cross=0;
int cross_cnt=0;//岔路口检测3.
int i=0;
int circle=0;//统计圈数从岔路口出来即为完成一圈。
int flag_stop=0;
int flag_wait=0;
int wait_time=0;
int stop_time=0;
int start_time =0;
int angle=0;
int flag_turn=0;
extern int blue_flag_stop , blue_flag_wait;//蓝牙传递的停车标志

extern float G_X,G_Z,G_Y;

float angle_z=0;
//函数原型
float targer_speed = 0.1;
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
	MPU_Init();	
	while(mpu_dmp_init());
		
	//Play();

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
	//初始化串口2
	init_drv_Uart3(9600);
	
}
	//初始化完成


	//进入主循环


//while 1执行频率为0.1s
	while(1)
	{	
		position_PID.kp =0;
		position_PID.ki =data_array[1]/10000.0;
		position_PID.kd =-data_array[2]/100.0;
		targer_speed  =data_array[3]/100.0;
		
		distance_PID.kp =0.012;
		distance_PID.kd =-data_array[5]/10000.0;
		speed_PID.kp =2000;
		speed_PID.ki =data_array[7]/100.0;
		speed_PID.kd =-data_array[8]/1.0;
		
		
		
		VDDA =Get_Adc_Average(12,5);
		VDDA = VDDA *3.3*11*1.1/4096;
		sprintf( str, "%d", mode);
			LCD_ShowString(40,0,str,BLUE,WHITE,16,0);
			//pitch
			sprintf( str, "%d", is_leader);
			LCD_ShowString(80,16,str,BLUE,WHITE,16,0);
			//yaw
			sprintf( str, "%3.1f",targer_speed);
			LCD_ShowString(48,32,str,BLUE,WHITE,16,0);
		
		/*获取姿态*/
			
		sprintf( str, "%4d",L_code);	
			LCD_ShowString(48,48,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%4d",R_code);	
			LCD_ShowString(48,64,str,BLUE,WHITE,16,0);
		
		
		
		sprintf( str, "%4.4f",L_speed);	
		LCD_ShowString(120,48,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%4.4f",R_speed);	
		LCD_ShowString(120,64,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%3d",L_angle);	
		LCD_ShowString(64,80,str,BLUE,WHITE,16,0);
		
		
		sprintf( str, "%3d",R_angle);	
		LCD_ShowString(64,96,str,BLUE,WHITE,16,0);
			
		
		
		sprintf( str, "%6d",L_PWM);	
		LCD_ShowString(48,112,str,BLUE,WHITE,16,0);

		sprintf( str, "%6d",R_PWM);	
		LCD_ShowString(48,128,str,BLUE,WHITE,16,0);	
		
		//循迹控制pid参数
		sprintf( str, "%4.2f",position_PID.kp);	
		LCD_ShowString(0,144,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",position_PID.ki);	
		LCD_ShowString(80,144,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",position_PID.kd);	
		LCD_ShowString(160,144,str,BLUE,WHITE,16,0);
		//速度控制pid参数
		sprintf( str, "%4.2f",speed_PID.kp);	
		LCD_ShowString(0,160,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",speed_PID.ki);	
		LCD_ShowString(80,160,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",speed_PID.kd);	
		LCD_ShowString(160,160,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",distance_PID.kp);	
		LCD_ShowString(0,176,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",distance_PID.ki);	
		LCD_ShowString(80,176,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",distance_PID.kd);	
		LCD_ShowString(160,176,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%4.2f",angle_z);	
		LCD_ShowString(160,192,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%4.2f",distance_speed);	
		LCD_ShowString(80,192,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%4.2f",G_Z);	
		LCD_ShowString(0,192,str,BLUE,WHITE,16,0);
		
		sprintf( str, "%2.2f",VDDA);	
		LCD_ShowString(160,0,str,BLUE,WHITE,16,0);	
		
		
		LCD_ShowString(0,208,rec,BLUE,WHITE,16,0);
		
	
		
		sprintf( str, "%4.2f",length);
		LCD_ShowString(0,224,str,BLUE,WHITE,16,0);
		//printf("hello world");
		
	}
}




extern "C" void TIM2_IRQHandler(void) 
{
	//频率为0.005s,200hz
	//检查 TIM2更新 中断发生与否
	int index;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		//清除 TIM2更新 中断 标志
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); 	
		//if(i >= 10)
		Get_Angle(2);
		UltraSonic_valuetance();
		angle_z+=(G_Z+0.97)*0.005;//xiao1			
		i++;
		L_angle=openmv_data_array[0];
		R_angle=openmv_data_array[1];
		int L_bias=L_angle-90;
		int R_bias=R_angle-90;
		angle =angle_z;
		
		
		//总控制部分
		
		if(is_leader ==1 )
		{
			if(start_time <1000)
				start_time ++;
			else if(start_time ==1000)
			{
				switch (mode)
				{
				case 0:printf("b");break;						
				case 1:printf("c");break;
				case 2:printf("d");break;						
				case 3:printf("e");break;
				}	
				start_time++;
			
			
			}
			else
			{
				flag_stop = openmv_data_array[3];//3表示停止线位于远，2表示停止线位于中，1表示停止线位于近，0则表示无停止线。
				flag_wait = openmv_data_array[4];//1表示检测到等待停止线
				//领头小车 模式1 
				//沿外圈走一圈停车
				if( mode == 0)
				{
					targer_speed = 0.3;
							
					if( (flag_stop == 1 && angle_z >=340)|| stop_time < 0)
						{
						TraceMove(0,0);//停车
						stop_time--;
					}			
					else 
						TraceMove(R_bias,targer_speed);
						
				}
				//领头小车 模式2
				//沿外圈走两圈到 A点停止，即停止线停止
				if(mode == 1)
				{
					targer_speed = 0.5;
							
					if( (flag_stop == 1 && angle_z >=700)|| stop_time < 0)
						{
						TraceMove(0,0);//停车
						stop_time--;
					}			
					else 
						TraceMove(R_bias,targer_speed);
						
				}
				
				//领头小车 模式3
				//外圈-外圈-内圈，速度大于0.3即可，A点停止，即停止线停止
				
				
				if( mode == 2)
				{
					targer_speed = 0.5;
							
					if( (flag_stop == 1 && angle_z >=1000)|| stop_time < 0)
						{
						TraceMove(0,0);//停车
						stop_time--;
					}			
					else if(angle_z >= 520 && angle_z <= 700 )
					{
						distance_speed= DistancePID(length);
						TraceMove(R_bias,targer_speed+distance_speed);
					}	 
					else if( angle_z >=700)
					{
						
						TraceMove(L_bias,targer_speed);			
					}
					else 
						TraceMove(R_bias,targer_speed);
						
				}
				
				//领头小车 模式4
				//沿外圈走一圈 ，E点等五秒,速度为1
				if(mode == 3)
				{
					//targer_speed = 1;
							
					if( (flag_stop == 1 && angle_z >=700)|| stop_time < 0)
						{
						TraceMove(0,0);//停车
						stop_time--;
					}			
					
					else if ( flag_wait ==1 && wait_time <=1000)
					{
						TraceMove(0,0);//停车
						wait_time ++;
					}		
					else 
						TraceMove(R_bias,targer_speed);				
				}

				
				
			}
			
		}
		
		
		if(is_leader ==0)
		{
			if(openmv_data_array[3] >0 && blue_flag_stop > 0)
			{
				if(openmv_data_array[3] < blue_flag_stop)
					flag_stop =openmv_data_array[3];
				else
					flag_stop =blue_flag_stop;
			}
			else
				flag_stop = openmv_data_array[3] + blue_flag_stop;
			
			flag_wait = openmv_data_array[4] || blue_flag_wait ;//1表示检测到等待停止线
			
			
						
			
			//追随小车 模式1
			
			if(mode == 0)
			{
				targer_speed = 0.3;
						
				if( (flag_stop == 1 && angle_z >=340)|| stop_time < 0)
					{
					TraceMove(0,0);//停车
					stop_time--;
				}			
				else 
				{
					distance_speed= DistancePID(length);
					TraceMove(R_bias,targer_speed+distance_speed);				
				}
			}
			//追随小车 模式2
			//从E点出发追领头小车
			
			if(is_leader == 0 && mode == 1)
			{
				//targer_speed = 0.5;
						
				if( (flag_stop == 1 && angle_z >=700)|| stop_time < 0)
					{
					TraceMove(0,0);//停车
					stop_time--;
				}							
				else 
				{
					distance_speed= DistancePID(length);
					TraceMove(R_bias,targer_speed+distance_speed);
				}	
			}
			
			//追随小车 模式3
			//外圈-内圈-内圈，速度大于0.3即可，A点停止，即停止线停止
			
			
			if(is_leader == 0 && mode == 2)
			{
				//targer_speed = 0.5;
				
				if( (flag_stop == 1 && angle_z >=1000)|| stop_time < 0)
					{
					TraceMove(0,0);//停车
					stop_time--;
				}			
				else if( angle_z >= 340  && angle_z <=700)
					TraceMove(L_bias,targer_speed);
				else 
				{
					distance_speed= DistancePID(length);
					TraceMove(R_bias,targer_speed+distance_speed);
				}	
			}
			
			//领头小车 模式4
			//沿外圈走一圈 ，E点等五秒,速度为1
			if(is_leader == 0 && mode == 3)
			{
				//targer_speed = 1;
						
				if( (flag_stop == 1 && angle_z >=1000)|| stop_time < 0)
					{
					TraceMove(0,0);//停车
					stop_time--;
				}			
				
				else if ( flag_wait ==1 && wait_time <=1000)
				{
					TraceMove(0,0);//停车
					wait_time ++;
				}		
				else 
				{
					distance_speed= DistancePID(length);
					TraceMove(R_bias,targer_speed+distance_speed);
				}				
			}

		
		}
		
		
		
		
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


