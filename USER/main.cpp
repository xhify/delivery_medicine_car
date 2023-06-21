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
float VDDA;
float target_angle = 0; // 旋转目标角度
bool is_turning = false; // 是否正在旋转
	int i=0,j=0;///////400          15    140
float Middle_angle=6.5 ;
int 	Velocity_Kp=20000,Balance_Kp=34000,Balance_Kd  =2100;
float 	Velocity_Ki =Velocity_Kp/200;
float    Turn_Kp=0,Turn_Kd=100;
float   turn_PWM;
int   turn_angle=0;
	int L_code;
	int R_code;
	int L_count=0;
	int R_count=0;;
	int L_PWM=200;
	int R_PWM=200;
extern float G_X,G_Z,G_Y;
float angle_z=0;
	char str[50];
int PWM;
extern float Pitch,Roll,Yaw; 

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
	MPU_Init();	
	BEEP_Init();
	
	//关闭了mpu6050的初始化
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
	data_array[0]=Balance_Kp;
	data_array[1]=Balance_Kd;
	data_array[2]=Velocity_Kp;
	data_array[3]=Middle_angle*100+1000;

	
	
	float length;
	
//while 1执行频率为0.1s
	while(1)
	{	
		Balance_Kp =data_array[0];
		Balance_Kd =data_array[1];
		Velocity_Kp =data_array[2];
		Middle_angle=(data_array[3]-1000)/100.0;
		Velocity_Ki =Velocity_Kp/200.0;
		

		VDDA =Get_Adc_Average(12,5);
		VDDA = VDDA *3.3*11*1.1/4096;
		sprintf( str, "%3.1f", Roll);
			LCD_ShowString(40,0,str,BLUE,WHITE,16,0);
			//pitch
			sprintf( str, "%3.1f", Pitch);
			LCD_ShowString(40,16,str,BLUE,WHITE,16,0);
			//yaw
			sprintf( str, "%3.1f",Yaw);
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
		sprintf( str, "%6f",Middle_angle);	
		LCD_ShowString(0,144,str,BLUE,WHITE,16,0);	
		
		sprintf( str, "%6d",Balance_Kp);	
		LCD_ShowString(0,160,str,BLUE,WHITE,16,0);	

		sprintf( str, "%6d",Balance_Kd);	
		LCD_ShowString(120,160,str,BLUE,WHITE,16,0);	
		
	sprintf( str, "%6d",Velocity_Kp);	
		LCD_ShowString(0,176,str,BLUE,WHITE,16,0);	

		sprintf( str, "%6.2f",Velocity_Ki);	
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
		
		
		L_PWM =PWM-turn_PWM+turn_angle;
		R_PWM =PWM+turn_PWM-turn_angle;

	if(L_PWM <0)
		L_PWM -=200;
	else if(L_PWM >0)
		L_PWM +=200;
	
		if(R_PWM <0)
		R_PWM -=200;
	else if(R_PWM >0)
		R_PWM +=200;
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

int Balance(float Angle,float Gyro)
{
float Angle_bias,Gyro_bias;
	
int balance;// 185   5

Angle_bias=Middle_angle-Angle; //求出平衡的角度中值 和机械相关
Gyro_bias=0-Gyro;
balance=-Balance_Kp*Angle_bias/100.0-Gyro_bias*Balance_Kd/100.0; //计算平衡控制的电机 PWM PD 控制 kp 是 P 系数 kd 是 D 系数
	/*if(Angle_bias>35)
		balance=0;
	else if(Angle_bias<-35 )
		balance=0;
	*/

	
return balance;
}
int Velocity(int encoder_left,int encoder_right)
{
static float velocity,Encoder_Least,Encoder_bias,Movement,Target_Velocity;
	

static float Encoder_Integral;
	
							Target_Velocity=400;                 
		if(1==Flag_Qian)    	Movement=Target_Velocity/Flag_sudu;	         //===前进标志位置1 
		else if(1==Flag_Hou)	Movement=-Target_Velocity/Flag_sudu;         //===后退标志位置1
	  else  Movement=0;	
	
	  
	
		Flag_Qian=0;
		Flag_Hou=0;
	
//=============速度 PI 控制器=======================//
Encoder_Least =0-(encoder_left+encoder_right); //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）
Encoder_bias *= 0.8; //一阶低通滤波器
Encoder_bias+= Encoder_Least*0.2; //一阶低通滤波器
//相当于上次偏差的 0.8 + 本次偏差的 0.2，减缓速度差值，减少对直立的干扰
Encoder_Integral +=Encoder_bias; //积分出位移 积分时间：5ms
Encoder_Integral=Encoder_Integral+Movement;                       //===接收遥控器数据，控制前进后退
		
velocity=-Encoder_bias*Velocity_Kp/100.0-Encoder_Integral*Velocity_Ki/100.0;//速度控制
if(Pitch == Middle_angle) Encoder_Integral=0;
//电机关闭后清除积分
return velocity;
}

int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	int temp;
	temp =encoder_left-encoder_right;
	if(temp <0)
		temp =-temp;
	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			if(++Turn_Count==1)
			Encoder_temp=temp;
			Turn_Convert=50/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
	
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		Turn=-Turn_Target*Kp -gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;
}
int turn_90_degrees(float angle_z) {
  const float angle_tolerance = 2.0; // 旋转角度容差
  const int turn_pwm = 300; // 转向PWM值
	float end;
  if (1) {
		if(turn90left)
    target_angle = angle_z + 90.0; // 设置旋转目标角度为当前角度加90度
		
		if(turn90right)
    target_angle = angle_z - 90.0; 
		
		if(turn180left)
    target_angle = angle_z + 180.0; 
		
		if(turn180right)
    target_angle = angle_z - 180.0; 
		
		if(turn360left)
    target_angle = angle_z + 360.0; 
		
		if(turn360right)
    target_angle = angle_z - 360.0; 
    /*if (target_angle > 360.0) {
      target_angle -= 360.0;
    }
		else if (target_angle < -360.0) {
      target_angle += 360.0;
    }*/
    is_turning = true;
		turn90left=0;
		turn90right=0;
		turn180left=0;
		turn180right=0;
		turn360left=0;
		turn360right=0;
  }
		end=angle_z - target_angle;
	if(end <0)
		end=-end;
  if (end < angle_tolerance) {
    is_turning = false; // 如果已经到达目标角度，停止旋转
    return 0;
  } else {
    if (angle_z > target_angle) {
      return turn_pwm; // 返回正向PWM值以顺时针旋转
    } else {
      return -turn_pwm; // 返回负向PWM值以逆时针旋转
    }
  }
}
