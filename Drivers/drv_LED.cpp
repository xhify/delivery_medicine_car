#include "stm32f10x.h"
#include "drv_LED.hpp"
#include "Basic.hpp"
#include "delay.h"
#include "GUI.hpp"
#include "GUI_Images.hpp" 
#define APB1TIMERCLK 36000000
extern int picture_id;
float ExtLedR = 0;
float ExtLedG = 0;
float ExtLedB = 0;


//亮度线性补偿函数
static inline float led_linear_compensation( float in )
{
	if( in <= 8 )
		return in / 903.3f * 100;
	else
		return 0;
}

/*
	LED调亮度函数
	R、G、B：亮度百分比（0-100）
*/
void set_LedBrightness( float R , float G , float B )
{
	if( B >= 0 && B <= 100 )
	{
		B = led_linear_compensation(B);
		ExtLedB = B;
		TIM5->CCR1 = B/100*TIM5->ARR;
	}
	if( G >= 0 && G <= 100 )
	{
		G = led_linear_compensation(G);
		ExtLedG = G;
		TIM5->CCR2 = G/100*TIM5->ARR;
	}
	if( R >= 0 && R <= 100 )
	{
		R = led_linear_compensation(R);
		ExtLedR = R;
		TIM5->CCR3 = R/100*TIM5->ARR;
	}
}

static LED_Mode led_status = normal;

/*
	蜂鸣器频率调节函数
	freq:蜂鸣器频率
*/
void set_BuzzerFreq( unsigned short freq )
{
	float B = (float)TIM5->CCR1/(float)TIM5->ARR;
	float G = (float)TIM5->CCR2/(float)TIM5->ARR;
	float R = (float)TIM5->CCR3/(float)TIM5->ARR;
	if( freq < 200 )
		freq = 200;
	TIM5->ARR = 10e6 / freq;
	if( TIM5->CCR4 != 0 )
		TIM5->CCR4 = TIM5->ARR / 2;
	TIM5->CCR1 = B*TIM5->ARR;
	TIM5->CCR2 = G*TIM5->ARR;
	TIM5->CCR3 = R*TIM5->ARR;
}

/*
	蜂鸣器鸣响函数
	on:是否鸣响
*/
void set_BuzzerOnOff( bool on )
{
	if(on)
		TIM5->CCR4 = TIM5->ARR / 2;
	else
		TIM5->CCR4 = 0;
}


//设置LED模式
void Set_LED_Mode(LED_Mode mode)
{
	led_status = mode;
}

//LED闪烁函数
void Led_Flash()
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
	GPIO_SetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
}

//TIM2 中断


//LED初始化函数
void init_drv_LED(void)
{
	/*
		D3_LED(GPIOE_P0)  PE0
		LED_B(TIM5_CH1)   PA0
		LED_G(TIM5_CH2)   PA1
		LED_R(TIM5_CH3)   PA2
		Buzzer(TIM5_CH4)  PA3
	*/
	//开启GPIOE、GPIOA外设时钟,开启复用功能时钟
	RCC->APB2ENR|=(1<<6)|(1<<2)|(1<<0);
	//D3初始化
	os_delay(1e-2);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	//使能定时器 5 时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);	   
	
	//配置引脚输出模式
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	    //使能PE端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			    	//D3-->PE.0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO口速度为50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);			     		//初始化GPIOE.0
	GPIO_SetBits(GPIOE,GPIO_Pin_0);											//PE.0 输出高
	
	//GPIO初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	    //使能PA端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;			    	
	GPIO_InitStructure.GPIO_Mode = GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;; 	 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     		//初始化GPIOA.0~3
	
	//定时器5初始化
	TIM_TimeBaseStructure.TIM_Period = 1000; 										//设置在自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler = 71; 										//设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 								//设置时钟分割 :TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 						//初始化 TIMx
	
	//初始化 TIM5 Channel PWM 模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
	TIM_OC1Init(TIM5, &TIM_OCInitStructure); //初始化外设 TIM5 OC1
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_OC2Init(TIM5, &TIM_OCInitStructure); //初始化外设 TIM5 OC2
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_OC3Init(TIM5, &TIM_OCInitStructure); //初始化外设 TIM5 OC3
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_OC4Init(TIM5, &TIM_OCInitStructure); //初始化外设 TIM5 OC4
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_Cmd(TIM5, ENABLE); //使能 TIM5
	
}