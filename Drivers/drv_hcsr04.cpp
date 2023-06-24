#include "drv_hcsr04.h"
#include "sys.h"
extern char value[];    //存储转换后的值
extern int Length;
int Val=0;
int countnum=0;
int time=0;
float lengths=0;
void hcsr04Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(HCSR04_CLK,ENABLE);
	RCC_APB1PeriphClockCmd(HCSR04_TIM_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=HCSR04_TRIG;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  		//推挽输出
	GPIO_Init(HCSR04_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG);    
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;	//浮空输入
	GPIO_InitStructure.GPIO_Pin=HCSR04_ECHO;
	GPIO_Init(HCSR04_PORT, &GPIO_InitStructure);  
	
	
//GPIOB.9 中断线以及中断初始化配置,下降沿触发
 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);//③
 EXTI_InitStructure.EXTI_Line=EXTI_Line8;
 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上降沿触发
 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 EXTI_Init(&EXTI_InitStructure); //④初始化中断线参数
 
 NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //使能按键外部中断通道
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级 2，
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03; //子优先级 2
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
NVIC_Init(&NVIC_InitStructure);//⑤初始化 NVIC
	
	
	
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructure.TIM_Period = 65535; 
	TIM_TimeBaseStructure.TIM_Prescaler =71; 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //初始化 

	TIM_ClearFlag(TIM5, TIM_FLAG_Update);   //清除更新中断，免得一打开中断立即产生中断
 	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);    //打开定时器更新中断
	TIM_Cmd(TIM5,DISABLE);
}


float UltraSonic_valuetance(void)   //测量超声波距离
{
float lengths=0,sum=0;
		int i=0;
	
				
		GPIO_SetBits(HCSR04_PORT, HCSR04_TRIG);   //拉高电平信号
		delay_us(20);   //高电平至少10uS
		GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG); //拉低电平信号    
	
	
	
	
	
		
	
  
	
		return 0;
}
	
extern "C" void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		countnum++;
	}
}

extern "C" void EXTI9_5_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)//判断某个线上的中断是否发生 
{
		//delay_ms(1);
		if(GPIO_ReadInputDataBit(HCSR04_PORT,HCSR04_ECHO)==1)
			TIM_Cmd(TIM5,ENABLE); 
		else
		{
				TIM_Cmd(TIM5,DISABLE); 
				time=int(TIM_GetCounter(TIM5)+countnum*65535);
				lengths=(time)*17/1000.0;
			if(lengths<=0)
			{
				lengths=0;
			}
			TIM_SetCounter(TIM5,0);  //取出TIM5的counter寄存器里的值
		}
			
			
EXTI_ClearITPendingBit(EXTI_Line8); //清除 LINE12 上的中断标志位  
}
}


