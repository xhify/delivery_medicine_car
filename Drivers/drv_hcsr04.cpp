#include "drv_hcsr04.h"
#include "sys.h"
extern char value[];    //存储转换后的值
extern int Length;
int Val=0;
int countnum=0;
void hcsr04Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
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
	int time=0;
		while(i!=8){		
		GPIO_SetBits(HCSR04_PORT, HCSR04_TRIG);   //拉高电平信号
		delay_us(20);   //高电平至少10uS
		GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG); //拉低电平信号    
		//等待回响信号
			delay_us(10);
	while(GPIO_ReadInputDataBit(HCSR04_PORT,HCSR04_ECHO)==0);  //接收到信号ECHO为高电平
		TIM_Cmd(TIM5,ENABLE);        //使能TIM5定时器
			i++;
	while(GPIO_ReadInputDataBit(HCSR04_PORT,HCSR04_ECHO)==1);//直到回响信号消失
		TIM_Cmd(TIM5,DISABLE);       //失能TIM5定时器
	time=int(TIM_GetCounter(TIM5)+countnum*65535);
	
	lengths=(time)*17/1000;
		if(lengths<=0)
		{
			lengths=0;
		}
		sum+=lengths;
		countnum=0;
		TIM_SetCounter(TIM5,0);  //取出TIM5的counter寄存器里的值
	}
  lengths=sum/8.0;
	
		return lengths;
}
	
extern "C" void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		countnum++;
	}
}
/*
//定时器 5 通道 1 输入捕获配置
TIM_ICInitTypeDef TIM5_ICInitStructure;
void TIM5_Cap_Init(u16 arr,u16 psc)
{
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //①使能 TIM5 时钟
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //①使能 GPIOA 时钟
	//初始化 GPIOA.0 ①
	GPIO_InitStructure.GPIO_Pin=HCSR04_TRIG;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;  		//推挽输出
	GPIO_Init(HCSR04_PORT,&GPIO_InitStructure);
	GPIO_ResetBits(HCSR04_PORT, HCSR04_TRIG);
	
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //PA0 设置 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入 
GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化 GPIOA.0
GPIO_ResetBits(GPIOC,GPIO_Pin_7); //PA0 下拉
//②初始化 TIM5 参数
TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值
TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频器 
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // TDTS = Tck_tim
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //初始化 TIMx
//③初始化 TIM5 输入捕获通道 1
TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //选择输入端 IC1 映射到 TI1 上
TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //上升沿捕获
TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到 TI1 上
TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //配置输入分频,不分频
TIM5_ICInitStructure.TIM_ICFilter = 0x00; //IC1F=0000 配置输入滤波器 不滤波
TIM_ICInit(TIM5, &TIM5_ICInitStructure); //初始化 TIM5 输入捕获通道 1
//⑤初始化 NVIC 中断优先级分组
NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; //TIM3 中断
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级 2 级
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级 0 级
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
NVIC_Init(&NVIC_InitStructure); //初始化 NVIC
TIM_ITConfig( TIM5,TIM_IT_Update|TIM_IT_CC1,ENABLE);//④允许更新中断捕获中断
TIM_Cmd(TIM5,ENABLE ); //⑥使能定时器 5
}
u8 TIM5CH1_CAPTURE_STA=0; //输入捕获状态 
u16 TIM5CH1_CAPTURE_VAL;//输入捕获值
//⑤定时器 5 中断服务程序
extern "C" void TIM5_IRQHandler(void)
{ 
if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
{ 
if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
{ 
if(TIM5CH1_CAPTURE_STA&0X40) //已经捕获到高电平了
{if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
{
TIM5CH1_CAPTURE_STA|=0X80; //标记成功捕获了一次
TIM5CH1_CAPTURE_VAL=0XFFFF;
}else TIM5CH1_CAPTURE_STA++;
}
}
if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) //捕获 1 发生捕获事件
{
if(TIM5CH1_CAPTURE_STA&0X40) //捕获到一个下降沿
{ 
TIM5CH1_CAPTURE_STA|=0X80; //标记成功捕获到一次上升沿
TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5);
 TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //设置为上升沿捕获
}else //还未开始,第一次捕获上升沿
{
TIM5CH1_CAPTURE_STA=0; //清空
TIM5CH1_CAPTURE_VAL=0;
TIM_SetCounter(TIM5,0);
TIM5CH1_CAPTURE_STA|=0X40; //标记捕获到了上升沿
 TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling); //设置为下降沿捕获
} 
} 
}
 TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
}
*/



