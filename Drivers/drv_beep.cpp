#include "drv_beep.h"



void Play(void)
{
	u16 i,e;
	//低7  1   2   3   4   5   6   7  高1 高2 高3 高4 高5 不发音
	uc16 tone[] = {247,262,294,330,349,392,440,294,523,587,659,698,784,1000};//音频数据表
	//玫瑰少年
	u8 music[]=
	{
		8,2,5,3,2,3,5,3,3,13,
		2,3,6,2,3,6,3,3,13,
		2,3,6,2,3,6,5,3,1,7,5,3,
		2,3,5,3,3,13,
		2,3,6,2,3,6,3,3,13,
		2,3,6,2,3,6,5,3,13,13,13
	};     
	u8 time[] = 
	{       
		3,3,4,4,2,2,2,1,4,2,
		1,1,2,1,1,2,2,4,2,
		1,1,4,1,1,1,1,4,4,4,4,4,
		2,2,2,2,4,2,
		1,1,2,1,1,2,2,4,2,
		1,1,2,1,1,2,2,4,4,4,4,4
		
	};     
	u32 yanshi;
	
	yanshi = 10;
	for(i=0;i<2;i++)
	{
		for(e=0;e<((u16)time[i])*tone[music[i]]/yanshi;e++)
		{
			//打开扬声器
			Sound((u32)tone[music[i]]);
		}      
	}
}
void Sound(u16 frq)
{   
    u32 n;
    BEEP = 0;
    if(frq != 1000) 
    {
        n = 500000/((u32)frq);
        BEEP = 0;
        delay_us(n);
        BEEP = 1;
        delay_us(n);
    }
    else
    {
    	delay_us(1000);
    }         
}
void BEEP_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
}