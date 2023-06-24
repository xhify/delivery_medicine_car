#ifndef __HCSR04_H
#define	__HCSR04_H
#include "Basic.hpp"
#include "stm32f10x.h"
#include "delay.h"

#define HCSR04_PORT    		 GPIOB
#define HCSR04_CLK    	   RCC_APB2Periph_GPIOB
#define HCSR04_TIM_CLK     RCC_APB1Periph_TIM5
#define HCSR04_TRIG   	   GPIO_Pin_8
#define HCSR04_ECHO        GPIO_Pin_9

void hcsr04Init(void);
float UltraSonic_valuetance(void);
//void TIM5_Cap_Init(u16 arr,u16 psc);
#endif /*__HCSR04_H */
