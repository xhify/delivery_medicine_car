#pragma once

#include "delay.h"
#include "stm32f10x_it.h" 

#define BEEP PAout(3)

void Sound(u16 frq);
void Play(void);
void BEEP_Init(void);