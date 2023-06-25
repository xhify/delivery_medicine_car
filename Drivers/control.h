
#include "drv_Encoder.hpp"
#include "drv_PWMOut.hpp"
#include "stm32f10x_gpio.h"
#include "Basic.hpp"
#include "drv_PwmOut.hpp"

struct PID
{
	float kp;
	float ki;
	float kd;
};
void TraceMove(int TraceDate,float TarSpeed);
float DistancePID(float length);