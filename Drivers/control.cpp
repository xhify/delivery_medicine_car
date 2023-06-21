#include "control.h"
#include "drv_Encoder.hpp"


struct PID
{
	float kp;
	float ki;
	float kd;
};

//默认向左转向 turn_speed为负数，向右转向turn_speed 为正数

int get_speed(){




	return 0;
}
//位置式PID控制器
//用来控制转向
int PositionPID(float deviation)
{
	PID pid;
	pid.kp=100;
	pid.ki=0;
	pid.kp=-30;
	float Position_KP=pid.kp,Position_KI=pid.ki,Position_KD=pid.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                                 //求出偏差的积分
	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias); //位置式PID控制器
	Last_Bias=Bias;                                      	 //保存上一次偏差 
	return Pwm;    
}
int PositionPIDToSpd(float deviation)
{
	PID pid;
	pid.kp=-50;pid.ki=-6;pid.kd=0;
	int MAX_MOTOR_PWM=7000;
	float Position_KP=pid.kp,Position_KI=pid.ki,Position_KD=pid.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias,pwmKI=0;
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                                 //求出偏差的积分
	pwmKI=Position_KI*Integral_bias;
	if(pwmKI>MAX_MOTOR_PWM) Integral_bias=MAX_MOTOR_PWM/Position_KI;
	Pwm=Position_KP*Bias+pwmKI+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	Last_Bias=Bias;                                      	 //保存上一次偏差 
	return Pwm;    
}

int ChangeTraceTurn(int TraceDate)
{
	int pwm=0;
	int bias;
	bias=TraceDate;
	pwm=PositionPID(bias);
	return pwm;
}

/*@brief:根据pid调节左边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前左电机编码器测速值
 *        [in]float TarSpdL:左边电机目标速度,最大速度越1.19m/s
 * @return: 返回左边电机计算后的pwm占空比
 */
int ChangeSpeedMotorL(int NowEncodeSpdL,float TarSpdL)
{
	int pwm=0;
	int bias;
	int TarEncodeSpdL;
	//TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//根据目标速度求出目标编码器速度
	bias=NowEncodeSpdL-TarEncodeSpdL;
	pwm=PositionPIDToSpd(bias);
	return pwm;
}

/*@brief:根据pid调节右边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前右电机编码器测速值
 *        [in]float TarSpdL:右边电机目标速度,最大速度越1.27m/s
 * @return: 返回右边电机计算后的pwm占空比
 */
int ChangeSpeedMotorR(int NowEncodeSpdR,float TarSpdR)
{
	int pwm=0;
	int bias;
	int TarEncodeSpdR;
	//TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//根据目标速度求出目标编码器速度
	bias=NowEncodeSpdR-TarEncodeSpdR;
	return pwm;
}

/*@brief: 让小车根据循迹黑线走
 *@param:
 *        [in]TraceDate: 循迹传感器输出的值
 *        [in]TarSpeed:循迹的目标速度
 *@return: 到达目标点返回1，否则返回0
 */
void TraceMove(int TraceDate,float TarSpeed)
{
	int turnpwm=0;
	int spdpwml=0,spdpwmr=0;
	int pwml=0,pwmr=0;
	
	turnpwm=ChangeTraceTurn(TraceDate);
	
	int Encode_Left=Read_Encoder(4);
	int Encode_Right= Read_Encoder(3);
	
	spdpwml=ChangeSpeedMotorL(Encode_Left,TarSpeed);
	spdpwmr=ChangeSpeedMotorR(Encode_Right,TarSpeed);
	
	
	pwmr=turnpwm+spdpwmr;
	
	pwml=-turnpwm+spdpwml;
	
	
	R_PWMset(pwmr);
	L_PWMset(pwml);
	

}



