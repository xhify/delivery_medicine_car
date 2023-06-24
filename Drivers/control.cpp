#include "control.h"
#include "drv_Encoder.hpp"




struct PID position_PID,speed_PID;


int L_code;
int R_code;
int L_PWM ,R_PWM;
float L_speed, R_speed;

//位置式PID控制器
//用来控制转向
int PositionPID(float deviation)
{
	
	
	float Position_KP=position_PID.kp,Position_KI=position_PID.ki,Position_KD=position_PID.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                                 //求出偏差的积分
	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias); //位置式PID控制器
	Last_Bias=Bias;                                      	 //保存上一次偏差 
	return Pwm;    
}
int L_SpeedPID(float deviation)
{
	
	int MAX_MOTOR_PWM=7000;
	float Position_KP=speed_PID.kp,Position_KI=speed_PID.ki,Position_KD=speed_PID.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias,pwmKI=0;
	
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                                 //求出偏差的积分
	pwmKI=Position_KI*Integral_bias;
	if(pwmKI>MAX_MOTOR_PWM) Integral_bias=MAX_MOTOR_PWM/Position_KI;
	
	Pwm=Position_KP*Bias+pwmKI+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	Last_Bias=Bias;                                      	 //保存上一次偏差 
	return Pwm;    
}
int R_SpeedPID(float deviation)
{
	
	int MAX_MOTOR_PWM=7000;
	float Position_KP=speed_PID.kp,Position_KI=speed_PID.ki,Position_KD=speed_PID.kd;
	static float Bias,Pwm,Integral_bias,Last_Bias,pwmKI=0;
	Bias=deviation;                         		         //计算偏差
	Integral_bias+=Bias;	                                 //求出偏差的积分
	pwmKI=Position_KI*Integral_bias;
	if(pwmKI>MAX_MOTOR_PWM) Integral_bias=MAX_MOTOR_PWM/Position_KI;
	Pwm=Position_KP*Bias+pwmKI+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	Last_Bias=Bias;                                      	 //保存上一次偏差 
	return Pwm;    
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
	TarEncodeSpdL=(int)(TarSpdL/3.14/0.07*1560*0.005);
	
		
	bias=TarEncodeSpdL - NowEncodeSpdL;
	pwm=L_SpeedPID(bias);
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
	TarEncodeSpdR=(int)(TarSpdR/3.14/0.07*1560*0.005);

	bias=TarEncodeSpdR - NowEncodeSpdR;
	pwm=L_SpeedPID(bias);
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
	
	
	turnpwm=PositionPID(TraceDate);
	
	L_code=Read_Encoder(4);
	R_code= -Read_Encoder(3);
	R_speed =(float)(R_code*200.0/1560.0*0.07*3.14);
	L_speed = L_code*200/1560.0*0.07*3.14;
	spdpwml=ChangeSpeedMotorL(L_code,TarSpeed);
	spdpwmr=ChangeSpeedMotorR(R_code,TarSpeed);
	
	
	R_PWM=turnpwm+spdpwmr;
	L_PWM=-turnpwm+spdpwml;
	
	//R_PWM=2000;
	//L_PWM=2000;
	R_PWMset(R_PWM);
	L_PWMset(L_PWM);
}



