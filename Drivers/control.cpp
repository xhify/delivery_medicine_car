#include "control.h"
#include "drv_Encoder.hpp"
#include "drv_hcsr04.h"

/// @brief 循迹的pid
struct PID follow_PID = {720, 0, 0};
/// @brief 速度的pid
struct PID speed_PID;
/// @brief 控制跟随距离的pid
struct PID distance_PID;
/// @brief 前进指定距离的pid
struct PID position_PID;
/// @brief 转向的pid
struct PID turn_PID;
int L_code;
int R_code;
int L_PWM, R_PWM;
float L_speed, R_speed;
extern float length;
/**
 * @brief 转向一定角度的pid
 *
 * @param bias 角度偏差
 * @return int 返回pwm
 */
int turnPID(float Bias)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Integral_bias += Bias;																			 // 求出偏差的积分
	Pwm = follow_PID.kp * Bias + follow_PID.ki * Integral_bias + follow_PID.kd * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																				 // 保存上一次偏差
	return Pwm;
}
/**
 * @brief 循迹的PID控制
 *
 * @param deviation 偏差
 * @return int 返回pwm
 */
int followPID(float deviation)
{

	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = deviation;																				 // 计算偏差
	Integral_bias += Bias;																			 // 求出偏差的积分
	Pwm = follow_PID.kp * Bias + follow_PID.ki * Integral_bias + follow_PID.kd * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																				 // 保存上一次偏差
	return Pwm;
}
/**
 * @brief 前进指定距离的pid
 *
 * @param bias 误差
 * @return int 返回pwm
 */
int positionPID(float bias)
{
	static float Integral_bias, Last_Bias, pwmKI = 0;
	int Pwm;
	// 计算偏差
	Integral_bias += bias;																				   // 求出偏差的积分
	Pwm = position_PID.kp * bias + position_PID.ki * Integral_bias + position_PID.kd * (bias - Last_Bias); // 位置式PID控制器
	Last_Bias = bias;
}

int L_SpeedPID(float deviation)
{

	int MAX_MOTOR_PWM = 7000;
	static float Bias, Integral_bias, Last_Bias, pwmKI = 0;
	int Pwm;
	Bias = deviation;																			  // 计算偏差
	Integral_bias += Bias;																		  // 求出偏差的积分
	Pwm = speed_PID.kp * Bias + speed_PID.ki * Integral_bias + speed_PID.kd * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																			  // 保存上一次偏差
	return Pwm;
}
int R_SpeedPID(float deviation)
{

	int MAX_MOTOR_PWM = 7000;
	float Position_KP = speed_PID.kp, Position_KI = speed_PID.ki, Position_KD = speed_PID.kd;
	static float Bias, Pwm, Integral_bias, Last_Bias, pwmKI = 0;
	Bias = deviation;	   // 计算偏差
	Integral_bias += Bias; // 求出偏差的积分
	pwmKI = Position_KI * Integral_bias;
	if (pwmKI > MAX_MOTOR_PWM)
		Integral_bias = MAX_MOTOR_PWM / Position_KI;
	Pwm = Position_KP * Bias + pwmKI + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;													 // 保存上一次偏差
	return Pwm;
}

/// @brief 距离pid，用来实现跟随
/// @param length 超声波模块测得的距离
/// @return 速度
float DistancePID(float length)
{

	float bias, Integral_bias, last_bias;
	float LengthTarspeed;
	if (length < 40.0)
		bias = length - 25.0;
	else
		bias = 20.0;
	Integral_bias += bias; // 求出偏差的积分
	LengthTarspeed = distance_PID.kp * bias;
	last_bias = bias;
	return LengthTarspeed;
}

/*@brief:根据pid调节左边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前左电机编码器测速值
 *        [in]float TarSpdL:左边电机目标速度,最大速度越1.19m/s
 // @return: 返回左边电机计算后的pwm占空比
 */
int ChangeSpeedMotorL(int NowEncodeSpdL, float TarSpdL)
{
	int pwm = 0;
	int bias;
	int TarEncodeSpdL;
	// TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//根据目标速度求出目标编码器速度
	TarEncodeSpdL = (int)(TarSpdL / 3.14 / 0.07 * 1560 * 0.005);

	bias = TarEncodeSpdL - NowEncodeSpdL;
	pwm = L_SpeedPID(bias);
	return pwm;
}

/*@brief:根据pid调节右边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前右电机编码器测速值
 *        [in]float TarSpdL:右边电机目标速度,最大速度越1.27m/s
 * @return: 返回右边电机计算后的pwm占空比
 */
int ChangeSpeedMotorR(int NowEncodeSpdR, float TarSpdR)
{
	int pwm = 0;
	int bias;
	int TarEncodeSpdR;
	// TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//根据目标速度求出目标编码器速度
	TarEncodeSpdR = (int)(TarSpdR / 3.14 / 0.07 * 1560 * 0.005);

	bias = TarEncodeSpdR - NowEncodeSpdR;
	pwm = R_SpeedPID(bias);
	return pwm;
}

/*@brief: 让小车根据循迹黑线走
 *@param:
 *        [in]TraceDate: 循迹传感器输出的值
 *        [in]TarSpeed:循迹的目标速度
 *@return: 到达目标点返回1，否则返回0
 */
void TraceMove(int TraceDate, float TarSpeed)
{
	/// @brief 转向的pid
	int turn_pwm = 0;
	
	int trace_pwm=0;
	static int spdpwml = 0, spdpwmr = 0;
	float distancespd = 0;

	turn_pwm = followPID(TraceDate);

	L_code = Read_Encoder(4);
	R_code = -Read_Encoder(3);
	R_speed = (float)(R_code * 200.0 / 1560.0 * 0.07 * 3.14);
	L_speed = L_code * 200 / 1560.0 * 0.07 * 3.14;

	spdpwml = ChangeSpeedMotorL(L_code, TarSpeed);
	spdpwmr = ChangeSpeedMotorR(R_code, TarSpeed);

	R_PWM = turn_pwm + spdpwmr;
	L_PWM = -turn_pwm + spdpwml;


	R_PWMset(R_PWM);
	L_PWMset(L_PWM);
}
