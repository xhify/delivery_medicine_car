#include "control.h"
#include "drv_Encoder.hpp"
#include "drv_hcsr04.h"

/// @brief ѭ����pid
struct PID follow_PID = {720, 0, 0};
/// @brief �ٶȵ�pid
struct PID speed_PID;
/// @brief ���Ƹ�������pid
struct PID distance_PID;
/// @brief ǰ��ָ�������pid
struct PID position_PID;
/// @brief ת���pid
struct PID turn_PID;
int L_code;
int R_code;
int L_PWM, R_PWM;
float L_speed, R_speed;
extern float length;
/**
 * @brief ת��һ���Ƕȵ�pid
 *
 * @param bias �Ƕ�ƫ��
 * @return int ����pwm
 */
int turnPID(float Bias)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Integral_bias += Bias;																			 // ���ƫ��Ļ���
	Pwm = follow_PID.kp * Bias + follow_PID.ki * Integral_bias + follow_PID.kd * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																				 // ������һ��ƫ��
	return Pwm;
}
/**
 * @brief ѭ����PID����
 *
 * @param deviation ƫ��
 * @return int ����pwm
 */
int followPID(float deviation)
{

	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = deviation;																				 // ����ƫ��
	Integral_bias += Bias;																			 // ���ƫ��Ļ���
	Pwm = follow_PID.kp * Bias + follow_PID.ki * Integral_bias + follow_PID.kd * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																				 // ������һ��ƫ��
	return Pwm;
}
/**
 * @brief ǰ��ָ�������pid
 *
 * @param bias ���
 * @return int ����pwm
 */
int positionPID(float bias)
{
	static float Integral_bias, Last_Bias, pwmKI = 0;
	int Pwm;
	// ����ƫ��
	Integral_bias += bias;																				   // ���ƫ��Ļ���
	Pwm = position_PID.kp * bias + position_PID.ki * Integral_bias + position_PID.kd * (bias - Last_Bias); // λ��ʽPID������
	Last_Bias = bias;
}

int L_SpeedPID(float deviation)
{

	int MAX_MOTOR_PWM = 7000;
	static float Bias, Integral_bias, Last_Bias, pwmKI = 0;
	int Pwm;
	Bias = deviation;																			  // ����ƫ��
	Integral_bias += Bias;																		  // ���ƫ��Ļ���
	Pwm = speed_PID.kp * Bias + speed_PID.ki * Integral_bias + speed_PID.kd * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																			  // ������һ��ƫ��
	return Pwm;
}
int R_SpeedPID(float deviation)
{

	int MAX_MOTOR_PWM = 7000;
	float Position_KP = speed_PID.kp, Position_KI = speed_PID.ki, Position_KD = speed_PID.kd;
	static float Bias, Pwm, Integral_bias, Last_Bias, pwmKI = 0;
	Bias = deviation;	   // ����ƫ��
	Integral_bias += Bias; // ���ƫ��Ļ���
	pwmKI = Position_KI * Integral_bias;
	if (pwmKI > MAX_MOTOR_PWM)
		Integral_bias = MAX_MOTOR_PWM / Position_KI;
	Pwm = Position_KP * Bias + pwmKI + Position_KD * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;													 // ������һ��ƫ��
	return Pwm;
}

/// @brief ����pid������ʵ�ָ���
/// @param length ������ģ���õľ���
/// @return �ٶ�
float DistancePID(float length)
{

	float bias, Integral_bias, last_bias;
	float LengthTarspeed;
	if (length < 40.0)
		bias = length - 25.0;
	else
		bias = 20.0;
	Integral_bias += bias; // ���ƫ��Ļ���
	LengthTarspeed = distance_PID.kp * bias;
	last_bias = bias;
	return LengthTarspeed;
}

/*@brief:����pid������ߵ����Ŀ���ٶ�
 * @param:
 *        [in]int EncodeSpdL: ��ǰ��������������ֵ
 *        [in]float TarSpdL:��ߵ��Ŀ���ٶ�,����ٶ�Խ1.19m/s
 // @return: ������ߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorL(int NowEncodeSpdL, float TarSpdL)
{
	int pwm = 0;
	int bias;
	int TarEncodeSpdL;
	// TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//����Ŀ���ٶ����Ŀ��������ٶ�
	TarEncodeSpdL = (int)(TarSpdL / 3.14 / 0.07 * 1560 * 0.005);

	bias = TarEncodeSpdL - NowEncodeSpdL;
	pwm = L_SpeedPID(bias);
	return pwm;
}

/*@brief:����pid�����ұߵ����Ŀ���ٶ�
 * @param:
 *        [in]int EncodeSpdL: ��ǰ�ҵ������������ֵ
 *        [in]float TarSpdL:�ұߵ��Ŀ���ٶ�,����ٶ�Խ1.27m/s
 * @return: �����ұߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorR(int NowEncodeSpdR, float TarSpdR)
{
	int pwm = 0;
	int bias;
	int TarEncodeSpdR;
	// TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100)+0.5f);//����Ŀ���ٶ����Ŀ��������ٶ�
	TarEncodeSpdR = (int)(TarSpdR / 3.14 / 0.07 * 1560 * 0.005);

	bias = TarEncodeSpdR - NowEncodeSpdR;
	pwm = R_SpeedPID(bias);
	return pwm;
}

/*@brief: ��С������ѭ��������
 *@param:
 *        [in]TraceDate: ѭ�������������ֵ
 *        [in]TarSpeed:ѭ����Ŀ���ٶ�
 *@return: ����Ŀ��㷵��1�����򷵻�0
 */
void TraceMove(int TraceDate, float TarSpeed)
{
	/// @brief ת���pid
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
