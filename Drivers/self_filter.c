#include "self_filter.h"
#include <math.h>

	

#include <math.h>



void kalman_filter_init(KalmanFilter *kf, float init_angle, float Q_angle, float Q_bias, float R_measure) {
    kf->angle = init_angle;
    kf->bias = 0.0;
    kf->P[0][0] = 1.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 1.0;
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
}

float kalman_filter_update(KalmanFilter *kf, float gyro_rate, float accel_angle,	float dt) {
    // 1. 预测状态

    kf->angle += dt * (gyro_rate - kf->bias);

    // 2. 预测误差协方差矩阵
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 3. 计算卡尔曼增益22
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 4. 更新状态（后验估计）
    float y = accel_angle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // 5. 更新误差协方差矩阵
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;

	}

void custom_low_pass_filter_init(CustomLowPassFilter *lpf, float alpha, float R) {
    lpf->last_value = 0;
    lpf->last_estimate = 0;
    lpf->alpha = alpha;
    lpf->R = R;
}


float custom_low_pass_filter_update(CustomLowPassFilter *lpf, float measurement) {
    // 先验估计
    float prior_estimate = lpf->last_estimate;

    // 后验估计
    float K = lpf->R / (lpf->R + lpf->alpha * lpf->alpha);
    float posterior_estimate = prior_estimate + K * (measurement - prior_estimate);

    // 更新滤波器状态
    lpf->last_estimate = posterior_estimate;

    return posterior_estimate;
}