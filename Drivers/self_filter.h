

typedef struct {
    float angle;
    float bias;
    float P[2][2];
    float Q_angle; // 角度过程噪声协方差
    float Q_bias;  // 偏差过程噪声协方差
    float R_measure; // 测量噪声协方差
} KalmanFilter;

typedef struct {
    float last_value;
    float last_estimate;
    float alpha; // 时间常数
    float R; // 测量噪声协方差
} CustomLowPassFilter;

#ifdef __cplusplus
extern "C" {
#endif
void kalman_filter_init(KalmanFilter *kf, float init_angle, float Q_angle, float Q_bias, float R_measure);
float kalman_filter_update(KalmanFilter *kf, float gyro_rate, float accel_angle,float dt);
void custom_low_pass_filter_init(CustomLowPassFilter *lpf, float alpha, float R);
float custom_low_pass_filter_update(CustomLowPassFilter *lpf, float measurement);
	#ifdef __cplusplus
}
#endif