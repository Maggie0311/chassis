#ifndef __KALMAN_FORWARD_v_H__
#define __KALMAN_FORWARD_v_H__

#include "drv_utils.h"

typedef struct {
    // 状态向量: [位移, 速度]
    float x[2];
    
    // 协方差矩阵
    float P[2][2];
    
    // 系统矩阵
    float F[2][2];  // 状态转移矩阵
    float B[2];     // 控制输入矩阵
    
    // 观测矩阵 (只能观测速度)
    float H[2];     // [0, 1] - 只能观测速度
    
    // 噪声参数
    float Q[2][2];  // 过程噪声协方差
    float R;        // 观测噪声方差
    
    // 时间参数
    float dt;
    
    // 初始化标志
    int initialized;
    float acc_noise;
    float vel_noise;

}Robotmotion_Kalman_t;

void RobotMotion_Kalman_Init(Robotmotion_Kalman_t *kf, float dt, 
                      float accel_noise_std, float velocity_noise_std);

void RobotMotionKalman_Predict(Robotmotion_Kalman_t *kf, float acceleration);
void RobotMotionKalman_Update(Robotmotion_Kalman_t *kf, float measured_velocity);
void RobotMotionKalman_Process(Robotmotion_Kalman_t *kf, float measured_velocity, float acceleration);
float RobotMotionKalman_Getdisplacement(Robotmotion_Kalman_t *kf);
float RobotMotionKalman_Getvelocity(Robotmotion_Kalman_t *kf);


#endif
