#ifndef __KALMAN_FORWARD_v_H__
#define __KALMAN_FORWARD_v_H__

#include "drv_utils.h"
typedef struct
{
    float v_input;  // 当前测量的速度（传感器读数）
    float a_input;
    float last_a_input;
    float P_covariance;
    float Kalman_gain;
    float v_prior;  // 预测的速度（根据模型计算得到的先验估计）
    float Q;        // 过程噪声（模型不准确度）
    float R;        // 观测噪声（传感器不准确度）
    float v_observe;    // 最终观测的速度（融合结果）
    float period;
    char Inited;
} Kalman_Forward_v_one_dimension;

typedef struct
{
    float input[2];     // 0号位是高度,1号位是速度
    float a_input;
    float last_a_input;
    float P_covariance[2][2];   // 协方差矩阵
    float Kalman_gain[2][2];
    matrix Kalman_gain_m;
    float State_prior[2];   // 预测状态：[预测高度, 预测速度]  
    float Q[2][2];
    float R[2][2];
    float State_observe[2];
    float period;
    char Inited;

    // 中间变量
    float temp_Kt_dividend[2][2]; // 计算卡尔曼增益的被除矩阵
    matrix temp_Kt_dividend_m;
    float temp_Kt_divide[2][2];
    matrix temp_Kt_divide_m;
    float temp_inverse[2][2];
    matrix temp_inverse_m;
} Kalman_Height_t;

typedef struct 
{
    //协方差
    matrix P_m;
    union
    {    struct
        {
            float P1;
            float P_temp1;
            float P_temp2;
            float P2;
        };
        float P[2][2];
    };
    //卡尔曼增益
    matrix K_m;
    float K[2][2];
    //过程噪声
    float Q1;
    float Q2;
    //观测噪声
    float R1;
    float R2;
    //当前输入的加速度，上次的加速度
    float a_input;
    float last_a_input;
    //状态变量
    matrix X_m;
    union
    {
        struct
        {
            float v_total_state;
            float w_state;
        };
        float X_state_matix[2];
    };
    //观测变量
    matrix Z_m;
    union
    {
        struct
        {
            float v_left_ob;
            float v_right_ob;
        };
        float Z_ob_matix[2];
    };
    //车身半径
    matrix H_m;
    union
    {
        struct
        {
            float constant_1;
            float r_positive;
            float constant_2;
            float r_negetive;
        };
        float H[2][2];
    };
    matrix H_T_m;
    float H_T[2][2];
    //初始化标志位
    char Inited;
    float period;
    //临时变量
    matrix K_up_m;
    float K_up[2][2];
    matrix K_down_m;
    float K_down[2][2];
    matrix temp_m;
    float temp[2];
    matrix temp1_m;
    float temp1[2];

}Kaliman_v_w_t;

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

// 一位卡尔曼滤波器计算，状态观测量只有一个速度
extern float Kalman_calculate(Kalman_Forward_v_one_dimension *Kalman, float v_now, float a_now);

// 初始化二阶卡尔曼
extern void Kalman_height_init(Kalman_Height_t *Kalman, float preiod, float Q1, float Q2, float R1, float R2);



// 二阶卡尔曼滤波器初始化
extern void Kalman_height_calculate(Kalman_Height_t *Kalman, float height, float height_v, float height_a);

// 融合角速度和速度卡尔曼滤波器初始化
extern void Kalman_v_w_init(Kaliman_v_w_t *Kalman, float period, float Q1, float Q2, float R, float r);



#endif
