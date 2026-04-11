#include "Kalman.h"
#include "rtthread.h"
#include "drv_IMU.h"
#include "robotdata.h"
#include "ex_motor.h"

/**
 * @brief 防打滑额外检测处理
 * 
 * @param Innovation 新息（观测减去预测值）
 * @note 通过改变观测噪声R来减少卡尔曼更新
 */
void Anti_Slip_Process(Robotmotion_Kalman_t *kf,float Innovation)
{
//   #define COMPLEENTARY_FACTOR 0.99f
     #define SLIP_THRESHOLD 0.5f   // 判断打滑阈值
     #define INORMAL_R      1000.f // 异常大的观测噪声R，间接减小K，避免打滑速度突变影响
//   
//   float accel_integral += Get_Chassis_IMU_Data(IMU_Y_ACCL) * CTRL_PERIOD  ;
//   float measured_velocity = (Hub_Read_NowSpeed(HUB_R) + Hub_Read_NowSpeed(HUB_L))/2 * HUB_RADIUS; // 轮毂半径乘以角速度得到线速度
//   float integrated_velocity = measured_velocity * (1-COMPLEENTARY_FACTOR) + accel_integral * COMPLEENTARY_FACTOR; // 互补滤波，加速度为主，编码器为辅缓解加速度计零漂

//   if(fabsf(integrated_velocity - measured_velocity) > 0.5f) // 速度差大于0.5m/s认为打滑 // TODO 后续可以加上rt_tick_get()计时器，短时内突变辅助判断打滑
//   {
//       
//   }
    if(fabsf(Innovation) > SLIP_THRESHOLD) // TODO 后续可以加上rt_tick_get()计时器，短时内突变辅助判断打滑
    {
        kf->R = INORMAL_R;
    }else{
        kf->R = kf->vel_noise; // 其他情况下变回原来正常的R
    }
}

/**
 * @brief 机体速度、位移卡尔曼滤波初始化
 * 
 * @param kf 
 * @param dt 控制周期
 * @param accel_noise_std 过程噪声Q越大，K变大，更加信新观测。 Q = 加速度测量标准差的平方。加速度波动大则Q大
 * @param velocity_noise_std R大（测量不靠谱）：K变小，更加信自己预测 。 R = 传感器标定得来的标准差平方。
 */
void RobotMotion_Kalman_Init(Robotmotion_Kalman_t *kf, float dt, 
                      float accel_noise_std, float velocity_noise_std) 
{
    kf->acc_noise = accel_noise_std;
    kf->vel_noise = velocity_noise_std; // 仅用于记录最开始的观测噪声
    kf->dt = dt;
    kf->initialized = 1;
    
    // 初始状态: 假设从静止开始
    kf->x[0] = 0.0f;  // 位移
    kf->x[1] = 0.0f;  // 速度
    
    // 初始协方差: 较大的不确定性 P = [1, 0; 0, 1]
    kf->P[0][0] = 1.0f;  // 位移方差
    kf->P[0][1] = 0.0f;  // 位移-速度协方差
    kf->P[1][0] = 0.0f;  // 速度-位移协方差  
    kf->P[1][1] = 1.0f;  // 速度方差
    
    // 状态转移矩阵 F = [1, dt; 0, 1]
    kf->F[0][0] = 1.0f; kf->F[0][1] = dt;
    kf->F[1][0] = 0.0f; kf->F[1][1] = 1.0f;
    
    // 控制输入矩阵 B = [0.5*dt²; dt]
    kf->B[0] = 0.5f * dt * dt;
    kf->B[1] = dt;
    
    // 观测矩阵 H = [0, 1] (只能观测速度)
    kf->H[0] = 0.0f;
    kf->H[1] = 1.0f;
    
    // 过程噪声协方差 Q
    // 基于加速度噪声推导
    float var_a = accel_noise_std * accel_noise_std;
    kf->Q[0][0] = 0.25f * dt * dt * dt * dt * var_a;
    kf->Q[0][1] = 0.5f * dt * dt * dt * var_a;
    kf->Q[1][0] = 0.5f * dt * dt * dt * var_a;
    kf->Q[1][1] = dt * dt * var_a;
    
    // 观测噪声方差 R (速度测量噪声)
    kf->R = velocity_noise_std * velocity_noise_std;
   
}

/**
 * @brief 卡尔曼滤波预测
 * 
 * @param kf 
 * @param acceleration 
 * @note  仅凭上一时刻的状态加上加速度计数据得到先验估计 
 *        x(k) = F*x(k-1) + B*u(k-1) 这里加速度作为控制输入u
 */
void RobotMotionKalman_Predict(Robotmotion_Kalman_t *kf, float acceleration) 
{
   if (!kf->initialized) return;
    
    // 状态预测: x(k) = F*x(k-1) + B*u(k-1)   
    float x0_prior = kf->F[0][0] * kf->x[0] + kf->F[0][1] * kf->x[1] + kf->B[0] * acceleration;
    float x1_prior = kf->F[1][0] * kf->x[0] + kf->F[1][1] * kf->x[1] + kf->B[1] * acceleration;
    
    kf->x[0] = x0_prior;
    kf->x[1] = x1_prior;
    
    // 协方差预测: P = F*P*F^T + Q
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];
    
    // F * P
    float FP00 = kf->F[0][0] * P00 + kf->F[0][1] * P10;
    float FP01 = kf->F[0][0] * P01 + kf->F[0][1] * P11;
    float FP10 = kf->F[1][0] * P00 + kf->F[1][1] * P10;
    float FP11 = kf->F[1][0] * P01 + kf->F[1][1] * P11;
    
    // (F * P) * F^T  这里矩阵乘法拆成标量运算
    kf->P[0][0] = FP00 * kf->F[0][0] + FP01 * kf->F[0][1] + kf->Q[0][0];
    kf->P[0][1] = FP00 * kf->F[1][0] + FP01 * kf->F[1][1] + kf->Q[0][1];
    kf->P[1][0] = FP10 * kf->F[0][0] + FP11 * kf->F[0][1] + kf->Q[1][0];
    kf->P[1][1] = FP10 * kf->F[1][0] + FP11 * kf->F[1][1] + kf->Q[1][1];
}


/**
 * @brief 更新卡尔曼滤波器的状态估计
 * 
 * @param kf 卡尔曼滤波器结构体指针
 * @param measured_velocity 测量得到的速度值
 * @example RobotMotionKalman_Update(&Kalman_Robotmotion, measured_velocity);
 * @note 观测方程：z(k) = H*x(k) + v(k),v(k)为观测噪声，这里是速度测量值；而这里的x(k)已经变成了先验估计值。
 *       计算新息协方差: S = H*P*H^T + R  总不确定性 = 预测的不确定性+传感器本身不确定性
 *       计算卡尔曼增益: K = P * H^T * S^-1 由前式，R越大（传感器噪声或者异常值），S越大，那么卡尔曼增益越小，对观测的速度传感器值置信度越小
 */      
void RobotMotionKalman_Update(Robotmotion_Kalman_t *kf, float measured_velocity) 
{
   if (!kf->initialized) return;
    
    // 计算新息 (测量残差): y = z - H*x  观测值减去预测值
    float y = measured_velocity - (kf->H[0] * kf->x[0] + kf->H[1] * kf->x[1]);
    
   // Anti_Slip_Process(kf,y); // 利用新息进行打滑处理

    // 计算新息协方差
    // H*P = [0, 1] * [P00, P01; P10, P11] = [P10, P11] 
    float HP0 = kf->H[0] * kf->P[0][0] + kf->H[1] * kf->P[1][0]; // = P10
    float HP1 = kf->H[0] * kf->P[0][1] + kf->H[1] * kf->P[1][1]; // = P11
    
    // H*P*H^T = [P10, P11] * [0; 1] = P11 这里相当于速度的方差
    float S = HP1 + kf->R;
    
    // 计算卡尔曼增益: K = P * H^T * S^-1  
    float K[2];
    K[0] = (kf->P[0][0] * kf->H[0] + kf->P[0][1] * kf->H[1]) / S; // = P01 / S
    K[1] = (kf->P[1][0] * kf->H[0] + kf->P[1][1] * kf->H[1]) / S; // = P11 / S
    
    // 更新状态估计: x = x + K*y
    kf->x[0] += K[0] * y;
    kf->x[1] += K[1] * y;
    
    // 更新协方差: P = (I - K*H) * P
    // 先计算 K*H
    float KH00 = K[0] * kf->H[0]; // = 0
    float KH01 = K[0] * kf->H[1]; // = K[0]
    float KH10 = K[1] * kf->H[0]; // = 0  
    float KH11 = K[1] * kf->H[1]; // = K[1]
    
    // I - K*H 这里I为单位阵，对角线元素给1就行
    float I_minus_KH[2][2];
    I_minus_KH[0][0] = 1.0f - KH00;
    I_minus_KH[0][1] = 0.0f - KH01;
    I_minus_KH[1][0] = 0.0f - KH10;
    I_minus_KH[1][1] = 1.0f - KH11;
    
    // 临时保存P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    float P10_temp = kf->P[1][0];
    float P11_temp = kf->P[1][1];
    
    // P = (I - K*H) * P
    kf->P[0][0] = I_minus_KH[0][0] * P00_temp + I_minus_KH[0][1] * P10_temp;
    kf->P[0][1] = I_minus_KH[0][0] * P01_temp + I_minus_KH[0][1] * P11_temp;
    kf->P[1][0] = I_minus_KH[1][0] * P00_temp + I_minus_KH[1][1] * P10_temp;
    kf->P[1][1] = I_minus_KH[1][0] * P01_temp + I_minus_KH[1][1] * P11_temp;
}

/**
 * @brief 获取卡尔曼滤波后的机体位移
 * 
 * @example RobotMotionKalman_Getdisplacement(&Kalman_Robotmotion);
 */
float RobotMotionKalman_Getdisplacement(Robotmotion_Kalman_t *kf)
{
    if (kf->initialized) 
    {
        return kf->x[0];   // 位移状态量赋值
    } 
    else 
    {
        return 0.0f;
    }
}
/**
 * @brief 获取卡尔曼滤波后的机体速度
 * 
 * @example RobotMotionKalman_Getvelocity(&Kalman_Robotmotion);
 */
float RobotMotionKalman_Getvelocity(Robotmotion_Kalman_t *kf)
{
    if (kf->initialized) 
    {
        return kf->x[1];   // 速度状态量
    } 
    else 
    {
        return 0.0f;
    }
}

// 一步到位的卡尔曼滤波更新（预测+更新）
void RobotMotionKalman_Process(Robotmotion_Kalman_t *kf, float measured_velocity, float acceleration) 
{
    RobotMotionKalman_Predict(kf, acceleration);
    RobotMotionKalman_Update(kf, measured_velocity);
}


// 一位卡尔曼滤波器计算，状态观测量只有一个速度
float Kalman_calculate(Kalman_Forward_v_one_dimension *Kalman,float v_now,float a_now)
{
    if (Kalman->Inited==0)
    {
        // 初始化的时候赋初值
        Kalman->Inited = 1;
        Kalman->last_a_input = a_now;
        Kalman->v_observe = v_now;
    }
    else 
    {
        //更新上一次的加速度
        Kalman->last_a_input = Kalman->a_input;
    }
    // 填写输入
    Kalman->a_input = a_now;
    Kalman->v_input = v_now;
    // 计算先验的速度估计值
    Kalman->v_prior = Kalman->v_observe + Kalman->period * Kalman->last_a_input;
    // 更新协方差
    Kalman->P_covariance = Kalman->P_covariance+Kalman->Q;
    // 计算卡尔曼增益
    Kalman->Kalman_gain = Kalman->P_covariance / (Kalman->P_covariance + Kalman->R);
    // 最终观测出来的速度
    Kalman->v_observe = Kalman->v_prior + Kalman->Kalman_gain * (Kalman->v_input - Kalman->v_prior);
    // 更新协方差
    Kalman->P_covariance = (1 - Kalman->Kalman_gain) * Kalman->P_covariance;
    return Kalman->v_observe;
}

// 方便协方差计算打代码做的替换
#define P1 (Kalman->P_covariance[0][0])
#define P2 (Kalman->P_covariance[0][1])
#define P3 (Kalman->P_covariance[1][0])
#define P4 (Kalman->P_covariance[1][1])

#define Kt1 (Kalman->Kalman_gain[0][0])
#define Kt2 (Kalman->Kalman_gain[0][1])
#define Kt3 (Kalman->Kalman_gain[1][0])
#define Kt4 (Kalman->Kalman_gain[1][1])

void Kalman_height_calculate(Kalman_Height_t*Kalman,float height,float height_v,float height_a)
{
    if (Kalman->Inited == 0)
    {
        // 初始化的时候赋初值
        Kalman->Inited = 1;
        Kalman->last_a_input = height_a;
        // 赋初始状态
        Kalman->State_observe[0] = height;
        Kalman->State_observe[1] = height_v;
    }
    else
    {
        // 更新上一次的加速度
        Kalman->last_a_input = Kalman->a_input;
    }
    // 输入填入
    Kalman->a_input = height_a;
    Kalman->input[0] = height;
    Kalman->input[1] = height_v;
    // 先验估计
    Kalman->State_prior[0] = Kalman->State_observe[0] 
                            + Kalman->State_observe[1] * Kalman->period 
                            + 0.5f * Kalman->period * Kalman->period * Kalman->last_a_input;
    Kalman->State_prior[1] = Kalman->State_observe[1] + Kalman->last_a_input * Kalman->period;
    // 协方差计算
    P1 = P1 + (P2 + P3) * Kalman->period + P4 * Kalman->period * Kalman->period + Kalman->Q[0][0];
    P2 = P2 + P4 * Kalman->period + Kalman->Q[0][1];
    P3 = P3 + P4 * Kalman->period + Kalman->Q[1][0];
    P4 = P4 + Kalman->Q[1][1];
    // 计算卡尔曼增益
    Kalman->temp_Kt_dividend[0][0] = P1;
    Kalman->temp_Kt_dividend[0][1] = P2;
    Kalman->temp_Kt_dividend[1][0] = P3;
    Kalman->temp_Kt_dividend[1][1] = P4;

    Kalman->temp_Kt_divide[0][0] = P1 + Kalman->R[0][0];
    Kalman->temp_Kt_divide[0][1] = P2 + Kalman->R[0][1];
    Kalman->temp_Kt_divide[1][0] = P3 + Kalman->R[1][0];
    Kalman->temp_Kt_divide[1][1] = P4 + Kalman->R[1][1];
    // 矩阵除法求卡尔曼增益
    matrix_inverse(&Kalman->temp_Kt_divide_m, &Kalman->temp_inverse_m);
    matrix_multiply(&Kalman->temp_Kt_dividend_m, &Kalman->temp_inverse_m, &Kalman->Kalman_gain_m);
    // 求最优估计值
    float temp_height = Kalman->input[0] - Kalman->State_prior[0];
    float temp_height_v = Kalman->input[1] - Kalman->State_prior[1];
    Kalman->State_observe[0] = Kalman->State_prior[0] + Kt1 * temp_height + Kt2 * temp_height_v;
    Kalman->State_observe[1] = Kalman->State_prior[1] + Kt3 * temp_height + Kt4 * temp_height_v;
    // 更新协方差矩阵
    float temp_p[2][2];
    temp_p[0][0] = P1 * (1 - Kt1) + P3 * (0 - Kt2);
    temp_p[0][1] = P2 * (1 - Kt1) + P4 * (0 - Kt2);
    temp_p[1][0] = P1 * (0 - Kt3) + P3 * (1 - Kt4);
    temp_p[1][1] = P2 * (0 - Kt3) + P4 * (1 - Kt4);
    P1 = temp_p[0][0];
    P2 = temp_p[0][1];
    P3 = temp_p[1][0];
    P4 = temp_p[1][1];
    return;
}
#undef P1
#undef P2
#undef P3
#undef P4
#undef Kt1
#undef Kt2
#undef Kt3
#undef Kt4

// 卡尔曼速度加速度融合观测器计算
void  Kalman_v_w_calculate(Kaliman_v_w_t *Kalman,float v_left,float v_right,float a_input,float w_input)
{
    if (Kalman->Inited == 0)
    {
        // 初始化的时候赋初值
        Kalman->Inited = 1;
        Kalman->last_a_input = a_input;
        // 赋初始状态
        Kalman->v_total_state = 0;
        Kalman->w_state = w_input;
    }
    else
    {
        // 更新上一次的加速度
        Kalman->last_a_input = Kalman->a_input;
    }
    Kalman->a_input = a_input;
    //状态转移
    Kalman->v_total_state = Kalman->v_total_state + Kalman->last_a_input * Kalman->period;
    Kalman->w_state = w_input;
    //测量值更新
    Kalman->v_left_ob = v_left;
    Kalman->v_right_ob = v_right;

    // 协方差计算
    Kalman->P[0][0] += Kalman->Q1;
    Kalman->P[0][1] = 0;
    Kalman->P[1][0] = 0;
    Kalman->P[1][1] = Kalman->Q2;

    //计算卡尔曼增益
    matrix_multiply(&Kalman->P_m, &Kalman->H_T_m, &Kalman->K_up_m);
    matrix_multiply(&Kalman->H_m, &Kalman->P_m, &Kalman->K_down_m);
    matrix_multiply(&Kalman->K_down_m, &Kalman->H_T_m, &Kalman->K_m);
    Kalman->K[0][0] += Kalman->R1;
    Kalman->K[1][1] += Kalman->R2;
    matrix_inverse(&Kalman->K_m, &Kalman->K_down_m);
    matrix_multiply(&Kalman->K_up_m, &Kalman->K_down_m, &Kalman->K_m);

    // 最优估计计算
    matrix_multiply(&Kalman->H_m, &Kalman->X_m, &Kalman->temp_m);
    matrix_sub(&Kalman->Z_m, &Kalman->temp_m, &Kalman->temp1_m);
    matrix_multiply(&Kalman->K_m, &Kalman->temp1_m, &Kalman->temp_m);
    Kalman->X_state_matix[0] += Kalman->temp[0];
    Kalman->X_state_matix[1] += Kalman->temp[1];

    //更新协方差方程
    matrix_multiply(&Kalman->K_m, &Kalman->H_m, &Kalman->K_down_m);
    Kalman->K_up[0][0] = 1 - Kalman->K_down[0][0];
    Kalman->K_up[0][1] = 0 - Kalman->K_down[0][1];
    Kalman->K_up[1][0] = 0 - Kalman->K_down[1][0];
    Kalman->K_up[1][1] = 1 - Kalman->K_down[1][1];
    matrix_multiply(&Kalman->K_up_m, &Kalman->P_m, &Kalman->K_down_m);
    Kalman->P[0][0] = Kalman->K_down[0][0];
    Kalman->P[0][1] = Kalman->K_down[0][1];
    Kalman->P[1][0] = Kalman->K_down[1][0];
    Kalman->P[1][1] = Kalman->K_down[1][1];
    //计算应该有的状态
    matrix_multiply(&Kalman->H_m,&Kalman->X_m,&Kalman->Z_m);
}