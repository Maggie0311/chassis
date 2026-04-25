#include "Kalman.h"
#include "rtthread.h"

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
    kf->innovation = y;  // 保存新息供外部前馈使用
    
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

/**
 * @brief 获取卡尔曼滤波器当前的新息（innovation）
 *
 * @note 新息 = 正解算速度 - 卡尔曼预测速度
 *       正值：正解算比预测快（轮子可能在打滑，编码器虚高）
 *       负值：正解算比预测慢
 *       该值是打滑的最早期信号，比滤波后的速度至少快一拍
 */
float RobotMotionKalman_GetInnovation(Robotmotion_Kalman_t *kf)
{
    if (kf->initialized)
    {
        return kf->innovation;
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

/**
 * @brief 根据IMU加速度幅值动态调整Q和R，然后执行预测+更新
 *
 * 核心思路：
 *   |accel| 大（急加减速/打滑风险高）→ 更信 IMU 积分 → Q 小, R 大
 *   |accel| 小（匀速平稳）           → 更信正解算   → Q 大, R 小
 *
 * 使用 sigmoid 型平滑插值，避免硬切换带来的跳变：
 *   alpha = |accel|^2 / (|accel|^2 + threshold^2)
 *   alpha ∈ [0, 1]，|accel| >> threshold 时趋近1，|accel| << threshold 时趋近0
 *
 *   Q_std = Q_min + (Q_max - Q_min) * (1 - alpha)   // 加速大→Q小
 *   R_std = R_min + (R_max - R_min) * alpha          // 加速大→R大
 *
 * @param kf 卡尔曼滤波器
 * @param measured_velocity 正解算速度 (m/s)
 * @param acceleration IMU加速度 (m/s²)
 */
void RobotMotionKalman_AdaptiveProcess(Robotmotion_Kalman_t *kf, float measured_velocity, float acceleration)
{
    /* ---- 可调参数 ---- */
    #define ACCEL_THRESHOLD  0.4f   // 加速度分界点(m/s²)，在此值附近alpha≈0.5
    #define Q_STD_MIN        0.05f  // 加速度大时的Q_std（更信IMU，Q小→预测噪声小）
    #define Q_STD_MAX        0.4f   // 加速度小时的Q_std（更信正解算，Q大→预测噪声大）
    #define R_STD_MIN        0.05f  // 加速度小时的R_std（更信正解算，R小→观测噪声小）
    #define R_STD_MAX        0.4f   // 加速度大时的R_std（更信IMU，R大→观测噪声大）

    /* ---- 计算平滑插值因子 alpha ---- */
    float abs_acc = fabsf(acceleration);
    float acc_sq = abs_acc * abs_acc;
    float thr_sq = ACCEL_THRESHOLD * ACCEL_THRESHOLD;
    float alpha = acc_sq / (acc_sq + thr_sq);  // sigmoid型，[0,1]

    /* ---- 动态计算噪声标准差 ---- */
    float q_std = Q_STD_MAX + (Q_STD_MIN - Q_STD_MAX) * alpha;  // 加速大→Q小
    float r_std = R_STD_MIN + (R_STD_MAX - R_STD_MIN) * alpha;  // 加速大→R大

    /* ---- 更新 Q 矩阵（基于加速度噪声标准差推导） ---- */
    float var_a = q_std * q_std;
    float dt = kf->dt;
    kf->Q[0][0] = 0.25f * dt * dt * dt * dt * var_a;
    kf->Q[0][1] = 0.5f  * dt * dt * dt * var_a;
    kf->Q[1][0] = 0.5f  * dt * dt * dt * var_a;
    kf->Q[1][1] = dt * dt * var_a;

    /* ---- 更新 R（观测噪声方差） ---- */
    kf->R = r_std * r_std;

    /* ---- 正常执行预测+更新 ---- */
    RobotMotionKalman_Predict(kf, acceleration);
    RobotMotionKalman_Update(kf, measured_velocity);
}


