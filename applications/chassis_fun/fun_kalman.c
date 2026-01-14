#include "fun_kalman.h"

/**
 * @brief  二阶卡尔曼滤波
 * @param {kalman2_state_t} *state
 * @param {float} data_x 待到的融合数据，一般为直接采样数据
 * @param {float} data_xdot 待到的融合数据的导数
 * @param {float} time_interval 时间差
 */
float kalman2_filter(kalman2_state_t *state, float data_x, float data_xdot, float time_interval)
{
    float d_data_x, d_data_xdot; // 先验预测量与测量量的差值

    state->dt = time_interval;

    /*------------------------------------预测部分----------------------------------------------------------*/

    // 期望预测
    state->PData.x = state->PosData.x + state->dt * state->PosData.xdot;
    state->PData.xdot = state->PosData.xdot;

    // 协方差预测
    state->PData.cov[0] = state->PosData.cov[0] + state->dt * (state->PosData.cov[1] + state->PosData.cov[2]) + state->Q_x;
    state->PData.cov[1] = state->PosData.cov[1] + state->dt * state->PosData.cov[3];
    state->PData.cov[2] = state->PosData.cov[2] + state->dt * state->PosData.cov[3];
    state->PData.cov[3] = state->PosData.cov[3] + state->Q_xdot;

    // 更新部分
    //  计算 卡尔曼增益矩阵
    state->k_det = 1 / ((state->PData.cov[0] + state->R_x) * (state->PData.cov[3] + state->R_xdot) - state->PData.cov[1] * state->PData.cov[2]);
    state->k[0] = state->k_det * (state->PData.cov[0] * (state->PData.cov[3] + state->R_xdot) - state->PData.cov[1] * state->PData.cov[2]);
    state->k[1] = state->k_det * (state->PData.cov[1] * (state->PData.cov[0] + state->R_x) - state->PData.cov[0] * state->PData.cov[1]);
    state->k[2] = state->k_det * (state->PData.cov[2] * (state->PData.cov[3] + state->R_xdot) - state->PData.cov[2] * state->PData.cov[3]);
    state->k[3] = state->k_det * (state->PData.cov[3] * (state->PData.cov[0] + state->R_x) - state->PData.cov[1] * state->PData.cov[2]);

    // 更新期望（即生成后验期望）
    d_data_x = data_x - state->PData.x;
    d_data_xdot = data_xdot - state->PData.xdot;
    state->PosData.x = state->PData.x + state->k[0] * d_data_x + state->k[1] * d_data_xdot;
    state->PosData.xdot = state->PData.xdot + state->k[2] * d_data_x + state->k[3] * d_data_xdot;

    // 更新协方差矩阵（即后验协方差矩阵）
    state->PosData.cov[0] = state->PData.cov[0] * (1 - state->k_det * state->k[0]) - state->k_det * state->k[1] * state->PData.cov[2];
    state->PosData.cov[1] = state->PData.cov[1] * (1 - state->k_det * state->k[0]) - state->k_det * state->k[1] * state->PData.cov[3];
    state->PosData.cov[2] = state->PData.cov[2] * (1 - state->k_det * state->k[3]) - state->k_det * state->k[2] * state->PData.cov[0];
    state->PosData.cov[3] = state->PData.cov[3] * (1 - state->k_det * state->k[3]) - state->k_det * state->k[2] * state->PData.cov[1];

    // 完成迭代，返回期望角度
    return state->PosData.x;
}


// 状态误差
void setQMatrix(kalman2_state_t *state, float Q_x, float Q_xdot)
{
    state->Q_x = Q_x;
    state->Q_xdot = Q_xdot;
}

// 估计误差
void setRMatrix(kalman2_state_t *state, float R_x, float R_xdot)
{
    state->R_x = R_x;
    state->R_xdot = R_xdot;
}

void kalman_DeInit(kalman2_state_t *state)
{
    state->Q_x = 0.1;
    state->Q_xdot = 1;

    state->R_x = 0.1;
    state->R_xdot = 1;

    state->PData.x = 0;
    state->PData.xdot = 0;

    state->PData.cov[0] = 0.2;
    state->PData.cov[1] = 0;
    state->PData.cov[2] = 0;
    state->PData.cov[3] = 0.2;
}

/*------------------------------------一阶卡尔曼滤波----------------------------------------------------------*/
/**
 * @brief 一阶卡尔曼滤波
 *  状态方程: x = A*x + w; w~N(0,Q)
 * 	测量方程: y = C*x + v; v~N(0,R)
 * @param {kalman1State_t} *kalman
 * @param {float} y 待滤波数据
 */
float kalman1_Filter(kalman1State_t *kalman, float y)
{
    // 状态预测
    kalman->x = kalman->A * kalman->x;
    // 误差协方差预测
    kalman->P = kalman->A2 * kalman->P + kalman->Q;
    // 计算卡尔曼滤波增益
    kalman->K = kalman->P * kalman->C / (kalman->C2 * kalman->P + kalman->R);

    // 状态估计校正
    kalman->x = kalman->x + kalman->K * (y - kalman->C * kalman->x);
    // 误差协方差估计校正
    kalman->P = (1 - kalman->K * kalman->C) * kalman->P;

    return kalman->C * kalman->x; // 输出滤波后的y
}

void kalman1_init(kalman1State_t *kalman, float A, float C, float Q, float R)
{
    kalman->A = A;
    kalman->A2 = A * A;
    kalman->C = C;
    kalman->C2 = C * C;

    kalman->Q = Q;
    kalman->R = R;

    kalman->x = 0;
    kalman->P = Q;
    kalman->K = 1;
}
void kalman1SetQ(kalman1State_t *kalman, float Q)
{
    kalman->Q = Q;
    kalman->P = Q;
}
void kalman1SetR(kalman1State_t *kalman, float R)
{
    kalman->R = R;
}

/*------------------------------------XXX----------------------------------------------------------*/

void kalman_filter_init(kalman_filter_t *kalman)
{
    kalman->accel = 0;
    kalman->accel_cov = 0.01;
    kalman->Q_accel = 0.05;
    kalman->R_accel = 0.004;
    kalman->vel = 0;
    kalman->vel_cov = 0.03;
    kalman->Q_vel = 0.005;
    kalman->R_vel = 0.008;
}

float kalman_filter_update(kalman_filter_t *filter, float vel_measurement,float accel_measurement,  float dt)
{
    // 预测步骤
    filter->vel += filter->accel * dt; // 预测速度

    // 更新步骤（基于加速度测量）
    float K_accel = filter->vel_cov / (filter->vel_cov + filter->R_accel); // 计算加速度测量的卡尔曼增益
    float accel_estimate = accel_measurement - filter->accel;              // 估计的加速度
    filter->vel += K_accel * accel_estimate;                               // 更新速度估计值
    filter->vel_cov = (1 - K_accel) * filter->vel_cov;                     // 更新速度协方差

    // 预测步骤（基于更新后的速度）
    filter->vel += filter->accel * dt; // 预测速度
    filter->vel_cov += filter->Q_vel;  // 更新速度协方差

    // 更新步骤（基于速度测量）
    float K_vel = filter->vel_cov / (filter->vel_cov + filter->R_vel); // 计算速度测量的卡尔曼增益
    float vel_estimate = vel_measurement - filter->vel;                // 估计的速度
    filter->vel += K_vel * vel_estimate;                               // 更新速度估计值
    filter->vel_cov = (1 - K_vel) * filter->vel_cov;                   // 更新速度协方差

    // 更新加速度
    filter->accel = accel_measurement;
    filter->accel_cov += filter->Q_accel; // 更新加速度协方差

    filter->vel_cov += filter->Q_vel; // 更新速度协方差

    return filter->vel;
}
