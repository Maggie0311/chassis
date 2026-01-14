#ifndef _FUN_KALMAN_H_
#define _FUN_KALMAN_H_


// 先验数据
typedef struct
{

    float x;
    float xdot;
    float cov[4]; // 协方差矩阵
} prioriData_t;

// 后验数据
typedef struct
{
    float x;
    float xdot;
    float cov[4]; // 协方差矩阵
} PosterioriData_t;

typedef struct
{
    prioriData_t PData;       // 先验数据
    PosterioriData_t PosData; // 后验数据
    float dt;                 // 采样间隔时间
    float Q_x;                // 过程噪声
    float Q_xdot;             // 预测模型的方差
    float R_x;                // 观察噪声
    float R_xdot;             // 观测模型的方差

    float k_det; // 卡尔曼增益矩阵行列式
    float k[4];  // 卡尔曼增益矩阵

} kalman2_state_t;

/**
 * 	x = A*x + w; w~N(0,Q)
 * 	y = C*x + v; v~N(0,R)
 */
typedef struct scalar_kalman_s
{
    float A, C;
    float A2, C2; // A,C 的平方

    float Q, R;
    float K, P;

    float x;
} kalman1State_t;

typedef struct
{
    float vel;       // 状态量（速度）
    float vel_cov;   // 速度协方差
    float accel;     // 加速度
    float accel_cov; // 加速度协方差
    float Q_vel;     // 速度状态噪声协方差
    float Q_accel;   // 加速度状态噪声协方差
    float R_accel;   // 加速度测量噪声协方差
    float R_vel;     // 速度测量噪声协方差
} kalman_filter_t;

//二阶卡尔曼滤波
float kalman2_filter(kalman2_state_t *state, float angle_m, float gyr_m, float time_interval);
void setQMatrix(kalman2_state_t *state, float Q_x, float Q_xdot);
void setRMatrix(kalman2_state_t *state, float R_x, float R_xdot);
void kalman_DeInit(kalman2_state_t *state);
void kalman_filter_init(kalman_filter_t *kalman);
float kalman_filter_update(kalman_filter_t *filter, float vel_measurement, float accel_measurement, float dt);
// 一阶卡尔曼滤波
float kalman1_Filter(kalman1State_t *kalman, float y);
void kalman1_init(kalman1State_t *kalman, float A, float C, float Q, float R);
void kalman1SetQ(kalman1State_t *kalman, float Q);
void kalman1SetR(kalman1State_t *kalman, float R);
#endif
