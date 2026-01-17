#ifndef __FUNC_ANTISLIP_H__
#define __FUNC_ANTISLIP_H__

#include "controlMsg.h"

// 当出现最多次数判断打滑
#define SLIP_THRESHOLD 10

// 当误差大于多少判断为打滑
#define MAX_DIFFERENCE 0.5f

// 设定值和实际值相差多大判断打滑
#define LEAST_SQUARE_DATANUM 100

typedef struct
{
    float vx;
    float vy;
    float vz; // 用于拓展用
} IMU_Speed_t;
typedef enum
{
    axis_x,
    axis_y,
    axis_z,

} IMU_AxisType_e;
typedef struct
{
    float k;
    float b;
} Equation_t;

typedef struct
{
    float x;      // 平移速度矢量，单位mm/s
    float y;      // 平移速度矢量，单位mm/s
    float angvel; // 角速度，单位0.1°/s

} Ctrl_speed_t;
typedef enum
{
    eqution_x,
    eqution_y,
    eqution_w,
    eqution_all,

} IsSlipEquation_e;

typedef struct
{
    // 实时用于校准的数据
    float slip_setData[LEAST_SQUARE_DATANUM];
    float slip_imuData[LEAST_SQUARE_DATANUM];
    Equation_t motion_eqa[eqution_all]; // x y w
    uint8_t isCali;
    uint8_t isfinish;

} SlipCaliData_t;

typedef struct
{
    Mot_base_t Set_speed;   // 当前设定值速度 单位mm/s
    IMU_Speed_t Acc_speed;  // 当前加速度计得到的速度
    Equation_t Equation[3]; // 测量的加速度计和设定速度正常情况下的对应关系
    uint16_t Slip_Counts;   // 打滑阈值

    uint8_t IsSlip;

} IsSlipInfo_t;

/**
 * @brief :判断是否打滑
 * @return {*}返回1表示打滑
 */
uint8_t Chassis_IsSlip(Mot_base_t Movemotion);

/**
 * @brief 最小二乘法拟合两个数据的线性关系
 * @param {float} *datax 方程的x
 * @param {float} *datay 方程的y
 * @param {uint16_t} dataNum 拟合的数据量
 * @param {Equation_t} *Equation 得到的方程系数 y=kx+b
 */
void leastSquareData(float *datax, float *datay, uint16_t dataNum, Equation_t *Equation);
#endif //__FUNC_ANTISLIP_H__
