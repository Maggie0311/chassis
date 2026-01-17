/**
 * @file drv_EnergyConservation.c
 * @brief 本文件用于跟随模式下底盘侧移时修改底盘正方向以实现节能底盘的效果
 * @author fwlh
 * @version 1.0
 * @date 2022-04-20
 *
 * @copyright Copyright (c) 2022  哈尔滨工业大学(威海)HERO战队
 */
#include "fun_EnergyConservation.h"
#include "drv_GimMotor.h"
#include "drv_IMU.h"
#include "drv_utils.h"

#define NORMAL_SPEED_SET 800 // 不需要节能底盘的横移速度

// 用于记录上一次访问时的云台 IMU 数据
float LastGimbalYaw = 0.f;

float last_value = 0.f; // 平滑时的上次输出值
/**
 * @brief 根据实际设定速度的大小计算平滑之后的角度
 * @author fwlh
 * @param  RealFixedFolAngle 底盘实际的运动方向角
 * @param  threshold         底盘设定速度大小变化速度
 * @return float             加权后的底盘跟随角度设定值
 */
static float GetMixedAngle(float RealFixedFolAngle, float threshold)
{
    if (utils_angle_difference(RealFixedFolAngle, last_value) > threshold)
        last_value += threshold;
    else if (utils_angle_difference(RealFixedFolAngle, last_value) < -threshold)
        last_value -= threshold;
    else
        last_value = RealFixedFolAngle;
    utils_norm_angle(&last_value);
    return last_value;
}
float Last_Follow_Angle = 0.f; // 用于记录上一次的目标跟随角度
/**
 * @brief 跟随模式下会根据云台给定的运动方向纠正底盘的正方向
 * @author fwlh
 * @param  Vxy              xy 方向的速度设定值(云台坐标系)
 * @param  fol_angle        云台设定的跟随角度
 * @param  YawAngle         Yaw 轴电机当前角度
 */
void Follow_Gimbal_Energy_Fix(const Vector2_t *Vxy, float *fol_angle, float YawAngle)
{
    float Speed2 = Vxy->x * Vxy->x + Vxy->y * Vxy->y;
    float Real_Follow_Angles = 0.f;
    // 目标跟随角度改变时直接改变本地记录数值
    if (fabsf(utils_angle_difference(Last_Follow_Angle, *fol_angle)) > 1.f)
    {
        Last_Follow_Angle = *fol_angle;
        last_value = *fol_angle;
    }
    // 速度比较小的时候就逐渐恢复正常的跟随模式
    if (Speed2 < NORMAL_SPEED_SET * NORMAL_SPEED_SET)
    {
        // 写入新的跟随角设定值, 每次运行该函数会恢复 0.35°
        *fol_angle = GetMixedAngle(*fol_angle, 0.35f);
        // 记录此时的云台 Yaw 轴 IMU 数据
        LastGimbalYaw = Get_Gimbal_IMU_Data(IMU_YAW);
        return;
    }

    // 需要进行节能, 所以先计算新的跟随角度的方向(向右为 x 正方向, 向前为 y 正方向)
    if ((Vxy->y < 0.f) || (Vxy->x > 0.f))
        Real_Follow_Angles = atan2f(Vxy->y, Vxy->x) / 3.14159f * 180.f;
    else // 该函数的两个参数都小于 0 的时候会出问题, 具体原因未知
        Real_Follow_Angles = atan2f(-Vxy->y, -Vxy->x) / 3.14159f * 180.f;
    // 根据计算出来的作用线计算真实的运动方向线
    Real_Follow_Angles = 90.f - Real_Follow_Angles;
    // 就近寻找转向方向, 计算两个角度的和是否大于 95 度
    if (fabsf(utils_angle_difference(Real_Follow_Angles, -YawAngle)) > 95.f)
    {
        if (Real_Follow_Angles < 0.f)
            Real_Follow_Angles += 180.f;
        else
            Real_Follow_Angles -= 180.f;
    }

    // 将新的跟随角度添加到跟随角度的设定值中(想要的运动速度越快, 设定值改变得越快), 这里角度设定值记得加符号
    *fol_angle = GetMixedAngle(-Real_Follow_Angles, sqrtf(Speed2) / 2500);
    // 更新 Yaw 轴数据
    LastGimbalYaw = Get_Gimbal_IMU_Data(IMU_YAW);
}
