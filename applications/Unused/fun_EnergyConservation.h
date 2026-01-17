#ifndef __DRV_ENERGYCONSERVATION_H__
#define __DRV_ENERGYCONSERVATION_H__

#include "controlMsg.h"
/**
 * @brief 跟随模式下会根据云台给定的运动方向纠正底盘的正方向
 * @author fwlh
 * @param  Vxy              xy 方向的速度设定值(云台坐标系)
 * @param  fol_angle        云台设定的跟随角度
 * @param  YawAngle         Yaw 轴电机当前角度
 */
extern void Follow_Gimbal_Energy_Fix(const Vector2_t *Vxy, float *fol_angle, float YawAngle);

#endif /* __DRV_ENERGYCONSERVATION_H__ */
