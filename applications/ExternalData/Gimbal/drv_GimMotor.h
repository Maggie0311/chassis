#ifndef __DRV_GIMMOTOR_H__
#define __DRV_GIMMOTOR_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_motor_new.h"

/**
 * @brief 注册云台电机
 * 
 * @param motor_instance 电机实例
 * @return int 1为错误，0为无错误
 */
int Gim_Motor_Attach(MotorInstance *motor_instance);
/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @return   数据范围(0°~360°]
 ***/
float Get_YAW_ANGLE_360f(void);
/**
 * @brief 获取云台pitch电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 * 
 * @return 数据范围(0°~360°] 
 */
float Get_PITCH_ANGLE_360f(void);
/***
 * @brief    获取云台电机速度数据  yaw轴电机速度为正表示：在俯视图底盘坐标系下云台枪管逆时针旋转
 * @param    kind 云台yaw or pitch电机 pitch轴电机速度为正表示：云台枪管抬升
 * @return   单位：°/s
 ***/
float Get_GM_Speed(MotorInstance *motor_instance);
/***
 * @brief    获取云台枪管正方向相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void);
/**
 * @brief 获取云台 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
rt_bool_t Get_Yaw_Motor_Offline_State(void);
/**
 * @brief 获取云台 pitch 轴电机的离线状态
 * @return rt_bool_t        返回真代表 pitch 轴电机离线(200ms)
 */
rt_bool_t Get_Pitch_Motor_Offline_State(void);
//todo 该函数待处理
uint8_t GetGimMotorOffline(void);
#endif
