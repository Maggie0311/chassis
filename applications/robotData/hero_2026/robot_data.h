#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

#include "rtconfig.h"

// #define GIMBAL_CAN_DEV USED_CAN2        // 云台can设备
#if GIMBAL_CAN_DEVICE_NAME == 1
#define GIMBAL_CAN_DEV "can1"
#elif GIMBAL_CAN_DEVICE_NAME == 2
#define GIMBAL_CAN_DEV "can2"
#else 
#error "GIMBAL_CAN_DEVICE_NAME is not defined right!"
#endif

// #define SCPR_CAN_DEV USED_CAN1          // 超级电容设备
#if SCPR_CAN_DEVICE_NAME == 1
#define SCPR_CAN_DEV "can1"
#elif SCPR_CAN_DEVICE_NAME == 2
#define SCPR_CAN_DEV "can2"
#else 
#error "SCPR_CAN_DEVICE_NAME is not defined right!"
#endif

#define GIMBAL_SPI_USE 2//使用从机2
#define SCPR_SPI_USE 2//使用从机2

/*功率控制相关参数*/

/*轮子相关数据*/
#define USING_MECANUM_RESOLVE
#define VEHICLE_LONG (426.f)       // 前后轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_WIDTH (376.f)      // 左右轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_DIAMETER (568.2f) // 对角轮间距（用于计算全向轮解算），单位mm
#define WHEEL_RADIUS (60.f)       // 轮子半径 单位毫米

// // 底盘方位角参数,底盘正前方为0°，逆时针为正方向,单位/°，在舵向电机正解算时需要
// #define RF_Wheel_Azimuth (-45.f)
// #define LF_Wheel_Azimuth (45.f)
// #define LB_Wheel_Azimuth (135.f)
// #define RB_Wheel_Azimuth (-135.f)

/*跟随角pid相关参数*/
#define FOLLOW_PID_1 15, 0, 10, 0, 4000, -4000 // kp, ki, kd, i_lim, out_lim_up, out_lim_down   // 一直在跟随模式下的跟随角pid参数
#define FOLLOW_PID_2 40, 0, 10, 0, 4000, -4000 // 由非跟随进入跟随时的跟随角pid参数，目的是为了实现非跟随模式下响应能够较快

/*设定值规划及前馈相关参数*/
/*底盘速度设定值的平滑，SMOOTH_ADD/SMOOTH_PERIOD m/s^2（或100°/s^2） (类型float)*/
#define WEIGHT 20.0f                   // 质量 kg
#define rotational_inertia 0.5f        // 转动惯量 kg·m²
#define PLAN_PERIOD (float)0.005       // s,频率为200Hz
#define VEL_MAX_X (float)5000          //    mm/s
#define VEL_MAX_Y (float)5000          //    mm/s
#define VEL_MAX_W (float)10000         //   0.1°/s
#define ACC_MAX_X (float)4000          // mm/s²
#define ACC_MAX_Y (float)4000          // mm/s²
#define ACC_MAX_W (float)30000         // 0.1°/s²
#define DOUBLE_ACC_MAX_X (float)30000  // mm / s³
#define DOUBLE_ACC_MAX_Y (float)30000  // mm / s³
#define DOUBLE_ACC_MAX_W (float)200000 // 0.1°/ s³

#endif

// can1:   底盘轮向四个3508电机
// can2 :  拨弹盘0x201 云台两个6020电机0x205 0x207 

