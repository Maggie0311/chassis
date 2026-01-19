#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

#include "rtconfig.h"
/*底盘电机ID*/
#define RUN_RF_MOTOR_ID 0x201
#define RUN_LF_MOTOR_ID 0x202
#define RUN_LB_MOTOR_ID 0x203
#define RUN_RB_MOTOR_ID 0x204

/*云台通信ID*/
/*云台电机ID(仅接收)（rx_gimMotor）*/
#ifdef YAW_MOTOR_6020
#define YAW_MOTOR_RXID 0x205  
#define PITCH_MOTOR_RXID 0x206 
#endif

#ifdef YAW_MOTOR_DM
#define YAW_MOTOR_RXID 0x03
#define PITCH_MOTOR_RXID 0x04 
#endif

/*超电通信ID*/
#define SCPR_RX 0x096 // 从超级电容端接收
#define SCPR_TX 0x095 // 向超级电容端发送

/*CAN设备挂载*/
#if RF_CAN_DEVICE_NAME == 1
#define RF_CAN_DEV USED_CAN1
#elif RF_CAN_DEVICE_NAME == 2
#define RF_CAN_DEV USED_CAN2
#endif
#if LF_CAN_DEVICE_NAME == 1
#define LF_CAN_DEV USED_CAN1
#elif LF_CAN_DEVICE_NAME == 2
#define LF_CAN_DEV USED_CAN2
#endif
#if LB_CAN_DEVICE_NAME == 1
#define LB_CAN_DEV USED_CAN1
#elif LB_CAN_DEVICE_NAME == 2
#define LB_CAN_DEV USED_CAN2
#endif
#if RB_CAN_DEVICE_NAME == 1
#define RB_CAN_DEV USED_CAN1
#elif RB_CAN_DEVICE_NAME == 2
#define RB_CAN_DEV USED_CAN2
#endif

// #define GIMBAL_CAN_DEV USED_CAN2        // 云台can设备
#if GIMBAL_CAN_DEVICE_NAME == 1
#define GIMBAL_CAN_DEV "can1"
#elif GIMBAL_CAN_DEVICE_NAME == 2
#define GIMBAL_CAN_DEV "can2"
#else 
#error "GIMBAL_CAN_DEVICE_NAME is not defined right!"
#endif

// #define IMU_GIMBAL_CAN_DEV USED_CAN2    // 云台IMUcan设备
#if IMU_GIMBAL_CAN_DEVICE_NAME == 1
#define IMU_GIMBAL_CAN_DEV "can1"
#elif IMU_GIMBAL_CAN_DEVICE_NAME == 2
#define IMU_GIMBAL_CAN_DEV "can2"
#else 
#error "IMU_GIMBAL_CAN_DEVICE_NAME is not defined right!"
#endif

// #define SCPR_CAN_DEV USED_CAN1          // 超级电容设备
#if SCPR_CAN_DEVICE_NAME == 1
#define SCPR_CAN_DEV "can1"
#elif SCPR_CAN_DEVICE_NAME == 2
#define SCPR_CAN_DEV "can2"
#else 
#error "SCPR_CAN_DEVICE_NAME is not defined right!"
#endif

// #define IMU_CHASSIS_CAN_DEV USED_CAN1   // 底盘IMUcan设备
#if IMU_CHASSIS_CAN_DEVICE_NAME == 1
#define IMU_CHASSIS_CAN_DEV USED_CAN1
#elif IMU_CHASSIS_CAN_DEVICE_NAME == 2
#define IMU_CHASSIS_CAN_DEV USED_CAN2
#else 
#error "IMU_CHASSIS_CAN_DEVICE_NAME is not defined right!"
#endif
/*功率控制相关参数*/

/*轮子相关数据*/
#define USING_MECANUM_RESOLVE
#define VEHICLE_LONG (342.f)       // 前后轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_WIDTH (342.f)      // 左右轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_DIAMETER (483.66f) // 对角轮间距（用于计算全向轮解算），单位mm
#define WHEEL_RADIUS (63.5f)       // 轮子半径 单位毫米

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