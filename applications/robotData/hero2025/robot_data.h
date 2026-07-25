#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

#include "rtconfig.h"

/*
 * hero2025 —— 25年老英雄（四轮麦轮）底盘参数
 * 普通构型：4×3508 麦轮 + 本机 6020 云台（yaw/pitch），无抬升/舵轮/上坡前馈。
 * 说明：以下机械/动态参数均为占位值，标 TODO 的项需按实车测量后修改。
 */

// 云台/超电通信设备（Hero_Gimbal / 超电模块使用）TODO：按实际接线确认
#define GIMBAL_CAN_DEV "can2" // TODO: 云台通信 CAN 设备
#define SCPR_CAN_DEV   "can1" // TODO: 超级电容通信 CAN 设备
#define GIMBAL_SPI_USE 0      // TODO: 云台通信 SPI 从机号（0=不使用SPI）
#define SCPR_SPI_USE   2      // TODO: 超电通信 SPI 从机号

/*---------------------- 轮子相关数据（麦轮）----------------------*/
#define USING_MECANUM_RESOLVE // 麦轮解算
#define VEHICLE_LONG (504.f)       // TODO: 前后轮间距 mm
#define VEHICLE_WIDTH (410.f)      // TODO: 左右轮间距 mm
#define VEHICLE_DIAMETER (649.7f)  // TODO: 对角轮间距 mm
#define WHEEL_RADIUS (76.f)        // TODO: 轮子半径 mm

/*小陀螺角度补偿相关参数*/
#define YAW_OFFSET_FAST (0.f) // TODO: 陀螺角度补偿（快）
#define YAW_OFFSET_SLOW (0.f) // TODO: 陀螺角度补偿（慢）

#define KEY_C_FUN 1            // 超级电容C键功能选择：0:无功能;1:超电开关功能;2:电量保留功能
#define SCAP_MAX_CURRENT (8.f) // 电容最大放电响应电流

/*跟随角pid相关参数*/
#define FOLLOW_PID_1 15, 0, 10, 0, 4000, -4000 // kp,ki,kd,i_lim,out_up,out_down（跟随模式）TODO:待整定
#define FOLLOW_PID_2 40, 0, 10, 0, 4000, -4000 // 由非跟随进入跟随时 TODO:待整定

/*设定值规划及前馈相关参数*/
#define WEIGHT (35.0f)                   // TODO: 整车质量 kg
#define rotational_inertia 0.3f          // TODO: 转动惯量 kg·m²
#define PLAN_PERIOD (float)0.02          // TODO: 规划周期 s（50Hz）
#define VEL_MAX_X (float)2900            // TODO: x向最大线速度 mm/s
#define VEL_MAX_Y (float)2900            // TODO: y向最大线速度 mm/s
#define VEL_MAX_W (float)6000            // TODO: 最大角速度 0.1°/s
#define ACC_MAX_X (float)4000            // mm/s²
#define ACC_MAX_Y (float)4000            // mm/s²
#define ACC_MAX_W (float)30000           // 0.1°/s²
#define DOUBLE_ACC_MAX_X (float)30000    // mm / s³
#define DOUBLE_ACC_MAX_Y (float)20000    // mm / s³
#define DOUBLE_ACC_MAX_W (float)200000   // 0.1°/ s³

#endif

// can1: 底盘四个麦轮 3508 电机（0x201-0x204）
// can2: 云台 yaw/pitch 6020（0x205/0x207）+ 超电/通信
