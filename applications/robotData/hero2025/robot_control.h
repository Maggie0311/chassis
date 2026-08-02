#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

/*
 * hero2025 —— 25年老英雄（四轮麦轮）电机配置与控制接口
 * 普通构型：4×3508 麦轮 + 本机 6020 云台（yaw/pitch），无抬升/舵轮。
 */

#include "controlMsg.h"
#include "robot_data.h"

#define USING_MECANUM_RESOLVE

int chassis_motor_init(void);
int gim_motor_init(void);
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);
void Follow_pid_init(void);
void Fix_Speed_pid_init(void);
void Forward_Kinematics(void);
// 云台电机索引
#define YAW_MOTOR 0
#define PITCH_MOTOR 1

// TODO: 以下 PID/功率参数均需按实车整定
#define RUN_PID_Kp 96
#define RUN_PID_Ki 0.5
#define RUN_PID_Kd 0.8

#define POWER_CTRL 0       // 功率控制开关：1开 0不开
#define Power_C_sum (4.0f) // 固定静态功率（换板子也影响）
#define Wheel_Power_K1 (50.f)
#define Wheel_Power_K2 (0.00097)
#define Wheel_Power_C (7.f)
#define Steer_Power_K1 (10.f) // 麦轮车型不用舵轮功率，保留占位
#define Steer_Power_K2 (0.0000015)
#define Steer_Power_C (0.f)

#define WHEELS_GROUP_NUM 4
#define PID_CAL_PERIOD 1
#define CHASSIS_CTRL_PERIOD 2

#endif
