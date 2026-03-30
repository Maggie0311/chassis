#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "controlMsg.h"
#include "robot_data.h"

#define USING_STEER_RESOLVE
int chassis_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);

int gim_motor_init();
void Follow_pid_init(void);

float get_power_lim_speed(float power_max);




// 云台电机是否大小yaw
#define YAW_MOTOR 0
#define PITCH_MOTOR 1
#define hero

// TODO:调Pid这堆
#define RUN_PID_Kp 100
#define RUN_PID_Ki 0.6
#define RUN_PID_Kd 7

#define POWER_CTRL 1//功率控制开关，1为开启，0为不开

#define Power_C_sum (4.0f) // 固定静态功率（换板子也会影响该数据）

#define Power_C_sum    (6.0f)        // 固定静态功率（换板子也会影响该数据）

#define Wheel_Power_K1 (50.f)       //(4.5e+02) //(1.15e+02) //(1.0e+02)
#define Wheel_Power_K2 (0.00097)   //(1.0e-07) //(6.0e-07)  //(4.1e-07)
#define Wheel_Power_C (2.f)          // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Wheel_Power_C_sum (6.0f)   // 固定静态功率（换板子也会影响该数据）
// TODO 此处参数需要测试修改
#define Steer_Power_K1 (0.2769f)//(10.f)      //
#define Steer_Power_K2 (0.0107) //
#define Steer_Power_C (0.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用

void Forward_Kinematics(void);

#endif