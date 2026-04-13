#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "controlMsg.h"
#include "robot_data.h"
#define USING_OMNI_RESOLVE
int chassis_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);

int gim_motor_init();
void Follow_pid_init(void);

// 云台电机是否大小yaw
#define YAW_MOTOR 0
#define PITCH_MOTOR 1

// TODO:调Pid这堆
#define RUN_PID_Kp 96
#define RUN_PID_Ki 0.5

#define RUN_PID_Kd 0.8

#define POWER_CTRL 0//功率控制开关，1为开启，0为不开

#define Power_C_sum (4.0f) // 固定静态功率（换板子也会影响该数据）

#define Wheel_Power_K1 (240.f)     //(4.5e+02) //(1.15e+02) //(1.0e+02)
#define Wheel_Power_K2 (2.009e-07) //(1.0e-07) //(6.0e-07)  //(4.1e-07)
#define Wheel_Power_C (0.f)
// #define Wheel_Power_C_sum (6.0f)   // 固定静态功率（换板子也会影响该数据）
// TODO 此处参数需要测试修改
#define Steer_Power_K1 (10.f)      //
#define Steer_Power_K2 (0.0000015) //
#define Steer_Power_C (0.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Steer_Power_C_sum (10.0f)  // 固定静态功率（换板子也会影响该数据）

//TODO:卡尔曼滤波器参数调这儿
#define ACCEL_NOISE_STD 0.2f    //越大：更信正解算速度，跟随快但噪声大；越小：更信 IMU 积分，平滑但响应慢
#define VELOCITY_NOISE_STD 0.2f //越大： 更信 IMU，抗打滑能力强；越小：更信正解算，跟随快
#define PID_CAL_PERIOD 1

extern Vector2_t fused_speed_xy;

void Fix_Speed_pid_init(void);
/*电机初始化*/
int chassis_motor_init();
int gim_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);
void Forward_Kinematics(void);

#endif