#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "controlMsg.h"
#include "robot_data.h"

#define USING_STEER_RESOLVE
int chassis_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw,uint8_t motion_mode);

int gim_motor_init();
void Follow_pid_init(void);

void Motor_Speed_Read();
void Up_Machine_pid_init(void);
int If_Target_Reached(Screw_Motor_e motor,float target);
void Up_Motor_Angle_Read();
void Screw_Motor_Move(Screw_Motor_e motor, float target, uint8_t up_or_down);
int If_Ready_To_Down(float target);
int If_Ready_To_Down_F(float target);
// 云台电机是否大小yaw
#define YAW_MOTOR 0
#define PITCH_MOTOR 1
#define hero

// TODO:调Pid这堆
#define RUN_PID_Kp 70
#define RUN_PID_Ki 0.1
#define RUN_PID_Kd 0.8


#define UP_PID_Kp_Angle 48
#define UP_PID_Ki_Angle 0
#define UP_PID_Kd_Angle 5

#define UP_PID_Kp_Speed 20
#define UP_PID_Ki_Speed 0.02
#define UP_PID_Kd_Speed 0

#define DOWN_PID_Kp_Angle 10
#define DOWN_PID_Ki_Angle 0
#define DOWN_PID_Kd_Angle 5

#define DOWN_PID_Kp_Speed 4
#define DOWN_PID_Ki_Speed 0.01
#define DOWN_PID_Kd_Speed 0


#define UP 0
#define DOWN 1

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
#define WHEELS_GROUP_NUM 4

#define CHASSIS_CTRL_PERIOD 10
#define PID_CAL_PERIOD 5
uint8_t Zero_Check(void);
void chasis_zero_x(void);
void Forward_Kinematics(void);
void Fix_Speed_pid_init(void);
void chasis_dangling_x(void);
#endif