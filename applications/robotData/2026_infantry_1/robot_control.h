#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "controlMsg.h"
#include "robot_data.h"

// TODO:调Pid这堆
#define RUN_PID_Kp 320
#define RUN_PID_Ki 2
#define RUN_PID_Kd 0.8

#define Power_C_sum (4.0f) // 固定静态功率（换板子也会影响该数据）

#define Wheel_Power_K1 (240.f)     //(4.5e+02) //(1.15e+02) //(1.0e+02)
#define Wheel_Power_K2 (2.009e-07) //(1.0e-07) //(6.0e-07)  //(4.1e-07)
#define Wheel_Power_C (4.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Wheel_Power_C_sum (6.0f)   // 固定静态功率（换板子也会影响该数据）
// TODO 此处参数需要测试修改
#define Steer_Power_K1 (10.f)      //
#define Steer_Power_K2 (0.0000015) //
#define Steer_Power_C (0.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Steer_Power_C_sum (10.0f)  // 固定静态功率（换板子也会影响该数据）

/*电机初始化*/
int chassis_motor_init();
int gim_motor_init();
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw);
void omni_Forward_Kinematics_temp(void);//对底盘速度与角速度正解算
Mot_base_t Get_realmotion(void);//获取正解算后数据
#endif