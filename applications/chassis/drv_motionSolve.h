#ifndef __DRV_MECANOMNI_H__
#define __DRV_MECANOMNI_H__
#include <rtthread.h>
#include "controlMsg.h"
#include "robot_data.h"

/*------------------------------------位操作和常量定义宏----------------------------------------------------------*/
#define PI (3.1415926f)
#define DEGTORAD ((float)(PI / 180.0f))
#define RADTODEG ((float)(180.0f / PI))
#define RADTORPM ((float)(30.0f / PI))
#define RPMTORAD ((float)(PI / 30.0f))
#define ROOT2 ((float)(1.41421f))

typedef struct J3_3 {
    float data[3][3];  // 3x3 矩阵
} J3_3;

typedef struct J4_3  //4*3矩阵
{
    float data[4][3];
}J4_3;

typedef struct J3_4 {
    float data[3][4];  // 3x4 矩阵
} J3_4;

//麦克纳姆轮正逆解算
#ifdef USING_MECANUM_RESOLVE

void Mecanum_Forward_Kinematics(float wheel_rpm[4], Mot_base_t* chassis_vel);

void Mecanum_Inverse_Kinematics(Mot_base_t target_xyw, float resolve_data[4]);

#endif
//全向轮正逆解算
#ifdef USING_OMNI_RESOLVE
void Omni_Forward_Kinematics_120(float wheel_rpm[4], Mot_base_t* chassis_vel);

void Omni_Inverse_Kinematics_120(Mot_base_t target_xyw, float resolve_data[4]);
void Omni_Inverse_Kinematics_45(Mot_base_t target_xyw, float resolve_data[4]);
void Omni_Forward_Kinematics_45(float wheel_rpm[4], Mot_base_t *chassis_velocity);


#endif
//舵轮正逆解算
#ifdef USING_STEER_RESOLVE

void Steer_Forward_Kinematics(float wheel_rpm[4], Mot_base_t* chassis_vel);

void Steer_Inverse_Kinematics(Mot_base_t target_xyw, float resolve_data[4]);

#endif

#endif
