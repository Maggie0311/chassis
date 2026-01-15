#ifndef __DRV_MECANOMNI_H__
#define __DRV_MECANOMNI_H__
#include <rtthread.h>
#include "controlMsg.h"

#define DAGTORAD ((float)(PI / 180.0f))
#define RADTODAG ((float)(180.0f / PI))
#define RADTORPM ((float)(30.0f / PI))
#define RPMTORAD ((float)(PI / 30.0f))
#define ROOT2 ((float)(1.41421f))
//********************************逆解算部分************************************** */
/// <summary>麦轮解算，单个轮组
///
/// </summary>
/// <param name="target_xyw"></param>
/// <param name="resolve_data"></param>
// void Mecan_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float* resolve_data);

/// <summary>全向轮解算，单个轮组
///
/// </summary>
/// <param name="target_xyw"></param>
/// <param name="resolve_data"></param>
// void Omni_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group,float *resolve_data);

/// <summary>
/// 舵轮解算改进版,要求底盘轮系所在位置必须为正方形底盘
/// </summary>
// void SteerWheel_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float* resolve_data);

/**
 * @brief 底盘运动解算函数，输入底盘的速度和角速度，输出各个电机的速度
 *      * 该函数会根据底盘的轮系类型进行不同的解算
 *      * 向右为x轴正方向，向前为y轴正方向，底盘角速度为正表示底盘逆时针旋转
 * @param Vxyw 速度和角速度，单位mm/s和0.1°/s
 */
void Motion_Resolve(Mot_base_t *Vxyw, float motorSetData[4][2]);
//********************************逆解算部分************************************** */

//********************************正解算部分************************************** */
/// <summary>
/// 获取底盘目前各个电机状态，轮向电机速度，单位mm/s，舵向电机角度，单位0.1°/s，范围[-1800,+1800],向前为0，向左为负，向右为正
/// </summary>
/// <param name=""></param>
void GetEncode(void);
/**
 * @brief 麦轮/全向轮底盘正运动解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方
 * @param {坐标系}底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
 * @param {float} encoderData[4] 编码器得到的电机的数据，快转子的转速(rpm)
 * @param {Mot_base_t} *real_xyw 解算出来的车身的速度
 * @return real_xyw->vy 单位m/s real_xyw->vx 单位mm/s  real_xyw->angvel单位0.1°/s
 */
void fromEncoderGetV(Mot_base_t *real_xyw);
/**
 * @brief 获得底盘实际运动状态
 *
 * @param real_xyw 返回底盘实际运动状态，x,y,w的单位分别为mm/s，mm/s，0.1°/s
 */
void Get_real_motion(Mot_base_t *real_xyw);
//********************************正解算部分************************************** */

#endif
