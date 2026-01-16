#ifndef __DRV_WHEEL_H__
#define __DRV_WHEEL_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "robot_data.h"

#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

/**
 * @brief 更新日期：2025年4月      更新内容：设计通用底盘，合并之前的全向麦轮和舵轮
 * @author Li Jichen
 * 
 */

/**
 * @brief 读取底盘电机当前的离线情况
 * 从低位到高位顺序为舵向电机的RF-LF-LB-RB，轮向电机的RF-LF-LB-RB 对应位置 1 代表离线
 * @return rt_uint8_t
 */
rt_uint8_t Read_Wheels_Offline_State(void);

#endif
