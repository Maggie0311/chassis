#ifndef __APP_CHASSISCTRL_H__
#define __APP_CHASSISCTRL_H__
#include <rtthread.h>
#include "controlMsg.h"

// 底盘控制线程周期，单位ms


// 底盘控制初始化
rt_err_t Chassis_Ctrl_Init(void);

Mot_base_t Get_Chassis_Set_Speed(void);

#endif
