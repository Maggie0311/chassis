#ifndef __FUNC_MONHANDLING_H__
#define __FUNC_MONHANDLING_H__

#include <rtthread.h>

// 复位底盘单片机(不要在中断中调用)
extern rt_err_t Robot_Chassis_Reset(rt_bool_t status);
void MX_TIM10_Init(void);
void rtt_ErrorHandle(void);
#endif
