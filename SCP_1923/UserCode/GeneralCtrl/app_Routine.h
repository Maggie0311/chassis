#ifndef __APP_ROUTINE_H__
#define __APP_ROUTINE_H__

#include "main.h"

#define DEBUG 0

// 这里注意，因为主定时器和子定时器频率不一样，子定时器的频率是主定时器的四倍，所以这里的周期数也被设置为了主定时器中一个触发周期的四倍
#define __GLOBAL_HRTIM_PERIOD_COUNT_MAX 38400
// 死区时间200nm左右
#define __GLOBAL_HRTIM_DEADTIME_COUNT 700
// 给寄存器赋值的时间
#define REG_WRITE_TIME 11500

/**
 * @brief Init the General Ctrl status.
 * @author fwlh
 */
extern void GeneralCtrl_Init(void);

#endif
