#ifndef __FUNC_SAFETYPROTECT_H__
#define __FUNC_SAFETYPROTECT_H__

#include "main.h"

// 过压欠压停机阈值
#define SET_OCAP (28.5f)
#define SET_UCAP (6.0f)
#define SET_OBAT (30.0f)
#define SET_UBAT (15.0f)
#define SET_OIBH (20.0f)

// 正常范围
#define BAT_UMAX (28.0f)
#define BAT_UMIN (17.0f)
#define IBH_MAX (12.0f)

// 底盘断电停机阈值
#define BAT_OUTAGE (0.05f) // 0.05f
#define CAP_OUTAGE (-0.1f)

/**
 * @brief 异常情况监测
 * @param PWMCount 时刻
 * @return int 0：正常；1：异常
 */
int BreakSituation_Protect(uint32_t PWMCount);

#endif
