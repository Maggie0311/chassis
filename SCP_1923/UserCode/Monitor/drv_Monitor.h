#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#include "main.h"

#include "iwdg.h"

#include "drv_Time.h"

typedef enum
{
    HRTIM_Working = 0U,   // HRTIM 在正常中断
    TIM2_Working,         // TIM2 在正常中断
    TIM7_Working,         // TIM7 在正常中断
    ADC_Refreshing,       // ADC 数据在更新
} Monitoring_Objcet_Enum; // 监视器主要监视的对象, 这些对象没有正常运行就不会喂硬件看门狗了

extern void Monitor_Objcet_Check(void);
extern void MOnitor_Objcet_Feed(Monitoring_Objcet_Enum Objcet);

#endif /* #ifdef __IWDG_H__ */
