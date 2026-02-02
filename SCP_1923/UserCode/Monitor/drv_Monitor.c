/**
 * @file drv_Monitor.c
 * @brief
 * @author fwlh
 * @version 1.0
 * @date 2021-12-15
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "GeneralMacros.h"
#if (WATCHDOG_ENABLED == 1)

#include "drv_Monitor.h"

// 该数字用于检查是否所有标志位都被置 1 了
#define OBJECT_CHECK_NUMBER ((1 << 4) - 1)

struct Monitoring_Objcet_Structure
{
    unsigned HRTIM_Working_Status : 1;
    unsigned TIM2_Working_Status : 1;
    unsigned TIM7_Working_Status : 1;
    unsigned ADC_Refreshing_Status : 1;
} Monitoring_Objcet_Struct; // 用于标记被监视对象是否有正常发生响应

/**
 * @brief 检查主监视对象是不是都有正常运行, 如果都在正常运行就喂看门狗
 * @author fwlh
 */
void Monitor_Objcet_Check(void)
{
    static uint8_t *const p = (uint8_t *)(&Monitoring_Objcet_Struct);

    // 异或运算会对所有相应的位取反, 如果上述判断为真, 说明还有部分被监视对象没有触发事件
    if ((*p) ^ (OBJECT_CHECK_NUMBER))
    {
        return;
    }
    // 喂独立看门狗
    HAL_IWDG_Refresh(&hiwdg);
    *p = 0;
}

/**
 * @brief 监视器的主监视对象的喂狗事件, 说明正在正常运行
 * @author fwlh
 * @param  Objcet      喂狗的主监视对象
 */
void MOnitor_Objcet_Feed(Monitoring_Objcet_Enum Objcet)
{
    static uint8_t *const p = (uint8_t *)(&Monitoring_Objcet_Struct);
    uint8_t bit = (uint8_t)Objcet;

    *p |= 1 << bit;
}

#endif /* WATCHDOG_ENABLED */
