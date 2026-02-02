/**
 * @file drv_Time.c
 * @brief 该文件用于时间相关的定义
 * @author fwlh
 * @version 1.0
 * @date 2021-11-30
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_Time.h"

#include "stdlib.h"
#include "drv_Time.h"
#include "drv_BridgeMath.h"

// 全套代码使用的时间变量
static Time_Struct_t Global_Time = {0, (uint32_t)(-1), F};

/**
 * @brief 获取一个当前时间的浅拷贝
 * @author fwlh
 * @param  time             需要被拷贝的时间
 * @return Time_Struct_t*   拷贝得到的时间结构体
 */
Time_Struct_t *Copy_Time(const Time_Struct_t *time)
{
    if (time == NULL)
        return NULL;
    Time_Struct_t *copy = (Time_Struct_t *)malloc(sizeof(Time_Struct_t));
    if (copy)
        return copy;
    else
        return NULL;
}

/**
 * @brief 时间片发生运转
 * @author fwlh
 * @param  void
 */
void Time_Run(void)
{
    ++Global_Time.Timer_Count;
}

/**
 * @brief 读取当前的时间
 * @author fwlh
 * @return uint32_t
 */
uint32_t Read_Current_Time(void)
{
    return Global_Time.Timer_Count;
}

/**
 * @brief 获取全局时间的配置参数
 * @author fwlh
 * @return Time_Struct_t*   全局时间的参数结构体
 */
Time_Struct_t *Read_Time_Parameter(void)
{
    return &Global_Time;
}

/**
 * @brief 传入两个时间结构体可以返回时间 1 和时间 2 之间的时间差即前者比后者早了多长时间
 * @author fwlh
 * @param  time1            需要比较的第一个时间
 * @param  time2            需要比较的第二个时间
 * @return float            数值单位为秒
 */
float Time_Struct_Compare(Time_Struct_t *time1, Time_Struct_t *time2)
{
    // 判断两个时间数据是否具有可比性
    assert_param(time1->Timer_Count_Max == time2->Timer_Count_Max);
    assert_param(time1->Count_Per_Second == time2->Count_Per_Second);

    float time_difference = 0;

    // 判断是否出现计数值越界回环情况
    if ((time1->Timer_Count > (time1->Timer_Count_Max / 2)) ==
        (time2->Timer_Count > (time2->Timer_Count_Max / 2)))
        time_difference = (time1->Timer_Count - time2->Timer_Count) * 1.0 / time1->Count_Per_Second;
    else
        time_difference = (time1->Timer_Count - time2->Timer_Count + time1->Timer_Count_Max) * 1.0 / time1->Count_Per_Second;
    return time_difference;
}

/**
 * @brief 传入两个时间可以返回时间 1 和时间 2 之间的时间差即前者比后者早了多长时间
 * @author fwlh
 * @param  time1            需要比较的第一个时间
 * @param  time2            需要比较的第二个时间
 * @return float            前者比后者早了多少时间
 */
float Time_Compare(Time_Struct_t *time1, uint32_t time2)
{
    float time_difference = 0;

    // 判断是否出现计数值越界回环情况
    if ((time1->Timer_Count > (time1->Timer_Count_Max / 2)) ==
        (time2 > (time1->Timer_Count_Max / 2)))
        time_difference = (time1->Timer_Count - time2) / time1->Count_Per_Second;
    else
        time_difference = (time1->Timer_Count - time2 + time1->Timer_Count_Max) / time1->Count_Per_Second;
    return time_difference;
}
