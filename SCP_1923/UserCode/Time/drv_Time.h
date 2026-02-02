#ifndef __DRV_TIME_H__
#define __DRV_TIME_H__

#include "main.h"

typedef struct
{
    uint32_t Timer_Count;      // 当前的时间计数值
    uint32_t Timer_Count_Max;  // 当前类型数据的最大计数值
    uint32_t Count_Per_Second; // 当前计数频率下每秒钟的计数值
} Time_Struct_t;

extern Time_Struct_t *Copy_Time(const Time_Struct_t *time);
extern void Time_Run(void);
extern Time_Struct_t *Read_Time_Parameter(void);
extern uint32_t Read_Current_Time(void);
extern float Time_Struct_Compare(Time_Struct_t *time1, Time_Struct_t *time2);
extern float Time_Compare(Time_Struct_t *time1, uint32_t time2);

#endif /* __DRV_TIME_H__ */
