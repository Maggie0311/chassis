#ifndef __ONLYCHASSIS_H__
#define __ONLYCHASSIS_H__
#include <board.h>

rt_err_t OnlyChassis_Init(void);

typedef struct
{
    struct rt_completion completion;
    rt_timer_t timer; // 定时器
} Synthread1_t;

typedef struct
{
    rt_timer_t timer; // 定时器
    rt_sem_t sem;     // 信号量
} Synthread2_t;

typedef struct
{
    rt_timer_t timer; // 定时器
    struct rt_event event;
} Synthread3_t;

#endif
