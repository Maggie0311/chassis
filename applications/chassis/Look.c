#include "Look.h"

static struct rt_timer Look_Timer;   /*线程定时器*/

float Look_Hz[2] = {0.0f};//1是控制线程，2是pid线程
uint32_t Look_Cnt[2] = {0, 0};

static void Look_IRQHandler(void *parameter) //
{
    Look_Hz[0] = (float)Look_Cnt[0];
    Look_Hz[1] = (float)Look_Cnt[1];
    Look_Cnt[0] = 0;
    Look_Cnt[1] = 0;
}

rt_err_t Look_Init(void)
{
    rt_err_t res = 0;
    rt_thread_t thread = RT_NULL;

    // 创建线程定时器 定时释放信号量 PID_Cal_Thread
    rt_timer_init(&Look_Timer,
                  "Look_Timer",
                  Look_IRQHandler,
                  RT_NULL,
                  1000,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);//200Hz

    // 启动定时器
    res = rt_timer_start(&Look_Timer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}