#include "drv_CPUUsage.h"
#include <main.h>

volatile float CPUUsage = 100.0f;

#define SCOPE_PERIOD_MS 500         // Loading结算周期
#define INIT_PERIOD_MS 20           // 初始化时长

static volatile uint64_t loopCNT;   // 循环计数
static uint32_t msr_tick;           // 下一次结算CPUloading的时刻
static uint64_t loopCNT_MAX;        // SCOPE_PERIOD_MS内的实测最大计数值
static int ReturnA = 0;             // 用于区分初始化和常规测量状态的变量

extern uint32_t HAL_GetTick(void);

int CPUUsage_Scope(void) {
    uint32_t ticknow = HAL_GetTick();
    loopCNT++;
    if (ticknow < msr_tick) {
        return 1;
    }
    else {
        CPUUsage = (1 - (loopCNT / (float)loopCNT_MAX)) * 100.0f;
        msr_tick = ticknow + SCOPE_PERIOD_MS;
        if (ReturnA != 0)
        {
            loopCNT = 0;
        }
        return ReturnA;
    }
}

void CPUUsage_Init(void) {
    uint32_t starttick;
    loopCNT = 0;
    loopCNT_MAX = 1;
    UNUSED(CPUUsage);

    // 等待整数tick
    starttick = HAL_GetTick();
    msr_tick = starttick + 2 + INIT_PERIOD_MS;
    while (HAL_GetTick() < starttick + 2);
    // 循环执行scope函数
    while (CPUUsage_Scope());
    CPUUsage = 100.0f;
    loopCNT_MAX = (int64_t)(loopCNT * (SCOPE_PERIOD_MS / (float)INIT_PERIOD_MS));
    loopCNT = 0;
    ReturnA = 1;
}
