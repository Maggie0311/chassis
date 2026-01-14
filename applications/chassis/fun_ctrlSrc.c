#include "fun_ctrlSrc.h"
#include "controlMsg.h"
#include "stddef.h"
#include "fun_protocolGim.h"
/*------------------------------------pub----------------------------------------------------------*/

// 数据源结构体初始化
void Source_Struct_Init(Ctrl_source_t *cs, CtrlSrcNum_e level, uint8_t if_used)
{
    cs->level = level;
    cs->data = NULL;
    cs->if_used = if_used;
    cs->period = SRC_DE_PERIOD;
}

// 设置调度控制块
void ctrlScheduleInit(Ctrl_schedule_t *const Ctrl_schedule,
                      Schedule_mode_e mode, CtrlSrcNum_e src, uint8_t srcNum)
{
    Ctrl_schedule->ctrl_mode = mode;
    Ctrl_schedule->fix_source = src;
    Ctrl_schedule->srcNum = srcNum;
}
/**
 *
 * @brief   数据源有效性规划
 * @brief   将有实时数据的且优先级最大的数据源置为有效数据源：其if_valid会置1,其余数据源的if_valid都置0
 * @return  有效数据源的下标；若无有效数据源，返回0，即无数据源
 * @author  Nanike
 */
// 数据源有效性规划（有实时数据的且优先级最大的数据源为有效数据源），并得到有效数据源的下标 当均无效时返回0数据源
static CtrlSrcNum_e valid_schedule(Ctrl_source_t *Ctrl_sourceArr, CtrlSrcNum_e srcNum)
{
    uint8_t index = 0, temp_max = 0;
    for (int cs = 1; cs < SOURCE_NUM; cs++)
    {
#ifdef CORE_USING_SENTRY // 如果是哨兵
        if (cs == CS_Visual)
        {                                       // 当前是视觉控制源
            if (getIsSentryMode())              // 云台遥控器打开了哨兵模式
                Ctrl_sourceArr[cs].if_used = 1; // 使用视觉控制源(备注：视觉控制源优先级高于云台控制)
            else
            {
                Ctrl_sourceArr[cs].if_used = 0;
                Ctrl_sourceArr[cs].if_valid = 0;
            }
        }
#endif
        // 数据源未使用，跳过
        if (Ctrl_sourceArr[cs].if_used == 0)
            continue;
        // 数据源在使用，先置无效，保证最终只有一个数据源有效
        Ctrl_sourceArr[cs].if_valid = 0;
        // 数据源超时断联，跳过
        if (rt_tick_get() - Ctrl_sourceArr[cs].tick > 100)
            continue;
        // 接收到了实时数据，判断优先级
        if (Ctrl_sourceArr[cs].level > temp_max)
        {
            temp_max = Ctrl_sourceArr[cs].level;
            index = cs;
        }
    }
    Ctrl_sourceArr[index].if_valid = 1;
    return index;
}

// 得到在使用的数据源的最大优先级 当均无效时返回0
uint8_t GetMaxSrcPrio(Ctrl_source_t *Ctrl_sourceArr, CtrlSrcNum_e srcNum)
{
    uint8_t temp_max = 0;

    for (int cs = 0; cs < srcNum; cs++)
    {
        // 数据源未使用，跳过
        if (Ctrl_sourceArr[cs].if_used == 0)
            continue;
        // 判断较大值
        if (Ctrl_sourceArr[cs].level > temp_max)
        {
            temp_max = Ctrl_sourceArr[cs].level;
        }
    }
    return temp_max;
}

// 设置数据源优先级
void Source_Set_Priority(Ctrl_source_t *Ctrl_sourceArr, CtrlSrcNum_e level)
{
    Ctrl_sourceArr->level = level;
}

// 设置当前数据源为最大优先级
void Source_Set_MaxPrio(Ctrl_schedule_t *const Ctrl_schedule, Ctrl_source_t *Ctrl_sourceArr)
{
    // 设置为最大优先级+1
    Ctrl_sourceArr->level = GetMaxSrcPrio(Ctrl_sourceArr, Ctrl_schedule->srcNum) + 1;
}

// 设置数据源是否有效状态
void Source_Set_State(Ctrl_source_t *Ctrl_sourceArr, uint8_t state)
{
    Ctrl_sourceArr->if_valid = state;
}

// 获得调度模式 固定式还是抢占式
Schedule_mode_e Schedule_Get_CtrlMode(const Ctrl_schedule_t *const Ctrl_schedule)
{
    return Ctrl_schedule->ctrl_mode;
}

// 设置调度模式 固定式还是抢占式
void Schedule_Set_CtrlMode(Ctrl_schedule_t *const Ctrl_schedule, Schedule_mode_e mode)
{
    Ctrl_schedule->ctrl_mode = mode;
}

// 获得固定数据源
CtrlSrcNum_e Schedule_Get_FixSource(const Ctrl_schedule_t *const Ctrl_schedule)
{
    return Ctrl_schedule->fix_source;
}

// 设置固定数据源
void Schedule_Set_FixSource(Ctrl_schedule_t *const Ctrl_schedule, CtrlSrcNum_e source)
{
    Ctrl_schedule->fix_source = source;
}

// 设置数据源超时时间 单位ms
void setSrcTimeOut(Ctrl_source_t *const src, uint32_t timeOut)
{
    src->period = timeOut;
}

//  数据源写入数据（）
uint8_t Source_Write_Data(const Ctrl_schedule_t *const Schedule, Ctrl_source_t Ctrl_sourceArr[],
                          uint8_t srcNum, uint16_t cs, void *ctrlData)
{
    if (cs > srcNum)
        return 0;

    Ctrl_sourceArr[cs].data = ctrlData;
    Ctrl_sourceArr[cs].lastTick = Ctrl_sourceArr[cs].tick;
    Ctrl_sourceArr[cs].tick = rt_tick_get();

    return 1;
}

// 获得当前数据源 更新数据源是否有效
Ctrl_source_t *getNowCtrlSrc(const Ctrl_schedule_t *const Schedule,
                             Ctrl_source_t Ctrl_source[], uint8_t srcNum)
{
    Ctrl_source_t *nowCtrlSrc = NULL;
    uint8_t index = 0;
    switch (Schedule->ctrl_mode)
    {
    case PREEMPTIVE: // 抢占式老代码逻辑bug太多了并且写的太复杂，现在完全简化重写了一套
    {
        // 得到有效数据源下标
        index = valid_schedule(Ctrl_source, Schedule->srcNum);
        break;
    }
    case FIXATION: // 固定式代码逻辑还没看，还不知道有没有bug
    {
        index = Schedule->fix_source;

        if (rt_tick_get() - Ctrl_source[index].tick > Ctrl_source[index].period)
        {
            Ctrl_source[index].if_valid = 0;
        }

        break;
    }
    default:
        break;
    }
    // 记录当前数据源
    nowCtrlSrc = &Ctrl_source[index];
    return nowCtrlSrc;
}
