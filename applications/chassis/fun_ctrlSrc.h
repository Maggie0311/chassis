#ifndef __FUN_CTRL_H__
#define __FUN_CTRL_H__
#include <board.h>

typedef uint16_t CtrlSrcNum_e;
#define SRC_DE_PERIOD 3000

/*调度模式*/
typedef enum
{
    PREEMPTIVE, // 抢占式
    FIXATION,   // 固定不变式

} Schedule_mode_e;

/*控制源对象结构体*/
typedef struct
{
    CtrlSrcNum_e level; // 优先级,
    void *data;         // 控制数据
    uint8_t if_valid;   // 是否有效

    uint32_t period;   // 最大连续多长时间认为数据无效
    uint32_t tick;     // 数据刷新时间
    uint32_t lastTick; // 最近一次有效时间
    uint8_t if_used;   // 是否使用该数据源
} Ctrl_source_t;

/*控制调度器结构体*/
typedef struct
{
    CtrlSrcNum_e srcNum;       // 数据源数量
    Schedule_mode_e ctrl_mode; // 调度模式，固定或者抢占
    CtrlSrcNum_e fix_source;   // 当前使用的控制源

} Ctrl_schedule_t;

// 设置调度控制块
void ctrlScheduleInit(Ctrl_schedule_t *const Ctrl_schedule,
                      Schedule_mode_e mode, CtrlSrcNum_e src, uint8_t srcNum);
void Source_Struct_Init(Ctrl_source_t *cs, CtrlSrcNum_e level, uint8_t if_used);

// 得到在使用的数据源的最大优先级 当均无效时返回0
uint8_t GetMaxSrcPrio(Ctrl_source_t *Ctrl_sourceArr, CtrlSrcNum_e srcNum);

// 设置数据源优先级
void Source_Set_Priority(Ctrl_source_t *Ctrl_sourceArr, CtrlSrcNum_e level);

// 设置当前数据源为最大优先级
void Source_Set_MaxPrio(Ctrl_schedule_t *const Ctrl_schedule, Ctrl_source_t *Ctrl_sourceArr);

// 设置数据源是否有效状态
void Source_Set_State(Ctrl_source_t *Ctrl_sourceArr, uint8_t state);

Schedule_mode_e Schedule_Get_CtrlMode(const Ctrl_schedule_t *const Ctrl_schedule);
void Schedule_Set_CtrlMode(Ctrl_schedule_t *const Ctrl_schedule, Schedule_mode_e mode);
CtrlSrcNum_e Schedule_Get_FixSource(const Ctrl_schedule_t *const Ctrl_schedule);
void Schedule_Set_FixSource(Ctrl_schedule_t *const Ctrl_schedule, CtrlSrcNum_e source);

//  数据源写入数据 置有效
uint8_t Source_Write_Data(const Ctrl_schedule_t *const Schedule, Ctrl_source_t Ctrl_sourceArr[],
                          uint8_t srcNum, uint16_t cs, void *ctrlData);

// 获得当前数据源 更新数据源是否有效
Ctrl_source_t *getNowCtrlSrc(const Ctrl_schedule_t *const Schedule,
                             Ctrl_source_t Ctrl_source[], uint8_t srcNum);
#endif
