#include "controlMsg.h"

// 控制方式如云台，底盘控制模式等 是否抢占
static Ctrl_schedule_t ctrl_schedulce = {.ctrl_mode = PREEMPTIVE, .fix_source = CS_GIMBAL};
// 控制具体信息 如由云台can发送更新
Ctrl_source_t ctrl_source[SOURCE_NUM];

/*------------------------------------private----------------------------------------------------------*/

/*初始化数据源结构体 安排数据源优先级等*/
void ctrlSrcInit_s(void)
{
    ctrlScheduleInit(&ctrl_schedulce, PREEMPTIVE, CS_GIMBAL, SOURCE_NUM);
    Source_Struct_Init(&ctrl_source[CS_No], 0, 1);

    /*初始化数据源结构体 安排数据源优先级等*/
    Source_Struct_Init(&ctrl_source[CS_GIMBAL], SE_GIMBAL_LEVEL, SE_GIMBAL_IF_USED);
    Source_Struct_Init(&ctrl_source[CS_TEST], SE_TEST_LEVEL, SE_TEST_IF_USED);
    Source_Struct_Init(&ctrl_source[CS_Visual], SE_VISUAL_LEVEL, SE_VISUAL_IF_USED);
    Source_Struct_Init(&ctrl_source[CS_MONITOR], SE_MONITOR_LEVEL, SE_MONITOR_IF_USED);
}

// 获得本文件的控制数据
Ctrl_source_t *getNowCtrlSrc_s(void)
{
    return getNowCtrlSrc(&ctrl_schedulce, ctrl_source, sizeof(ctrl_source) / sizeof(ctrl_source[0]));
}

// 向数据源写入数据 写入有效则返回1
uint8_t Source_WriteData_s(Ctrl_source_e cs, void *ctrlData)
{
    return Source_Write_Data(&ctrl_schedulce, ctrl_source,
                             sizeof(ctrl_source) / sizeof(ctrl_source[0]),
                             cs, ctrlData);
}

// 设置数据源是否有效
void setSourceIsValid_s(Ctrl_source_e cs, uint8_t isvalid)
{
    ctrl_source[cs].if_valid = isvalid;
}

uint8_t getNowCtrlSrcIndex(void)
{
    for (int i = 1; i < SOURCE_NUM; i++)
    {
        if (ctrl_source[i].if_valid == 1)
            return i;
    }
    return 0;
}

rt_tick_t getCtrlSrcTick(Ctrl_source_e cs)
{
    return ctrl_source[cs].tick;
}
