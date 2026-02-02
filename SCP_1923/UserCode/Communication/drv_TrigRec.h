#ifndef __DRV_TRIGREC_H__
#define __DRV_TRIGREC_H__

/* 用于触发式高速记录数据的软件模块 */

// 用于记录一组触发信息的结构体
typedef struct
{
    int RunFrq_Scale; // 执行指定函数时进行的分频数 设0为不分频
    int ScaleCounter; // 分频计数器
    int TimeToRelease; // 下一次允许触发的时刻
    int Times_Run; // 已运行的次数 第一次运行时为0，以此类推，重新触发时重置为0
    int Times_Left; // 剩余运行次数
    void (*FuncToRun)(int RunCount); // 被触发后执行的函数
} Trig_Recorder_s;

// 传参：输入当前时刻，触发后需要运行的次数，触发释抑时长，要触发的控制块
// 触发一次数据记录
extern int TrigRec_Trig(int TimeNow, int RecNum, int ReleaseTime, Trig_Recorder_s *Recorder);

// 对触发进行处理的需要被定周期轮询的函数
extern int TrigRec_Routine(int TimeNow, Trig_Recorder_s *Recorder);

// 初始化触发控制块
extern void TrigRec_InitStr(Trig_Recorder_s *Recorder, void (*FuncToRun)(int RunCount), int ScaleSet);

#endif
