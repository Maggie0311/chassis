#include "drv_TrigRec.h"

#ifndef NULL
#define NULL 0
#endif

// 传参：输入当前时刻，触发后需要运行的次数，触发释抑时长，要触发的控制块
// 触发一次数据记录
int TrigRec_Trig(int TimeNow, int RecNum, int ReleaseTime, Trig_Recorder_s* Recorder)
{
    if (Recorder->Times_Left > 0 || Recorder->TimeToRelease > TimeNow)
    {
        // 上次触发还没有完成响应，或触发释抑时间未到
        return -1;
    }

    // 可以正常进行下一次触发
    Recorder->ScaleCounter = 0;
    Recorder->Times_Run = 0; // 已经运行的次数清零
    Recorder->Times_Left = RecNum; // 记录接下来需要运行的次数
    Recorder->TimeToRelease = TimeNow + ReleaseTime; // 刷新触发释抑时刻
    return 0; // 触发正常
}

// 对触发进行处理的需要被定周期轮询的函数
int TrigRec_Routine(int TimeNow, Trig_Recorder_s* Recorder)
{
    if(Recorder->Times_Left>0)
    { // 还需继续执行
        Recorder->ScaleCounter++;
        if(Recorder->ScaleCounter>Recorder->RunFrq_Scale)
        { // 进行分频计算
            Recorder->ScaleCounter = 0;
            Recorder->Times_Left--;
            if (Recorder->FuncToRun != NULL)
            { // 如果设定了需要触发的函数
                Recorder->FuncToRun(Recorder->Times_Run);
            }
            Recorder->Times_Run++;
            return 1; // 执行了一次指定函数
        }
    }
    return 0; // 没有执行指定函数Q
}

// 初始化触发控制块
void TrigRec_InitStr(Trig_Recorder_s *Recorder, void (*FuncToRun)(int RunCount), int ScaleSet)
{
    Recorder->FuncToRun = FuncToRun;
    Recorder->Times_Left = 0;
    Recorder->Times_Run = 0;
    Recorder->TimeToRelease = 0;
    Recorder->ScaleCounter = 0;
    Recorder->RunFrq_Scale = ScaleSet;
}
