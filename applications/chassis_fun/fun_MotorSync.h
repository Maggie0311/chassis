#ifndef __DRV_MOTORSYNC_H__
#define __DRV_MOTORSYNC_H__

/* 使用方法：*
先自己定义MotorSyncSettings_S结构体并填好设置
定义MotorSync_Data_S结构体并使用Init函数进行初始化
每次计算转速闭环之前调用Input函数依次写入各个电机当前的相关数据
调用Sync_Alpha_Cal函数计算当前需要的Alpha
下次闭环时，将各个电机的设定值分别乘Alpha后再应用。
*/
#define MOTOR_MAX_SET 4 // 最多对4个电机进行同步

typedef struct
{
    float LowSpeed_Set;
    float LossMax_Set;
    float ADJ_K; // 调整速度系数
    float OverFlow_K;
    float Motor_Count;
    int IDat_Sign;        // 若外部 piderror = set - now 则取1否则为-1
    float Alpha_Max;      // 限制算法运行过程中Alpha的最大值 可取 2
    float Alpha_Min;      // 限制算法运行过程中Alpha的最小值 可取-1
    float Break_K;        // 减速情况下，失控程度变量计算值会取原值的1/Break_K倍
    float Alpha_Filter_K; // alpha 输出值的之后滤波系数, 该系数为新数据的信任比例
    float Speed_Filter_K; // 单电机 error 的滤波系数, 该系数为新数据的信任比例
    float Recover_K;      // alpha 在回调速度(需要在(0,1)之间)
    float IData_damp;     // 额外计算的Idata的衰减系数
    float IData_Max;      // 额外计算的Idata的限幅
    float IData_Min;      // 额外计算的Idata的限幅
    float Input_Gain;     // 输入误差增益
} MotorSyncSettings_S;

typedef struct
{
    float SpeedSetNow;
    float SpeedSenseNow;
    float SpeedSet_BeforeAlpha;
} MotorInfo_Input_S;

typedef struct
{
    float LowSpeedFlag;
    float PIDError;
    float IdataNow;
    float Loss;
    float Motor_SpeedFilterNow;
} MotorInfo_Temp_S;

typedef struct
{
    MotorInfo_Input_S MotorInput[MOTOR_MAX_SET];
    MotorInfo_Temp_S MotorTemp[MOTOR_MAX_SET];
    float Sync_Alpha_Now;
    volatile float Set_Sum;  // 调试时可以使用Jscope观察数值
    volatile float Loss_Sum; // 调试时可以使用Jscope观察数值
} MotorSync_Data_S;

// 单电机数据输入
extern void MotorSync_MotorInput(MotorInfo_Input_S *InputData,
                                 MotorSync_Data_S *SyncData_Str,
                                 int MotorIndex);

extern float Sync_Alpha_Cal(MotorSyncSettings_S *Settings, MotorSync_Data_S *Data);

// 初始化：MotorSync_Data_S* Data
extern void MotorSyncCTRL_Init(MotorSyncSettings_S *Settings, MotorSync_Data_S *Data);

#endif
