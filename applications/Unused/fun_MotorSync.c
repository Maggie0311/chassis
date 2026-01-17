#include "fun_MotorSync.h"
#include "drv_utils.h"

// 单电机数据输入
void MotorSync_MotorInput(MotorInfo_Input_S *InputData,
                          MotorSync_Data_S *SyncData_Str,
                          int MotorIndex)
{
    SyncData_Str->MotorInput[MotorIndex].SpeedSenseNow = InputData->SpeedSenseNow;
    SyncData_Str->MotorInput[MotorIndex].SpeedSetNow = InputData->SpeedSetNow;
    SyncData_Str->MotorInput[MotorIndex].SpeedSet_BeforeAlpha = InputData->SpeedSet_BeforeAlpha;
}

// 单电机失控程度计算函数
static void MotorSync_Loss_Fresh(MotorInfo_Input_S *InputInfo,
                                 MotorSyncSettings_S *Settings,
                                 MotorInfo_Temp_S *Output_TempData)
{
    int Sign;
    float IdatMULErr_Temp;
    float IdataTemp;

    Output_TempData->Motor_SpeedFilterNow = InputInfo->SpeedSenseNow * Settings->Speed_Filter_K + Output_TempData->Motor_SpeedFilterNow * (1 - Settings->Speed_Filter_K);

    // 判断是否处于低速状态
    if (fabsf(InputInfo->SpeedSetNow) < Settings->LowSpeed_Set)
    {
        Output_TempData->LowSpeedFlag = 1;
    }
    else
    {
        Output_TempData->LowSpeedFlag = 0;
    }
    Output_TempData->PIDError = Settings->Input_Gain * (InputInfo->SpeedSetNow - InputInfo->SpeedSenseNow);

    // 本地计算积分
    IdataTemp = Output_TempData->IdataNow + Output_TempData->PIDError - Settings->IData_damp * Output_TempData->IdataNow;
    if (IdataTemp > Settings->IData_Max)
    {
        IdataTemp = Settings->IData_Max;
    }
    else if (IdataTemp < Settings->IData_Min)
    {
        IdataTemp = Settings->IData_Min;
    }
    Output_TempData->IdataNow = IdataTemp;

    // Alpha增大时，Set增大
    Sign = SIGN_F(Output_TempData->IdataNow) * SIGN_F(InputInfo->SpeedSet_BeforeAlpha);

    // 计算失控程度的数值
    IdatMULErr_Temp = fabsf(Output_TempData->IdataNow);

    Output_TempData->Loss = IdatMULErr_Temp * Sign;
}

float Sync_Alpha_Cal(MotorSyncSettings_S *Settings, MotorSync_Data_S *Data)
{
    int fori;

    float SetSum = 0;
    float Loss_Sum = 0;
    float Loss_Sum_OverFlow;
    float AlphaTemp;

    for (fori = 0; fori < Settings->Motor_Count; fori++)
    {
        // 计算当前各个电机转速设定值绝对值之和 后续计算Alpha时需用
        SetSum += fabsf(Data->MotorInput[fori].SpeedSet_BeforeAlpha);
        // 根据当前输入的数据进行各个电机的失控数值计算
        MotorSync_Loss_Fresh(&Data->MotorInput[fori], Settings, &Data->MotorTemp[fori]);
        Loss_Sum += Data->MotorTemp[fori].Loss;
    }

    Data->Set_Sum = SetSum;

    // 当Loss过大时，对OverFlow_K做相关处理 得到对于不同的Loss_Sum_OverFlow Loss_Sum
    if (Loss_Sum > 0)
    {
        if (Loss_Sum > Settings->LossMax_Set * Settings->OverFlow_K)
        {
            Loss_Sum_OverFlow = Settings->LossMax_Set * Settings->OverFlow_K;
            Loss_Sum = Settings->LossMax_Set;
        }
        else if (Loss_Sum > Settings->LossMax_Set)
        {
            Loss_Sum_OverFlow = Loss_Sum;
            Loss_Sum = Settings->LossMax_Set;
        }
        else
        {
            Loss_Sum_OverFlow = Loss_Sum;
        }
    }
    else
    { // Loss小于零
        if (Loss_Sum < -Settings->LossMax_Set * Settings->OverFlow_K)
        {
            Loss_Sum_OverFlow = -Settings->LossMax_Set * Settings->OverFlow_K;
            Loss_Sum = -Settings->LossMax_Set;
        }
        else if (Loss_Sum < -Settings->LossMax_Set)
        {
            Loss_Sum_OverFlow = Loss_Sum;
            Loss_Sum = -Settings->LossMax_Set;
        }
        else
        {
            Loss_Sum_OverFlow = Loss_Sum;
        }
    }
    Data->Loss_Sum = Loss_Sum;

    if (SetSum <= Settings->LowSpeed_Set * Settings->Motor_Count)
    { // SetSum太小时，取一个稍大的固定的最小值代替
        SetSum = Settings->LowSpeed_Set * Settings->Motor_Count;
    }

    float tempx = (fabsf(Loss_Sum) / Settings->LossMax_Set);
    AlphaTemp = Data->Sync_Alpha_Now - Settings->ADJ_K * (Loss_Sum_OverFlow / SetSum) +
                (1 - Data->Sync_Alpha_Now) * (1 - tempx * tempx * tempx) * Settings->Recover_K;
    if (isnan(AlphaTemp))
    {
        AlphaTemp = 1; // 如果出现问题则进行纠错
    }
    if (AlphaTemp < Settings->Alpha_Min)
    {
        AlphaTemp = Settings->Alpha_Min;
    }
    else if (AlphaTemp > Settings->Alpha_Max)
    {
        AlphaTemp = Settings->Alpha_Max;
    }

    Data->Sync_Alpha_Now = Data->Sync_Alpha_Now * (1 - Settings->Alpha_Filter_K) + AlphaTemp * Settings->Alpha_Filter_K;

    return Data->Sync_Alpha_Now;
}

// 初始化：MotorSync_Data_S* Data
void MotorSyncCTRL_Init(MotorSyncSettings_S *Settings, MotorSync_Data_S *Data)
{
    int fori;
    if (Settings->Motor_Count > MOTOR_MAX_SET)
    {
        while (1)
            continue;
    }
    for (fori = 0; fori < Settings->Motor_Count; fori++)
    {
        Data->MotorInput[fori].SpeedSenseNow = 0;
        Data->MotorInput[fori].SpeedSetNow = 0;
        Data->MotorTemp[fori].IdataNow = 0;
        Data->MotorTemp[fori].Loss = 0;
        Data->MotorTemp[fori].LowSpeedFlag = 1;
        Data->MotorTemp[fori].PIDError = 0;
    }
    Data->Set_Sum = 0;
    Data->Sync_Alpha_Now = 1;
}
