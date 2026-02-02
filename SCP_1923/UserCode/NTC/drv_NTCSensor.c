/**
 * @file drv_NTCSensor.c
 * @brief NTC 电阻的温度计算文件
 * @author fwlh
 * @version 1.0
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_NTCSensor.h"

#include "math.h"

const float E = 2.71828;

/**
 * @brief NTC 电阻的初始化
 * @author fwlh
 * @param  NTC_Param        NTC 电阻的数据结构体
 * @param  B                NTC 电阻的 B 参数
 * @param  T                NTC 电阻的原始温度数据, 单位 K
 * @param  R                NTC 电阻的原始电阻数据, 单位 KΩ
 */
void NTCSensor_Init(NTC_Parameter *NTC_Param, float B, float T, float R)
{
    NTC_Param->B = B;
    NTC_Param->R = R;
    NTC_Param->T = T;
}

/**
 * @brief 传入电阻参数计算温度
 * @author fwlh
 * @param  NTC_Param        NTC 电阻的数据结构体
 * @param  R                当前电阻值
 * @return float            
 */
float NTCSensor_Temperature_Calc(NTC_Parameter *NTC_Param, float R)
{
    return 1 / (1 / NTC_Param->T + (logf(R) - logf(NTC_Param->R))/ (NTC_Param->B));
}
