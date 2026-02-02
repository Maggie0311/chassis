#ifndef __DRV_NTCSENSOR_H__
#define __DRV_NTCSENSOR_H__

typedef struct
{
    float B;
    float T;
    float R;
} NTC_Parameter;

// NTC 电阻的温度, 单位 K, 电阻数据, 单位 KΩ
// 初始化一个 NTC 电阻结构体
extern void NTCSensor_Init(NTC_Parameter* NTC_Param,float B,float T,float R);

// 传入当前电阻计算当前温度
extern float NTCSensor_Temperature_Calc(NTC_Parameter *NTC_Param, float R);

#endif /* __DRV_NTCSENSOR_H__ */
