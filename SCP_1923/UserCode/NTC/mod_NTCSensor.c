/**
 * @file mod_NTCSensor.c
 * @brief NTC 电阻电路计算温度
 * @author fwlh
 * @version 1.0
 * @date 2021-12-07
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "mod_NTCSensor.h"

#include "drv_NTCSensor.h"

NTC_Parameter NTC_Param;
float Temperature; // NTC 电阻传感到的温度
static uint8_t NTC_Calc_Prescaler = 1u;
// 用于标记超级电容由于温度过高进入限制模式
static uint8_t Cap_Attenuation = 0;
// 超温回调停机的处理函数
static void (*Stop_Bridge_Action)(void *param);
// 温度降低重新开机的处理函数
static void (*Restart_Bridge_Action)(void *param);

/**
 * @brief 初始化使用的 NTC
 * @author fwlh
 * @param  prescaler              温度计算的分频因子
 * @param  StopBridgeAction       停止 H 桥的函数指针
 * @param  RestartBridgeAction    重新开启 H 桥的函数指针
 */
void NTC_Init(uint8_t prescaler, void (*StopBridgeAction)(void *param),
              void (*RestartBridgeAction)(void *param))
{
    NTCSensor_Init(&NTC_Param, 4150, 298.15, 47);
    NTC_Calc_Prescaler = prescaler;
    Stop_Bridge_Action = StopBridgeAction;
    Restart_Bridge_Action = RestartBridgeAction;
}

/**
 * @brief 根据现在使用的 NTC 电阻电路计算温度
 * @author fwlh
 * @param  ADC_Data         ADC 采集得到的电压数据
 */
void NTC_Temperature_Calc(float ADC_Data_T)
{
    // TODO: 检查一下这里的电阻分压
    // 电阻分压计算
    float NTC_Resistor = ADC_Data_T / ((3.3f - ADC_Data_T) / 10) * 2;
    Temperature = NTCSensor_Temperature_Calc(&NTC_Param, NTC_Resistor) - 273.15f;
}

/**
 * @brief 获取 NTC 电阻的温度
 * @author fwlh
 * @return float            获取到的实际温度
 */
float NTC_Read_Temperature(void)
{
    return Temperature;
}

/**
 * @brief 用于读取超级电容是不是进入了温度限制模式
 * @author fwlh
 * @return uint8_t          返回值为真代表进入限制模式
 */
uint8_t Capatity_Enter_Attenuation(void)
{
    return Cap_Attenuation;
}

// 标志是否已经由于超温度发生了停机
static uint8_t NTC_Stop_Bridge = 0;
// 根据预分频因子判断是否需要进行计算
static uint8_t Calculate_Flag = 0;
// 上一次的返回值, 安全起见设定初始值为 1 已经发生了完全停机事件
static float Return_Value = 0.0f;
/**
 * @brief 进行温度的计算以及温度相关的停机重启等操作
 * @author fwlh
 * @param  ADC_Data_T       MOS 管温度
 * @return float            是否已经发生了停机事件或者衰减事件(发生衰减事件时返回衰减比)
 */
float NTC_Temperature_Operate(float ADC_Data_T)
{
    // 判断本模块是不是被正常初始化了
    if (!Stop_Bridge_Action)
        return 0;

    // 对温度计算次数进行分频
    ++Calculate_Flag;
    if (Calculate_Flag < NTC_Calc_Prescaler)
        return Return_Value;
    else if (Calculate_Flag >= NTC_Calc_Prescaler)
        Calculate_Flag = 0;

    // 进行温度计算
    NTC_Temperature_Calc(ADC_Data_T);

    // 温度超过 75 度就停止 H 桥的正常工作
    if (!NTC_Stop_Bridge && (Temperature > STOP_CRITICAL_TEMPERATURE))
    {
        NTC_Stop_Bridge = 1;
        // 关断 H 桥
        if (Stop_Bridge_Action)
            Stop_Bridge_Action(NULL);
        Return_Value = 1.0f;
    }
    // 温度降低到 65 度以下就重新开机
    else if (Temperature < RESTART_CRITICAL_TEMPERATURE)
    {
        // 当前处于停止模式
        if (NTC_Stop_Bridge)
        {
            NTC_Stop_Bridge = 0;
            // 重新开启 H 桥
            if (Restart_Bridge_Action)
                Restart_Bridge_Action(NULL);
            Return_Value = 0.0f;
        }
        // 当前处于限制模式
        else if (Cap_Attenuation)
        {
            Cap_Attenuation = 0;
            Return_Value = 0.0f;
        }
    }
    // 温度上升到 70 度以上但是没到达 75 度就开始逐渐衰减充放电功率
    else if (!NTC_Stop_Bridge && (Temperature > ATTENUATION_CRITICAL_TEMPREATURE))
    {
        NTC_Stop_Bridge = 0;
        // 向底盘发送信息说明进入限制模式
        Cap_Attenuation = 1;
        Return_Value = (Temperature - ATTENUATION_CRITICAL_TEMPREATURE) / 5.0f;
    }

    return Return_Value;
}
