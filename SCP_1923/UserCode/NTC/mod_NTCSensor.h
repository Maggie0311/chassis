#ifndef __MOD_NTCSENSOR_H__
#define __MOD_NTCSENSOR_H__

#include "main.h"

// 停机的临界温度
#define STOP_CRITICAL_TEMPERATURE 75.0f
// 停机后重新开机的临界温度
#define RESTART_CRITICAL_TEMPERATURE 65.0f
// 开始衰减充放电速率得临界温度
#define ATTENUATION_CRITICAL_TEMPREATURE 70.0f

/**
 * @brief 初始化使用的 NTC
 * @author fwlh
 * @param  prescaler              温度计算的分频因子
 * @param  StopBridgeAction       停止 H 桥的函数指针
 * @param  RestartBridgeAction    重新开启 H 桥的函数指针
 */
extern void NTC_Init(uint8_t prescaler, void (*StopBridgeAction)(void *param),
                     void (*RestartBridgeAction)(void *param));

/**
 * @brief 根据现在使用的 NTC 电阻电路计算温度
 * @author fwlh
 * @param  ADC_Data         ADC 采集得到的电压数据
 */
extern void NTC_Temperature_Calc(float ADC_Data_T);

/**
 * @brief 获取 NTC 电阻的温度
 * @author fwlh
 * @return float            获取到的实际温度
 */
extern float NTC_Read_Temperature(void);

/**
 * @brief 用于读取超级电容是不是进入了温度限制模式
 * @author fwlh
 * @return uint8_t          返回值为真代表进入限制模式
 */
extern uint8_t Capatity_Enter_Attenuation(void);

/**
 * @brief 进行温度的计算以及温度相关的停机重启等操作
 * @author fwlh
 * @param  ADC_Data_T       MOS 管温度
 * @return float            是否已经发生了停机事件或者衰减事件(发生衰减事件时返回衰减比)
 */
extern float NTC_Temperature_Operate(float ADC_Data_T);

#endif
