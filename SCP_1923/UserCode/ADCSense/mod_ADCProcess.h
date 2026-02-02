#ifndef __MOD_ADCPROCESS_H__
#define __MOD_ADCPROCESS_H__

#include "drv_ADCSense.h"

typedef enum
{
    ADC_Initial_Data = 0,
    ADC_Initial_Voltage,
    ADC_Raw_Data_Uncalibed,
    ADC_Raw_Data_Calibed,
    ADC_Filted_Data,
} ADC_Data_e;

/**
 * @brief 处理 ADC 的原始数据
 * @author fwlh
 * @param  Time             当前的时刻
 */
extern void ADC_Routine(uint32_t Time);

/**
 * @brief 获得当前的 ADC 计算以后的数据
 * @author fwlh
 * @param  DataType         需要获取的数据类型
 * @param  Methods          数据的处理方式
 * @return float            获得到的实际数据
 */
extern float Read_Now_Data(Sense_Data_Type_e DataType, ADC_Data_e Methods);

/**
 * @brief 初始化 ADC 数据转换模块
 * @author fwlh
 * @return int     返回真代表初始化失败
 */
extern int ADC_Module_Init(void);

#endif /* __MOD_ADCPROCESS_H__ */
