#ifndef __FUNC_ADCLINEARCVT_H__
#define __FUNC_ADCLINEARCVT_H__

typedef struct
{
    float Gain;
    float Bias;
} LinearConvert_s; // 数据线性变换

/**
 * @brief  将数据进行标定, 标定公式为 Data*K+B
 * @param  Data             需要标定的数据
 * @param  Fix              标定参数结构体
 * @return float            标定后的结果
 */
static inline float ADCDataGetFix(float Data,
                                  const LinearConvert_s *Fix)
{
    return Data * Fix->Gain + Fix->Bias;
}

/**
 * @brief 将现在测量到的数据转化为实际被测对象的量纲, 全部使用国际单位制
 * @author fwlh
 * @param  Data             本次测量到的数据
 * @param  Convert          数据转换结构体
 * @return float            转换完以后的数据
 */
static inline float ADC_Voltage_ConvertTo_Real(float Data,
                                               const LinearConvert_s *Convert)
{
    return (Data - Convert->Bias) * Convert->Gain;
}

#endif /* __FUNC_ADCLINEARCVT_H__ */
