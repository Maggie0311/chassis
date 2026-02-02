#ifndef __DRV_ADCSENSE_H__
#define __DRV_ADCSENSE_H__

#include "main.h"

#define PWM_FREQ 120000.0f                                                     // H桥PWM的频率
#define ADC1_SAMPLE_PERIOD_COUNT 3                                              // 计算使用的采样点对应的PWM周期数
#define ADC1_SAMPLE_RANK 7                                                      // ADC一轮采样的RANK数
#define ADC1_SAMPLE_TIME ((7.5 + 12.5) / 72000000)                              // ADC采样的时间(s)
#define ADC1_CONVERT_TIME (1.0f / PWM_FREQ / ADC1_SAMPLE_TIME / ADC1_SAMPLE_RANK) // 一个PWM周期内ADC可以采样的轮数（7通道）
#define ADC1_CAL_TIMES ((int)(ADC1_SAMPLE_PERIOD_COUNT * ADC1_CONVERT_TIME))      // 用于计算的ADC采样轮数
#define ADC1_BUFFER_LEN (ADC1_CAL_TIMES * 2)                                    // ADC数据缓冲区的大小，单位为采样轮数

// TODO 硬件低通滤波频率（3倍以上）

#define ADC2_SAMPLE_PERIOD_COUNT 3                                                // 计算使用的采样点对应的PWM周期数
#define ADC2_SAMPLE_RANK 7                                                        // ADC一轮采样的RANK数
#define ADC2_SAMPLE_TIME ((7.5 + 12.5) / 72000000)                                // ADC采样的时间(s)
#define ADC2_CONVERT_TIME (1.0f / PWM_FREQ / ADC2_SAMPLE_TIME / ADC2_SAMPLE_RANK) // 一个PWM周期内ADC可以采样的轮数（7通道）
#define ADC2_CAL_TIMES ((int)(ADC2_SAMPLE_PERIOD_COUNT * ADC2_CONVERT_TIME))      // 用于计算的ADC采样轮数
#define ADC2_BUFFER_LEN (ADC2_CAL_TIMES * 2)                                      // ADC数据缓冲区的大小，单位为采样轮数

// #define ADC_SAMPLE_TIMES_LCM 412 // 采样次数的最小公倍数
// #if (((ADC_SAMPLE_TIMES_LCM / 2) % ADC1_SAMPLE_TIMES != 0) || \
//     ((ADC_SAMPLE_TIMES_LCM / 2) % ADC2_SAMPLE_TIMES != 0))
// #error "Please enter the right lcm of sample times!"
// #endif

// 填写ADC能测的的电流的最大值 单位 A
#define ADC_IA_MAX 25
#define ADC_IB_MAX 25

typedef enum
{
    ADC1_DataType_I_L = 0,
    ADC1_DataType_I_BH, // H 桥电源端的电流(注意不是输入电流)
    ADC1_DataType_U_IN,
    ADC1_DataType_U_CAP,

    ADC1_DataType_All,
} ADC1_DataType_e;

typedef enum
{
    ADC2_DataType_I_IN,

    ADC2_DataType_All,
} ADC2_DataType_e;

typedef struct
{
    // 数据被更新的标志位
    uint8_t Data_FreshFlag;
    // 读指针
    uint8_t ReadPos;
    // 写指针
    uint8_t WritePos;
    // 原始数据
    uint16_t RawData[2][ADC1_BUFFER_LEN][ADC1_SAMPLE_RANK];
    // 采样数据求和计算的结果的 2 倍
    uint16_t SampleData_x2[(int)ADC1_DataType_All][ADC1_BUFFER_LEN];
} ADC1_Data_s;

typedef struct
{
    // 数据被更新的标志位
    uint8_t Data_FreshFlag;
    // 读指针
    uint8_t ReadPos;
    // 写指针
    uint8_t WritePos;
    // 原始数据
    uint16_t RawData[2][ADC2_BUFFER_LEN][ADC2_SAMPLE_RANK];
    // 采样数据求和计算的结果的 2 倍
    uint16_t SampleData_x2[(int)ADC2_DataType_All][ADC2_BUFFER_LEN];
} ADC2_Data_s;

typedef enum
{
    DataType_I_L = 0, // 0. 电感电流
    DataType_I_BH,    // 1. H 桥电源端的电流(注意不是输入电流)
    DataType_U_IN,    // 2. 输入端电压
    DataType_U_CAP,   // 3. 电容端电压
    DataType_U_NTC,   // 4. NTC 温度电压
    DataType_I_IN,    // 5. 电源输入电流

    DataType_All,

    DataType_ADC1_Start = DataType_I_L,
    DataType_ADC1_End = DataType_U_CAP,
    DataType_ADC2_Start = DataType_I_IN,
    DataType_ADC2_End = DataType_I_IN,
} Sense_Data_Type_e; // 传感到的数据类型

// 中断回调函数类型
typedef void (*InteruptCallback)(void);

/**
 * @brief 根据 ADC 获得到的原始数据计算得出现在的等效同相位数据信息
 * @author fwlh
 * @param  PWMCount         当前时刻
 */
extern void ADC_GetSynchronous_Data(uint32_t PWMCount);

/**
 * @brief 为 DMA 的传输完成中断回调函数, 这里每次 ADC 转换完成时只有 ADC1 会触发此函数的回调
 * @author fwlh
 * @param  Set_DMA_InteruptCallback  要设置的回调函数
 */
extern void ADC_Sense_SetCallback(InteruptCallback Set_DMA_InteruptCallback);

/**
 * @brief 获得距离本次解算最近一次的 ADC_SAMPLE_TIMES 轮 ADC 采样的数据, 结果以数组形式返回
 * @author fwlh
 * @param  SenseData_Type   需要的数据类型
 * @return float*           答案所在的数组
 */
extern uint16_t *ADC_Read_2xSampleData(Sense_Data_Type_e SenseData_Type);

/**
 * @brief 获取相关数据是否在本周期内被更新过了
 * @author fwlh
 * @param  SenseData_Type   数据类型
 * @return int              返回真代表数据被更新了
 */
extern int Read_ADC_Data_FreshFlag(Sense_Data_Type_e SenseData_Type);

/**
 * @brief 读取 ADC 采到的基准值电压
 *
 */
uint16_t Read_ADC_V_ref_Rawdata(void);

// ADC检测初始化
extern void ADCSense_Init(void);
void TIM15_ADC_TRIGGER(void);
#endif
