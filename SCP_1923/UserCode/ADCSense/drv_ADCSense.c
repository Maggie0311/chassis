#include "drv_ADCSense.h"

#include "main.h"
#include "hrtim.h"

#include "adc.h"
#include "dma.h"

#include <string.h>
#include "GeneralMacros.h"

#ifdef ADC_RAWDATA_SEND
#include "drv_UartSendCtrl.h"
#include "drv_TrigRec.h"
#include "drv_Time.h"
#include <stdio.h>
#endif

// 存储 ADC 外设的原始和处理中的数据
ADC1_Data_s ADC1Data = {0};
ADC2_Data_s ADC2Data = {0};
#ifdef ADC_RAWDATA_SEND
#define RECORD_TIMES_MAX 30                      // 记录数据的数量
#define SEND_DATA_INDEX 0                        // 发送的数据是采样原始数据数组里面的哪一个下标的数据
static Trig_Recorder_s ADCRawDataRec_Ctrl = {0}; // 用于数据触发记录的模块句柄
static uint8_t start_rec_flag = 0;               // 开始记录数据标志位
#endif

// DMA 中断回调函数(包括半满和全满中断)
static InteruptCallback DMA_InteruptCallback = NULL;

/**
 * @brief 根据 ADC 获得到的原始数据计算得出现在的等效同相位数据信息
 * @author fwlh
 * @param  PWMCount         当前时刻
 */
void ADC_GetSynchronous_Data(uint32_t PWMCount)
{
    // 为了防止读写冲突而设立的中间变量
    uint8_t ADC1ReadPos, ADC2ReadPos;
    ADC1ReadPos = ADC1Data.ReadPos;
    ADC2ReadPos = ADC2Data.ReadPos;
    // 将按照同相位采样得到的数据点融合成为一个数据点
    for (int times = ADC1_BUFFER_LEN - ADC1_CAL_TIMES; times < ADC1_BUFFER_LEN; ++times)
    {
        ADC1Data.SampleData_x2[(int)ADC1_DataType_I_BH][times] =
            ADC1Data.RawData[ADC1ReadPos][times][3] +
            ADC1Data.RawData[ADC1ReadPos][times][4];
        ADC1Data.SampleData_x2[(int)ADC1_DataType_U_IN][times] =
            ADC1Data.RawData[ADC1ReadPos][times][1] +
            ADC1Data.RawData[ADC1ReadPos][times][5];
        ADC1Data.SampleData_x2[(int)ADC1_DataType_U_CAP][times] =
            ADC1Data.RawData[ADC1ReadPos][times][2] +
            ADC1Data.RawData[ADC1ReadPos][times][6];
    }
    ADC1Data.Data_FreshFlag = 1;
    for (int times = ADC2_BUFFER_LEN - ADC2_CAL_TIMES; times < ADC2_BUFFER_LEN; ++times)
    {
        ADC2Data.SampleData_x2[(int)ADC2_DataType_I_IN][times] =
            (ADC2Data.RawData[ADC2ReadPos][times][0] +
             ADC2Data.RawData[ADC2ReadPos][times][1] +
             ADC2Data.RawData[ADC2ReadPos][times][2] +
             ADC2Data.RawData[ADC2ReadPos][times][3] +
             ADC2Data.RawData[ADC2ReadPos][times][4] +
             ADC2Data.RawData[ADC2ReadPos][times][5]) /
            3;
    }
    ADC2Data.Data_FreshFlag = 1;

#ifdef ADC_RAWDATA_SEND
    // 记录发送数据
    if (start_rec_flag)
    {
        start_rec_flag = 0;
        TrigRec_Trig(Read_Current_Time(), RECORD_TIMES_MAX, 5000, &ADCRawDataRec_Ctrl);
    }
    TrigRec_Routine(Read_Current_Time(), &ADCRawDataRec_Ctrl);
#endif
}

/**
 * @brief 获得距离本次解算最近一次的 ADC_SAMPLE_TIMES 轮 ADC 采样的数据, 结果以数组形式返回
 * @author fwlh
 * @param  SenseData_Type   需要的数据类型
 * @return float*           答案所在的数组
 */
__inline uint16_t *ADC_Read_2xSampleData(Sense_Data_Type_e SenseData_Type)
{
    switch (SenseData_Type)
    {
    case DataType_I_L:
        return ADC1Data.SampleData_x2[(int)ADC1_DataType_I_L];
    case DataType_I_BH:
        return ADC1Data.SampleData_x2[(int)ADC1_DataType_I_BH];
    case DataType_U_IN:
        return ADC1Data.SampleData_x2[(int)ADC1_DataType_U_IN];
    case DataType_U_CAP:
        return ADC1Data.SampleData_x2[(int)ADC1_DataType_U_CAP];
    case DataType_I_IN:
        return ADC2Data.SampleData_x2[(int)ADC2_DataType_I_IN];
    default:
        return NULL;
    }
}

/**
 * @brief 获取相关数据是否在本周期内被更新过了
 * @author fwlh
 * @param  SenseData_Type   数据类型
 * @return int              返回真代表数据被更新了
 */
int Read_ADC_Data_FreshFlag(Sense_Data_Type_e SenseData_Type)
{
    switch (SenseData_Type)
    {
    case DataType_I_L:
    case DataType_I_BH:
    case DataType_U_IN:
    case DataType_U_CAP:
        return ADC1Data.Data_FreshFlag;
    case DataType_I_IN:
        return ADC2Data.Data_FreshFlag;
    default:
        return 0;
    }
}

/**
 * @brief 读取 ADC 采到的基准值电压
 *
 */
uint16_t Read_ADC_V_ref_Rawdata(void)
{
    return (ADC1Data.RawData[ADC1Data.ReadPos][3][1] & 0xFFF);
}

/**
 * @brief DMA 的传输完成中断
 * @author fwlh
 * @param  hadc            ADC 外设句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        if (ADC1Data.ReadPos == ADC1Data.WritePos)
        {
        }

        ADC1Data.ReadPos = ADC1Data.WritePos;
        if (DMA_InteruptCallback)
            DMA_InteruptCallback(); // Tasklist1();
    }
    else if (hadc == &hadc2)
    {
    }
}

/**
 * @brief DMA 的传输半完成中断
 * @author fwlh
 * @param  hadc            ADC 外设句柄
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        return;
    }
    else if (hadc == &hadc2)
    {
        ADC2Data.ReadPos = 0;
        // 需要开始处理 ADC2 的数据
    }
}

/**
 * @brief 为 DMA 的传输完成中断回调函数, 这里每次 ADC 转换完成时只有 ADC1 会触发此函数的回调
 * @author fwlh
 * @param  Set_DMA_InteruptCallback  要设置的回调函数
 */
void ADC_Sense_SetCallback(InteruptCallback Set_DMA_InteruptCallback)
{
    if (Set_DMA_InteruptCallback)
        DMA_InteruptCallback = Set_DMA_InteruptCallback;
}

#ifdef ADC_RAWDATA_SEND
/**
 * @brief  发送一轮 ADC 原始数据
 * @author fwlh
 * @param  RunCount         当前计数时刻
 */
static void ADCRawDataSend(int RunCount)
{
    static int index_adc1 = ((SEND_DATA_INDEX < 0 ? 0 : SEND_DATA_INDEX) > ADC1_SAMPLE_TIMES
                                 ? ADC1_SAMPLE_TIMES
                                 : SEND_DATA_INDEX);
    static int index_adc2 = ((SEND_DATA_INDEX < 0 ? 0 : SEND_DATA_INDEX) > ADC2_SAMPLE_TIMES
                                 ? ADC2_SAMPLE_TIMES
                                 : SEND_DATA_INDEX);
    for (int i = (int)DataType_I_L; i < (int)DataType_All; i++)
    {
        // 需要判断数组下标
        if (i <= DataType_ADC1_End)
            Uart1Trans_float(1.f * ADC_Read_2xSampleData((Sense_Data_Type_e)i)[index_adc1]);
        else
            Uart1Trans_float(1.f * ADC_Read_2xSampleData((Sense_Data_Type_e)i)[index_adc2]);
    }
    printf("%c%c%c%c", 0x00, 0x00, 0x80, 0x7f);
}
#endif

void TIM15_ADC_TRIGGER(void)
{
    ADC1Data.WritePos = (ADC1Data.WritePos == 0 ? 1 : 0);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&(ADC1Data.RawData[ADC1Data.WritePos]), sizeof(ADC1Data.RawData[ADC1Data.WritePos]) / sizeof(uint16_t));
}

// ADC检测初始化
void ADCSense_Init(void)
{
    memset(&ADC1Data, 0, sizeof(ADC1Data));
    memset(&ADC2Data, 0, sizeof(ADC2Data));
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_Delay(10);
    // 为了校准读写指针的正确性, 中断里进行一整轮的数据采样, 保证下一次定时器开始的时候写入位置固定为缓冲区头部
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&(ADC1Data.RawData), sizeof(ADC1Data.RawData) / sizeof(uint16_t));
    // HAL_Delay(10);
    ADC2Data.WritePos = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&(ADC2Data.RawData[ADC2Data.WritePos]), sizeof(ADC2Data.RawData[ADC2Data.WritePos]) / sizeof(uint16_t));

#ifdef ADC_RAWDATA_SEND
    TrigRec_InitStr(&ADCRawDataRec_Ctrl, ADCRawDataSend, 0); // 发送不分频
#endif
}
