#include "mod_ADCProcess.h"
#include "GeneralMacros.h"
#include "drv_utils.h"
#include "func_ADCLinearCVT.h"
#include "drv_Monitor.h"
#define fabs(x) (x > 0 ? x : -x)

typedef struct
{
    struct
    {
        // 硬件设计决定的由 ADC 测量数据到实际被测对象的电压/电流放大系数
        const LinearConvert_s DataConvert[(int)DataType_All];
        // 默认 ADC 量纲换算
        const LinearConvert_s DefaultFix[(int)DataType_All];
        // ADC 滞后滤波系数, 新数据置信度
        const float DefaultFilter[(int)DataType_All];
    } Data_Process_Param; // 数据处理参数
    struct
    { // 输出的被采样数据(滤波前)(量纲保持为 ADC 原始数据量纲, 有 24 倍的放大)
        uint32_t ADC_Data_Raw[(int)DataType_All];
        // 本次测量到的 ADC 电压(滤波后)(单位: V)
        float ADC_Data_Voltage_Raw[(int)DataType_All];
        // 本次测量到的数据(滤波前且未标定)(单位: V/A)
        float ADC_Data_Out_Raw_Uncalibed[(int)DataType_All];
        // 本次测量到的数据(滤波前但已经标定完成)(单位: V/A)
        float ADC_Data_Out_Raw_Calibed[(int)DataType_All];
        // 本次计算最终输出的被采样数据(滤波后且已经标定完成)(单位: V/A)
        float ADC_Data_Out_Filtered[(int)DataType_All];
    } Data_Field; // 数据域
    struct
    {
        float ADC_Last_Data[(int)DataType_All];
        float ADC_Data_Err;
    } ADC_Compare_Protect;
} ADC_Data_Process_Structure; // ADC 数据处理结构体

ADC_Data_Process_Structure ADC_Data_Process = {
    .Data_Process_Param = {
        .DataConvert = {
            [(int)DataType_I_L] = {.Gain = 1.f / 0.06f, .Bias = 1.65f},
            [(int) DataType_I_BH] = {.Gain = 1.f / 0.15f, .Bias = 1.65f},
            [(int) DataType_U_IN] = {.Gain = 11.f, .Bias = 0.f},
            [(int) DataType_U_CAP] = {.Gain = 11.f, .Bias = 0.f},
            [(int) DataType_I_IN] = {.Gain = 1.f / 0.15f, .Bias = 1.65f},
            [(int) DataType_U_NTC] = {.Gain = 1.f, .Bias = 0.f}},
        .DefaultFix = {
            [(int)DataType_I_L] = {.Bias = 0.f, .Gain = 1.f},
            [(int) DataType_I_BH] = {.Bias = 0.f, .Gain = 1.f},
            [(int) DataType_U_IN] = {.Bias = 0.0f, .Gain = 1.f},
            [(int) DataType_U_CAP] = {.Bias = 0.f, .Gain = 1.f},
            [(int) DataType_I_IN] = {.Bias = 0.f, .Gain = 1.f},
            [(int) DataType_U_NTC] = {.Bias = 0.f, .Gain = 1.f},
        },
        .DefaultFilter = {
            [(int)DataType_I_L] = 0.1f,
            [(int) DataType_I_BH] = 0.8f,
            [(int) DataType_U_IN] = 0.1f,
            [(int) DataType_U_CAP] = 0.1f,
            [(int) DataType_I_IN] = 0.8f,
            [(int) DataType_U_NTC] = 0.1f,
        }},
    .Data_Field = {0},
#if defined ADC_DATA_SEND
    .UART_Debug_Settings = {.Start_Rec_Flag = 0, .ADCDataRec_Ctrl = {0}}
#endif
};

/**
 * @brief ADC 数据的通用处理流程
 * @author fwlh
 * @param  DataType         需要处理的数据类型
 * @param  Data             数据处理结构体
 */
static inline void ADC_Data_General_Process(Sense_Data_Type_e DataType,
                                            ADC_Data_Process_Structure *Data)
{
    int type = (int)DataType;
    // 转化为标准电压
    Data->Data_Field.ADC_Data_Voltage_Raw[type] =
        Data->Data_Field.ADC_Data_Raw[type] * 1.f /
        4096 * 3.3f;
    // 转化为实际的电压数值
    Data->Data_Field.ADC_Data_Out_Raw_Uncalibed[type] =
        ADC_Voltage_ConvertTo_Real(
            Data->Data_Field.ADC_Data_Voltage_Raw[type],
            &Data->Data_Process_Param.DataConvert[type]);
    // 计算标定以后的数据
    Data->Data_Field.ADC_Data_Out_Raw_Calibed[type] = ADCDataGetFix(
        Data->Data_Field.ADC_Data_Out_Raw_Uncalibed[type],
        &Data->Data_Process_Param.DefaultFix[type]);
    // 滞后滤波
    Data->Data_Field.ADC_Data_Out_Filtered[type] = UTILS_LP_FAST(
        Data->Data_Field.ADC_Data_Out_Filtered[type],
        Data->Data_Field.ADC_Data_Out_Raw_Calibed[type],
        Data->Data_Process_Param.DefaultFilter[type]);
}

uint8_t ADC_Refresh_Check(ADC_Data_Process_Structure *Data)
{
    uint8_t Refresh_Check_Result = 1;
    if (Data->Data_Field.ADC_Data_Out_Filtered[DataType_I_IN] == Data->ADC_Compare_Protect.ADC_Last_Data[DataType_I_IN] || Data->Data_Field.ADC_Data_Out_Filtered[DataType_I_BH] == Data->ADC_Compare_Protect.ADC_Last_Data[DataType_I_BH] || Data->Data_Field.ADC_Data_Out_Filtered[DataType_U_IN] == Data->ADC_Compare_Protect.ADC_Last_Data[DataType_U_IN] || Data->Data_Field.ADC_Data_Out_Filtered[DataType_U_CAP] == Data->ADC_Compare_Protect.ADC_Last_Data[DataType_U_CAP])
    {
        Refresh_Check_Result = 0;
    }
    Data->ADC_Compare_Protect.ADC_Last_Data[DataType_I_IN] = Data->Data_Field.ADC_Data_Out_Filtered[DataType_I_IN];
    Data->ADC_Compare_Protect.ADC_Last_Data[DataType_I_BH] = Data->Data_Field.ADC_Data_Out_Filtered[DataType_I_BH];
    Data->ADC_Compare_Protect.ADC_Last_Data[DataType_U_IN] = Data->Data_Field.ADC_Data_Out_Filtered[DataType_U_IN];
    Data->ADC_Compare_Protect.ADC_Last_Data[DataType_U_CAP] = Data->Data_Field.ADC_Data_Out_Filtered[DataType_U_CAP];

    return Refresh_Check_Result;
}

/**
 * @brief 处理 ADC 的原始数据
 * @author fwlh
 * @param  Time             当前的时刻
 */
void ADC_Routine(uint32_t Time)
{
    // ADC 原始数据处理
    ADC_GetSynchronous_Data(Time);

    // 将信号的平均值输出
    if (Read_ADC_Data_FreshFlag(DataType_ADC1_Start)) // ADC1 数据存在更新
    {
        for (int type = (int)DataType_ADC1_Start;
             type <= (int)DataType_ADC1_End; ++type)
        {
            // 计算原始数据的平均值
            register uint32_t sum = 0;
            register int times;
            uint16_t *Read_ADC_DataType = ADC_Read_2xSampleData((Sense_Data_Type_e)type);
            for (times = ADC1_BUFFER_LEN - ADC1_CAL_TIMES; times < ADC1_BUFFER_LEN; ++times)
            {
                sum += Read_ADC_DataType[times];
            }

            // ADC_Data_Process.Data_Field.ADC_Data_Raw[type] = sum / ADC1_SAMPLE_TIMES / 2 / 4;
            ADC_Data_Process.Data_Field.ADC_Data_Raw[type] = sum / 2 / (ADC1_BUFFER_LEN - ADC1_CAL_TIMES);
            // 通用数据处理
            ADC_Data_General_Process((Sense_Data_Type_e)type, &ADC_Data_Process);
        }
        if (Read_ADC_Data_FreshFlag(DataType_ADC2_Start)) // ADC2 数据存在更新
        {
            int type = (int)DataType_ADC2_Start;
            {
                // 计算原始数据的平均值
                register uint32_t sum = 0;
                for (int times = ADC2_BUFFER_LEN - ADC2_CAL_TIMES; times < ADC2_BUFFER_LEN; ++times)
                    sum += ADC_Read_2xSampleData((Sense_Data_Type_e)type)[times];
                ADC_Data_Process.Data_Field.ADC_Data_Raw[type] = sum / 2 / (ADC2_BUFFER_LEN - ADC2_CAL_TIMES);
                // 通用数据处理
                ADC_Data_General_Process((Sense_Data_Type_e)type, &ADC_Data_Process);
            }
        }
#if (WATCHDOG_ENABLED == 1)
        // 喂看门狗
        if (ADC_Refresh_Check(&ADC_Data_Process))
        {
            MOnitor_Objcet_Feed(ADC_Refreshing);
        }
#endif /* WATCHDOG_ENABLED */
    }
}

/**
 * @brief 获得当前的 ADC 计算以后的数据
 * @author fwlh
 * @param  DataType         需要获取的数据类型
 * @param  Methods          数据的处理方式
 * @return float            获得到的实际数据
 */
float Read_Now_Data(Sense_Data_Type_e DataType, ADC_Data_e Methods)
{
    switch (Methods)
    {
    case ADC_Initial_Data:
        return ADC_Data_Process.Data_Field.ADC_Data_Raw[(int)DataType];
    case ADC_Initial_Voltage:
        return ADC_Data_Process.Data_Field.ADC_Data_Voltage_Raw[(int)DataType];
    case ADC_Raw_Data_Uncalibed:
        return ADC_Data_Process.Data_Field.ADC_Data_Out_Raw_Uncalibed[(int)DataType];
    case ADC_Raw_Data_Calibed:
        return ADC_Data_Process.Data_Field.ADC_Data_Out_Raw_Calibed[(int)DataType];
    case ADC_Filted_Data:
        return ADC_Data_Process.Data_Field.ADC_Data_Out_Filtered[(int)DataType];
    default:
        return 0.f;
    }
}

/**
 * @brief 初始化 ADC 数据转换模块
 * @author fwlh
 * @return int     返回真代表初始化失败
 */
int ADC_Module_Init(void)
{
    // 初始化 ADC 硬件
    ADCSense_Init();
    return 0;
}
