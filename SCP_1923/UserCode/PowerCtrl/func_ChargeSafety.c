/**
 * @file drv_ChargeSafety.c
 * @brief 该文件用于监视处理充电的安全性问题
 * @author fwlh
 * @version 1.0
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "func_ChargeSafety.h"

#include "drv_Time.h"

#include "hrtim.h"

#include <stdlib.h>

#include "drv_ADCSense.h"

#include "GeneralMacros.h"

#define ABS(a) ((a > 0) ? a : -a)
// 这个阈值比较器会在输入数值 a 大于阈值的绝对值时输出 1 , 小于阈值绝对值时输出 -1 , 否则输出 0
#define THRESHOLD_COMPARE(a, threshold) ((a > ABS(threshold)) ? 1 : ((a < -ABS(threshold)) ? -1 : 0))

const float I_referee_min = 0.1;               // 判断裁判系统端的电流是否为“比较大”的阈值
const float I_motor_min = 0.1;                 // 判断裁判底盘的电流是否为“比较大”的阈值
const float DisCharge_Recovering_Times = 0.1f; // 上次判断完底盘没有断电以后 100ms 才需要重新判断

static void (*Charge_System_Stop)(void *param) = NULL;    // 关闭充电的函数指针
static void (*Charge_System_Restart)(void *param) = NULL; // 重新开始充电的函数指针

static Charge_System_Permission_Check_Structure DisCharge_Check_Status; // 用于记录底盘供电状态的结构体类型
static uint8_t PowerOnSelfTest_Result = 0;                              // 上电自检的结果, 值为 1 代表自检成功

/**
 * @brief 该函数用于初始化充电准许控制系统
 * @author fwlh
 * @param  ChargeSystemStop       关闭充电的函数指针
 * @param  ChargeSystemRestart    重新开始充电的函数指针
 * @return void
 */
void Charge_System_Check_Init(void (*ChargeSystemStop)(void *param),
                              void (*ChargeSystemRestart)(void *param))
{
    Charge_System_Stop = ChargeSystemStop;
    Charge_System_Restart = ChargeSystemRestart;

    DisCharge_Check_Status.Charge_Permission_Flag = 0;
    DisCharge_Check_Status.Charge_UnPermission_Times = 0;
    DisCharge_Check_Status.DisCharge_Attempt_Status = 0;
    DisCharge_Check_Status.DisCharge_Attempt_Judge = 0;

    DisCharge_Check_Status.DisCharge_LastCheck_Time = (Time_Struct_t *)malloc(sizeof(Time_Struct_t));
    if (DisCharge_Check_Status.DisCharge_LastCheck_Time == NULL)
        while (1)
            continue;
    DisCharge_Check_Status.DisCharge_LastCheck_Time->Count_Per_Second = 8000;
    DisCharge_Check_Status.DisCharge_LastCheck_Time->Timer_Count_Max = (uint32_t)(-1);
    DisCharge_Check_Status.DisCharge_LastCheck_Time->Timer_Count = Read_Current_Time();

    DisCharge_Check_Status.DisCharge_Attempt_Time = (Time_Struct_t *)malloc(sizeof(Time_Struct_t));
    if (DisCharge_Check_Status.DisCharge_Attempt_Time == NULL)
        while (1)
            continue;
    DisCharge_Check_Status.DisCharge_Attempt_Time->Count_Per_Second = 8000;
    DisCharge_Check_Status.DisCharge_Attempt_Time->Timer_Count_Max = (uint32_t)(-1);
    DisCharge_Check_Status.DisCharge_Attempt_Time->Timer_Count = Read_Current_Time();
}

/**
 * @brief (堵塞进行)上电自检(两个下桥打开测试源端上桥是否短路, 源端上桥打开测试剩下三个 MOS 是否短路)
 * @author fwlh
 * @return int8_t          返回值为 0 代表自检过程正常进行但是自检不通过, 为 1 代表自检通过, 为 -1 代表欠压无法自检
 */
int8_t Power_On_Self_Test(void)
{
#if (SELFCHECK_ALWAYS_PASS == 0)
    // TODO: 准备进行自检
    ADC_DataOut_s ADC_RAW = {0};
    PowerOnSelfTest_Result = 0;

    // 先初始化 H 桥对应的 GPIO
    HAL_HRTIM_MspDeInit(&hhrtim1);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 关闭所有半桥进行 ADC 数据的检测
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_RESET);
    ADC_Routine(0);
    ADC_FreshOutData(&ADC1Data, &ADC2Data, &ADC_RAW);
    // 电源端电压低于 10V 说明欠压不能通过自检
    if (ADC_RAW.U_IN < 10)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_RESET);
        return -1;
    }
    // 电源端输入电流大于 500mA 说明应该至少有两个 MOS 击穿了
    if (ABS(ADC_RAW.I_IN) > 0.5f)
        return 0;

    // 关闭两个上桥打开两路下桥进行 ADC 数据的检测
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_RESET);
    ADC_Routine(0);
    ADC_FreshOutData(&ADC1Data, &ADC2Data, &ADC_RAW);
    // 电源端输入电流大于 500mA 说明应该是电源端上桥击穿了
    if (ABS(ADC_RAW.I_IN) > 0.5f)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_RESET);
        return 0;
    }

    // 只打开电源端上桥进行 ADC 数据的检测
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
    ADC_Routine(0);
    ADC_FreshOutData(&ADC1Data, &ADC2Data, &ADC_RAW);
    // 电源端输入电流大于 500mA 说明应该是剩下三个 MOS 至少击穿了一个
    if (ABS(ADC_RAW.I_IN) > 0.5f)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_10, GPIO_PIN_RESET);
        return 0;
    }

    // 自检完毕, 重新初始化 HRTIM 定时器
    HAL_HRTIM_MspDeInit(&hhrtim1);
    MX_HRTIM1_Init();
#endif
    // 置位自检成功标志位
    PowerOnSelfTest_Result = 1;
    return 1;
}

/**
 * @brief 外部查询自检是否成功
 * @author fwlh
 * @return uint8_t          返回值为 1 代表自检成功
 */
uint8_t Inquire_PowerOnSelfTest_Result(void)
{
#if (SELFCHECK_ALWAYS_PASS)
    PowerOnSelfTest_Result = 1;
#endif
    return PowerOnSelfTest_Result;
}

/**
 * @brief 进行充电允许性的判断
 * @author fwlh
 * @param  I_cap            超级电容端的电流, 超级电容输出为正方向
 * @param  I_referee        裁判系统端的电流, 裁判系统输出为正方向
 * @return uint8_t          返回 0 说明不允许充电
 */
uint8_t Charge_System_Permission_Check(float I_cap, float I_referee)
{
    float I_motor = I_referee + I_cap; // 底盘端的电流, 输入到底盘为正
    DisCharge_Check_Status.DisCharge_Attempt_Judge = 0;

    // 如果现在处于尝试断电的状态, 判断一下断电时间就可以直接返回了
    if (DisCharge_Check_Status.DisCharge_Attempt_Status)
    {
        // 时间还没到就直接返回不允许通电的信号
        if (Time_Compare(DisCharge_Check_Status.DisCharge_Attempt_Time, Read_Current_Time()) < 0.01f)
        {
            DisCharge_Check_Status.Charge_UnPermission_Times = 0;
            return DisCharge_Check_Status.Charge_Permission_Flag;
        }
        // 时间到了就恢复通电
        else
        {
            DisCharge_Check_Status.DisCharge_Attempt_Status = 0;
            DisCharge_Check_Status.Charge_Permission_Flag = 1;
            DisCharge_Check_Status.DisCharge_LastCheck_Time->Timer_Count = Read_Current_Time();
            (*Charge_System_Restart)(NULL);
        }
    }
    // 判断一下是不是不久之前刚进行过尝试断电
    else if (Time_Compare(DisCharge_Check_Status.DisCharge_LastCheck_Time, Read_Current_Time()) < 0.1f)
    {
        return DisCharge_Check_Status.Charge_Permission_Flag;
    }

    // 进行电流的判断是不是处于异常情况
    switch (THRESHOLD_COMPARE(I_referee, I_referee_min))
    {
    case 1:
        // 裁判系统正电流比较大, 显然此时裁判系统正在正常供电, 所以正常充电即可
        if (!DisCharge_Check_Status.Charge_UnPermission_Times)
            --DisCharge_Check_Status.Charge_UnPermission_Times;
        break;
    case -1:
        // 裁判系统负电流比较大, 就对底盘电流进行判断
        switch (THRESHOLD_COMPARE(I_motor, I_motor_min))
        {
        case 0:
            // 底盘用电电流接近于 0 说明超级电容在单独给裁判系统供电, 应该要断电
            ++DisCharge_Check_Status.Charge_UnPermission_Times;
            if (DisCharge_Check_Status.Charge_UnPermission_Times > 6000)
                DisCharge_Check_Status.Charge_UnPermission_Times = 6000;
            break;
        default:
            // 底盘用电电流绝对值远大于 0 说明底盘在减速, 同时超级电容充满了, 所以保持正常充电
            if (!DisCharge_Check_Status.Charge_UnPermission_Times)
                --DisCharge_Check_Status.Charge_UnPermission_Times;
            break;
        }
        break;
    case 0:
        // 裁判系统电流绝对值比较小, 说明超级电容电流和底盘电流比较接近, 单独判断底盘电流
        switch (THRESHOLD_COMPARE(I_motor, I_motor_min))
        {
        case 1:
            // 底盘在用电但是只有超级电容在供电, 说明裁判系统可能已经断电了, 超级电容需要断电
            ++DisCharge_Check_Status.Charge_UnPermission_Times;
            if (DisCharge_Check_Status.Charge_UnPermission_Times > 6000)
                DisCharge_Check_Status.Charge_UnPermission_Times = 6000;
            break;
        case -1:
            // 如果底盘电流远小于 0 说明底盘在减速但是超级电容在充电, 保持正常充电即可
            if (!DisCharge_Check_Status.Charge_UnPermission_Times)
            {
                --DisCharge_Check_Status.Charge_UnPermission_Times;
            }
            break;
        case 0:
            // 底盘电流和超级电容电流都比较小, 难以判断, 需要尝试性断电
            DisCharge_Check_Status.DisCharge_Attempt_Judge = 1;
            ++DisCharge_Check_Status.Charge_UnPermission_Times;
            if (DisCharge_Check_Status.Charge_UnPermission_Times > 6000)
                DisCharge_Check_Status.Charge_UnPermission_Times = 6000;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    // 进行滞回比较, 是否允许充电的标志位相当于记录了当前进行比较的阈值
    // 上一次不允许充电的通电请求需要当前计数值小于 1000
    if ((!DisCharge_Check_Status.Charge_Permission_Flag) &&
        (DisCharge_Check_Status.Charge_UnPermission_Times < 1000))
    {
        DisCharge_Check_Status.Charge_Permission_Flag = 1;
        (*Charge_System_Restart)(NULL);
    }
    // 上一次允许充电的断电请求需要计数值大于 5000
    else if ((DisCharge_Check_Status.Charge_Permission_Flag) &&
             (DisCharge_Check_Status.Charge_UnPermission_Times > 5000))
    {
        DisCharge_Check_Status.Charge_Permission_Flag = 0;
        (*Charge_System_Stop)(NULL);

        // 判断本次断电是不是在尝试性断电, 如果是就记录当前时间
        if (DisCharge_Check_Status.DisCharge_Attempt_Judge)
        {
            DisCharge_Check_Status.DisCharge_Attempt_Status = 1;
            DisCharge_Check_Status.DisCharge_Attempt_Time->Timer_Count = Read_Current_Time();
        }
    }

    return DisCharge_Check_Status.Charge_Permission_Flag;
}

static uint16_t Key_Times = 3000;  // 用于记录按键按下的计数值
static uint8_t return_value = 0;   // 当前预备的返回值
static uint8_t first_run_flag = 0; // 用于记录是否第一次运行的标志位
/**
 * @brief 通过开关判断下一步是不是要继续充电
 * @author fwlh
 * @return uint8_t        返回值为真代表继续充电
 */
uint8_t Key_Protect_Charge(void)
{
    // 首次进入该函数时按照现在的按键状态决定初值以后退出
    if (!first_run_flag)
    {
        ++first_run_flag;
        if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
        {
            return_value = 1;
            Key_Times = 6000;
        }
        else
        {
            return_value = 0;
            Key_Times = 0;
        }
        return return_value;
    }
    // 从第二次进入该函数开始, 就需要按照正常逻辑运行了
    // 按读取按键是否按下
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
    {
        // 按键按下则计数值上涨, 不过要注意限幅
        ++Key_Times;
        if (Key_Times > 6000)
            Key_Times = 6000;
    }
    else if (Key_Times)
        --Key_Times;

    // 进行滞回比较
    if (return_value && (!Key_Times))
    { // 按键松开一段时间而且上次返回为真说明需要在这一次停止充电
        return_value = 0;
        if (Charge_System_Stop)
            Charge_System_Stop(NULL);
    }
    else if ((!return_value) && (Key_Times == 6000))
    { // 按键按下一段时间而且上次返回为假说明需要在这一次开启充电锁
        return_value = 1;
        if (Charge_System_Restart)
            Charge_System_Restart(NULL);
    }

    return return_value;
}
