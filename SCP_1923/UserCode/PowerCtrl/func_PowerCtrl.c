/**
 * @file func_PowerCtrl.c
 * @brief 用于控制底盘功率和缓冲能量闭环
 * @author fwlh
 * @version 1.0
 * @date 2021-12-31
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "func_PowerCtrl.h"

#include "func_CurrentCtrl.h"
#include "func_SafetyProtect.h"
#include "mod_ADCProcess.h"
#include "func_LED.h"

#include "hrtim.h"
#include "drv_utils.h"
#include "drv_UartSendCtrl.h"
#include "func_Observer.h"
#include "func_Sense.h"

#include "drv_CAN.h"

static uint8_t utils_truncate_number_uint16_t(uint16_t number, int min, int max)
{
    uint8_t did_trunc = 0;

    if (number > max)
        did_trunc = 1;
    else if (number < min)
        did_trunc = 1;

    return did_trunc;
}
/*2024新*/
// 旧电容组【立式】
// const float halfC = 50.0f / 11.0f / 2; // 电容组电容大小的一半
// const float Fullbattary = 50.0f / 11.0f / 2 * (CTRLSET_UCAP_MAX * CTRLSET_UCAP_MAX - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN);
// 新电容组【平铺式】
const float halfC = 44.f / 10.f / 2; // 电容组电容大小的一半
const float Fullbattary = 44.f / 10.f / 2 * (HYSTERESIS_PARAMENT_UMAX * HYSTERESIS_PARAMENT_UMAX - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN);
/*2024废*/
// const float halfC = 60.f / 9.f / 2; // 电容组电容大小的一半
// const float Fullbattary = 60.f / 9.f / 2 * (CTRLSET_UCAP_MAX * CTRLSET_UCAP_MAX - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN);

/*二代电容组*/
// const float halfC = 60.0f / 11.0f / 2; // 电容组电容大小的一半
// const float Fullbattary = 60.0f / 11.0f / 2 * (CTRLSET_UCAP_MAX * CTRLSET_UCAP_MAX - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN);

Energy_Control_Structure_s Energy_Control =
    {
        .Power_Limit = POWER_SET_LOW,
        .Buffer_Energy = 0.0f,
        .Local_Buffer_Energy = 0.0f,
        .Receive_time.Count_Per_Second = F,
        .Receive_time.Timer_Count_Max = (uint32_t)(-1),
        .Work_state = 1

};

static uint8_t OverTime_Flag = 0;
static uint32_t Last_Receive_TimeCount = 0;
/**
 * @brief 检查通信是不是发生了超时
 * @author fwlh
 * @param  void
 * @return uint8_t          返回值为真代表发生了超时
 */
static uint8_t Communication_Check_Normal(void)
{

    CAN_Receive_Data_t *Receive_Data = Read_Canreceive_Data();

    // 如果目前状态不是超时状态就比较当前时间和上一次接收到数据的时间
    if ((!OverTime_Flag) &&
        (Time_Struct_Compare(Read_Time_Parameter(), &(Receive_Data->Receive_Time)) > OVER_TIME))
    {
        OverTime_Flag = 1;
    }
    // 如果当前状态是超时状态就判断当前记录的上一次收到的时间是不是发生变化
    else if (OverTime_Flag)
    {
        if (Last_Receive_TimeCount != Receive_Data->Receive_Time.Timer_Count)
        {
            OverTime_Flag = 0;
        }
    }

    // 对上一次收到信息的时间进行记录
    Last_Receive_TimeCount = Receive_Data->Receive_Time.Timer_Count;

    return OverTime_Flag;
}

struct
{
    Key_S Key;            // 用于检测状态
    uint8_t Trigger_Flag; // 用于标志PowerOutage触发
    uint8_t EN_Flag;      // 用于表示PowerOutage使能
} PowerOutage;

/**
 * @brief PowerOutage事件检测轮询函数（10hz）
 * @author dty
 */
void PowerOutage_Routine(void)
{
    float I_BH = Read_Now_Data(DataType_I_BH, ADC_Filted_Data);
    float I_IN = Read_Now_Data(DataType_I_IN, ADC_Filted_Data);

    if (Communication_Check_Normal())
    {
        PowerOutage.Trigger_Flag = 1;
    }
    else
    {
        PowerOutage.Trigger_Flag = 0;
    }

    if (PowerOutage.EN_Flag)
    { // 已经认为发生了PowerOutage事件
        // 判断是否要退出PowerOutage事件,即检测Trigger_Flag为 0 的时间
        Key_Rec(&(PowerOutage.Key), !PowerOutage.Trigger_Flag);
        PowerOutage.Trigger_Flag = 0;

        if (Key_GetPress(&(PowerOutage.Key), 1))
        { // 不再持续发生PowerOutage中断
            // 认为没有再发生PowerOutage事件
            PowerOutage.EN_Flag = 0;
            // 打开输出
            PWM_Output_Ctrl(POWEROUTAGEPROTECT, 1);
        }
    }
    else
    { // 未认为发生了PowerOutage事件
        // 判断是否要进入PowerOutage事件,即检测Trigger_Flag为 1 的时间
        Key_Rec(&(PowerOutage.Key), PowerOutage.Trigger_Flag);
        PowerOutage.Trigger_Flag = 0;

        if (Key_GetPress(&(PowerOutage.Key), 1))
        { // 持续发生PowerOutage中断
            // 认为发生了PowerOutage事件
            PowerOutage.EN_Flag = 1;
            // 关闭输出
            PWM_Output_Ctrl(POWEROUTAGEPROTECT, 0);
        }
    }
}

/**
 * @brief 初始化PowerOutage检测(10hz)
 * @author dty
 */
static void PowerOutage_Init(void)
{
    Key_Init(&(PowerOutage.Key), 5, 0, 0);
    PowerOutage.EN_Flag = 0;
    PowerOutage.Trigger_Flag = 0;
}

/**
 * @brief 功率控制过程的初始化
 * @author fwlh
 */
void Power_Control_Init(void)
{
    PID_Init(&Energy_Control.Buffer_Energy_PID,
             10, 0, 0, 0, 0, 125, -250);
    PowerOutage_Init();

    // TODO: 进行通信失败回调函数和缓冲能量计算成功回调函数的注册
}

float set = 0;

/**
* @brief 对功率环进行计算

* @param U_BAT    此时的电源电压
* @param I_IN     此时的输入电流
* @param Energy_set 设定的闭环能量
* @return void
 */
float Power_Output_Sum = 0;
float Power_Cap_Sum = 0;
float V_Cap_Last = 0;
float V_Cap = 0;
float efficiency = 0;
uint8_t stopflag = 0;
static void Power_Current_Control_Calc(float U_BAT, float I_IN, float Energy_set)
{
    static float Last_set = 0;
    //    static int32_t Delay_Time = 0;
    float temp = 0;
    // 对剩余能量进行闭环，输出值为我们所需的功率
    // P = E/t，裁判系统检测频率100Hz，t=0.1s
    PID_Calc(&Energy_Control.Buffer_Energy_PID, Energy_Control.Local_Buffer_Energy - Energy_set);

    if (U_BAT < BAT_UMIN)
    { // 刚上电或欠压，不进行控制
        set = Last_set = 0;
        Current_FreshSet(set);
        return;
    }
    // 不需要这个功能暂时
    // // TODO: 若正在进行飞坡，则最大释放电流
    // if (Read_Canreceive_Data()->Fly_flag == 1)
    // {
    //     Current_FreshSet(MAX_DISCHARGE_CURRENT_PERMITTED);
    //     Delay_Time = 2500;
    //     return;
    // }
    // else if (Delay_Time > 0)
    // {
    //     Current_FreshSet(MAX_DISCHARGE_CURRENT_PERMITTED);
    //     Delay_Time--;
    //     return;
    // }


    // 这里注意各单位的正方向
    // 超级电容功率 = 总限制功率 - 底盘功率 + 回复缓冲能量功率（与PID符号有关）
    temp = (Energy_Control.Buffer_Energy_PID.out + Energy_Control.Power_Limit - U_BAT * I_IN) / U_BAT;

    set = UTILS_LP_FAST(Last_set, (temp + Read_Now_Data(DataType_I_BH, ADC_Filted_Data)), 0.2f);

    Last_set = set;

    // 刷新电流设定值
		if (stopflag && set <0)
			set = 0;		
    Current_FreshSet(set);
    V_Cap_Last = V_Cap;
    V_Cap = Read_Now_Data(DataType_U_CAP, ADC_Filted_Data);
    if (V_Cap > 27)
    {
        Power_Output_Sum = 0;
        Power_Cap_Sum = 0;
    }
    else if (V_Cap < 12)
    {
        efficiency = Power_Output_Sum / (900);
			stopflag = 1;
    }
    else if (set < 0)
    {
        Power_Output_Sum += set * Read_Now_Data(DataType_U_IN, ADC_Filted_Data) * 0.001;
    }
}

/**
@brief 更新本地缓冲能量的计算

@param U_BAT    此时的电源电压
@param I_IN     此时的输入电流
@param F_Power_Ctrl        更新频率
 */
static void Local_Energy_Updata(float U_BAT, float I_IN)
{
    // TODO:这里的缓冲能量参考下面的梳理进行考虑，不要擅自赋值
    Energy_Control.Local_Buffer_Energy += Energy_Control.Power_Limit / F_Power_Ctrl - U_BAT * I_IN / F_Power_Ctrl;
    // 根据是否飞坡更改本地能量的上限
    // if (Read_Canreceive_Data()->Fly_flag)
    //     utils_truncate_number(&Energy_Control.Local_Buffer_Energy, 0, BUFFER_ENERGY_FLY_MAX);
    // else
    utils_truncate_number(&Energy_Control.Local_Buffer_Energy, 0, BUFFER_ENERGY_MAX); // 限幅
}

static void CAN_Data_Load(void)
{
    // TODO:仔细梳理裁判系统数据的准确性
    /**
    * 1.考虑超级电容关机或复位的情况，涉及后续的一系列情况
    // * 2.考虑裁判系统延时（ 800 ms左右 ）
    // * 3.缓冲能量的计算（ 考虑飞坡情况 ）
    // * 4.可以考虑增加发送的数据，实时功率
    // * 5.可以考虑从数据自洽方面入手，根据它更改到底信谁的数据
    // * 6.功率
    // * 7.发送底盘是否超功率
    // * 8.记得改超级电容放电时的下限处理
    * 9.充放电的判断，可以考虑采用继电器等物理方法
    // * 10.work_state处理
    // * 11.发超级电容电量沟通
    *
     */

    static float Power_Before = POWER_SET_LOW / POWER_K_SET;
    // 如果更新数据则，对结构体进行赋值
    if (Read_Canreceive_Data()->Data_Is_New == 1)
    {
        Energy_Control.ACK_Flag = 1;
        /*
        // 检查底盘是否让正常工作
        if (Read_Canreceive_Data()->Work_state == 0)
        { // 底盘不让你工作了
            PWM_Output_Ctrl(CHASSISCTRL, 0);
            Read_Canreceive_Data()->Data_Is_New = 0;
            return;
        }
        else
        {
            PWM_Output_Ctrl(CHASSISCTRL, 1);
        }
        */
        //        if (Read_Canreceive_Data()->Power_HP_Deduction == 1)
        //        { // 底盘功率超限扣血

        //            Energy_Control.Local_Buffer_Energy = BUFFER_ENERGY_SET; // 缓冲能量设为目标值，不闭环
        //            Energy_Control.Power_Limit = POWER_SET_LOW;                        // 功率设定为保守值
        //            Read_Canreceive_Data()->Data_Is_New = 0;
        //            return;
        //        }

        // 更新底盘工作标志
        Chassis_EnableSet(Read_Canreceive_Data()->Work_state);
        // 如果设定功率未超过正常范围
        if (utils_truncate_number_uint16_t(Read_Canreceive_Data()->Power_Set, POWER_SET_LOW, POWER_SET_MAX) == 0)
        {
            Power_Before = Energy_Control.Power_Limit;
            Energy_Control.Power_Limit = Read_Canreceive_Data()->Power_Set;
        }
        else
        {
            Power_Before = Energy_Control.Power_Limit;
            Energy_Control.Power_Limit = POWER_SET_LOW;
        }

        // 如果缓冲能量未超过正常范围,考虑飞坡
        if ((/* (Read_Canreceive_Data()->Fly_flag == 0) && */ (utils_truncate_number_uint16_t(Read_Canreceive_Data()->Energy_Buff, 0, BUFFER_ENERGY_MAX) == 0)) /* ||
            ((Read_Canreceive_Data()->Fly_flag == 1) && (utils_truncate_number_uint16_t(Read_Canreceive_Data()->Energy_Buff, 0, BUFFER_ENERGY_FLY_MAX) == 0)) */
        )
        {

            // 缓冲能量变化不大于最大步长，认为合理
            if (Time_Struct_Compare(&(Energy_Control.Receive_time), &(Read_Canreceive_Data()->Receive_Time)) * POWER_OUT_LIMIT >
                fabsf(Energy_Control.Buffer_Energy - Read_Canreceive_Data()->Energy_Buff))
            {
                // VOFA_Add_SendBuff(Read_Canreceive_Data()->Energy_Buff);
                // VOFA_Add_SendBuff(Energy_Control.Buffer_Energy);

                Energy_Control.Buffer_Energy = Read_Canreceive_Data()->Energy_Buff;
                Energy_Control.Local_Buffer_Energy = Read_Canreceive_Data()->Energy_Buff;
                Energy_Control.Receive_time.Timer_Count = Read_Canreceive_Data()->Receive_Time.Timer_Count;

#ifdef LED_ONCAR
                LED_Ctrl(2, 2, LED_Flash, 100);
#endif
            }
            else
            {
                Energy_Control.Buffer_Energy = Read_Canreceive_Data()->Energy_Buff;

#ifdef LED_ONCAR
                LED_Ctrl(2, 2, LED_OFF, 100);
#endif
            }
        }
        Read_Canreceive_Data()->Data_Is_New = 0;

        return;
    }

    // 检查是否超时
    if (Communication_Check_Normal())
    {

        Energy_Control.Local_Buffer_Energy = BUFFER_ENERGY_SET;  // 缓冲能量设为目标值，不闭环
        Energy_Control.Power_Limit = POWER_K_SET * Power_Before; // 功率限制设为上次的POWER_K_SET
#ifdef LED_ONCAR
        LED_Ctrl(1, 2, LED_OFF, 100);
        LED_Ctrl(2, 2, LED_OFF, 100);
#endif
        return;
    }
}

static uint32_t count = 0;
uint8_t Send_Flag = 0;

static void CAN_Send_CAP_State(float U_CAP, float I_IN, float U_BAT, float I_BH)
{
    static uint8_t data[8];
    static float battary;
    // 计算此时的电量
    battary = (U_CAP * U_CAP - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN) * halfC;
    battary /= Fullbattary;

    if (I_IN < 0)
        I_IN = 0.001;

    // 对电量进行限幅，其值在 0-1 之间
    utils_truncate_number(&battary, 0, 1);

#ifdef LED_ONCAR
    if (battary > 0.9f)
    {
        LED_Ctrl(1, 4, LED_ON, 1);
        LED_Ctrl(2, 4, LED_ON, 1);
    }
    else if (battary > 0.5f)
    {
        LED_Ctrl(1, 4, LED_Flash, (uint16_t)((0.9f - battary) * 1000));
        LED_Ctrl(2, 4, LED_ON, 1);
    }
    else if (battary > 0.1f)
    {
        LED_Ctrl(1, 4, LED_OFF, 1);
        LED_Ctrl(2, 4, LED_Flash, (uint16_t)((0.5f - battary) * 1000));
    }
    else
    {
        LED_Ctrl(1, 4, LED_OFF, 1);
        LED_Ctrl(2, 4, LED_OFF, 1);
    }
#endif

    data[0] = Energy_Control.ACK_Flag | ((Current_CtrlRead()->Close) & 0xfe);
    data[1] = (uint8_t)(battary * 100);
    *((uint16_t *)(&(data[2]))) = (uint16_t)(U_CAP * 1000);
    *((uint16_t *)(&(data[4]))) = (uint16_t)(U_BAT * 1000);
    *((uint16_t *)(&(data[6]))) = (uint16_t)((I_IN) * 1000);

    Energy_Control.ACK_Flag = 0;

    Send_Flag = CAN_Send(data);
}

/**
 * @brief 功率控制循环函数
 *
 */
void Power_Ctrl_Routine(void)
{
    float U_BAT = Read_Now_Data(DataType_U_IN, ADC_Filted_Data);
    float U_CAP = Read_Now_Data(DataType_U_CAP, ADC_Filted_Data);
    if (U_CAP < CTRLSET_UCAP_MIN)
        U_CAP = CTRLSET_UCAP_MIN;
    if (U_CAP > HYSTERESIS_PARAMENT_UMAX)
        U_CAP = HYSTERESIS_PARAMENT_UMAX;

    float I_IN = Read_Now_Data(DataType_I_IN, ADC_Filted_Data);
    float I_BH = Read_Now_Data(DataType_I_BH, ADC_Filted_Data);

    count++;

    Local_Energy_Updata(U_BAT, I_IN);

    // 装载 CAN 收到的数据
    CAN_Data_Load();

    //  此时频率满足要求并且没有发生保护事件，证明电容在正常工作，此时才能发送数据
    if ((count % 10 == 0))
        CAN_Send_CAP_State(U_CAP, I_IN, U_BAT, I_BH);
    // 计算对应的电流设定值
    Power_Current_Control_Calc(U_BAT, I_IN, BUFFER_ENERGY_SET);
    // 发送超级电容信息数据
}

void Battery_Cal(void)
{
    static float battary;

    float U_BAT = Read_Now_Data(DataType_U_IN, ADC_Filted_Data);
    float U_CAP = Read_U_obs();
    float I_IN = Read_Now_Data(DataType_I_IN, ADC_Filted_Data);
    float I_BH = Read_Now_Data(DataType_I_BH, ADC_Filted_Data);

    // 计算此时的电量
    battary = (U_CAP * U_CAP - CTRLSET_UCAP_MIN * CTRLSET_UCAP_MIN) * halfC;
    battary /= Fullbattary;

    // 对电量进行限幅，其值在 0-1 之间
    utils_truncate_number(&battary, 0, 1);

    if (battary > 0.9f)
    {
        LED_Ctrl(1, 4, LED_ON, 1);
        LED_Ctrl(2, 4, LED_ON, 1);
    }
    else if (battary > 0.5f)
    {
        LED_Ctrl(1, 4, LED_Flash, (uint16_t)((0.9f - battary) * 500));
        LED_Ctrl(2, 4, LED_ON, 1);
    }
    else if (battary > 0.1f)
    {
        LED_Ctrl(1, 4, LED_OFF, 1);
        LED_Ctrl(2, 4, LED_Flash, (uint16_t)((0.5f - battary) * 500));
    }
    else
    {
        LED_Ctrl(1, 4, LED_OFF, 1);
        LED_Ctrl(2, 4, LED_OFF, 1);
    }
}
