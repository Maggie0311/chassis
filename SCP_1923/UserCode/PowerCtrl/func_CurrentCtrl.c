#include "func_CurrentCtrl.h"

#include "mod_ADCProcess.h"
#include "func_SafetyProtect.h"
#include "drv_BridgeHW.h"
#include "drv_BridgeMath.h"
#include "hrtim.h"

#include "drv_SetPlanning.h"
#include "drv_Time.h"
#include "func_LED.h"
#include "pid.h"
#include "func_Observer.h"

#include "drv_utils.h"
#include "app_Routine.h"
SetPlanning_Str I_Planning;
SetPlanSettings_Str Planning_Config =
    {
        .Accl_Max = 4000,
        .dt = 1 / F,
        .Speed_Max = 40,
        .POS_Error_Max = 2,
};

Current_Ctrl_s Current_Ctrl =
    {
        .Mode = CurrentCtrl_Mode_Boost,
        .ChassisEnableFlag = 1,
};

/**
 * @brief 修改底盘决定工作的标志位
 *
 * @param Flag 底盘是否工作标志位 1 工作 0 不工作
 */
void Chassis_EnableSet(uint8_t Flag)
{
    switch (Flag)
    {
    case 0:
        Current_Ctrl.ChassisEnableFlag = 0;
        break;
    case 1:
        Current_Ctrl.ChassisEnableFlag = 1;
        break;
    default:
        break;
    }
}

/**
 * @brief 修改电流环设定值，对其进行限幅
 * @param NewSet 电流设定值
 */
void Current_FreshSet(float NewSet)
{
    if (isnan(NewSet))
    { // 如果出现问题则直接设置 0 电流
        Current_Ctrl.I_SetReq = 0;
        return;
    }

    // 对于太大或者太小的设定值进行限幅
    Current_Ctrl.I_SetReq = utils_clamp(
        NewSet, MAX_DISCHARGE_CURRENT_PERMITTED, MAX_CHARGE_CURRENT_PERMITTED);
}

/**
 * @brief 查询目前PWM输出是否关闭
 * @author dty
 * @param type 要查询的保护类型
 * @return uint8_t 1：关闭；0：未关闭
 */
uint8_t IS_PWM_Disable(uint32_t type)
{
    if (Current_Ctrl.Close & type)
        return 1;
    else
        return 0;
}

/**
 * @brief PWM输出控制（上下桥完全关断）
 * @author dty
 * @param type 类型
 * @param state 0：关闭；1：打开。
 */
void PWM_Output_Ctrl(uint32_t type, uint8_t state)
{

    if (state && (Current_Ctrl.Close & type))
    { // 要打开PWM输出
        // 复位当前的标志位
        Current_Ctrl.Close &= (~type);
        // 查看是否仍被关闭
        if (Current_Ctrl.Close)
            return;
        HRTIM_Enable_Output(NULL);
        pid_clear(&Current_Ctrl.CurrentPID);
#ifdef LED_ONCAR
        LED_Ctrl(1, 3, LED_ON, 1);
        LED_Ctrl(2, 3, LED_OFF, 1);
#endif
    }
    else if ((!state) && (!(Current_Ctrl.Close & type)))
    {
        // 把对应位置1，表示已关闭
        Current_Ctrl.Close |= type;
        HRTIM_Disable_Output(NULL);
#ifdef LED_ONCAR
        LED_Ctrl(1, 3, LED_OFF, 1);
        LED_Ctrl(2, 3, LED_ON, 1);
#endif
    }
}

/**
 * @brief 电容充放电控制
 * @param Max
 * @param Min
 * @param I_SetReq
 */
void CAP_ChargeAndDischarge_Ctrl(float *Max, float *Min, float I_SetReq)
{
    static uint8_t flag = 0; // 用于滞回比较

    // 对U_A中电流带来的升压进行修正
    float U_A = Read_Now_Data(DataType_U_CAP, ADC_Filted_Data);
    float U_B = Read_Now_Data(DataType_U_IN, ADC_Filted_Data);
    *Max = MAX_CHARGE_CURRENT_PERMITTED;
    *Min = MAX_DISCHARGE_CURRENT_PERMITTED;

    // 根据现在的距离电容充满电压的剩余值规划我们最大能充电的电流
    // 计算充电时的最大电流
    float temp = (CTRLSET_UCAP_MAX - U_A) * 3.f;
    // if (temp < 1.5f)
    //     temp = 1.5f;
    if (*Max > temp)
        *Max = temp;
    temp = (U_A - CTRLSET_UCAP_MIN) * 2.f;
		if (temp < 0)
			temp = 0;
    if (*Max > temp)
        *Max = temp;
    // 计算放电时的最大电流
    float temp2 = -U_A / U_B * 15.f;
    temp = (CTRLSET_UCAP_MIN - U_A) * 2.f;
    if (temp < temp2)
    {
        temp = temp2;
    }
    // if (temp < 1.5f)
    //     temp = 1.5f;
    if (*Min < temp)
        *Min = temp;

    switch (flag)
    {
    case 0: // 正常工作情况
        if (U_A > HYSTERESIS_PARAMENT_UMAX)
        {
            flag = 1;
            break;
        }
        else if (U_A < HYSTERESIS_PARAMENT_UMIN)
        {
            flag = 2;
            break;
        }
        else
        {
            break;
        }
    case 1: // 电容满电时的保护措施
        if (U_A > (HYSTERESIS_PARAMENT_UMAX - HYSTERESIS_PARAMENT_OV))
        {
            if (I_SetReq > 0)
            {
#ifdef LED_DEBUG
                LED_Ctrl(1, 1, LED_ON, 1);
#endif
                PWM_Output_Ctrl(CHARGE_CAP_OV, 0);
            }
            else if (I_SetReq < 0)
            {
#ifdef LED_DEBUG
                LED_Ctrl(1, 1, LED_OFF, 1);
#endif
                PWM_Output_Ctrl(CHARGE_CAP_OV, 1);
            }

            break;
        }
        else
        {
            flag = 0;
#ifdef LED_DEBUG
            LED_Ctrl(1, 1, LED_OFF, 1);
#endif
            PWM_Output_Ctrl(CHARGE_CAP_OV, 1);
            break;
        }
    case 2: // 电容低压时的保护措施
        if (U_A < (HYSTERESIS_PARAMENT_UMIN + HYSTERESIS_PARAMENT_UV))
        {
            if (I_SetReq < 0)
            {

                PWM_Output_Ctrl(CHARGE_CAP_UV, 0);
            }
            else if (I_SetReq > 0)
            {

                PWM_Output_Ctrl(CHARGE_CAP_UV, 1);
            }

            break;
        }
        else
        {
            flag = 0;
            PWM_Output_Ctrl(CHARGE_CAP_UV, 1);
            break;
        }
    default:
        break;
    }
}

/**
 * @brief 底盘控制充放电，底盘标志位0不放电，1放电
 *
 * @param I_SetReq 当前设定的电流值
 */
void Chassis_Charge_Ctrl(float I_SetReq)
{
    switch (Current_Ctrl.ChassisEnableFlag)
    {
    case 0:
        // 底盘不让工作，只充不放
        if (I_SetReq < 0)
        {
            PWM_Output_Ctrl(CHASSISCTRL, 0);
        }
        else if (I_SetReq > 0)
        {
            PWM_Output_Ctrl(CHASSISCTRL, 1);
        }
        break;
    case 1:
        // 底盘允许工作，正常充放
        PWM_Output_Ctrl(CHASSISCTRL, 1);
        break;
    case 2:
        // 无线充电模式，只放不充
        if (I_SetReq > 0)
        {
            PWM_Output_Ctrl(CHASSISCTRL, 0);
        }
        else if (I_SetReq < 0)
        {
            PWM_Output_Ctrl(CHASSISCTRL, 1);
        }
        break;
    default:
        // 数据异常，正常充放
        PWM_Output_Ctrl(CHASSISCTRL, 1);
        break;
    }
}

/**
 * @brief 电流环 PID 的限幅比较
 */
static void Current_PID_Limit_Protect(void)
{
    float U_A = Read_U_obs();
    float U_B = Read_Now_Data(DataType_U_IN, ADC_Filted_Data);
    float I_B = Read_Now_Data(DataType_I_BH, ADC_Filted_Data);
    float Accl_Max, Accl_Min; // 电流变化率限幅
    float ISet_Max, ISet_Min; // 电流设定值限幅

    // 按照加载的数据计算Boost能达到的电流加速度范围
    BoostA2B_LimCalculate(U_A, U_B, I_B);
    // 按照加载的数据计算Buck能达到的电流加速度范围
    BuckA2B_LimCalculate(U_A, U_B, I_B);
    // 按照加载的数据计算Buck-Boost能达到的电流加速度范围
    Buck_BoostA2B_LimCalculate(U_A, U_B, I_B);

    // 获取限幅
    Accl_Max = Get_AcclLim_Max(BridgeMath_State_Buck);
    Accl_Min = Get_AcclLim_Min(BridgeMath_State_Boost);

    // 确定 PID 输出的上下限幅
    Current_Ctrl.CurrentPID.out_limit_up = Accl_Max;
    Current_Ctrl.CurrentPID.out_limit_down = Accl_Min;

    // 根据电容充放电控制处理确定当前的电流实际设定值
    CAP_ChargeAndDischarge_Ctrl(&ISet_Max, &ISet_Min, Current_Ctrl.I_SetReq);
    // 根据底盘控制标志决定是否放电
    Chassis_Charge_Ctrl(Current_Ctrl.I_SetReq);
    Current_Ctrl.CurrentPID.set = utils_clamp(Current_Ctrl.I_SetReq, ISet_Min, ISet_Max);
}

/**
 * @brief 分段PID参数
 */
static void Pid_Param_change(void)
{
    if (I_Planning.Output.pos < 0)
    {
        Current_Ctrl.CurrentPID.kp = 1.f;
        Current_Ctrl.CurrentPID.ki = 0.1f;
        Current_Ctrl.CurrentPID.kd = 0.f;

        if (Current_Ctrl.Mode == CurrentCtrl_Mode_Boost)
        {
            Current_Ctrl.CurrentPID.kp = 0.2f;
            Current_Ctrl.CurrentPID.ki = 0.04f;
        }
    }
    else
    {
        Current_Ctrl.CurrentPID.kp = 1.f;
        Current_Ctrl.CurrentPID.ki = 0.1f;
        Current_Ctrl.CurrentPID.kd = 0.f;
    }
}

/**
 * @brief 计算电流变化率
 */
static void Current_Ctrl_Calculate(void)
{
    // 如果PWM输出被关闭，则将设定值设为0；
    if (IS_PWM_Disable(ALL_PWMDISABLE))
        Current_Ctrl.CurrentPID.set = 0;

    // 更新设定值规划数据
    Planning_updata(Read_Now_Data(DataType_I_BH, ADC_Filted_Data), 0, Current_Ctrl.CurrentPID.set, 10000, &I_Planning);

    // 设定值规划计算
    SetPlanning_Cal(&I_Planning);

    // 根据现在的电流方向，更改 PID 参数
    Pid_Param_change();

    // 电流环 PID 计算
    PID_Calc(&Current_Ctrl.CurrentPID,
             I_Planning.Output.pos - Read_Now_Data(DataType_I_BH, ADC_Filted_Data));

    Current_Ctrl.Accl_Now = Current_Ctrl.CurrentPID.out;
}

/**
 * @brief 根据设定的电流值计算输出电流时 H 桥的相关参数，包括模式和实际输出内容如占空比等
 */
static void Current_Output_Calculate(void)
{
    // Boost与Buck的电流变化率限幅交界值
    float Accl_Mid = (Get_AcclLim_Max(BridgeMath_State_Boost) +
                      Get_AcclLim_Min(BridgeMath_State_Buck)) /
                     2;
    float U_A = Read_U_obs();
    float U_B = Read_Now_Data(DataType_U_IN, ADC_Filted_Data);
    float I_B = Read_Now_Data(DataType_I_BH, ADC_Filted_Data);

    // 必须保证 Getduty 函数里有限幅
    switch (Current_Ctrl.Mode)
    { // 这里是做了滞回
    case CurrentCtrl_Mode_Boost:
        if (Current_Ctrl.Accl_Now > Accl_Mid)
            Current_Ctrl.Mode = CurrentCtrl_Mode_Buck_Boost;
        break;
    case CurrentCtrl_Mode_Buck_Boost:
        if (BridgeHW_TGGLRead(0, 1) >= 0.97f)
            Current_Ctrl.Mode = CurrentCtrl_Mode_Boost;
        else if (BridgeHW_TGGLRead(0, 1) <= 0.751f)
            Current_Ctrl.Mode = CurrentCtrl_Mode_Buck;
        break;
    case CurrentCtrl_Mode_Buck:
        if (Current_Ctrl.Accl_Now < Accl_Mid)
            Current_Ctrl.Mode = CurrentCtrl_Mode_Buck_Boost;
        break;
    }

    if (Current_Ctrl.Mode == CurrentCtrl_Mode_Buck)
    {
        BuckA2B_GetDuty(U_A, U_B, I_B, Current_Ctrl.Accl_Now);
#ifdef LED_DEBUG
        LED_Ctrl(2, 1, LED_ON, 1);
        LED_Ctrl(2, 2, LED_OFF, 1);
        LED_Ctrl(2, 3, LED_OFF, 1);
#endif
    }
    else if (Current_Ctrl.Mode == CurrentCtrl_Mode_Buck_Boost)
    {
        Buck_BoostA2B_GetDuty(U_A, U_B, I_B, Current_Ctrl.Accl_Now);
#ifdef LED_DEBUG
        LED_Ctrl(2, 1, LED_OFF, 1);
        LED_Ctrl(2, 2, LED_ON, 1);
        LED_Ctrl(2, 3, LED_OFF, 1);
#endif
    }
    else if (Current_Ctrl.Mode == CurrentCtrl_Mode_Boost)
    {
        BoostA2B_GetDuty(U_A, U_B, I_B, Current_Ctrl.Accl_Now);
#ifdef LED_DEBUG
        LED_Ctrl(2, 1, LED_OFF, 1);
        LED_Ctrl(2, 2, LED_OFF, 1);
        LED_Ctrl(2, 3, LED_ON, 1);
#endif
    }
}

/**
 * @brief 按照设定值进行电流环计算(8k,20230707)
 * @param PWMCount 时刻
 */
void CurrentCtrl_Routine(uint32_t PWMCount)
{
    // 过压、欠压、过流保护
    BreakSituation_Protect(PWMCount);

    // 计算电流环的输出限幅
    Current_PID_Limit_Protect();

    // 计算电流变化率
    Current_Ctrl_Calculate();

    // 根据设定的电流值计算输出电流时 H 桥的相关参数
    Current_Output_Calculate();

    // 驱动 H 桥硬件按照设定方式工作
    BridgeHW_SetPWM(Read_Now_Data(DataType_I_BH, ADC_Filted_Data));

    __disable_irq(); // 禁止全局中断
    while ((hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CNTxR) > (__GLOBAL_HRTIM_PERIOD_COUNT_MAX - REG_WRITE_TIME))
    {
    }
    Bridge_TIM_Set_Comp();
    __enable_irq(); // 使能全局中断
}

/**
 * @brief 电流环初始化
 */
void CurrentCtrl_Init(void)
{
    BridgeHW_Init();

    BridgeMath_Init();

    // 电流闭环控制器初始化
    PID_Init(&Current_Ctrl.CurrentPID,
             1.5f, 0.3f, 0.02f, 0.0f, 40, 50.0f, -50.0f);
    // 电流环设定值规划
    SetPlanning_Init(&I_Planning, &Planning_Config);
    // 默认设定电流为 0
    Current_FreshSet(0.0f);
}

/**
 * @brief 读取 Current_Ctrl 变量
 * @return Current_Ctrl_s*
 */
Current_Ctrl_s *Current_CtrlRead(void)
{
    return &Current_Ctrl;
}
