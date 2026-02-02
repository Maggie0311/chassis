#include "drv_BridgeMath.h"

// 下属文件
#include "drv_BridgeHW.h"
// 数学类文件
#include <math.h>
#include "drv_utils.h"
// C语言文件
#include <string.h>

// 电感值
// 旧板子30或28
#define INDUCTOR_SET 33e-6f
//  这里内阻为大概估算
//  电容内阻
#define RESISTANCE_CAP_SET 0.107f
// 电源内阻
#define RESISTANCE_BAT_SET 0.05f
// 最大占空比
#define DUTYMAX_SET 0.95f

typedef struct
{
    float Max;
    float Min;
} Lim_s;

// H桥计算相关数据
typedef struct
{
    float Duty_Max;       // 规定的PWM最大占空比
    float Inductor;       // 电感值
    float Resistance_BAT; // 电源端的内阻
    float Resistance_CAP; // 电容端的内阻
    Lim_s Buck_AcclLim;
    Lim_s Boost_AcclLim;
    Lim_s Buck_Boost_AcclLim;
} Bridge_Math_s;

Bridge_Math_s Bridge_Math;

// TODO:deleat it
float Duty_test;

/**
 * @brief 计算Buck状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int BuckA2B_LimCalculate(float U_A, float U_B, float I_B)
{
    float K, MaxLim_CAL, MinLim_CAL;
    float K2DIVIND;
    float Temp;
    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B))
    {
        // 电压传感异常，无法计算
        Bridge_Math.Buck_AcclLim.Max = 0;
        Bridge_Math.Buck_AcclLim.Min = 0;
        return -1;
    }
    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // 计算范围
    K = Bridge_Math.Duty_Max;
    K2DIVIND = K * K / Bridge_Math.Inductor;
    MaxLim_CAL = K2DIVIND * U_B;
    MinLim_CAL = -K2DIVIND * (U_A - U_B);

    if (MaxLim_CAL < MinLim_CAL)
    {
        Temp = MinLim_CAL;
        MinLim_CAL = MaxLim_CAL;
        MaxLim_CAL = Temp;
    }
    // 把单位换成 A
    Bridge_Math.Buck_AcclLim.Max = MaxLim_CAL / F;
    Bridge_Math.Buck_AcclLim.Min = MinLim_CAL / F;
    return 0;
}

/**
 * @brief 获得Buck状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常；-2:传参异常
 */
int BuckA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set)
{
    float BuckDuty; // Buck状态下A侧H通的时长/周期
    float L = Bridge_Math.Inductor;
    float K = Bridge_Math.Duty_Max;
    float temp;
    int ret = 0; // 默认为0

    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B) || isnan(Iaccl_Set))
    {
        ret = -2;
        return ret;
    }

    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // 限幅，确保计算不会出错，若限幅则返回1
    ret = utils_truncate_number(&Iaccl_Set, Bridge_Math.Buck_AcclLim.Min, Bridge_Math.Buck_AcclLim.Max);

    Iaccl_Set *= F; // 对齐单位

    if (fabsf(U_A) < 0.1f)
        // A侧电压很低，此时开关没有意义
        BuckDuty = 0; // 直接不开关，保持低电平
    else
        // A侧电压正常
        // 计算理论占空比
        BuckDuty = (-Iaccl_Set * L / K + U_B * K) / U_A; // 这里对考虑过最大占空比的表达式进行了化简

    if (BuckDuty > K)
    {
        BuckDuty = K;
        ret = -1;
    }
    else if (BuckDuty < 0)
    {
        BuckDuty = 0;
        ret = -1;
    }

    temp = BuckDuty + 1 - K;
    if (temp >= 1)
    {
        temp = 0.98f;
        ret = -1;
    }

    Duty_test = BuckDuty;

    // 按照规定写入TGGL值
    BridgeHW_TGGLWrite(BuckDuty, 0.98f, BuckDuty, temp);

    return ret;
}

/**
 * @brief 获得Boost状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int BoostA2B_LimCalculate(float U_A, float U_B, float I_B)
{
    float K, MaxLim_CAL, MinLim_CAL;
    float K2DIVIND;
    float Temp;

    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B))
    {
        // 电压传感异常，无法计算
        Bridge_Math.Boost_AcclLim.Max = 0;
        Bridge_Math.Boost_AcclLim.Min = 0;
        return -1;
    }

    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // Ub过低时特殊处理
    if (fabsf(U_B) < 0.01f)
        U_B = 0.01f;

    // 计算范围
    K = Bridge_Math.Duty_Max;
    K2DIVIND = K * K / Bridge_Math.Inductor; /* 这里乘了一个 k*k 参考之前的公式推导，除了单纯的上下限外还要考虑低电平的插入时间*/
    MinLim_CAL = -K2DIVIND * U_A * U_A / (4 * U_B);
    MaxLim_CAL = -K2DIVIND * (U_A - U_B);

    if (MaxLim_CAL < MinLim_CAL)
    {
        Temp = MinLim_CAL;
        MinLim_CAL = MaxLim_CAL;
        MaxLim_CAL = Temp;
    }
    // 把单位换成 A
    Bridge_Math.Boost_AcclLim.Max = MaxLim_CAL / F;
    Bridge_Math.Boost_AcclLim.Min = MinLim_CAL / F;
    return 0;
}

/**
 * @brief 获得Boost状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常
 */
int BoostA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set)
{
    float BoostDuty; // Buck状态下A侧H通的时长/周期
    float L = Bridge_Math.Inductor;
    float K = Bridge_Math.Duty_Max;
    float Delta; // 计算判别式
    float temp;

    int ret = 0; // 默认为零

    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B) || isnan(Iaccl_Set))
    {
        ret = -2;
        return ret;
    }

    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // 限幅，确保计算不会出错，若限幅则返回1
    ret = utils_truncate_number(&Iaccl_Set, Bridge_Math.Boost_AcclLim.Min, Bridge_Math.Boost_AcclLim.Max);

    Iaccl_Set *= F; // 对齐单位

    Delta = U_A * U_A + 4 * Iaccl_Set / (K * K) * U_B * L;
    if (Delta < 0)
    {
        Delta = 0;
        ret = -1; // 可能输入的设定值过大，导致无解，此时取最大输出电流对应的占空比
    }

    if (fabsf(U_B) < 0.1f)
        // B侧电压很低，此时开关没有意义
        BoostDuty = K; // 直接保持高电平
    else
    {
        // 电压正常
        // 计算理论占空比
        temp = U_A + sqrtf(Delta);
        if (temp > 0.0001f)
            BoostDuty = K * temp / 2 / U_B;
        else
            BoostDuty = 0;
    }

    if (BoostDuty > K)
    {
        BoostDuty = K;
        ret = -1;
    }

    else if (BoostDuty < 0)
    {
        BoostDuty = 0;
        ret = -1;
    }

    Duty_test = BoostDuty;

    // 换算成输出格式
    BridgeHW_TGGLWrite(K, 0.98f, BoostDuty, 0.98f);

    return ret;
}

/**
 * @brief 获得Buck-Boost状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int Buck_BoostA2B_LimCalculate(float U_A, float U_B, float I_B)
{
    float MaxLim_CAL, MinLim_CAL;
    float Temp;
    float a = BUCK_BOOST_SHIFTDUTY;
    float K = Bridge_Math.Duty_Max;

    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B))
    {
        // 电压传感异常，无法计算
        Bridge_Math.Buck_Boost_AcclLim.Max = 0;
        Bridge_Math.Buck_Boost_AcclLim.Min = 0;
        return -1;
    }

    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // Ub过低时特殊处理
    if (fabsf(U_B) < 0.01f)
        U_B = 0.01f;

    // 计算范围
    MinLim_CAL = a * a * U_B - a * K * U_A;
    MaxLim_CAL = K * K * U_B - a * K * U_A;

    if (MaxLim_CAL < MinLim_CAL)
    {
        Temp = MinLim_CAL;
        MinLim_CAL = MaxLim_CAL;
        MaxLim_CAL = Temp;
    }
    // 把单位换成 A
    Bridge_Math.Buck_Boost_AcclLim.Max = MaxLim_CAL / F / Bridge_Math.Inductor;
    Bridge_Math.Buck_Boost_AcclLim.Min = MinLim_CAL / F / Bridge_Math.Inductor;
    return 0;
}

/**
 * @brief 获得Buck-Boost状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常
 */
int Buck_BoostA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set)
{
    float Buck_BoostDuty; // Buck_Boost 状态下 B 侧 H 通的时长/周期
    float L = Bridge_Math.Inductor;
    float K = Bridge_Math.Duty_Max;
    float Delta; // 计算判别式
    float temp;
    float a = BUCK_BOOST_SHIFTDUTY; // 转换时的截止占空比

    int ret = 0; // 默认为零

    // 检查传参是否正常
    if (isnan(U_A) || isnan(U_B) || isnan(I_B) || isnan(Iaccl_Set))
    {
        ret = -2;
        return ret;
    }

    // 计算内阻影响
    U_A += Bridge_Math.Resistance_CAP * I_B;
    U_B -= Bridge_Math.Resistance_BAT * I_B;

    // 限幅，确保计算不会出错，若限幅则返回1
    ret = utils_truncate_number(&Iaccl_Set, Bridge_Math.Buck_Boost_AcclLim.Min, Bridge_Math.Buck_Boost_AcclLim.Max);

    Iaccl_Set *= F; // 对齐单位

    Delta = U_A * U_A * (K + a) * (K + a) + 4 * Iaccl_Set * (U_A + U_B) * L;
    if (Delta < 0)
    {
        Delta = 0;
        ret = -1; // 可能输入的设定值过大，导致无解，此时取最大输出电流对应的占空比
    }

    // 计算理论占空比
    temp = (2 * a + 2) * U_B + (2 + a - K) * U_A - sqrtf(Delta);
    Buck_BoostDuty = temp / 2 / (U_A + U_B);

    if (Buck_BoostDuty > 0.98f)
    {
        Buck_BoostDuty = 0.98f;
        ret = -1;
    }

    else if (Buck_BoostDuty < (a + 1 - K))
    {
        Buck_BoostDuty = a + 1 - K;
        ret = -1;
    }

    temp = Buck_BoostDuty - 1 + K;
    if (temp < a)
    {
        temp = a;
        ret = -1;
    }

    Duty_test = Buck_BoostDuty;

    // 换算成输出格式
    BridgeHW_TGGLWrite(temp, 0.98f, a, Buck_BoostDuty);

    return ret;
}

/**
 * @brief 获得电流变化率的限幅的最大值
 *
 * @param State 想要获得的状态
 * @return float 电流变化率的限幅的最大值
 */
float Get_AcclLim_Max(BridgeMath_State_e State)
{
    switch (State)
    {
    case BridgeMath_State_Boost:
        return Bridge_Math.Boost_AcclLim.Max;
    case BridgeMath_State_Buck:
        return Bridge_Math.Buck_AcclLim.Max;
    case BridgeMath_State_Buck_Boost:
        return Bridge_Math.Buck_Boost_AcclLim.Max;
    }
    return 0;
}

/**
 * @brief 获得电流变化率的限幅的最小值
 *
 * @param State 想要获得的状态
 * @return float 电流变化率的限幅的最小值
 */
float Get_AcclLim_Min(BridgeMath_State_e State)
{
    switch (State)
    {
    case BridgeMath_State_Boost:
        return Bridge_Math.Boost_AcclLim.Min;
    case BridgeMath_State_Buck:
        return Bridge_Math.Buck_AcclLim.Min;
    case BridgeMath_State_Buck_Boost:
        return Bridge_Math.Buck_Boost_AcclLim.Min;
    }

    return 0;
}

/**
 * @brief 初始化
 */
void BridgeMath_Init(void)
{
    memset(&Bridge_Math, 0, sizeof(Bridge_Math));
    Bridge_Math.Duty_Max = DUTYMAX_SET;
    Bridge_Math.Inductor = INDUCTOR_SET;
    Bridge_Math.Resistance_BAT = RESISTANCE_BAT_SET;
    Bridge_Math.Resistance_CAP = RESISTANCE_CAP_SET;
}
