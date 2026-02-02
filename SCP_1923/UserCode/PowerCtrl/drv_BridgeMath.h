#ifndef __DRV_BRIDGEMATH_H__
#define __DRV_BRIDGEMATH_H__

#include "main.h"


// 单位时间，用于计算真实的物理单位

#define BUCK_BOOST_SHIFTDUTY 0.7f

typedef enum
{
    BridgeMath_State_Buck,
    BridgeMath_State_Boost,
    BridgeMath_State_Buck_Boost,
} BridgeMath_State_e;

/**
 * @brief 获得Buck状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int BuckA2B_LimCalculate(float U_A, float U_B, float I_B);

/**
 * @brief 获得Buck状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常
 */
int BuckA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set);

/**
 * @brief 获得Boost状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int BoostA2B_LimCalculate(float U_A, float U_B, float I_B);

/**
 * @brief 获得Boost状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常
 */
int BoostA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set);

/**
 * @brief 获得Buck-Boost状态下的电流变化率限幅
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @return int 0：正常；-1：出错
 */
int Buck_BoostA2B_LimCalculate(float U_A, float U_B, float I_B);

/**
 * @brief 获得Buck-Boost状态下的该电流变化率对应的占空比，并写入TGGL
 *
 * @param U_A A端电压
 * @param U_B B端电压
 * @param I_B B端电流
 * @param Iaccl_Set 当前电流变化率设定值
 * @return int 0:正常；1：给的电流变化率超出限幅；-1：计算出的占空比或其他过程异常
 */
int Buck_BoostA2B_GetDuty(float U_A, float U_B, float I_B, float Iaccl_Set);

/**
 * @brief 获得电流变化率的限幅的最大值
 *
 * @param State 想要获得的状态
 * @return float 电流变化率的限幅的最大值
 */
float Get_AcclLim_Max(BridgeMath_State_e State);

/**
 * @brief 获得电流变化率的限幅的最小值
 *
 * @param State 想要获得的状态
 * @return float 电流变化率的限幅的最小值
 */
float Get_AcclLim_Min(BridgeMath_State_e State);

/**
 * @brief 初始化参数
 */
void BridgeMath_Init(void);
#endif
