#ifndef __DRV_BRIDGEHW_H__
#define __DRV_BRIDGEHW_H__

// 单片机文件
#include "main.h"

/**
 * @brief 按照TGGLA与TGGLB修改PWM占空比
 *
 * @param I_B   目前B端电流值（关注正负）
 */
void BridgeHW_SetPWM(float I_B);

// 初始化H桥硬件
int BridgeHW_Init(void);

/**
 * @brief 修改PWM参数
 *
 * @param TGGLA1 A 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
 * @param TGGLA2
 * @param TGGLB1 B 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
 * @param TGGLB2
 */
void BridgeHW_TGGLWrite(float TGGLA1, float TGGLA2, float TGGLB1, float TGGLB2);

/**
 * @brief 读取当前设定的TGGL值
 *
 * @param AorB      1：读取A端；0:读取B端
 * @param Num       0:读取第一个；1：读取第二个
 * @return float    返回该值
 */
float BridgeHW_TGGLRead(int8_t AorB, int8_t Num);

void Bridge_TIM_Set_Comp(void);
#endif
