#ifndef __FUNC_LED_H__
#define __FUNC_LED_H__

#include "main.h"

// #define LED_DEBUG

#ifndef LED_DEBUG
#define LED_ONCAR
#endif

typedef enum
{
    LED_ON,
    LED_OFF,
    LED_Flash
} LED_State_e;

/**
 * @brief
 * @author dty
 * @param a 第a行
 * @param b 第b列
 * @param ctrl LED_ON/LED_OFF/LED_Flash
 * @param period 闪烁周期（100ms整数倍）
 */
void LED_Ctrl(uint8_t a, uint8_t b, LED_State_e ctrl, uint16_t period);

/**
 * @brief 关闭所有LED
 * @author dty
 */
void LED_AllOff(void);

/**
 * @brief 开启所有LED
 * @author dty
 */
void LED_AllOn(void);

/**
 * @brief 初始化
 * @author dty
 */
void LED_Init(void);

/**
 * @brief LED轮询
 * @author dty
 */
void LED_Routine(void);

#endif /* __FUNC_LED_H__ */
