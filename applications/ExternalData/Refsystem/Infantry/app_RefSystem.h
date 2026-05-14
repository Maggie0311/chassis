#ifndef __APP_REFSYSTEM_H__
#define __APP_REFSYSTEM_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "ThreadCtrl.h"//初始化线程要用
#include "mod_RefSystem.h"

/* 英雄弹速上限档位 */
#define HERO_BS_1 10
#define HERO_BS_2 16

/* 步兵弹速上限档位 */
#define INFANTRY_BS_1 15
#define INFANTRY_BS_2 18
#define INFANTRY_BS_3 30

typedef struct
{
    rt_uint8_t robot_if_hurt;     // 机器人是否受伤
    rt_uint8_t robot_hurt_armor;  // 机器人受伤的装甲板ID
    rt_uint8_t build_if_attacked; // 建筑物是否受伤，低三位代表哨兵，前哨站，基地。位数据为1：掉血，为0：不掉血
    rt_uint16_t sentry_hp;        // 哨兵血量
    rt_uint16_t outpost_hp;       // 前哨站血量
    rt_uint16_t base_hp;          // 基地血量
    uint32_t tick;

} Ref_Hurt_t;

/*------------------------------------XXX----------------------------------------------------------*/


/**
 * @brief    获取枪口上限速度
 * @param    None
 * @return   单位 m/s
 */
rt_uint16_t Ref_Bullet_Speed_Limit(void);

/**
 * @brief    获取剩余弹量
 *		  	发送频率： 10Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
rt_uint16_t Ref_AmmoRemain(void);
/**
 * @brief 获取当前机器人是否飞坡增益
 *        根据规则：同一台机器人需在 10 秒内检测到一方场地两处飞坡增益点的场地交互模块卡，才能触发飞坡增益
 *        飞坡增益持续时间：20秒
 * @return uint8_t
 */
uint8_t RefGetIsFlySlope(void);

rt_err_t DJI_Init(void);
extern Ref_Hurt_t hurt;

#endif
