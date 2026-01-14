#ifndef __DRV_GIMBALCOMTX_H__
#define __DRV_GIMBALCOM_H__
#include <rtdevice.h>
#include "stdint.h"

#define TX_G_PERIOD 10

typedef __packed struct
{
    // 工作状态数据
    unsigned isStartGame : 1;   // 是否开始比赛
    unsigned hurtID : 3;        // 装甲板ID，未受攻击时为FF
    unsigned tempColor : 1;     // 红蓝方
    unsigned ourOutpost : 1;    // 己方前哨战是否存活
    unsigned enermyOutpost : 1; // 敌方前哨战是否存活
    unsigned speed : 16;        // 弹速
    unsigned SentryHP : 16;     // 哨兵的血量
#ifdef CORE_USING_SENTRY
    unsigned SentryIsFire : 1;
#endif
    uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
    uint16_t target_robot_ID; // 要作用的目标机器人 ID           当发送位置信息时，该项为 0
} RefState_TxGimMsg_t;        // 发送给云台的裁判系统数据

/**
 * @brief    初始化与云台通信，并控制底盘运动部分
 * @param [in]	无
 * @return   true:初始化成功	false:初始化失败
 */
rt_err_t Gimbal_Com_Init(void);

/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg);

/***
 * @brief    处理云台发送的各类数据报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Gimbaldata(struct rt_can_msg *msg);

#endif
