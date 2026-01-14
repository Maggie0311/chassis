#include "mod_GimbalComTx.h"
#include "ThreadCtrl.h"
#include "drv_canthread.h"
#include "drv_gimCtrlRx.h"
#include "drv_GimMotor.h"
#include "app_RefSystem.h"
#include "fun_getBulletSpeed_AVG.h"
#include "mod_motion.h"
// #include "func_ModeTrigger.h"
#include "drv_IMU.h"

RefState_TxGimMsg_t TxGimMsg;
uint8_t gimTxDebug = 1;

/***
 * @brief    向云台端发送热量和射速信息
 * @param    cooling_heat 枪口热量
 * @param    cooling_rate 枪口每秒冷却值
 * @param    cooling_limit 枪口热量上限
 * @param    speed_limit 枪口上限速度,单位m/s
 * @return   rt_err_t：can2报文发送成功or失败
 ***/
static rt_err_t Send_Shooter_Data(rt_uint16_t cooling_heat, rt_uint16_t cooling_rate, rt_uint16_t cooling_limit, rt_uint16_t speed_limit, rt_uint8_t rune_buff)
{
    struct rt_can_msg tx_gimbal;

    tx_gimbal.id = GIMBAL_SHOOT_TX; // 设置ID
    tx_gimbal.ide = RT_CAN_STDID;   // 标准帧
    tx_gimbal.rtr = RT_CAN_DTR;     // 数据帧
    tx_gimbal.priv = 1;             // 报文优先级次高
    tx_gimbal.len = 8;              // 长度8

    tx_gimbal.data[0] = (rt_uint8_t)(cooling_heat >> 8);
    tx_gimbal.data[1] = (rt_uint8_t)cooling_heat;
    tx_gimbal.data[2] = (rt_uint8_t)(cooling_rate >> 8) | (rune_buff << 7);
    tx_gimbal.data[3] = (rt_uint8_t)cooling_rate;
    tx_gimbal.data[4] = (rt_uint8_t)(cooling_limit >> 8);
    tx_gimbal.data[5] = (rt_uint8_t)cooling_limit;
    tx_gimbal.data[6] = (rt_uint8_t)(speed_limit >> 8);
    tx_gimbal.data[7] = (rt_uint8_t)speed_limit;

    if (!CanTrans(GIMBAL_CAN_DEV, &tx_gimbal))
        return RT_ERROR;
    else
        return RT_EOK;
}

/***
 * @brief   发送裁判系统数据给云台
 * @param   fin_xspeed/fin_yspeed 经过缓动后的速度大小
 * @return  rt_err_t 是否成功发送报文
 ***/
static rt_err_t Send_Mixed_Data(RefState_TxGimMsg_t *msg)
{
    struct rt_can_msg tx_gimbal;

    tx_gimbal.id = GIMBAL_DATA_TX; // 设置ID
    tx_gimbal.ide = RT_CAN_STDID;  // 标准帧
    tx_gimbal.rtr = RT_CAN_DTR;    // 数据帧
    tx_gimbal.priv = 1;            // 报文优先级次高
    tx_gimbal.len = 8;             // 长度8

    uint8_t size = sizeof(RefState_TxGimMsg_t);
    memcpy(&tx_gimbal.data[0], msg, size > 8 ? 8 : size);

    if (RT_ERROR == CanTrans(GIMBAL_CAN_DEV, &tx_gimbal))
        return RT_ERROR;
    else
        return RT_EOK;
}

static void getGimData(void)
{
    static uint32_t lastBulletT;
    uint8_t tm = 0;

    // 当装甲板未受攻击时为0XFF
    TxGimMsg.hurtID = getArmorHurtID();
    TxGimMsg.isStartGame = getIsStartGame();
    TxGimMsg.SentryHP = getSentryHP();

    // 前哨战信息
    EnemyInfo_t enermy;
    if (getOurOutpostHP())
        TxGimMsg.ourOutpost = 1;
    else
        TxGimMsg.ourOutpost = 0;
    enermy = GetEnemyInfo();    
//bug,好像是写错了吧？
    if (enermy.outpost_hp)
        TxGimMsg.ourOutpost = 1;
    else
        TxGimMsg.ourOutpost = 0;

    //  裁判系统更新弹速就发发送一次数据
    // if (lastBulletT < GetRefDataRefreshTick(Index_shoot_data))
    // {
    //     // 弹速单位为cm/s
        TxGimMsg.speed = (uint16_t)(Ref_Bullet_Speed());        //发送最新一次的弹速数据，只要不射击就不会变化，裁判系统只在射击时更新
    //     lastBulletT = GetRefDataRefreshTick(Index_shoot_data);
    // }
    // else
    //     TxGimMsg.speed = 0;

    // 云台手按键
    ext_robot_command_map_t mapData;
    getRefData(structGet_pram(mapData,ext_robot_command_map));
    TxGimMsg.target_robot_ID = mapData.target_robot_ID;
    TxGimMsg.commd_keyboard = mapData.commd_keyboard;
#ifdef CORE_USING_SENTRY
    uint32_t tick = 0;
    static uint32_t lastMapTick;
    tick = GetRefDataRefreshTick(Index_robot_command_map);
    if (mapData.commd_keyboard == 'F' && tick != lastMapTick)
    {
        lastMapTick = tick;
        TxGimMsg.SentryIsFire = !TxGimMsg.SentryIsFire;
    }
#endif
    // 0为红方，1为蓝方
    if (REF_TEAM_COLOR == RED_TEAM)
        REF_TEAM_COLOR = 0;
    else if (REF_TEAM_COLOR == BLUE_TEAM)
        REF_TEAM_COLOR = 1;
    TxGimMsg.tempColor = REF_TEAM_COLOR;
}

/***
 * @brief    与云台端通信线程,发送热量数据给云台，并从云台端读取控制数据
 ***/
static void Gimbal_Com_Thread(void *parameter)
{
    uint8_t tm = 0;
    while (1)
    {
        // 如果通信存在异常断连, 就直接发送 -1 告知云台
        if ((rt_tick_get() - GetRefDataRefreshTick(Index_game_robot_state) > 1500) || (rt_tick_get() - GetRefDataRefreshTick(Index_power_heat_data) > 1500) ||
            (!GetRefDataRefreshTick(Index_game_robot_state)) || (!GetRefDataRefreshTick(Index_power_heat_data)))
        {
            Send_Shooter_Data((rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint8_t)-1);
        }
        else
            // 发送枪管信息
            Send_Shooter_Data(Ref_Shooter_Cooling_Heat(), Ref_Shooter_Cooling_Rate(), Ref_Shooter_Cooling_Limit(), Ref_Bullet_Speed_Limit(), Ref_Power_Rune_Buff());

        // 发送各类信息
        // Vector2_t temp = ExMotMod_Get_xyDelta();
        tm = gimGetRobotHurt();
        if (gimTxDebug) // 给云台发送一些杂项数据
            getGimData();
        Send_Mixed_Data(&TxGimMsg);
        rt_thread_mdelay(TX_G_PERIOD);
    }
}

/**
 * @brief    初始化与云台通信，并控制底盘运动部分
 * @return   true:初始化成功	false:初始化失败
 */
rt_err_t Gimbal_Com_Init(void)
{
    rt_thread_t handler = RT_NULL; // 线程句柄

    // 注册can接受回调
    attachCanRXfun(rx_gimIMU, Refresh_Chassis_Gimbalcaled_IMU_Data);
    attachCanRXfun(rx_gimMotor, Refresh_GimbalMotor);
    attachCanRXfun(rx_gimctrl, Refresh_Ctldata);
    attachCanRXfun(rx_gimData, Refresh_Gimbaldata);

    // 初始化云台控制线程
    handler = rt_thread_create(
        "gimbal_ctrl",       // 线程名
        Gimbal_Com_Thread,   // 线程入口
        RT_NULL,             // 入口参数无
        THREAD_STACK_GIMBAL, // 线程栈
        THREAD_PRIO_GIMBAL,  // 线程优先级
        THREAD_TICK_GIMBAL); // 线程时间片大小

    // 线程创建失败返回false
    if (handler == RT_NULL)
        return RT_ERROR;

    // 线程启动失败返回false
    if (rt_thread_startup(handler) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
