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
#include "robot_control.h"
#define LOG_TAG     "gim_motor_Log"
#include <ulog.h>


RefState_TxGimMsg_t TxGimMsg;
uint8_t gimTxDebug = 1;

/**
 * @brief    获取枪口热量
 * @param    None
 * @return   原始数据，单位不变
 */
static rt_uint16_t Ref_Shooter_Cooling_Heat(void)
{
    uint16_t heat_temp;
    switch(REF_ROBO_ID)
    {
        case REF_ROBO_R1_HERO:
        case REF_ROBO_B1_HERO:
            getRefData(memberGet_parm(heat_temp,ext_power_heat_data,shooter_id1_42mm_cooling_heat));
            break;
        case REF_ROBO_R3_STANDARD:
        case REF_ROBO_B3_STANDARD:
        case REF_ROBO_R4_STANDARD:
        case REF_ROBO_B4_STANDARD:
            getRefData(memberGet_parm(heat_temp,ext_power_heat_data,shooter_id1_17mm_cooling_heat));
            break;
        case REF_ROBO_R7_SENTRY:
        case REF_ROBO_B7_SENTRY:
            getRefData(memberGet_parm(heat_temp,ext_power_heat_data,shooter_id1_17mm_cooling_heat));
            break;
        default:
            getRefData(memberGet_parm(heat_temp,ext_power_heat_data,shooter_id1_17mm_cooling_heat));
    }
    return heat_temp;
}

/**
 * @brief    获取枪口上限速度
 * @param    None
 * @return   单位 m/s
 */
static rt_uint16_t Ref_Bullet_Speed_Limit(void)
{
    #ifdef CORE_USING_HERO
        return 16;
    #else
        return 30;
    #endif
}

/***
 * @brief    向云台端发送热量和射速信息
 * @param    cooling_heat 枪口热量
 * @param    cooling_rate 枪口每秒冷却值
 * @param    cooling_limit 枪口热量上限
 * @param    speed_limit 枪口上限速度,单位m/s
 * @return   rt_err_t：can2报文发送成功or失败
 ***/
static rt_err_t Send_Shooter_Data(rt_uint16_t cooling_heat,rt_uint16_t cooling_rate,rt_uint16_t cooling_limit,rt_uint16_t speed_limit,rt_uint8_t rune_buff)
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

/**
 * @brief 当装甲板受攻击时(距上次数据更新<20ms)返回受击装甲板id
 * 
 * @return uint8_t 受击装甲板id(当装甲板未受攻击时为0XFF)
 */
static uint8_t getArmorHurtID(void)
{
    if (IsRefDataValid(Index_robot_hurt,20))// 20ms内都相信此数据
    {
        return hurt.robot_hurt_armor;
    }
    return 0xFF;
}

// 得到己方前哨战血量
static uint16_t getOurOutpostHP(void)
{
    if (IsRefDataValid(Index_robot_hurt,20))// 20ms内都相信此数据
    {
        return hurt.outpost_hp;
    }
    return 0;
}

static void getGimData(void)
{
    static uint32_t lastBulletT;
    uint8_t temp_8;
    uint16_t temp_16;
    float temp_f;

    // 当装甲板受攻击时(距上次数据更新<20ms)返回受击装甲板id
    TxGimMsg.hurtID = getArmorHurtID();

    //判断比赛是否开始
    getRefData(bitGet_parm(temp_8,ext_game_state,game_data,4,4));
    TxGimMsg.isStartGame = temp_8 == 4 ? 1:0;
    
    //得到己方哨兵血量
    getRefData(memberGet_parm(temp_16,ext_game_robot_survivors,ally_7_robot_HP));

    TxGimMsg.SentryHP = temp_16;

    if (getOurOutpostHP())
        TxGimMsg.ourOutpost = 1;
    else
        TxGimMsg.ourOutpost = 0;

    if (EnemyInfo.outpost_hp)
        TxGimMsg.ourOutpost = 1;
    else
        TxGimMsg.ourOutpost = 0;

    //  裁判系统更新弹速就发发送一次数据
    // if (lastBulletT < GetRefDataRefreshTick(Index_shoot_data))
    // {
    //     // 弹速单位为cm/s
    getRefData(memberGet_parm(temp_f,ext_shoot_data,bullet_speed));
    TxGimMsg.speed = (uint16_t)(temp_f * 100);        //发送最新一次的弹速数据，只要不射击就不会变化，裁判系统只在射击时更新
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
    uint8_t temp_8;
    uint16_t temp1_16,temp2_16;
    while (1)
    {
        // 如果通信存在异常断连, 就直接发送 -1 告知云台
        if ((rt_tick_get() - GetRefDataRefreshTick(Index_game_robot_state) > 1500) || (rt_tick_get() - GetRefDataRefreshTick(Index_power_heat_data) > 1500) ||
            (!GetRefDataRefreshTick(Index_game_robot_state)) || (!GetRefDataRefreshTick(Index_power_heat_data)))
        {
            Send_Shooter_Data((rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint8_t)-1);
        }
        else
        {
            // 发送枪管信息
            getRefData(memberGet_parm(temp1_16,ext_game_robot_status,shooter_cooling_rate));
            getRefData(memberGet_parm(temp2_16,ext_game_robot_status,shooter_cooling_limit));
            getRefData(memberGet_parm(temp_8,ext_buff_musk,cooling_buff));
            Send_Shooter_Data(Ref_Shooter_Cooling_Heat(), temp1_16, temp2_16, Ref_Bullet_Speed_Limit(),temp_8);
        }
            
        // 发送各类信息
        // Vector2_t temp = ExMotMod_Get_xyDelta();
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
//    attachCanRXfun(rx_gimMotor, Refresh_GimbalMotor);
    if(!gim_motor_init())
    {
        LOG_E("gim_motor init failed, err=%d",gim_motor_init());
        return RT_ERROR;
    }
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
