#include "drv_Gimbal_Com.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "mod_Log.h"

#include "app_RefSystem.h" //Tx数据来源
#include "controlMsg.h"    //云台控制数据去向
#include "drv_GimMotor.h"

#ifdef BSP_USING_LOG
#define LOG_TAG "gim_com"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#endif

/*定时发送信号量*/
static struct rt_semaphore gimbal_Tx_sem;

/*通信信息*/
CANInstance gim_caninstance_Tx[Msg_num_Tx];
Gimbal_TransDataGet_ptr GetTransData[Msg_num_Tx]; // 获取需要发送的数据的函数指针
CANInstance gim_caninstance_Rx[Msg_num_Rx];

/*提供数据读取*/
Gim_mixData_Rx_t gimbal_mixdata_msg;
imu_euler_t gimbal_imudata_msg;
ui_data_t gimbal_ui_data_msg;
rt_tick_t imu_data_freshtick = 0;
rt_tick_t gimbal_mixdata_msg_freshtick = 0;
rt_tick_t gimbal_ui_data_msg_freshtick = 0;

/*---------------------------信息接收-----------------------------------------------------------------------------*/
/*接收回调*/
/***
 * @brief    处理云台发送的运动控制报文
 * @param    instance
 ***/
static void Gimbal_CtrlData_Rx_call(CANInstance *instance)
{
    static Ctrl_data_info_t gimbal_ctrl;
    gimbal_ctrl.vel_x = (float)MSG_READ_2BYTES(instance->rx_buff[0], instance->rx_buff[1]);
    gimbal_ctrl.vel_y = (float)MSG_READ_2BYTES(instance->rx_buff[2], instance->rx_buff[3]);
    gimbal_ctrl.mode = (Motion_mode_e)(instance->rx_buff[6]);

    // ulog_d("gim","vx,vy:%.2f,%.2f",gimbal_ctrl.vel_x,gimbal_ctrl.vel_y);

    // 云台跟随角外部修改
    if (gimbal_ctrl.mode == NO_FOLLOW)
    {
        gimbal_ctrl.fol_ang = 0;
        gimbal_ctrl.angvel = 0;
    }
    if (gimbal_ctrl.mode == FOLLOW_GIMBAL)
    {
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(instance->rx_buff[4], instance->rx_buff[5]); // 云台发送角速度单位0.1度/s，逻辑为在跟随模式下，云台直接发送YAW轴角速度值给底盘
        gimbal_ctrl.fol_ang = 0;
    }
    else if (gimbal_ctrl.mode == SLOW_GYRO || gimbal_ctrl.mode == FAST_GYRO || gimbal_ctrl.mode == CHANGE_GYRO)
    {
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(instance->rx_buff[4], instance->rx_buff[5]);
        gimbal_ctrl.fol_ang = 0;
    }
    gimbal_ctrl.getYawAngel = Get_YAW_ANGLE_360f();
    // instance->rx_buff[7]为云台端裁认为裁判系统异常则会给出一个底盘最大功率，底盘这边没用
    Source_WriteData_s(CS_GIMBAL, &gimbal_ctrl);
}

/***
 * @brief    处理云台发送的各类数据报文
 * @param    instance
 ***/
static void Gimbal_MixData_Rx_call(CANInstance *instance)
{
    gimbal_mixdata_msg_freshtick = rt_tick_get();
    rt_memcpy(&gimbal_mixdata_msg, instance->rx_buff, sizeof(gimbal_mixdata_msg) < 8 ? sizeof(gimbal_mixdata_msg) : 8);
    // Package_Write(dataNum[Gim_data], gimbal_msg); // 更新数据服务器的数据
    if (gimbal_mixdata_msg.chassis_reset)
        Robot_Chassis_Reset(RT_TRUE);
}

/***
 * @brief    处理云台发送的imu数据报文
 * @param    instance
 ***/
static void Gimbal_imuData_Rx_call(CANInstance *instance)
{
    gimbal_imudata_msg.pitch = *((float *)&instance->rx_buff[0]);
    gimbal_imudata_msg.roll = *((float *)&instance->rx_buff[4]);
    imu_data_freshtick = rt_tick_get();
}

/**
 * @brief   处理云台发送的英雄UI绘制数据报文(引导灯)
 *
 */
static void Gimbal_ui_light_Rx_call(CANInstance *instance)
{
    gimbal_ui_data_msg_freshtick = rt_tick_get();
    gimbal_ui_data_msg.Ui_Guide_Light_Circle_x = (uint16_t)MSG_READ_2BYTES(instance->rx_buff[0], instance->rx_buff[1]);
    gimbal_ui_data_msg.Ui_Guide_Light_Circle_y = (uint16_t)MSG_READ_2BYTES(instance->rx_buff[2], instance->rx_buff[3]);
    gimbal_ui_data_msg.Ui_Guide_Light_Circle_radius = instance->rx_buff[4];
    gimbal_ui_data_msg.Ui_Guide_Light_Circle_freshtick = rt_tick_get();
}

/**
 * @brief 处理云台发送的英雄UI绘制数据报文(弹丸)
 *
 */
static void Gimbal_ui_bullet_Rx_call(CANInstance *instance)
{
    gimbal_ui_data_msg_freshtick = rt_tick_get();
    gimbal_ui_data_msg.Ui_Bullet_Circle_x = (uint16_t)MSG_READ_2BYTES(instance->rx_buff[0], instance->rx_buff[1]);
    gimbal_ui_data_msg.Ui_Bullet_Circle_y = (uint16_t)MSG_READ_2BYTES(instance->rx_buff[2], instance->rx_buff[3]);
    gimbal_ui_data_msg.Ui_Bullet_Circle_radius = instance->rx_buff[4];
    gimbal_ui_data_msg.Ui_Bullet_Circle_freshtick = rt_tick_get();
}

/**
 * @brief 判断云台通信是否掉线
 *
 * @return uint8_t
 */
uint8_t GetGimbalOffline(void)
{
    uint8_t gimbleoffline = 0;
    rt_tick_t gimbal_msg_freshtick = 0;

    /*获取最近一次报文接收时间*/
    for (int index = 0; index < Msg_num_Rx; index++)
        if (gimbal_msg_freshtick < gim_caninstance_Rx[index].rx_time)
            gimbal_msg_freshtick = gim_caninstance_Rx[index].rx_time;

    /*判断是否掉线*/
    if (rt_tick_get() - gimbal_msg_freshtick >= 500)
    {
        gimbleoffline |= 1 << 0;
    }
    return gimbleoffline;
}

/*---------------------------信息发送-----------------------------------------------------------------------------*/
/**
 * @brief 定时发送中断
 *
 * @param parameter
 */
static void Gimbal_Tx_IRQHandler(void *parameter)
{
    rt_sem_release(&gimbal_Tx_sem);
}

/**
 * @brief    获取枪口热量
 * @param    None
 * @return   原始数据，单位不变
 */
static rt_uint16_t Ref_Shooter_Cooling_Heat(void)
{
    uint16_t heat_temp;
    switch (REF_ROBO_ID)
    {
    case REF_ROBO_R1_HERO:
    case REF_ROBO_B1_HERO:
        getRefData(memberGet_parm(heat_temp, ext_power_heat_data, shooter_id1_42mm_cooling_heat));
        break;
    case REF_ROBO_R3_STANDARD:
    case REF_ROBO_B3_STANDARD:
    case REF_ROBO_R4_STANDARD:
    case REF_ROBO_B4_STANDARD:
    case REF_ROBO_R7_SENTRY:
    case REF_ROBO_B7_SENTRY:
        getRefData(memberGet_parm(heat_temp, ext_power_heat_data, shooter_id1_17mm_cooling_heat));
        break;
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
    return 16;
}

Gim_Reftrans1_t gim_reftrans_msg1;
Gim_Reftrans2_t gim_reftrans_msg2;

/**
 * @brief 转发裁判系统数据1的数据获取函数
 *
 * @param instance
 */
static void Gimbal_RefShootdataGet_Tx(CANInstance *instance)
{
    rt_memset(&gim_reftrans_msg1, 0, sizeof(gim_reftrans_msg1));
    uint16_t get_temp_16;
    /*枪口热量*/
    gim_reftrans_msg1.cooling_heat = Ref_Shooter_Cooling_Heat();
    /*枪口每秒冷却值*/
    getRefData(memberGet_parm(get_temp_16, ext_game_robot_status, shooter_cooling_rate));
    gim_reftrans_msg1.cooling_rate = get_temp_16;
    /*枪口热量上限*/
    getRefData(memberGet_parm(get_temp_16, ext_game_robot_status, shooter_cooling_limit));
    gim_reftrans_msg1.cooling_limit = get_temp_16;
    /*枪口速度上限*/
    gim_reftrans_msg1.speed_limit = Ref_Bullet_Speed_Limit();
    /*枪口冷却具体值*/
    //getRefData(memberGet_parm(get_temp_16, ext_buff_musk, cooling_buff));
    //gim_reftrans_msg1.cooling_buff = get_temp_16;

    //! 因为协议修改导致字节数超了，但暂时配合云台调试
    memcpy(instance->tx_buff, &gim_reftrans_msg1, sizeof(gim_reftrans_msg1) < 8 ? sizeof(gim_reftrans_msg1) : 8);
}

/**
 * @brief 转发裁判系统数据2的数据获取函数
 *
 * @param instance
 */
static void Gimbal_RefdataGet_Tx(CANInstance *instance)
{
    rt_memset(&gim_reftrans_msg2, 0, sizeof(gim_reftrans_msg2));
    uint8_t get_temp_8; // 暂存裁判系统信息
    uint16_t get_temp_16;
    float get_temp_float;

    /*是否开始比赛*/
    getRefData(bitGet_parm(get_temp_8, ext_game_state, game_data, 4, 4));
    if (get_temp_8 == 4) // 比赛中
        gim_reftrans_msg2.isStartGame = 1;
    else
        gim_reftrans_msg2.isStartGame = 0;

    /*当装甲板受攻击时(距上次数据更新<20ms)返回受击装甲板id*/
    if (IsRefDataValid(Index_robot_hurt, 20)) // 20ms内都相信此数据
        gim_reftrans_msg2.hurtID = hurt.robot_hurt_armor;
    else
        gim_reftrans_msg2.hurtID = 0xFF;

    /*获取己方颜色*/
    gim_reftrans_msg2.tempColor = REF_TEAM_COLOR;

    /*己方前哨站是否存活*/
    if (hurt.outpost_hp > 0)
        gim_reftrans_msg2.ourOutpost = 1;
    else
        gim_reftrans_msg2.ourOutpost = 0;

    //! 配合云台调试
    gim_reftrans_msg2.enermyOutpost = 0;

    /*弹速*/
    if (IsRefDataValid(Index_shoot_data, 20)) // 20ms内有射击则更新弹速，否则弹速为0
    {
        getRefData(memberGet_parm(get_temp_float, ext_shoot_data, bullet_speed));
        gim_reftrans_msg2.speed = (uint16_t)(get_temp_float * 100);
    }
    else
        gim_reftrans_msg2.speed = 0;

    /*哨兵血量*/
    getRefData(memberGet_parm(get_temp_16, ext_game_robot_survivors, ally_7_robot_HP));
    gim_reftrans_msg2.SentryHP = get_temp_16;

    /*云台手键盘信息*/
    getRefData(memberGet_parm(get_temp_8, ext_robot_command_map, commd_keyboard));
    gim_reftrans_msg2.commd_keyboard = get_temp_8;

    /*目标机器人ID（多机通信）*/
    getRefData(memberGet_parm(get_temp_16, ext_robot_command_map, target_robot_ID));
    gim_reftrans_msg2.target_robot_ID = get_temp_16;

    memcpy(instance->tx_buff, &gim_reftrans_msg2, sizeof(gim_reftrans_msg2) < 8 ? sizeof(gim_reftrans_msg2) : 8); // 写入报文
}

/**
 * @brief 给云台的信息发送线程
 *
 * @param parameter
 */
static void Gimbal_ComTx_Thread(void *parameter)
{
    while (1)
    {
        rt_sem_take(&gimbal_Tx_sem, RT_WAITING_FOREVER);
        for (int index = 0; index < Msg_num_Tx; index++) // 遍历发送注册报文
        {
            GetTransData[index](&gim_caninstance_Tx[index]); // 获取数据
            gim_caninstance_Tx[index].tx_len = 8;
            CAN_Transmit(&gim_caninstance_Tx[index]); // 发送数据
        }
    }
}

/*---------------------------通信初始化-----------------------------------------------------------------------------*/

/**
 * @brief 给云台发送报文注册
 *
 * @param index 报文索引
 * @param tx_id 发送ID
 * @param get_transdata 获取需要发送的数据的函数
 */
static int Gimbal_comTx_Attach(rt_uint8_t index, unsigned int tx_id, Gimbal_TransDataGet_ptr get_transdata)
{
    int res = 0;
    CAN_Init_Config_s config;
    rt_memset(&config, 0, sizeof(config));
    config.can_device_name = GIMBAL_CAN_DEV;
    config.tx_id = tx_id;
    config.spi_slave = GIMBAL_SPI_USE;
    GetTransData[index] = get_transdata;
    config.can_module_callback = NULL;
    res = CAN_Attach(&gim_caninstance_Tx[index], &config);
    return res;
}

/**
 * @brief 接收云台报文注册
 *
 * @param index 报文索引
 * @param rx_id 接收ID
 * @param can_module_callback 对应接收函数
 */
static int Gimbal_comRx_Attach(rt_uint8_t index, unsigned int rx_id, void (*can_module_callback)(struct CANInstance *))
{
    int res = 0;
    CAN_Init_Config_s config;
    rt_memset(&config, 0, sizeof(config));
    config.can_device_name = GIMBAL_CAN_DEV;
    config.rx_id = rx_id;
    config.spi_slave = GIMBAL_SPI_USE;
    config.can_module_callback = can_module_callback;
    res = CAN_Attach(&gim_caninstance_Rx[index], &config);
    return res;
}

/**
 * @brief 云台通信初始化
 *
 * @return rt_err_t
 */
rt_err_t Gimbal_Com_Init(void)
{
    int res = 0;
    rt_thread_t handler = RT_NULL; // 线程句柄

    /*注册发送can通信*/
    res += Gimbal_comTx_Attach(Shooter_msg_Index, GIMBAL_SHOOT_TX, Gimbal_RefShootdataGet_Tx);
    res += Gimbal_comTx_Attach(Mixed_msg_Index, GIMBAL_MIXDATA_TX, Gimbal_RefdataGet_Tx);
    if (res)
    {
        LOG_E("gim_Tx_msg init failed, err=%d", res);
        return RT_ERROR;
    }

    /*注册接收can通信*/
    res += Gimbal_comRx_Attach(Ctrl_msg_Index, GIMBAL_CTRL_RX, Gimbal_CtrlData_Rx_call);
    res += Gimbal_comRx_Attach(mixData_msg_Index, GIMBAL_DATA_RX, Gimbal_MixData_Rx_call);
    // res += Gimbal_comRx_Attach(Imu_msg_Index, GIMBAL_IMU_DATA_RX, Gimbal_imuData_Rx_call);
#if defined HREO_UI_SPECIAL
    // res += Gimbal_comRx_Attach(Ui_light_msg_Index, GIMBAL_UI_LIGHT_DATA_RX, Gimbal_ui_light_Rx_call);
    // res += Gimbal_comRx_Attach(Ui_bullet_msg_Index, GIMBAL_UI_BULLET_DATA_RX, Gimbal_ui_bullet_Rx_call);
#endif
    if (res)
    {
        LOG_E("gim_Rx_msg init failed, err=%d", res);
        return RT_ERROR;
    }

    /*注册云台电机*/
    res = gim_motor_init();
    if (res)
    {
        LOG_E("gim_motor init failed, err=%d", res);
        return RT_ERROR;
    }

    // 初始化信号量
    res = rt_sem_init(&gimbal_Tx_sem, "gim_tx_sem", 0, RT_IPC_FLAG_FIFO);
    if (res)
    {
        LOG_E("gimbal_Tx_sem init failed, err=%d", res);
        return RT_ERROR;
    }

    // 初始化云台控制线程
    handler = rt_thread_create(
        "gimbal_com",        // 线程名
        Gimbal_ComTx_Thread, // 线程入口
        RT_NULL,             // 入口参数无
        THREAD_STACK_GIMBAL, // 线程栈
        THREAD_PRIO_GIMBAL,  // 线程优先级
        THREAD_TICK_GIMBAL); // 线程时间片大小

    // 线程创建失败返回false
    if (handler == RT_NULL)
    {
        LOG_E("gimbal_com_thread create failed");
        return RT_ERROR;
    }

    // 线程启动失败返回false
    if (rt_thread_startup(handler) != RT_EOK)
    {
        LOG_E("gimbal_com_thread startup failed");
        return RT_ERROR;
    }

    // 创建线程定时器 定时释放信号量
    rt_timer_t timer = rt_timer_create("Gimbal_Tx_timer",
                                       Gimbal_Tx_IRQHandler,
                                       RT_NULL,
                                       Gim_Tx_PERIOD,
                                       RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (rt_timer_start(timer) != RT_EOK)
    {
        LOG_E("gimbal_com_timer start failed");
        return RT_ERROR;
    }

    return RT_EOK;
}
