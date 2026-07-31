#include "func_Visual_Com.h"
#include "ThreadCtrl.h"
#include "app_RefSystem.h"
#include "drv_utils.h"
#include "drv_wheel.h"
#include "app_monitor.h"
#include "mod_Log.h"
#include "fun_protocolGim.h"
#include "drv_cmd_com.h"
#include "robot_control.h"
#include "drv_motionSolve.h"
#include "drv_GimMotor.h"
#include "mod_motion.h"

#ifdef BSP_USING_LOG
#define LOG_TAG     "Visual_com"
#define LOG_LVL     LOG_LVL_DBG
#include <ulog.h>
#endif

struct RttUartMsg
{
    rt_device_t dev;
    rt_size_t size;
};
//设备，线程，信号量相关
static rt_device_t VisualData_Serial;                    // 与视觉通信串口
static rt_thread_t Visual_Thread = RT_NULL; // 线程
#ifdef VISUAL_USART_USING_NORMAL
static rt_timer_t Visual_Timer = RT_NULL; // 定时器
static struct rt_completion completion;   // 完成量
#endif
#ifdef VISUAL_USART_USING_RS485
static struct rt_semaphore Visual_sem; // 信号量
#endif


//内部数据
VisualCtrl_t VisualCtrl;        //视觉发送的控制相关数据最终转化为的控制数据（写入数据源中）

//接收视觉发送的数据
Ctrl_info_t visual_rx_ctrl;     //视觉发送的控制原数据，经过处理后设置到VisualCtrl中
sentry_cmd_info_t visual_rx_sentry_cmd_info; //视觉发送来的哨兵自主决策指令

//给视觉发送的数据
Game_info_t visual_tx_game_info;        //给视觉发送的比赛信息
Game_robot_survivors_t visual_tx_game_robot_survivors;//给视觉发送的机器人血量信息
Robot_info_t visual_tx_robot_info;      //给视觉发送的机器人数据（自己的数据）
Bullet_remaining_t visual_tx_bullet_remaining; // 给视觉发送子弹剩余发射数和剩余金币数量
Robot_wheel_odometry_t visual_tx_Robot_wheel_odometry;     //底盘正解算后的x\y方向速度
ext_sentry_info_t visual_tx_sentry_info;    //哨兵自主决策信息同步
com_opponent_position_t visual_tx_opponent_position; // 对方机器人位置信息
ext_robot_command_map_t visual_tx_robot_command_map; // 云台手发送的机器人位置
ext_ground_robot_position_t visual_tx_ground_robot_position; // 己方机器人位置信息

static uint8_t visual_com_tx_buff[2048]; // 视觉通信发送数据缓冲区
static uint8_t visual_com_rx_buff[512]; // 视觉通信接收数据缓冲区

static void get_visual_need_ref_data()
{
    //这里使用静态局部变量来保存刷新的数据，由于结构体类型名称不同，但内容是一样的，所以直接使用rt_memcpy来赋值到需要发送的数据中
    static ext_game_robot_survivors_t visual_tx_game_robot_survivors_temp={0};
    static ext_bullet_remaining_t visual_tx_bullet_remaining_temp={0};
    static ext_game_state_t visual_tx_game_state_temp ={0};
    static ext_event_data_t visual_tx_event_data_temp ={0};
    static ext_game_robot_status_t visual_tx_robot_info_temp = {0};

    uint16_t getdata_temp_16;
    uint8_t getdata_temp_8;

    /*比赛阶段信息*/
    getRefData(structGet_pram(visual_tx_game_state_temp,ext_game_state));
    visual_tx_game_info.game_type = visual_tx_game_state_temp.game_data.game_type;
    visual_tx_game_info.game_progress = visual_tx_game_state_temp.game_data.game_progress;
    getRefData(memberGet_parm(getdata_temp_16,ext_game_state,stage_remain_time));
    visual_tx_game_info.stage_remain_time = getdata_temp_16;
    getRefData(bitGet_parm(getdata_temp_8,ext_dart_info,dart_info,6,3));
    visual_tx_game_info.dart_target = getdata_temp_8;
    getRefData(structGet_pram(visual_tx_event_data_temp, ext_event_data));
    visual_tx_game_info.small_energy_state = visual_tx_event_data_temp.small_energy_state;  
    visual_tx_game_info.big_energy_state = visual_tx_event_data_temp.big_energy_state;  

    /*己方血量信息*/
    getRefData(structGet_pram(visual_tx_game_robot_survivors_temp,ext_game_robot_survivors));
    rt_memcpy(&visual_tx_game_robot_survivors,&visual_tx_game_robot_survivors_temp,sizeof(visual_tx_game_robot_survivors));

    /*己方位置信息*/
    getRefData(structGet_pram(visual_tx_ground_robot_position,ext_ground_robot_position));
    
    /*机器人信息*/
    getRefData(structGet_pram(visual_tx_robot_info_temp,ext_game_robot_status));
    visual_tx_robot_info.robot_id = visual_tx_robot_info_temp.robot_id;
    visual_tx_robot_info.robot_level = visual_tx_robot_info_temp.robot_level;
    visual_tx_robot_info.remain_HP = visual_tx_robot_info_temp.remain_HP;
    visual_tx_robot_info.max_HP = visual_tx_robot_info_temp.max_HP;
    // visual_tx_robot_info.shooter_cooling_rate = visual_tx_robot_info_temp.shooter_cooling_rate;
    visual_tx_robot_info.shooter_cooling_limit = visual_tx_robot_info_temp.shooter_cooling_limit;
    visual_tx_robot_info.chassis_power_limit = visual_tx_robot_info_temp.chassis_power_limit;
    visual_tx_robot_info.bullet_speed_limit = visual_tx_robot_info_temp.bullet_speed_limit;

    /*剩余发弹量与金币数*/
    getRefData(structGet_pram(visual_tx_bullet_remaining_temp,ext_bullet_remaining));
    visual_tx_robot_info.shooter_cooling_rate = visual_tx_bullet_remaining_temp.bullet_remaining_num_17mm;//视觉要求
    visual_tx_bullet_remaining.bullet_remaining_num_17mm = visual_tx_bullet_remaining_temp.bullet_remaining_num_17mm;
    visual_tx_bullet_remaining.coin_remaining_num = visual_tx_bullet_remaining_temp.coin_remaining_num;
    visual_tx_bullet_remaining.projectile_allowance_fortress = visual_tx_bullet_remaining_temp.projectile_allowance_fortress;

    /*哨兵自主决策信息*/
    getRefData(structGet_pram(visual_tx_sentry_info,ext_sentry_info));

    /*云台手发送的机器人位置*/
    getRefData(structGet_pram(visual_tx_robot_command_map,ext_robot_command_map));

    /*底盘正解算后的x\y方向速度*/
    static Mot_base_t chass_xyw={0};
    chass_xyw.vel=MotModule_Transform(Get_realmotion().vel,-Get_YAW_ANGLE_360f());
    visual_tx_Robot_wheel_odometry.vel_x = chass_xyw.vel.x;
    visual_tx_Robot_wheel_odometry.vel_y = chass_xyw.vel.y;
}

#ifdef VISUAL_USART_USING_NORMAL
// 定时发送
static void Visual_Timeout_Handler(void *parameter)
{
    rt_completion_done(&completion);
}
#endif
// 发送数据给视觉的线程
static void Visual_TX_Thread_Entry(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("Visual_TX_Thread", 3000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        SWDG_FEED(id);
#ifdef VISUAL_USART_USING_NORMAL
        rt_completion_wait(&completion, RT_WAITING_FOREVER);
#endif
#ifdef VISUAL_USART_USING_RS485
        rt_sem_take(&Visual_sem, RT_WAITING_FOREVER);
#endif
        if (VisualData_Serial)
        {
            // if (isVsDebug[1] != 1)
            // {
                // getVisualComData();
            // }
            // VisualSendData(VisualData_Serial, &ComTxdata);
            // visualComInfo.txTick = rt_tick_get();
            get_visual_need_ref_data();             //刷新数据
            Send_Cmd_Packet(VisualData_Serial,visual_com_tx_buff,sizeof(visual_com_tx_buff),0); //以命令包发送数据
        }
    }
}

rt_err_t VisualTxThreadInit(void)
{
#ifdef VISUAL_USART_USING_NORMAL
    /* 初始化完成量对象 */
    rt_completion_init(&completion);

    // 定时器创建
    Visual_Timer = rt_timer_create(
        "Visual timer", Visual_Timeout_Handler,
        RT_NULL, THREAD_PEROID_VISUAL, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (Visual_Timer)
    {
        if (rt_timer_start(Visual_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
#endif
    // 线程创建
    Visual_Thread = rt_thread_create(
        "Visual", Visual_TX_Thread_Entry, RT_NULL,
        THREAD_STACK_VISUAL, THREAD_PRIO_VISUAL, THREAD_TICK_VISUAL);

    // 查看是否创建成功
    if (Visual_Thread != RT_NULL)
    {
        if (rt_thread_startup(Visual_Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    return RT_EOK;
}

/*------------------------------------XXX----------------------------------------------------------*/
static struct rt_messagequeue Visual_mq;                  /* 消息队列控制块 */
static rt_uint8_t Visual_msg_pool[1024];                   /* 消息队列中用到的放置消息的内存池 存储字节大小和设备供读取串口DMA时使用*/
static rt_uint8_t VisualUartBuff[RT_SERIAL_RB_BUFSZ + 1]; // 串口接受的原始数据 DMA

// 串口接受回调
rt_err_t Visual_rxCallBack(rt_device_t dev, rt_size_t size)
{
    struct RttUartMsg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&Visual_mq, &msg, sizeof(msg));
    return result;
}

// 设置接受回调和发送回调
rt_err_t Visual_rtt_uart_Init(const char *name, rt_err_t (*rx_cb)(rt_device_t dev, rt_size_t size),
                              rt_err_t (*tx_cmpt_cb)(rt_device_t dev, void *buffer))
{

    rt_err_t res = RT_EOK;
    struct serial_configure config =
        {
            BSP_VISUAL_USART_BOUD,
            DATA_BITS_8,        /* 8 databits */
            STOP_BITS_1,        /* 1 stopbit */
            PARITY_NONE,        /* No parity  */
            BIT_ORDER_LSB,      /* LSB first sent */
            NRZ_NORMAL,         /* Normal mode */
            RT_SERIAL_RB_BUFSZ, /* Buffer size */
            0};

    VisualData_Serial = rt_device_find(name);
    if (!VisualData_Serial)
    {
        rt_kprintf("rt_device_find UART failed !\n");
        return RT_ERROR;
    }

    rt_device_control(VisualData_Serial, RT_DEVICE_CTRL_CONFIG, (void *)&config);

    res = rt_device_set_rx_indicate(VisualData_Serial, rx_cb);
    if (res != RT_EOK)
        return res;

    res = rt_device_set_tx_complete(VisualData_Serial, tx_cmpt_cb);
    if (res != RT_EOK)
        return res;

    rt_device_open(VisualData_Serial, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
    return RT_EOK;
}




/****************接收部分*************************************************** */

/**
 * @brief 有视觉导航时使用的电控处理
 */
static void Visual_Ctrl_Receive_callback(void* data_p)
{
    VisualCtrl.ctrlData.vel_x = visual_rx_ctrl.x;
    VisualCtrl.ctrlData.vel_y = visual_rx_ctrl.y;
    if (visual_rx_ctrl.is_tyro == 1)
    {
        VisualCtrl.ctrlData.mode = SLOW_GYRO;
        VisualCtrl.ctrlData.fol_ang = 0;
        VisualCtrl.ctrlData.angvel = visual_rx_ctrl.w;
    }
    else if (visual_rx_ctrl.virt_yaw <= 360)
    {
        VisualCtrl.ctrlData.mode = FOLLOW_GIMBAL;
        VisualCtrl.ctrlData.fol_ang = visual_rx_ctrl.virt_yaw;
        VisualCtrl.ctrlData.angvel = 0;
    }
    else
    {
        VisualCtrl.ctrlData.mode = NO_FOLLOW; // 根据视觉需要选择底盘模式
        VisualCtrl.ctrlData.fol_ang = 0;
        VisualCtrl.ctrlData.angvel = visual_rx_ctrl.w;
    }

    VisualCtrl.ctrlData.getYawAngel = Get_YAW_ANGLE_360f();
    Source_WriteData_s(CS_Visual, (void *)&VisualCtrl.ctrlData);//将数据源写入到控制数据中
}

static void visualRxThread(void *parameter)
{
    struct RttUartMsg msg;
    rt_err_t result;
    uint32_t read_data_size=0;  
    CREAT_ID(id);
    ADDTOMONITOR_ID("visualRxThread", 3000, NULL, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&Visual_mq, &msg, sizeof(msg), 500); // 等待回调函数的消息队列

        if (result == RT_EOK)
        {
            SWDG_FEED(id);
            read_data_size = rt_device_read(msg.dev, 0, VisualUartBuff, msg.size);
            Receive_Handle(VisualUartBuff, read_data_size); // 处理接受到的数据
#ifdef VISUAL_USART_USING_RS485
                rt_sem_release(&Visual_sem);
#endif
        }
    }
}
rt_err_t VisualRxThreadInit(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t Thread = RT_NULL;
    /* 初始化消息队列 */
    ret = rt_mq_init(&Visual_mq, "Visual_mq",
                     Visual_msg_pool,
                     sizeof(struct RttUartMsg),
                     sizeof(Visual_msg_pool),
                     RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
        return ret;

    Thread = rt_thread_create(
        "VofaRX", visualRxThread, RT_NULL,
        THREAD_STACK_VISUAL, THREAD_PRIO_VISUAL, THREAD_TICK_VISUAL);

    // 查看是否创建成功
    if (Thread != RT_NULL)
    {
        if (rt_thread_startup(Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
#ifdef VISUAL_USART_USING_RS485
    rt_sem_init(&Visual_sem, "VisualSem", 0, RT_IPC_FLAG_FIFO);
#endif
    return ret;
}

// 与视觉通信线程以及串口初始化
rt_err_t VisualInit(void)
{
    rt_err_t ret=RT_ERROR;
    // 初始化串口
    if(Visual_rtt_uart_Init(BSP_VISUAL_USART_NAME, Visual_rxCallBack, NULL))
        return RT_ERROR;
    
    //初始化发送命令码ID
    Attach_SendCmd(VisualData_Serial,CMD_ID_GAME_INFO,CMD_SET_GAME_INFO,sizeof(visual_tx_game_info), (void*)&visual_tx_game_info);
    Attach_SendCmd(VisualData_Serial,CMD_ID_GAME_ROBOT_survivors,CMD_SET_GAME_ROBOT_survivors,sizeof(visual_tx_game_robot_survivors), (void*)&visual_tx_game_robot_survivors);
    Attach_SendCmd(VisualData_Serial,CMD_ID_ROBOT_INFO,CMD_SET_ROBOT_INFO,sizeof(visual_tx_robot_info), (void*)&visual_tx_robot_info);
    Attach_SendCmd(VisualData_Serial,CMD_ID_Bullet_remaining,CMD_SET_Bullet_remaining,sizeof(visual_tx_bullet_remaining), (void*)&visual_tx_bullet_remaining);
    Attach_SendCmd(VisualData_Serial,CMD_ID_Ground_robot_position,CMD_SET_Ground_robot_position,sizeof(visual_tx_ground_robot_position), (void*)&visual_tx_ground_robot_position);
    // Attach_SendCmd(VisualData_Serial,CMD_ID_Robot_wheel_odometry,CMD_SET_Robot_wheel_odometry,sizeof(visual_tx_Robot_wheel_odometry),(void*)&visual_tx_Robot_wheel_odometry);
    Attach_SendCmd(VisualData_Serial,CMD_ID_Sentry_info,CMD_SET_Sentry_INFO,sizeof(visual_tx_sentry_info),(void*)&visual_tx_sentry_info);
    Attach_SendCmd(VisualData_Serial,CMD_ID_opponent_position,CMD_SET_opponent_position,sizeof(visual_tx_opponent_position),(void*)&visual_tx_opponent_position);
    Attach_SendCmd(VisualData_Serial, CMD_ID_Robot_command_map, CMD_SET_Robot_command_map, sizeof(visual_tx_robot_command_map), (void *)&visual_tx_robot_command_map);

    //初始化接收命令ID和回调函数
    Attach_Receive_callback(CMD_ID_VISUAL_CTRL, sizeof(visual_rx_ctrl), (void *)&visual_rx_ctrl, Visual_Ctrl_Receive_callback);
    Attach_Receive_callback(CMD_ID_SENTRY_CMD_INFO, sizeof(visual_rx_sentry_cmd_info), (void *)&visual_rx_sentry_cmd_info, NULL);
    // Attach_Receive_callback(CMD_ID_VISUAL_RADAR, sizeof(visual_rx_radar), (void *)&visual_rx_radar, vsDatadeal);
    

    // 先初始化接收线程以及串口初始化
    ret = VisualRxThreadInit();
    if (ret != RT_EOK)
        return RT_ERROR;

    ret = VisualTxThreadInit(); // 发送数据到视觉通信线程
    if (ret != RT_EOK)
        return RT_ERROR;

    return ret;
}
