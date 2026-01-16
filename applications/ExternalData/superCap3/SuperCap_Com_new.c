#include "SuperCap_Com_new.h"

#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "robot_data.h"

#include "app_RefSystem.h"
#include "fun_protocolGim.h"
#include "fun_powerctrl.h"
#include "drv_utils.h"

/*线程定时*/
static struct rt_semaphore scpr_Tx_sem;

/*超电通信信息*/
Scpr_ComRx_t scpr_comRx;
Scpr_ComTx_t scpr_comTx;
CANInstance scpr_caninstance;

/*超电调试开关；*/
uint8_t power_ctrl_close = 0;   // 超电是否关闭输出
uint8_t scprIsDebug = 0;

/*电压记录*/
uint16_t V_cap_data[VCAP_DATA_NUM]; // mv,本文件用于记录电压值，判断超电状态
uint8_t V_cap_data_index = 0;

/*超电状态*/
SupCap_State_e Scp_Now_State = (SupCap_State_e)0;

/*监视器ID*/
static uint8_t suprMontiorID;

/*----------------------------------------------超电线程-----------------------------------------------*/
static void Scpr_Tx_IRQHandler(void *parameter)
{
    rt_sem_release(&scpr_Tx_sem);
}

/**
 * @brief 超电CAN报文接收回调函数
 * 
 * @param instance 对应的can实例
 */
static void Refresh_Scprdata(CANInstance *instance)
{
    // 当没有在发送并且  收到应答
    if ((1 == instance->rx_buff[0])) // 0
        scpr_comRx.recvAck = 1;

    SWDG_FEED(suprMontiorID);
    V_cap_data[V_cap_data_index == VCAP_DATA_NUM ? (V_cap_data_index = 0, V_cap_data_index++) : V_cap_data_index++] = scpr_comRx.V_cap; // 更新前记录上一次数据

    scpr_comRx.cap = (instance->rx_buff[1]) / 100.f;           // 1
    scpr_comRx.V_cap = *(uint16_t *)(&instance->rx_buff[2]);   // 2-3
    scpr_comRx.voltage = *(uint16_t *)(&instance->rx_buff[4]); // 4-5
    scpr_comRx.current = *(uint16_t *)(&instance->rx_buff[6]); // 6-7
    scpr_comRx.power_cup = scpr_comRx.voltage * scpr_comRx.current / 1000.f/1000.f; // 计算功率，此处功率未进行滤波，在后续使用时需要考虑滤波
    scpr_comRx.recvTick = rt_tick_get();               //刷新数据接收时刻
}

/**
 * @brief 更新数据
 * 
 * @param Cmd 
 */
static void getSuprCapComData(Scpr_ComTx_t *Cmd)
{
    // 判断是否更新数据，只置标志位不清除
#define checkSuprComDataRefresh(now, last) \
    do                                     \
    {                                      \
        if (now != last)                   \
        {                                  \
            Cmd->isdataRef = 1;            \
        }                                  \
        last = now;                        \
    } while (0)

    static rt_tick_t powerLimTick, powerBufTick, overPowerTick;
    uint16_t now = 0;
    uint32_t nowTick = 0; // 使用32位通用

    // 发送前检查是否收到应答，收到则将标志位清0
    if (1 == scpr_comRx.recvAck)
    {
        scpr_comRx.recvAck = 0;
        Cmd->isdataRef = 0;
    }

    // 功率上限 0x0201
    now = GetRefDataRefreshTick(Index_game_robot_state);
    checkSuprComDataRefresh(now, powerLimTick);
    // 功率缓冲 0x0202
    now = GetRefDataRefreshTick(Index_power_heat_data);
    checkSuprComDataRefresh(now, powerBufTick);

    getRefData(memberGet_parm(now,ext_game_robot_status,chassis_power_limit));
    // 底盘功率限制上限--从裁判系统数据中读取
    checkSuprComDataRefresh(now, Cmd->chassis_powerLim);
    getRefData(memberGet_parm(now,ext_power_heat_data,chassis_power_buffer));      // 功率缓冲--从裁判系统数据中读取
    checkSuprComDataRefresh(now, Cmd->chassis_powerBuf);

    // 首先判断是否无线充电
    if(Get_Wireless_Charge())
    {
        Cmd->Whether_To_Output = 2; // 无线充电状态为真时，发给超电，让其只放电，不充电
    }
    else//如果没有打开无线充电，则只判断超电状态如何
    {
        if (power_ctrl_close == 1 || (rt_tick_get() - Get_Gim_FreshTick() > 500))  
        // 如果功率控制要求关闭，置0，关掉超电，云台通信丢失也关掉超电，防止底盘主控接到小电脑时，与超电继续通信，主动放电到底盘
            Cmd->Whether_To_Output = 0;
        else
            Cmd->Whether_To_Output = 1; // 否则打开
    }
    now = RefGetIsFlySlope(); // 是否rfid飞坡增益 0x209
    checkSuprComDataRefresh(now, Cmd->isFlySlope);
    // now = Ref_Chassis_IsOverPower(&nowTick); // 是否超功率扣血 0x206
    // if (nowTick != overPowerTick)
    // {
    //     overPowerTick = nowTick;
    //     Cmd->isdataRef = 1;
    // }
    // else
    //     Cmd->isOverPower = 0;

    if (powerBufTick == 0 && powerLimTick == 0)
    { // 当从未收到裁判系统数据
        Cmd->chassis_powerLim = POWER_REMANI;
        Cmd->isdataRef = 1;
        Cmd->chassis_powerBuf = (uint16_t)(-1);
    }
}

/**
 * @brief 发送信息给超电
 * 
 * @param cmd 
 * @return rt_err_t 
 */
static rt_err_t Send_To_Scpr(Scpr_ComTx_t *cmd)
{
    /*更新发送数据*/
    scpr_caninstance.tx_buff[0] = (uint8_t)(cmd->Whether_To_Output);
    scpr_caninstance.tx_buff[1] = (uint8_t)(cmd->isdataRef);
    scpr_caninstance.tx_buff[2] = (uint8_t)(cmd->isFlySlope);
    scpr_caninstance.tx_buff[3] = (uint8_t)(cmd->isOverPower);
    scpr_caninstance.tx_buff[4] = (uint8_t)((uint16_t)cmd->chassis_powerBuf >> 8);
    scpr_caninstance.tx_buff[5] = (uint8_t)(cmd->chassis_powerBuf);
    scpr_caninstance.tx_buff[6] = (uint8_t)(uint16_t)(cmd->chassis_powerLim >> 8);
    scpr_caninstance.tx_buff[7] = (uint8_t)(cmd->chassis_powerLim);

    /*发送*/
    CAN_Transmit(&scpr_caninstance);

    return RT_EOK;
}

/***
 * @brief    与超级电容端通信线程（发送给超级电容）
 ***/
static void Scpr_Thread(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("Scpr_Thread", 1500, NULL, ALARM_RED, 1, id);
    SWDG_START(id);

    ADDTOMONITOR_ID("Scpr_Rx", 5000, NULL, ALARM_RED, 0, suprMontiorID);
    SWDG_START(suprMontiorID);
    while (1)
    {
        rt_sem_take(&scpr_Tx_sem, RT_WAITING_FOREVER);
        if (scprIsDebug == 0)
            getSuprCapComData(&scpr_comTx);//获取裁判系统相关数据并处理
        Send_To_Scpr(&scpr_comTx);//发送信息
        SWDG_FEED(id);
    }
}

/**
 * @brief    初始化与超级电容端通信
 * @return   true:初始化成功	false:初始化失败
 * @author   chenhao
 */
rt_err_t Scpr_Com_Init(void)
{
    rt_thread_t Scpr_handler = RT_NULL; // 线程句柄

    /*注册CAN通信*/
    CAN_Init_Config_s config;
    config.can_device_name = SCPR_CAN_DEV;
    config.tx_id = SCPR_TX;
    config.rx_id = SCPR_RX;
    config.can_module_callback = Refresh_Scprdata;
    CAN_Attach(&scpr_caninstance,&config);

    // 初始化超级电容通信线程
    Scpr_handler = rt_thread_create(
        "SCR_ctrl",      // 线程名
        Scpr_Thread,     // 线程入口
        RT_NULL,         // 入口参数无
        THREAD_STACK_SC, // 线程栈
        THREAD_PRIO_SC,  // 线程优先级
        THREAD_TICK_SC); // 线程时间片大小

    // 线程创建失败返回false
    if (Scpr_handler == RT_NULL)
    {
        return RT_ERROR;
    }

    // 线程启动失败返回false
    if (rt_thread_startup(Scpr_handler) != RT_EOK)
    {
        return RT_ERROR;
    }

    // 创建线程定时器 定时释放信号量
    rt_timer_t timer = rt_timer_create("Scpr_Tx_timer",
                                       Scpr_Tx_IRQHandler,
                                       RT_NULL,
                                       SC_PERIOD,
                                       RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (rt_timer_start(timer)!= RT_EOK)
    {
        return RT_ERROR;
    }
    return RT_EOK;
}

/*----------------------------------------------外部超电数据获取接口-----------------------------------------------*/
/**
 * @brief 获取超级电容控制板当前是否在线
 * @author fwlh
 * @return rt_bool_t        超级电容控制板在线返回真
 */
rt_bool_t Read_Super_Capacity_Online(void)
{
    return ((scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 200)) ? RT_TRUE : RT_FALSE);
}

// 获得超电电压V
rt_err_t Get_SuprCapVoltage(float *V_cap)
{
    *V_cap = scpr_comRx.V_cap / 1000.f;
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 100)) 
        return RT_EOK;
    else
        return RT_ERROR;
}

// 获得超级电容充放电状态
SupCap_State_e Get_SuprCapState(void)
{
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 100))
    {
        if (scpr_comRx.V_cap > V_cap_data[V_cap_data_index == VCAP_DATA_NUM ? 0 : V_cap_data_index] + 5) //+x防止ADC数据浮动差影响
            Scp_Now_State = CAP_STATE_CHARGE;
        else if (scpr_comRx.V_cap < V_cap_data[V_cap_data_index == VCAP_DATA_NUM ? 0 : V_cap_data_index] - 5)
            Scp_Now_State = CAP_STATE_DISCHARGE;
        else
            Scp_Now_State = CAP_STATE_NO_WORK;
    }
    else
        Scp_Now_State = CAP_STATE_ERROR;
    return Scp_Now_State;
}

// 获得超级电容测得底盘总电流ma，电压mv，还有超级电容电压V
rt_bool_t getSuprCapMearData(float *volatge, float *current, float *V_Cap)
{
    if (volatge)
        *volatge = scpr_comRx.voltage;
    if (current)
        *current = scpr_comRx.current;
    if (V_Cap)
        *V_Cap = scpr_comRx.V_cap / 1000.f;
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 100))
        return RT_TRUE;
    else
        return RT_FALSE;
}

/***
 * @brief    获得超级电容剩余电量
 * @param    None
 * @return   剩余电容量,0~100。100为满电
 ***/
rt_bool_t Get_RemainCapcity(float *cap)
{
    *cap = scpr_comRx.cap * 100.f;
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 100))
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
}
//获取超级电容测量底盘功率
rt_bool_t Get_SuprCapPower(float *power)
{
    *power = scpr_comRx.power_cup;
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 100))
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
}

/**
 * @brief 获取超电是否输出
 * 
 * @return uint8_t 
 */
uint8_t Get_if_supercap_output(void)
{
    return !power_ctrl_close;
}

