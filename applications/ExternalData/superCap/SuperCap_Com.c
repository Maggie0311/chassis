#include "SuperCap_Com.h"
#include "drv_canthread.h"
#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "fun_protocolGim.h"
#include "app_RefSystem.h"

#define USED(x) ((void)(x))

static float remain_cap;         // 剩余电容量,0.0~1.0。1.0为满电
static float V_Cap;              // 超级电容电压（二代电容不要低于15V）
static rt_tick_t Fresh_Tick = 0; // 上次接受数据刷新的时刻
// static uint8_t suprID;
/***
 * @brief    向超级电容端发送数据
 * @note     当输入都为-1时，表示告知超级电容，裁判系统的通信断开
 * @param    Whether_To_Output 当前是否允许放电
 * @param    power_buffer      底盘功率缓冲
 * @param    power_limit       底盘功率上限
 * @return   rt_err_t：can2报文发送成功or失败
 ***/
static rt_err_t Send_To_Scpr(uint8_t Whether_To_Output, uint8_t recvFlag,
                             rt_uint16_t power_buffer, rt_uint16_t power_limit)
{
    struct rt_can_msg tx_Scpr;

    tx_Scpr.id = SCPR_TX;       // 设置ID
    tx_Scpr.ide = RT_CAN_STDID; // 标准帧
    tx_Scpr.rtr = RT_CAN_DTR;   // 数据帧
    tx_Scpr.priv = 2;           // 报文优先级次次高
    tx_Scpr.len = 8;            // 长度8

    if (power_limit == 0)
        power_limit = SC_POWERT_LIM;

    // 发送数据
    tx_Scpr.data[0] = Whether_To_Output;
    tx_Scpr.data[1] = recvFlag;
    tx_Scpr.data[4] = power_buffer >> 8;
    tx_Scpr.data[5] = power_buffer;
    tx_Scpr.data[6] = power_limit >> 8;
    tx_Scpr.data[7] = power_limit;

    if (!CanTrans(USED_CAN2, &tx_Scpr))
        return RT_ERROR;
    else
        return RT_EOK;
}

static void GetCtrlData(scpr_ctrlCmd_t *Cmd)
{
    static rt_tick_t lasttick;
    uint32_t tick = GetRefDataRefreshTick(Index_power_heat_data);
    if (lasttick != tick)
    {
        Cmd->powerRefFlag = 1;
        lasttick = tick;
    }
    // Cmd->Whether_To_Charge = Get_Charge_Cmd();
    getRefData(memberGet_parm(Cmd->chassis_powerBu,ext_power_heat_data,chassis_power_buffer));
    getRefData(memberGet_parm(Cmd->chassis_powerLim,ext_game_robot_status,chassis_power_limit));
    Cmd->heatDataTick = GetRefDataRefreshTick(Index_power_heat_data);
    Cmd->maxPower = GetGimMaxPower(); // 当云台端裁认为裁判系统异常则会给出一个底盘最大功率
}

/***
 * @brief    与超级电容端通信线程
 ***/
static void Scpr_Thread(void *parameter)
{
    rt_uint8_t first_fg = 0; // 是否第一次进入
    scpr_ctrlCmd_t ctrlCmd;

    CREAT_ID(id);
    ADDTOMONITOR_ID("Scpr_Thread", 1500, NULL, ALARM_RED, 1, id);
    SWDG_START(id);

    ADDTOMONITOR_ID("Scpr_RX", 3000, NULL, ALARM_RED, 1, suprID);
    SWDG_START(suprID);
    USED(remain_cap); /* To avoid compiler warnings */

    while (1)
    {
        GetCtrlData(&ctrlCmd);
        if (first_fg && (rt_tick_get() - ctrlCmd.heatDataTick < 800) &&
            ctrlCmd.chassis_powerLim >= SC_POWERT_LIM)
        {
            Send_To_Scpr(ctrlCmd.Whether_To_Charge, ctrlCmd.powerRefFlag,
                         ctrlCmd.chassis_powerBuf, ctrlCmd.chassis_powerLim);
        }
        else
        {
            /*发送给超级电容端的数据全部置-1,与裁判系统通信断掉，告知超级电容端*/
            // ctrlCmd.maxPower = 120; // debug用
            Send_To_Scpr(ctrlCmd.Whether_To_Charge, (uint8_t)-1, (uint16_t)-1, ctrlCmd.maxPower);

            /*第一次发送报文*/
            if (!first_fg)
                first_fg = 1;
        }

        SWDG_FEED(id);
        rt_thread_mdelay(SC_PERIOD);
    }
}

static void Refresh_Scprdata(struct rt_can_msg *msg);
/**
 * @brief    初始化与超级电容端通信
 * @param [in]	无
 * @return   true:初始化成功	false:初始化失败
 * @author  lfp
 */
rt_err_t Scpr_Com_Init(void)
{
    rt_thread_t Scpr_handler = RT_NULL; // 线程句柄

    attachCanRXfun(rx_scpr, Refresh_Scprdata); // 注册can接受

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

    return RT_EOK;
}

/*------------------------------------RX---------------------------------------------------------*/

/***
 * @brief    处理超级电容端发送报文
 * @param    msg can2报文
 ***/
static void Refresh_Scprdata(struct rt_can_msg *msg)
{
    remain_cap = *(float *)(&msg->data[0]);
    V_Cap = *(float *)(&msg->data[4]);
    Fresh_Tick = rt_tick_get();
    SWDG_FEED(suprID);
}

/**
 * @brief 获取超级电容控制板当前是否在线
 * @author fwlh
 * @return rt_bool_t        超级电容控制板在线返回真
 */
rt_bool_t Read_Super_Capacity_Online(void)
{
    return ((Fresh_Tick && (rt_tick_get() - Fresh_Tick < 200)) ? RT_TRUE : RT_FALSE);
}

/***
 * @brief    获得超级电容剩余电量
 * @param    None
 * @return   剩余电容量,0~100。100为满电
 ***/
float Get_RemainCapcity(void)
{
#ifdef USE_CAPACITY_2
    if (Fresh_Tick && (rt_tick_get() - Fresh_Tick < 500))
        return remain_cap * 100.f;
    else
        return 40.f;
#elif defined(USE_CAPACITY_3)
    if (scpr_comRx.recvTick && (rt_tick_get() - scpr_comRx.recvTick < 500))
        return scpr_comRx.cap * 100.f;
    else
        return 0;
#else
    return 100.f;
#endif /* USE_CAPACITY */
}

float Get_Cap_Voltage(void)
{
    if (rt_tick_get() - Fresh_Tick <= 500)
        return V_Cap;
    else
        return 0;
}
