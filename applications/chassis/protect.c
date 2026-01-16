#include "protect.h"
#include "ThreadCtrl.h"
#include "drv_wheel.h"
#include "func_MonHandling.h"
#include "rtconfig.h"
#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"
#endif
#include "drv_can_com.h"
#ifdef BSP_CHASSIS_USE_SPI_CAN
#include "drv_spiCanMaster.h"
#endif
#include "app_monitor.h"
#ifdef MONITOR_NEED_ALARM
#include "func_Alarm.h"
#endif
#include "SuperCap_Com_new.h"

static rt_thread_t protct_Thread = RT_NULL; // 线程
static rt_timer_t protect_Timer = RT_NULL;  // 定时器
static struct rt_completion completion;

uint8_t Motorstuck[2][4];
motorOfflineAlarm_t offlineCtrl[WHEELS_GROUP_NUM][WHEEL_NUM];

// 自定义间隔声音
#ifdef BSP_USING_BUZZER
#define Buzzer_Diff()           \
    do                          \
    {                           \
        rt_thread_mdelay(500);  \
        set_buzzer(900, 1);     \
        rt_thread_mdelay(120);  \
        set_buzzer(0, 1);       \
        rt_thread_mdelay(1000); \
    } while (0)
#endif

// debug 电机掉线 +电机堵转
static uint8_t isAlarm[2] = {1, 1};

static void protect_Timeout_Handler(void *parameter)
{
    rt_completion_done(&completion);
}

// 排序
static void getIndexCanID(uint32_t arr[], uint16_t num)
{
    int n = num, i, j, temp, gap;
    for (gap = n / 2; gap > 0; gap /= 2)
    {
        for (i = gap; i < n; i++)
        {
            for (j = i - gap; j >= 0 && arr[j] > arr[j + gap]; j -= gap)
            {
                temp = arr[j];
                arr[j] = arr[j + gap];
                arr[j + gap] = temp;
            }
        }
    }
    (void)getIndexCanID;
}
// TODO 需要检查保护线程里关于底盘电机掉线的程序
// 驱动电机掉线检查 根据电机ID响
static void checkCan(void)
{
    int16_t isErr = 0;
    int group = 0, num = 0;
    CanRecordID_t data;

    //    isErr = SwdgGetIsSErr_Name("Can1_Rx_Thread");
    uint8_t isfind = 0;
    if (!isErr)
    { // 如果只有个别电机通信断掉
      // 得到当前can线上已经有的id
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 1
        data = *getCanID(Rx_Can1);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_RF][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_RF][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_RF][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 2
        data = *getCanID(Rx_Can2);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_RF][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_RF][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_RF][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LF_CAN_DEVICE_NAME == 1
        data = *getCanID(Rx_Can1);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_LF][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_LF][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_LF][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LF_CAN_DEVICE_NAME == 2
        data = *getCanID(Rx_Can2);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_LF][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_LF][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_LF][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LB_CAN_DEVICE_NAME == 1
        data = *getCanID(Rx_Can1);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_LB][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_LB][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_LB][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LB_CAN_DEVICE_NAME == 2
        data = *getCanID(Rx_Can2);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_LB][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_LB][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_LB][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RB_CAN_DEVICE_NAME == 1
        data = *getCanID(Rx_Can1);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_RB][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_RB][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_RB][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RB_CAN_DEVICE_NAME == 2
        data = *getCanID(Rx_Can2);
        for (int j = 0; j < data.Cannum; j++)
        {
            if (offlineCtrl[WHEEL_RB][WHEEL_RUN].id == data.RXcanDataID[j])
            { // 找到当前id并且没有离线
                isfind = 1;
                if (rt_tick_get() - data.refTick[j] < 500)
                    offlineCtrl[WHEEL_RB][WHEEL_RUN].isOffline = 0;
                else
                    offlineCtrl[WHEEL_RB][WHEEL_RUN].isOffline = 1;
            }
        }
    #endif


        // for (i = 0; i < 4; i++)
        // {
        //     if (1 == offlineCtrl[i].isOffline && isAlarm[0])
        //     { // 电机离线则根据电机顺序报警
        //         for (num = 0; num < i; num++)
        //         {
        //             set_buzzer(3000, 1);
        //             rt_thread_mdelay(80);
        //             set_buzzer(0, 1);
        //             rt_thread_mdelay(300);
        //         }
        //         Buzzer_Diff();
        //     }
        // }
    }
}
uint8_t GetMotorOffline(void)
{
    uint8_t isoffline=0;
    if(offlineCtrl[WHEEL_RF][WHEEL_RUN].isOffline == 1)
    {
        isoffline |= 1<<0;
    }
    if(offlineCtrl[WHEEL_LF][WHEEL_RUN].isOffline == 1)
    {
        isoffline |= 1<<1;
    }
    if(offlineCtrl[WHEEL_LB][WHEEL_RUN].isOffline == 1)
    {
        isoffline|= 1<<2;
    }
    if(offlineCtrl[WHEEL_RB][WHEEL_RUN].isOffline == 1)
    {
        isoffline|= 1<<3;
    }
    // if(offlineCtrl[WHEEL_RF][WHEEL_ANGLE].isOffline == 1)
    // {
    //     return 5;
    // }
    // if(offlineCtrl[WHEEL_LF][WHEEL_ANGLE].isOffline == 1)
    // {
    //     return 6;
    // }
    // if(offlineCtrl[WHEEL_LB][WHEEL_ANGLE].isOffline == 1)
    // {
    //     return 7;
    // }
    // if(offlineCtrl[WHEEL_RB][WHEEL_ANGLE].isOffline == 1)
    // {
    //     return 8;
    // }
    
    return isoffline;
}

static void protct_Thread_Entry(void *parameter)
{
    // 等待初始化完成
    rt_thread_mdelay(4000);
    // 初始化电机
    for (int group = 0; group < 8; group++)
    {
        offlineCtrl[group][WHEEL_RUN].isOffline = 1;
        offlineCtrl[group][WHEEL_ANGLE].isOffline = 1;
    }
    offlineCtrl[WHEEL_RF][WHEEL_RUN].id = RUN_RF_MOTOR_ID;
    offlineCtrl[WHEEL_RF][WHEEL_ANGLE].id = ANGLE_RF_MOTOR_ID;
    offlineCtrl[WHEEL_LF][WHEEL_RUN].id = RUN_LF_MOTOR_ID;
    offlineCtrl[WHEEL_LF][WHEEL_ANGLE].id = ANGLE_LF_MOTOR_ID;
    offlineCtrl[WHEEL_LB][WHEEL_RUN].id = RUN_LB_MOTOR_ID;
    offlineCtrl[WHEEL_LB][WHEEL_ANGLE].id = ANGLE_LB_MOTOR_ID;
	offlineCtrl[WHEEL_RB][WHEEL_RUN].id = RUN_RB_MOTOR_ID;
    offlineCtrl[WHEEL_RB][WHEEL_ANGLE].id = ANGLE_RB_MOTOR_ID;
#ifdef MONITOR_NEED_ALARM
    // 当监视器没有报警则可以自定义报警
    attachUserAlarmFun(checkCan);
#endif
    while (1)
    {
        rt_completion_wait(&completion, RT_WAITING_FOREVER);

#ifdef CORE_USING_RTT_MONITOR
        // 检查rtt是否正常工作
        rtt_ErrorHandle();
#endif

#ifndef MONITOR_NEED_ALARM
        // 当有监视器没有报警时则自己定义报警
        checkCan();
#endif
    }
}

// 监视器复位
rt_err_t monitorErrHandle(rt_bool_t statusid)
{
    Robot_Chassis_Reset(RT_TRUE);
    return RT_EOK;
}

/*用于处理各种异常*/
rt_err_t protct_ThreadInit(void)
{
#ifdef CORE_USING_RTT_MONITOR
    MX_TIM10_Init();
#endif

    /* 初始化完成量对象 */
    rt_completion_init(&completion);

    // 定时器创建
    protect_Timer = rt_timer_create(
        "protect timer", protect_Timeout_Handler,
        RT_NULL, THREAD_TICK_PEROID_PROTECT, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (protect_Timer)
    {
        if (rt_timer_start(protect_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    // 线程创建
    protct_Thread = rt_thread_create(
        "protect", protct_Thread_Entry, RT_NULL,
        THREAD_STACK_PROTECT, THREAD_PRIO_PROTECT, THREAD_TICK_PROTECT);

    // 查看是否创建成功
    if (protct_Thread != RT_NULL)
    {
        if (rt_thread_startup(protct_Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    return RT_EOK;
}
