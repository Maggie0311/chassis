#include "drv_canthread.h"
#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "func_MonHandling.h"
#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"
#endif

static struct rt_semaphore can1_rx_sem; // 用于接收消息的信号量
static struct rt_semaphore can2_rx_sem; // 用于接收消息的信号量

// CAN 设备句柄
rt_device_t can1_dev;
rt_device_t can2_dev;

/**
 * @brief  can1接收回调
 * @retval RT_EOK
 */
static rt_err_t Can1_Rx_Call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&can1_rx_sem);
    return RT_EOK;
}

/**
 * @brief  can1读取线程
 */
static void Can1_Rx_Thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};

#ifdef RT_CAN_USING_HDR

    rt_err_t res;

    MYCAN1_FILTER_ITEM // 硬件过滤表项具体内容

        struct rt_can_filter_config cfg = {MYCAN1_FILTER_ITEM_NUM, 1, items}; /* 一共有 2 个过滤表 */

    /* 设置硬件过滤表 */
    res = rt_device_control(can1_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);

#endif

    CREAT_ID(id);
    ADDTOMONITOR_ID("Can1_Rx_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 1, id);
    SWDG_START(id);

    while (1)
    {
        // hdr 值为 - 1，表示直接从 uselist 链表读取数据
        rxmsg.hdr = -1;
        // 阻塞等待接收信号量
        rt_sem_take(&can1_rx_sem, RT_WAITING_FOREVER);
        // 从 CAN 读取一帧数据
        rt_device_read(can1_dev, 0, &rxmsg, sizeof(rxmsg));
        can1_rec(&rxmsg);
        SWDG_FEED(id);
    }
}

/**
 * @brief  can2接收回调
 * @retval RT_EOK
 */
static rt_err_t Can2_Rx_Call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&can2_rx_sem);
    return RT_EOK;
}

/**
 * @brief  can2读取线程
 */
static void Can2_Rx_Thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};

#ifdef RT_CAN_USING_HDR

    rt_err_t res;

    MYCAN2_FILTER_ITEM // 硬件过滤表项具体内容

        struct rt_can_filter_config cfg = {MYCAN2_FILTER_ITEM_NUM, 1, items}; /* 一共有 4 个过滤表 */

    /* 设置硬件过滤表 */
    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);

#endif
    CREAT_ID(id);
    ADDTOMONITOR_ID("Can2_Rx_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 1, id);
    SWDG_START(id);

    while (1)
    {
        // hdr 值为 - 1，表示直接从 uselist 链表读取数据
        rxmsg.hdr = -1;
        // 阻塞等待接收信号量
        rt_sem_take(&can2_rx_sem, RT_WAITING_FOREVER);
        // 从 CAN 读取一帧数据
        rt_device_read(can2_dev, 0, &rxmsg, sizeof(rxmsg));
        can2_rec(&rxmsg);
        SWDG_FEED(id);
    }
}

/**
 * @brief  can1初始化，can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can1_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    // can接收中断信号量
    res = rt_sem_init(&can1_rx_sem, "can1_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    can1_dev = rt_device_find(CAN1_DEV_NAME);
    if (!can1_dev)
    {
        return RT_ERROR;
    }

    // 配置can驱动
    res = rt_device_open(can1_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (res != RT_EOK)
        return res;

    res = rt_device_control(can1_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    if (res != RT_EOK)
        return res;

    res = rt_device_control(can1_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    if (res != RT_EOK)
        return res;

    // 设置接收回调函数
    res = rt_device_set_rx_indicate(can1_dev, Can1_Rx_Call);
    if (res != RT_EOK)
        return res;

    // can线程初始化
    thread = rt_thread_create("can1_rx", Can1_Rx_Thread, RT_NULL,
                              THREAD_STACK_CAN1RX,
                              THREAD_PRIO_CAN1RX,
                              THREAD_TICK_CAN1RX);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
    {
        return RT_ERROR;
    }
    return RT_EOK;
}

/**
 * @brief  can2初始化，can2数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can2_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    // can接收中断信号量
    res = rt_sem_init(&can2_rx_sem, "can2_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    can2_dev = rt_device_find(CAN2_DEV_NAME);
    if (!can2_dev)
    {
        return RT_ERROR;
    }

    // 配置can驱动
    res = rt_device_open(can2_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (res != RT_EOK)
        return res;

    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    if (res != RT_EOK)
        return res;

    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    if (res != RT_EOK)
        return res;

    // 设置接收回调函数
    res = rt_device_set_rx_indicate(can2_dev, Can2_Rx_Call);
    if (res != RT_EOK)
        return res;

    // can线程初始化
    thread = rt_thread_create("can2_rx", Can2_Rx_Thread, RT_NULL,
                              THREAD_STACK_CAN2RX,
                              THREAD_PRIO_CAN2RX,
                              THREAD_TICK_CAN2RX);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
    {
        return RT_ERROR;
    }

    //todo 电机检测掉线线程需要完善一下
#if !defined(USING_STEER_WHEEL) && (defined(USING_CHECK_WHEEL_CAN))
    // can线程初始化
    thread = rt_thread_create("checkCan", checkCan, RT_NULL,
                              512,
                              2,
                              1);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
    {
        return RT_ERROR;
    }
#endif
    return RT_EOK;
}
