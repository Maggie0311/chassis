#include "custom_message.h"
#include "app_monitor.h"
#include "ThreadCtrl.h"
#include "mod_RefSystem.h"
#include "string.h"
#include "drv_CRC.h"
#include "drv_utf.h"


static struct rt_timer Custom_Message_SendTimerSem; // 定时器
static rt_device_t serial;                          // 发送设备
static struct rt_event custom_message_event;        // 事件集

static uint8_t send_data[CUSTOM_DATA_NUM];

static uint8_t Senddata[64];

extern Message_t message[message_num];
extern Message_List_e Now_Message;

/***
* @name     帧头+ cmd_id (2-byte)+ data (n-byte) +frame_tail
* @brief	向裁判系统传输Custom_Message数据
* @param	senderID: 发送者ID:
            红方机器人	英雄 1; 工程 2; 步兵 3/4/5; 空中 6;哨兵 7；雷达站 9
            蓝方机器人  英雄 101; 工程 102; 步兵 103/104/105; 空中 106;哨兵 107；雷达站 109
* @param	receiverID: 接受者ID
            红方操作手客户端	英雄 0x101; 工程 0x102; 步兵 0x103/104/105; 空中 0x106
            蓝方操作手客户端    英雄 0x111; 工程 0x112; 步兵 0x113/114/115; 空中 0x116
***/
static rt_err_t Custom_Message_data_Send(void *send_data_p, uint8_t size, uint16_t senderID, uint16_t receiverID)
{
    rt_err_t res = RT_ERROR;

    uint8_t custom_data[size];
    memcpy(&custom_data, send_data_p, size);

    // 发送体初始化为0
    memset(Senddata, 0x00, sizeof(Senddata));

    /*命令码 机器人间交互数据，发送方触发发送，上限 30Hz*/
    /*帧头--SOF data_length seq CRC8---纯帧头5个字节*/
    Custom_Data_t data;
    data.txFrameHeader.SOF = HEADER_SOF;
    data.txFrameHeader.DataLength = sizeof(data.dataFrameHeader) + size; // 数据长度
    data.txFrameHeader.Seq = 0x01;
    data.txFrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&data.txFrameHeader,
                                                 sizeof(data.txFrameHeader) - sizeof(data.txFrameHeader.CRC8), CRC8_INIT);

    /*数据段的内容ID+发送者ID+接收者内容ID+数据段---所以数据长度加6*/
    data.CmdID = ID_custom_data;
    // data.dataFrameHeader.data_cmd_id = cmd_id;
    data.dataFrameHeader.sender_id = senderID;
    data.dataFrameHeader.receiver_id = receiverID;
    memcpy(&data.custom_data, &custom_data, size); /*数据*/

    /*帧尾*/
    // 用于帧尾校验的长度(除去帧尾以外的数据长度)
    uint8_t size_ex_tail = sizeof(data.txFrameHeader) + sizeof(data.CmdID) + sizeof(data.dataFrameHeader) + size;
    data.FrameTail = Get_CRC16_Check_Sum((uint8_t *)&data, size_ex_tail, CRC16_INIT);

    /*赋值给最终的发送数组*/
    memcpy(Senddata, &data, size_ex_tail);
    memcpy(Senddata + size_ex_tail, (const void *)&data.FrameTail, sizeof(data.FrameTail));

    res = rt_device_write(serial, 0, Senddata, size_ex_tail + sizeof(data.FrameTail));
    // 发送单行发送完成事件
    Custom_Message_sendEvent(custom_message_sendReady_eve);
    return res;
}

static void Custom_Message_SendTimerSem_IRQHandler(void *parameter)
{
    // rt_sem_release(&Custom_Message_sem);
    Custom_Message_sendEvent(custom_message_timer_eve);
}
/****************************************************************** */
static void Message_Init(void)
{
    uint16_t i = 0, j = 0;
    // 初始化用户消息数据
    for (i = 0; i < message_num; i++)
    {
        if ((message[i].message_type == Message_Type_SingleLine || message[i].message_type == Message_Type_MultiLine) &&
            message[i].message_line_num != 0)
        {
            for (j = 0; j < message[i].message_line_num; j++)
                utf8_to_utf16((uint8_t *)message[i].message_data_utf8[j], (uint16_t *)message[i].message_data_utf16[j], CUSTOM_DATA_NUM / 2);
        }
        else if ((message[i].message_type == Message_Type_Montage_SingleLine || message[i].message_type == Message_Type_Montage_MultiLine) &&
                 message[i].init_line_num != 0)
        {
            for (j = 0; j < message[i].init_line_num; j++)
                utf8_to_utf16((uint8_t *)message[i].message_data_utf8[j], (uint16_t *)message[i].message_data_utf16[j], CUSTOM_DATA_NUM / 2);
        }
    }
}

static void Custom_Message_Update_Thread(void *parameter)
{
    while (1)
    {
        Select_Send_Message();
        Custom_Message_sendEvent(custom_message_dataReady_eve);
    }
}

static void Custom_Message_Transmit_Thread(void *parameter)
{
    while (1)
    {
        Custom_Message_waitEvent(custom_message_dataReady_eve);
        // rt_sem_take(&Custom_Message_sem, RT_WAITING_FOREVER);
        if (message[Now_Message].message_type == Message_Type_Montage_SingleLine)
            message_cat_single(Now_Message);
        else if (message[Now_Message].message_type == Message_Type_Montage_MultiLine)
            message_cat_multi(Now_Message);
        do
        {
            Custom_Message_waitForSend();
            memset(send_data, 0x00, sizeof(send_data));
            rt_memcpy(send_data, (void *)message[Now_Message].message_data_utf16[message[Now_Message].send_line_num++], CUSTOM_DATA_NUM);
            // 获取当前机器人ID:REF_ROBO_ID
            // Ref_Robot_ID();//!
            Custom_Message_data_Send((void *)&send_data, CUSTOM_DATA_NUM, REF_ROBO_ID, REF_CLIENT_ID);
        } while (message[Now_Message].send_line_num != message[Now_Message].message_line_num);
        message[Now_Message].send_line_num = 0;
        Custom_Message_sendEvent(custom_message_sendFinish_eve);
    }
}

// 线程初始化
rt_err_t Custom_Message_Data_Send_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    serial = rt_device_find(BSP_REF_USART_NAME);
    if (!serial)
        return RT_ERROR;

    Message_Init();

    thread = rt_thread_create("Custom_Message_send",
                              Custom_Message_Transmit_Thread,
                              RT_NULL,
                              THREAD_STACK_CUSTOM_MESSAGE,
                              THREAD_PRIO_CUSTOM_MESSAGE,
                              THREAD_TICK_CUSTOM_MESSAGE);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    // 信号量初始化
    // rt_sem_init(&Custom_Message_sem, "custom_message_sem", 0, RT_IPC_FLAG_PRIO);
    // 初始化事件对象
    Custom_Message_event_Init();

    // 创建线程定时器
    rt_timer_init(&Custom_Message_SendTimerSem,
                  "sentry_task",
                  Custom_Message_SendTimerSem_IRQHandler,
                  RT_NULL,
                  350,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // 启动定时器
    res = rt_timer_start(&Custom_Message_SendTimerSem);
    if (res != RT_EOK)
        return res;

    // 得到待发送数据的线程 时间片轮询
    thread = rt_thread_create("Message_Updata_Thread",
                              Custom_Message_Update_Thread,
                              RT_NULL,
                              THREAD_STACK_CUSTOM_MESSAGE,
                              THREAD_PRIO_CUSTOM_MESSAGE - 1,
                              THREAD_TICK_CUSTOM_MESSAGE);
    if (thread != RT_NULL)
    {
        // 启动线程
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    return RT_EOK;
}

rt_err_t Custom_Message_event_Init(void)
{
    rt_err_t result = RT_ERROR;
    /* 初始化事件对象 */
    result = rt_event_init(&custom_message_event, "custom_message_event", RT_IPC_FLAG_PRIO);
    if (result != RT_ERROR)
    {
        Custom_Message_sendEvent(custom_message_sendReady_eve);
        Custom_Message_sendEvent(custom_message_sendFinish_eve); // 首先发送该事件表示上一个消息发送完成
    }
    return result;
}

/*
 每周期检查是否可以发送
 检查是否需要发送
 检查上一次是否发送完成即是否可以发送
*/
rt_err_t Custom_Message_waitForSend(void)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_recv(&custom_message_event, (custom_message_timer_eve | custom_message_sendReady_eve),
                           RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                           RT_WAITING_FOREVER, RT_NULL);
    return result;
}

rt_err_t Custom_Message_sendEvent(Custom_Message_threadsyn_e event)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_send(&custom_message_event, event);
    return result;
}

rt_err_t Custom_Message_waitEvent(Custom_Message_threadsyn_e event)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_recv(&custom_message_event, event, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                           RT_WAITING_FOREVER, RT_NULL);
    return result;
}

uint8_t message_cat_block(Message_List_e index, uint8_t det, uint8_t src)
{
    if (message[index].cnt >= CUSTOM_DATA_NUM)
        return 0;
    uint8_t temp = 0;
    temp = utf16_strnlen((void *)message[index].message_data_utf16[src], CUSTOM_DATA_NUM);
    if (temp > CUSTOM_DATA_NUM - message[index].cnt)
        return 0;
    rt_memcpy((void *)((uint8_t *)(message[index].message_data_utf16[det]) + message[index].cnt),
              (void *)message[index].message_data_utf16[src], temp);
    message[index].cnt += temp;
    return 1;
}

uint8_t message_cat_single(Message_List_e index)
{
    message[index].cnt = 0;
    rt_memset((void *)message[index].message_data_utf16[0], 0x00, CUSTOM_DATA_NUM);
    for (uint8_t i = 0; message[index].message_montage_order[i] != 0; i++)
        if (!message_cat_block(index, 0, message[index].message_montage_order[i]))
            return 0;
    return 1;
}

uint8_t message_cat_multi(Message_List_e index)
{
    message[index].cnt = 0;
    uint8_t i = 0;
    for (uint8_t det = 0; det < message[index].message_line_num; det++)
        rt_memset((void *)message[index].message_data_utf16[det], 0x00, CUSTOM_DATA_NUM);
    for (uint8_t det = 0; det < message[index].message_line_num; det++)
    {
        for (; message[index].message_montage_order[i] != 0; i++)
            if (!message_cat_block(index, 0, message[index].message_montage_order[i]))
                break;
        if (message[index].message_montage_order[i] == 0)
            return 1;
    }
    return 0;
}
