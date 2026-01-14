// 哨兵向裁判系统通讯发送功能文件
// 目前此文件的功能有：哨兵自主决策指令，机器人间通信(与雷达通信)
#include "sentry_cmd.h"
#include "app_monitor.h"
#include "ThreadCtrl.h"
#include "string.h"
#include "drv_CRC.h"
#include "app_RefSystem.h"
#include "func_Visual_Com.h"
// #include "HRef_ID.h"

static struct rt_timer SentrySendTimerSem; // 定时器
static struct rt_semaphore Sentry_cmd_sem; // 信号量


/*********发送和接收的数据（全局变量）******* */
sentry_cmd_info_t Sentry_cmd_data;
radar_RxData_t radar_Rxdata;                     /*接收到的雷达站的数据*/
radar_TxData_t radar_Txdata;                     /*发送给雷达站的数据*/
radar_cardata_t radar_data[VISUAL_DATA_NUM_MAX]; // 视觉给雷达的机器人识别数据缓存

uint8_t radar_Rx_falg;
uint8_t radar_Tx_flag;

// uint8_t senddata[Robot_Interact_DataLen]; // 要发送的用户数据
/*最终发送给裁判系统的数据-带帧头帧尾等数据 可以debug*/
uint8_t Sentry_Senddata[MAX_Sentry_Data_BUFF_SIZE];

static rt_device_t serial; // 发送设备

/***
* @name     帧头+ cmd_id (2-byte)+ data (n-byte) +frame_tail
* @brief	向裁判系统传输UI数据
* @param	senderID: 发送者ID:
            红方机器人	英雄 1; 工程 2; 步兵 3/4/5; 空中 6;哨兵 7；雷达站 9
            蓝方机器人  英雄 101; 工程 102; 步兵 103/104/105; 空中 106;哨兵 107；雷达站 109
* @param	receiverID: 接受者ID
            红方操作手客户端	英雄 0x101; 工程 0x102; 步兵 0x103/104/105; 空中 0x106
            蓝方操作手客户端    英雄 0x111; 工程 0x112; 步兵 0x113/114/115; 空中 0x116
***/
static rt_err_t Sentry_data_Send(void *send_data_p, uint8_t size, uint16_t cmd_id, uint16_t senderID, uint16_t receiverID)
{
    rt_err_t res = RT_ERROR;

    uint8_t custom_data[size];
    memcpy(&custom_data, send_data_p, size);

    // 发送体初始化为0
    memset(Sentry_Senddata, 0x00, sizeof(Sentry_Senddata));

    /*命令码 机器人间交互数据，发送方触发发送，上限 30Hz*/
    /*帧头--SOF data_length seq CRC8---纯帧头5个字节*/
    Interact_Data_t data;
    data.txFrameHeader.SOF = HEADER_SOF;
    data.txFrameHeader.DataLength = sizeof(data.dataFrameHeader) + size; // 数据长度
    data.txFrameHeader.Seq = 0x01;
    data.txFrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&data.txFrameHeader,
                                                 sizeof(data.txFrameHeader) - sizeof(data.txFrameHeader.CRC8), CRC8_INIT);

    /*数据段的内容ID+发送者ID+接收者内容ID+数据段---所以数据长度加6*/
    data.CmdID = ID_student_interactive_data;
    data.dataFrameHeader.data_cmd_id = cmd_id;
    data.dataFrameHeader.send_ID = senderID;
    data.dataFrameHeader.receiver_ID = receiverID;
    memcpy(&data.send_data, &custom_data, size); /*数据*/

    /*帧尾*/
    // 用于帧尾校验的长度(除去帧尾以外的数据长度)
    uint8_t size_ex_tail = sizeof(data.txFrameHeader) + sizeof(data.CmdID) + sizeof(data.dataFrameHeader) + size;
    data.FrameTail = Get_CRC16_Check_Sum((uint8_t *)&data, size_ex_tail, CRC16_INIT);

    /*赋值给最终的发送数组*/
    memcpy(Sentry_Senddata, &data, size_ex_tail);
    memcpy(Sentry_Senddata + size_ex_tail, (const void *)&data.FrameTail, sizeof(data.FrameTail));

    res = rt_device_write(serial, 0, Sentry_Senddata, size_ex_tail + sizeof(data.FrameTail));

    return res;
}

static void SentrySendTimerSem_IRQHandler(void *parameter)
{
    rt_sem_release(&Sentry_cmd_sem);
}
/****************************************************************** */

void protocolRadarData(const Frame_t *const frame)
{
    // Ref_Robot_ID();
    if (frame->Data.ext_rec_user_data.dataFrameHeader.data_cmd_id == 0x0200 &&
        frame->Data.ext_rec_user_data.dataFrameHeader.send_ID == REF_RADAR_ID &&
        frame->Data.ext_rec_user_data.dataFrameHeader.receiver_ID == REF_ROBO_ID)
    {
        rt_memcpy(&radar_Rxdata, (uint8_t *)frame->Data.ext_rec_user_data.userData.data, sizeof(radar_RxData_t) - sizeof(rt_tick_t));
        radar_Rxdata.tick = rt_tick_get();
    }
}

uint8_t get_Radar_Data(void)
{
    if (rt_tick_get() - radar_Rxdata.tick <= 110) // 110ms内都相信此数据
    {
        return radar_Rxdata.receive_flag;
    }
    return 0;
}

// 获取云台手是否有效按下雷达键值("R")
uint8_t get_radar_commd_keyboard(void)
{
    uint8_t ret = 0;
    uint8_t temp;
    if (rt_tick_get() - GetRefDataRefreshTick(Index_robot_command_map) <= 70) // 70ms内都相信此数据
    {
        getRefData(memberGet_parm(temp,ext_robot_command_map,commd_keyboard));
        if (temp == 'R')
            ret = 1;
    }
    return ret;
}

void Sentry_Transmit_Thread(void *parameter)
{
    //    static sentry_cmd_info_t Sentry_cmd_data_last; // 最近一次发送的数据
    while (1)
    {
        rt_sem_take(&Sentry_cmd_sem, RT_WAITING_FOREVER);

        Sentry_cmd_data = get_sentry_cmd_data();
        //        if (*(uint32_t *)&Sentry_cmd_data != *(uint32_t *)&Sentry_cmd_data_last)
        //        {
        // 获取当前机器人ID:REF_ROBO_ID
        // Ref_Robot_ID();
        // 发送数据
        // 接受者ID:REF_SERVER_ID:0x8080：裁判系统服务器（用于哨兵和雷达自主决策指令）
        Sentry_data_Send((void *)&Sentry_cmd_data, 4, 0x0120, REF_ROBO_ID, REF_SERVER_ID);
        //            Sentry_cmd_data_last = Sentry_cmd_data;
        //        }
        rt_sem_take(&Sentry_cmd_sem, RT_WAITING_FOREVER);
        // Ref_Robot_ID();
        radar_Rx_falg = get_Radar_Data();
        if (radar_Rx_falg == 1) // 雷达站表示已经收到
            radar_Tx_flag = 0;
        else if (get_radar_commd_keyboard())
            radar_Tx_flag = 1;
        radar_Txdata.send_flag = radar_Tx_flag;
        rt_memcpy(&radar_Txdata.radar_cardata, &radar_data, sizeof(radar_Txdata.radar_cardata));
        rt_memset(&radar_data, 0x00, sizeof(radar_data));
        if (REF_TEAM_COLOR == BLUE_TEAM)
            Sentry_data_Send((void *)&radar_Txdata, sizeof(radar_Txdata), 0x0201, REF_ROBO_ID, REF_ROBO_B9_RADAR);
        else if (REF_TEAM_COLOR == RED_TEAM)
            Sentry_data_Send((void *)&radar_Txdata, sizeof(radar_Txdata), 0x0201, REF_ROBO_ID, REF_ROBO_R9_RADAR);
    }
}

// 线程初始化
rt_err_t Sentry_Data_Send_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    serial = rt_device_find(BSP_REF_USART_NAME);
    if (!serial)
        return RT_ERROR;

    thread = rt_thread_create("sentry_cmd_send",
                              Sentry_Transmit_Thread,
                              RT_NULL,
                              THREAD_STACK_SENTRY,
                              THREAD_PRIO_SENTRY,
                              THREAD_TICK_SENTRY);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    // 信号量初始化
    rt_sem_init(&Sentry_cmd_sem, "sentry_cmd", 0, RT_IPC_FLAG_PRIO);

    // 创建线程定时器
    rt_timer_init(&SentrySendTimerSem,
                  "sentry_task",
                  SentrySendTimerSem_IRQHandler,
                  RT_NULL,
                  60,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // 启动定时器
    res = rt_timer_start(&SentrySendTimerSem);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}
