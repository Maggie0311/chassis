#include "drv_spiCanMaster.h"
#include "app_monitor.h"
#include "ThreadCtrl.h"
#include "HCanID_data.h"
#include "drv_can_com.h"

static uint16_t spiCanSwdgID;          // 看门狗id
static struct rt_event SPI_CAN_event;
static SpicanRx_fun_p spiCanRx_fun[spiCanrxAll]; // 接受回调

SPICAN_MsgOnTransfer_t SpiCanData[SPI_CAN_ALL] = {0}; // 发送的数据
uint8_t control[SPI_CAN_ALL] = {1, 1};                // debug
CanRecordID_t SpiCanID[SPI_CAN_ALL];                  // 统计的canID

typedef enum
{
    SPI_CAN1_event = 1 << 1,
    SPI_CAN2_event = 1 << 2,
} SPI_CAN_event_t;

// 0为CAN1,1为CAN2
CanRecordID_t *SPI_CAN_getCanID(uint8_t motor)
{
    if (motor < SPI_CAN_ALL)
        return &SpiCanID[motor];
    else
        return NULL;
}
//TODO 需要修改CAN接收部分逻辑
// CAN接收回调函数
void SPI_CAN_ReceCallback(SPICAN_MsgOnTransfer_t *Msg)
{
#if defined USING_STEER_WHEEL
    SWDG_FEED(spiCanSwdgID);
    if (Msg->ForCANx == SPI_CAN1)
    {
        RecordId(&SpiCanID[SPI_CAN1], Msg->id);
        switch (Msg->id)
        {
        case ANGLE_RF_MOTOR_ID: // 底盘电机
        case ANGLE_LF_MOTOR_ID:
        case ANGLE_LB_MOTOR_ID:
        case ANGLE_RB_MOTOR_ID:
            if (NULL != spiCanRx_fun[rx_chasMotor_steer])
                spiCanRx_fun[rx_chasMotor_steer](Msg);
            return;
        default:
            return;
        }
    }
    else if (Msg->ForCANx == SPI_CAN2)
    {
        RecordId(&SpiCanID[SPI_CAN2], Msg->id);
        switch (Msg->id)
        {
        case RUN_RF_MOTOR_ID:
        case RUN_LF_MOTOR_ID:
        case RUN_LB_MOTOR_ID:
        case RUN_RB_MOTOR_ID:
            if (NULL != spiCanRx_fun[rx_chasMotor_wheel])
                spiCanRx_fun[rx_chasMotor_wheel](Msg);
            return;
        default:
            return;
        }
    }
#endif
}

/**
 * @brief 添加从机CAN的回调
 * @param {SPI_CAN_CallBack_e} motorType 3508或者6020
 * @param {SpicanRx_fun_p} fun 参数为can的8字节的内容
 */
void AttachSpiCanCallBack(SPI_CAN_CallBack_e motorType, SpicanRx_fun_p fun)
{
    if (motorType < spiCanrxAll)
        spiCanRx_fun[motorType] = fun;
}
/**
 * 
 * @brief spi_can发送0数据
 * @param {St_Wheel_e} type
 * @return {*}
 */
//note 该函数不适配舵向电机或者轮向电机不在同一个can线上的情况
rt_err_t SPI_CANSTransPIDOut(SPI_CAN_e type, Motor_t *motor)
{
    rt_err_t res = RT_ERROR;
    int i = 0;
    SPI_CAN_e SPI_CAN_type = SPI_CAN1;

    SpiCanData[SPI_CAN_type].ide = 0;
    SpiCanData[SPI_CAN_type].rtr = 0;
    SpiCanData[SPI_CAN_type].len = 8;

    if (type == SPI_CAN1)
    {
        res = RT_EOK;
        SPI_CAN_type = type;                        // 发送数据缓冲
        SpiCanData[SPI_CAN_type].id = STDID_6020_1; // 控制报文标识符
        SpiCanData[SPI_CAN_type].ForCANx = SPI_CAN1;

        for (i = 0; i < 4; i++)
        {
            // 两个为一组 舵
            SpiCanData[SPI_CAN_type].data[(motor[i].dji.motorID - RBID_6020) * 2] = (rt_uint8_t)((rt_int16_t)motor[i].spe.out >> 8);
            SpiCanData[SPI_CAN_type].data[(motor[i].dji.motorID - RBID_6020) * 2 + 1] = (rt_uint8_t)((rt_int16_t)motor[i].spe.out);
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN1_event);
    }
    else if (type == SPI_CAN2)
    {
        res = RT_EOK;
        SPI_CAN_type = type;
        SpiCanData[SPI_CAN_type].id = STDID_3508_1; // 控制报文标识符
        SpiCanData[SPI_CAN_type].ForCANx = SPI_CAN2;

        for (i = 0; i < 4; i++)
        {
            // 两个为一组 轮
            SpiCanData[SPI_CAN_type].data[(motor[i].dji.motorID - RBID_3508) * 2] = (rt_uint8_t)((rt_int16_t)motor[i].spe.out >> 8);
            SpiCanData[SPI_CAN_type].data[(motor[i].dji.motorID - RBID_3508) * 2 + 1] = (rt_uint8_t)((rt_int16_t)motor[i].spe.out);
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN2_event);
    }
    return res;
}


/**
 * @brief spi_can发送0数据
 * @param {St_Wheel_e} type
 * @return {*}
 */
rt_err_t SPI_CANSTrans(SPI_CAN_e type, SPICAN_MsgOnTransfer_t *Msg)
{
    rt_err_t res = RT_ERROR;
    int i = 0;
    SPI_CAN_e SPI_CAN_type = type;

    if (type == SPI_CAN1)
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN1].ide = 0;    // 发送数据缓冲
        SpiCanData[SPI_CAN1].rtr = 0;
        SpiCanData[SPI_CAN1].len = 8;
        SpiCanData[SPI_CAN1].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN1].ForCANx = SPI_CAN1;

        for(i=0;i<sizeof(SpiCanData[SPI_CAN1].data) / sizeof(SpiCanData[SPI_CAN1].data[0]);i++)
        {
            SpiCanData[SPI_CAN1].data[i] = Msg->data[i];
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN1_event);
    }
    else if (type == SPI_CAN2)
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN2].ide = 0;
        SpiCanData[SPI_CAN2].rtr = 0;
        SpiCanData[SPI_CAN2].len = 8;
        SpiCanData[SPI_CAN2].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN2].ForCANx = SPI_CAN2;

        for(i=0;i<sizeof(SpiCanData[SPI_CAN2].data) / sizeof(SpiCanData[SPI_CAN2].data[0]);i++)
        {
            SpiCanData[SPI_CAN2].data[i] = Msg->data[i];
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN2_event);
    }
    else if(type == SPI_CAN_1_2)    //当舵向电机或者轮向电机不全在一个can线上时，需要将该数据包先后在can1和can2分别发送一次
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN1].ide = 0;
        SpiCanData[SPI_CAN1].rtr = 0;
        SpiCanData[SPI_CAN1].len = 8;
        SpiCanData[SPI_CAN1].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN1].ForCANx = SPI_CAN1;
        for(i=0;i<sizeof(SpiCanData[SPI_CAN1].data) / sizeof(SpiCanData[SPI_CAN1].data[0]);i++)
        {
            SpiCanData[SPI_CAN1].data[i] = Msg->data[i];
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN1_event);

        res = RT_EOK;
        SpiCanData[SPI_CAN2].ide = 0;
        SpiCanData[SPI_CAN2].rtr = 0;
        SpiCanData[SPI_CAN2].len = 8;
        SpiCanData[SPI_CAN2].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN2].ForCANx = SPI_CAN2;
        for(i=0;i<sizeof(SpiCanData[SPI_CAN2].data) / sizeof(SpiCanData[SPI_CAN2].data[0]);i++)
        {
            SpiCanData[SPI_CAN2].data[i] = Msg->data[i];
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN2_event);
    } 
    return res;
}

/**
 * @brief spi_can发送0数据
 * @param {St_Wheel_e} type
 * @return {*}
 */
rt_err_t SPI_CANSTransNULL(SPI_CAN_e type,SPICAN_MsgOnTransfer_t *Msg)
{
    rt_err_t res = RT_ERROR;

    int i = 0;
    SPI_CAN_e SPI_CAN_type = type;

    if (type == SPI_CAN1)
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN1].ide = 0;
        SpiCanData[SPI_CAN1].rtr = 0;
        SpiCanData[SPI_CAN1].len = 8;
        SpiCanData[SPI_CAN1].id = STDID_6020_1;  // 控制报文标识符
        SpiCanData[SPI_CAN1].ForCANx = SPI_CAN1; // 从机can1

        for (i = 0; i < sizeof(SpiCanData[SPI_CAN1].data) / sizeof(SpiCanData[SPI_CAN1].data[0]); i++)
            SpiCanData[SPI_CAN1].data[i] = 0;

        rt_event_send(&SPI_CAN_event, SPI_CAN1_event);
    }
    else if (type == SPI_CAN2)
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN2].ide = 0;
        SpiCanData[SPI_CAN2].rtr = 0;
        SpiCanData[SPI_CAN2].len = 8;
        SpiCanData[SPI_CAN2].id = STDID_3508_1;  // 控制报文标识符
        SpiCanData[SPI_CAN2].ForCANx = SPI_CAN2; // 从机can2

        for (i = 0; i < sizeof(SpiCanData[SPI_CAN2].data) / sizeof(SpiCanData[SPI_CAN2].data[0]); i++)
            SpiCanData[SPI_CAN2].data[i] = 0;

        rt_event_send(&SPI_CAN_event, SPI_CAN2_event);
    }
    else if(type == SPI_CAN_1_2)    //当舵向电机或者轮向电机不全在一个can线上时，需要将该数据包先后在can1和can2分别发送一次
    {
        res = RT_EOK;
        SpiCanData[SPI_CAN1].ide = 0;
        SpiCanData[SPI_CAN1].rtr = 0;
        SpiCanData[SPI_CAN1].len = 8;
        SpiCanData[SPI_CAN1].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN1].ForCANx = SPI_CAN1;
        for(i=0;i<sizeof(SpiCanData[SPI_CAN1].data) / sizeof(SpiCanData[SPI_CAN1].data[0]);i++)
        {
            SpiCanData[SPI_CAN1].data[i] = 0;
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN1_event);

        res = RT_EOK;
        SpiCanData[SPI_CAN2].ide = 0;
        SpiCanData[SPI_CAN2].rtr = 0;
        SpiCanData[SPI_CAN2].len = 8;
        SpiCanData[SPI_CAN2].id = Msg->id; // 控制报文标识符
        SpiCanData[SPI_CAN2].ForCANx = SPI_CAN2;
        for(i=0;i<sizeof(SpiCanData[SPI_CAN2].data) / sizeof(SpiCanData[SPI_CAN2].data[0]);i++)
        {
            SpiCanData[SPI_CAN2].data[i] = 0;
        }
        rt_event_send(&SPI_CAN_event, SPI_CAN2_event);
    } 
    return res;
}

rt_err_t DrvSpiCanInit(void)
{
    rt_err_t res = RT_ERROR;
    ADDTOMONITOR_ID("SPI_CAN_RX", 1000, MONITOR_DEHANDLER, ALARM_RED, 1, spiCanSwdgID);
    SWDG_START(spiCanSwdgID);

    // spi底层初始化
    drv_HW_SPI_Init();
    SPI_CAN_Init();

    // 设置SPI主机接收回调函数
    SPI_CAN_Set_ReceiveFun(0, SPI_CAN_ReceCallback);

    // spi_can发送线程
    res = SPI_CANThreadTX_Init();
    return res;
}

/*------------------------------------test----------------------------------------------------------*/

// 发送线程
static void SPI_CAN_TxThread(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("SPI_CAN_TxThread", 1000, NULL, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        rt_uint32_t event;
        rt_event_recv(&SPI_CAN_event, (SPI_CAN1_event | SPI_CAN2_event),
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &event);

        if (event == SPI_CAN1_event && control[SPI_CAN1])
        {
            SPI_CAN_SendMsg(0, &SpiCanData[SPI_CAN1]);
        }
        else if (event == SPI_CAN2_event&& control[SPI_CAN2])
        {
            SPI_CAN_SendMsg(0, &SpiCanData[SPI_CAN2] );
        }
        else
        {
            for (int i = 0; i < SPI_CAN_ALL; i++)
            {
                SPI_CANSTransNULL((SPI_CAN_e)i,&SpiCanData[SPI_CAN1]);
            }
        }
        SWDG_FEED(id);
    }
}

rt_err_t SPI_CANThreadTX_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    res = rt_event_init(&SPI_CAN_event, "SPI_CAN_event", RT_IPC_FLAG_PRIO);

    // 线程创建失败返回false
    if (res == RT_ERROR)
        return RT_ERROR;

    // 初始化控制线程
    thread = rt_thread_create(" SPI_CANThread_Thread",
                              SPI_CAN_TxThread,
                              RT_NULL,
                              THREAD_STACK_SPI_CAN, // 线程栈大小
                              THREAD_PRIO_SPI_CAN,  // 线程优先级
                              THREAD_TICK_SPI_CAN); // 线程时间片
    // 线程创建失败返回false
    if (thread == RT_NULL)
        return RT_ERROR;

    // 线程启动失败返回false
    if (rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
