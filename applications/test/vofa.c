#include "vofa.h"
#include "math.h"
#include "string.h"
#include "ThreadCtrl.h"
#include <board.h>
#include "rtdevice.h"
#include "stdio.h"
#include <stdarg.h>
#include "app_RefSystem.h"
#include "drv_utils.h"

/* 完成量控制块 */
static struct rt_completion completion;
static rt_device_t vofaData_Serial;
static rt_timer_t Vofa_Timer = RT_NULL; // 定时器
static const uint8_t Vofatail[4] = {0x00, 0x00, 0x80, 0x7f};

// 接受到的数据
vofa_idData_t vofa_idData;
// 发送的数据
vofa_float_t vofa_float[PER_DATA_NUM];

// 外部得到接受vofa的数据
float getVofaData(vofa_id_e id)
{
    switch (id)
    {
    case kp_data:
        return vofa_idData.kp;
    case ki_data:
        return vofa_idData.ki;
    case kd_data:
        return vofa_idData.kd;
    case vx_data:
        return vofa_idData.vx;
    case vy_data:
        return vofa_idData.vy;
    case w_data:
        return vofa_idData.w;
    default:
        return 0;
    }
}

static void Vofa_Timer_Timeout_Handler(void *parameter)
{
    rt_completion_done(&completion);
}

volatile uint16_t sendNUm;
void vofaSendFloat(rt_device_t rtt_dev, float data[], uint16_t numFloat)
{

    uint16_t num = numFloat < (PER_DATA_NUM - 1) ? numFloat : PER_DATA_NUM;
    for (int i = 0; i < num; i++)
        vofa_float[i].fdata = data[i];

    for (int i = 0; i < 4; i++)
        vofa_float[num].data[i] = Vofatail[i];

    sendNUm = rt_device_write(rtt_dev, 0, vofa_float, (numFloat + 1) * sizeof(float));
}

float vofaTxText[6];
static void VofaTXThread_Entry(void *parameter)
{
    while (1)
    {
        /* code */
        rt_completion_wait(&completion, RT_WAITING_FOREVER);
        if (vofaData_Serial)
        {
            vofaTxText[0] = Get_Chassis_IMU_Data(IMU_YAW);
            vofaTxText[1] = Get_Chassis_IMU_Data(IMU_PITCH);
            vofaTxText[2] = Get_Chassis_IMU_Data(IMU_ROLL);
            vofaTxText[3] = Get_Chassis_IMU_Data(IMU_ACCX);
            vofaTxText[4] = Get_Chassis_IMU_Data(IMU_ACCY);
            vofaTxText[5] = Get_Chassis_IMU_Data(IMU_ACCZ);
            vofaSendFloat(vofaData_Serial, vofaTxText, sizeof(vofaTxText) / sizeof(vofaTxText[0]));
        }
    }
}

rt_err_t VofaTXThreadInit(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t Thread = RT_NULL;

    /* 初始化完成量对象 */
    rt_completion_init(&completion);

    // 定时器创建
    Vofa_Timer = rt_timer_create(
        "Vofa timer", Vofa_Timer_Timeout_Handler,
        RT_NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (Vofa_Timer)
    {
        if (rt_timer_start(Vofa_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    Thread = rt_thread_create(
        "VofaThread", VofaTXThread_Entry, RT_NULL,
        THREAD_STACK_DEBUGTX, THREAD_PRIO_DEBUGTX, THREAD_TICK_DEBUGTX);

    // 查看是否创建成功
    if (Thread != RT_NULL)
    {
        if (rt_thread_startup(Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

    return ret;
}

/*------------------------------------串口接受----------------------------------------------------------*/
static struct rt_messagequeue vofa_mq;                   /* 消息队列控制块 */
static rt_uint8_t vofa_msg_pool[512];                    /* 消息队列中用到的放置消息的内存池 存储字节大小和设备供读取串口DMA时使用*/
static rt_uint8_t vofa_uartBuff[RT_SERIAL_RB_BUFSZ + 1]; // 串口接受的原始数据 DMA

// 串口接受回调
rt_err_t vofa_rxCallBack(rt_device_t dev, rt_size_t size)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&vofa_mq, &msg, sizeof(msg));
    return result;
}

// 设置接受回调和发送回调
rt_err_t drv_rtt_uart_Init(rt_err_t (*rx_cb)(rt_device_t dev, rt_size_t size),
                           rt_err_t (*tx_cmpt_cb)(rt_device_t dev, void *buffer))
{
    rt_err_t res = RT_EOK;
    struct serial_configure config =
        {
            BSP_TEST_USART_BOUD,
            DATA_BITS_8,        /* 8 databits */
            STOP_BITS_1,        /* 1 stopbit */
            PARITY_NONE,        /* No parity  */
            BIT_ORDER_LSB,      /* LSB first sent */
            NRZ_NORMAL,         /* Normal mode */
            RT_SERIAL_RB_BUFSZ, /* Buffer size */
            0};

    vofaData_Serial = rt_device_find(BSP_TEST_USART_NAME);
    if (!vofaData_Serial)
    {
        rt_kprintf("rt_device_find UART failed !\n");
        return RT_ERROR;
    }

    rt_device_control(vofaData_Serial, RT_DEVICE_CTRL_CONFIG, (void *)&config);

    res = rt_device_set_rx_indicate(vofaData_Serial, rx_cb);
    if (res != RT_EOK)
        return res;

    res = rt_device_set_tx_complete(vofaData_Serial, tx_cmpt_cb);
    if (res != RT_EOK)
        return res;

    rt_device_open(vofaData_Serial, RT_DEVICE_FLAG_DMA_RX);
    return RT_EOK;
}

int vofaRec(uint8_t *src, uint16_t srcSize)
{
    /*指向帧头*/
    uint8_t *pSrcData = src;

    for (; pSrcData < pSrcData + srcSize - 1; pSrcData++)
    {
        /*首先找到帧头*/
        while (*pSrcData != VOFA_HEADER_SOF)
        {
            pSrcData++;
            if (pSrcData - src > srcSize - 1) /*最大偏移量为Size-1*/
                return -1;
        }

        /*找到帧头之后剩余源数长度*/
        int16_t realSrcSize = srcSize - 1 - (pSrcData - src);
        if (realSrcSize < 0)
            return -1;

        uint8_t id = 0;
        vofa_float_t nowData;
        id = *(pSrcData + 1);
        for (int i = 0; i < 4; i++)
            nowData.data[i] = *(pSrcData + 2 + i);

        switch (id)
        {
        case kp_data:
            vofa_idData.kp = nowData.fdata;
            break;
        case ki_data:
            vofa_idData.ki = nowData.fdata;
            break;
        case kd_data:
            vofa_idData.kd = nowData.fdata;
            break;
        case vx_data:
        case vy_data:
        {
            vofa_idData.vx = nowData.fdata;
            memcpy(&vofa_idData.vy, pSrcData + 2 + sizeof(float), sizeof(vofa_idData.vy));
            break;
        }
        case w_data:
            vofa_idData.w = nowData.fdata;
            break;

        default:
            break;
        }
        /*指向下一帧的开始*/
        pSrcData += 5;
    }
    return 0;
}

static void VofaRXThread_Entry(void *parameter)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    while (1)
    {

        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&vofa_mq, &msg, sizeof(msg), 500); // 等待回调函数的消息队列

        if (result == RT_EOK)
        {
            rt_device_read(msg.dev, 0, vofa_uartBuff, msg.size);
            vofaRec(vofa_uartBuff, msg.size);
            rt_memset(&vofa_uartBuff, 0, sizeof(vofa_uartBuff));
        }
    }
}

rt_err_t VofaRxThreadInit(void)
{
    rt_err_t ret = RT_EOK;
    rt_thread_t Thread = RT_NULL;

    // 初始化串口
    drv_rtt_uart_Init(vofa_rxCallBack, NULL);

    /* 初始化消息队列 */
    ret = rt_mq_init(&vofa_mq, "vofa_mq",
                     vofa_msg_pool,
                     sizeof(struct DJI_Mxg),
                     sizeof(vofa_msg_pool),
                     RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
        return ret;

    Thread = rt_thread_create(
        "VofaRX", VofaRXThread_Entry, RT_NULL,
        THREAD_STACK_DEBUGRX, THREAD_PRIO_DEBUGRX, THREAD_TICK_DEBUGRX);

    // 查看是否创建成功
    if (Thread != RT_NULL)
    {
        if (rt_thread_startup(Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

    return ret;
}
void vofaInit(void)
{
    VofaRxThreadInit();
    VofaTXThreadInit();
}
