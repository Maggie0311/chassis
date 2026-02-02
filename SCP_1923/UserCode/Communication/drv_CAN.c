/**
 * @file drv_CAN.c
 * @brief 该文件用于处理 CAN 外设的底层驱动与再封装
 * @author fwlh
 * @version 1.0
 * @date 2021-11-30
 *
 * @copyright Copyright (c) 2021  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_CAN.h"

#include "func_LED.h"

#include "stdlib.h"

CAN_RxHeaderTypeDef RXHeader = {0}; // 接收帧的数据
CAN_Receive_Data_t CAN_Receive_Data;
//={
//	.Work_state=1//debug
//};
// 接收数据指针
CAN_TxHeaderTypeDef TXHandler =
    {.RTR = CAN_RTR_DATA,
     .DLC = 8,
     .IDE = CAN_ID_STD,
     .StdId = CAPACITY_CAN_ID,
     .TransmitGlobalTime = DISABLE}; // 发送帧的数据
uint8_t debug_rec_cnt = 0;

/**
 * @brief 硬件滤波器初始化
 * @author fwlh
 */
static void CAN_Receive_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterActivation = ENABLE;               // 打开过滤器
    sFilterConfig.FilterBank = 0;                          // 过滤器0 这里可设0-13
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;      // 采用掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;     // 采用32位掩码模式
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 采用FIFO0

    sFilterConfig.FilterIdHigh = (0x000 << 5);     // 设置过滤器ID高16位
    sFilterConfig.FilterIdLow = 0x0000;            // 设置过滤器ID低16位
    sFilterConfig.FilterMaskIdHigh = (0x000 << 5); // 设置过滤器掩码高16位
    sFilterConfig.FilterMaskIdLow = 0x0000;        // 设置过滤器掩码低16位

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) // 初始化过滤器
    {
        Error_Handler();
    }
}

/**
 * @brief  CAN 向底盘发送一次超级电容状态
 * @author fwlh
 * @param  Data_To_Send     待发送数据的结构体
 * @return uint8_t          代表数据是不是成功写到发送邮箱里面了
 */
uint8_t CAN_Send(uint8_t Data_To_Send[8])
{
    uint32_t TxMailbox;
    // 查找是不是有空闲的发送邮箱, 如果有再填入数据
    if (!HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2))
    {
        HAL_CAN_AddTxMessage(&hcan, &TXHandler, Data_To_Send, &TxMailbox);
        return HAL_CAN_IsTxMessagePending(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    else
    {
        return 0;
    }
}

/**
 * @brief CAN外设的初始化
 * @author fwlh
 */
void drv_CAN_Init(void)
{
    // 初始化接受过滤器
    CAN_Receive_Filter_Init();
    // 打开 CAN
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    // 开启接受邮箱 0 挂起中断
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN 外设 FIFO0 的中断接收回调函数
 * @author fwlh
 * @param  hcan             CAN 处理句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN)
    {
        // 获取数据并存到缓存区里面等待处理
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RXHeader, CAN_Receive_Data.CAN_Receive_Message);

        // 记录接收到报文的 ID
        CAN_Receive_Data.StdID = RXHeader.StdId;

        if (CAN_Receive_Data.StdID == CHASSIS_CAN_ID)
        { // 是底盘发来的数据
            debug_rec_cnt++;
#ifdef LED_ONCAR
            LED_Ctrl(1, 2, LED_Flash, 100);
#endif

            // 查看数据是否更新
            CAN_Receive_Data.Data_Is_New = CAN_Receive_Data.CAN_Receive_Message[1];
            if (CAN_Receive_Data.Data_Is_New == 1)
            {

                // 底盘是否让工作
                CAN_Receive_Data.Work_state = CAN_Receive_Data.CAN_Receive_Message[0];
                // CAN_Receive_Data.Work_state = 1;//debug
                //  如果底盘不让工作
                //  if (CAN_Receive_Data.Work_state == 0)
                //      return;

                // 如果底盘功率超限扣血
                CAN_Receive_Data.Power_HP_Deduction = CAN_Receive_Data.CAN_Receive_Message[3];
                // if (CAN_Receive_Data.Power_HP_Deduction == 1)
                //     return;

                CAN_Receive_Data.Receive_Time.Timer_Count = Read_Current_Time();
                // 解包
                CAN_Receive_Data.Fly_flag = CAN_Receive_Data.CAN_Receive_Message[2];
                CAN_Receive_Data.Energy_Buff = (((uint16_t)(CAN_Receive_Data.CAN_Receive_Message[4])) << 8) | CAN_Receive_Data.CAN_Receive_Message[5];
                CAN_Receive_Data.Power_Set = (((uint16_t)(CAN_Receive_Data.CAN_Receive_Message[6])) << 8) | CAN_Receive_Data.CAN_Receive_Message[7];
            }
        }
    }
}

/**
 * @brief 读取 can 接受到的数据
 *
 * @return CAN_Receive_Data_Struct*
 */
CAN_Receive_Data_t *Read_Canreceive_Data(void)
{
    return &CAN_Receive_Data;
}
