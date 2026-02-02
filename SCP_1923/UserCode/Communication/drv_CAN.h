#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "main.h"

#include "can.h"

#include "drv_Time.h"

#define CHASSIS_CAN_ID (0x095)
#define CAPACITY_CAN_ID (0x096)

typedef struct
{
    uint32_t StdID;                 // 接收到报文的 ID
    uint8_t CAN_Receive_Message[8]; // 收到数据的数组
    /**
     * @brief 数组成员
     * 0：Work_state
     * 1：是否应答（认为是新数据）
     * 2：Fly_flag
     * 3：Power_HP_Deduction
     * 4、5：Energy_Buff
     * 6、7：Power_Set
     *
     */
    Time_Struct_t Receive_Time; // 收到数据时的时间结构体
    uint8_t Work_state;         // 是否正常工作，1为正常工作
    uint8_t Fly_flag;           // 飞坡标志位，1为发生飞坡
    uint8_t Power_HP_Deduction; // 底盘超限扣血，1为发生扣血
    uint16_t Energy_Buff;       // 底盘功率缓冲能量数值
    uint16_t Power_Set;         // 底盘功率上限设定值
    uint8_t Data_Is_New;        // 是否为最新数据，为 1 标明为最新数据
} CAN_Receive_Data_t;

extern void drv_CAN_Init(void);

/**
 * @brief  CAN 向底盘发送一次超级电容状态
 * @author fwlh
 * @param  Data_To_Send     待发送数据的结构体
 * @return uint8_t          代表数据是不是成功写到发送邮箱里面了
 */
uint8_t CAN_Send(uint8_t Data_To_Send[8]);

CAN_Receive_Data_t *Read_Canreceive_Data(void);

#endif /* __DRV_CAN_H__ */
