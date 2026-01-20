#ifndef __DRV_CAN_COM_H__
#define __DRV_CAN_COM_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "HCanID_data.h"
#include "drv_dataserve.h"

typedef enum
{
    Rx_Can1,
    Rx_Can2,

    Can_all,
} canRxType_e;

typedef enum
{
    //此处通过控制是否使用SPI转CAN和是否使用舵轮来控制底盘电机挂载设备，
    //如果不使用舵轮或者不使用SPI，则将电机挂载到普通CAN设备上，在此处定义其枚举索引
    rx_chasMotor,
    rx_chasIMU_gyro,
    rx_chasIMU_elur,
    rx_chasIMU_acc,
    rx_chasIMU_Quaternion,

    rx_gimMotor,
    rx_gimIMU,
    rx_gimctrl,
    rx_gimData,
    rx_scpr,
    rx_All,
} canRxFun_e;

typedef struct
{
    canRxFun_e canRxFun;
    uint32_t CanId;
} canRxFuntable_t;

#define NUM 10
typedef struct
{
    uint32_t RXcanDataID[NUM];
    uint32_t refTick[NUM];
    uint8_t Cannum;
    uint8_t CanType;
    uint16_t f_Hz[NUM];

} CanRecordID_t;

typedef void (*canRx_fun_p)(struct rt_can_msg *msg);

CanRecordID_t *getCanID(canRxType_e canType);

/**
 * @brief 注册can接受函数
 * @param {canRxFun_e} ID 目前可能的接受函数ID
 * @param {canRx_fun_p fun} 接受函数 void (*canRx_fun_p)(struct rt_can_msg *msg)
 */
void attachCanRXfun(canRxFun_e ID, canRx_fun_p fun);

void can1_rec(struct rt_can_msg *msg); // can1数据接收函数
void can2_rec(struct rt_can_msg *msg); // can2数据接收函数

rt_err_t CanTrans(canDev_e canDev, struct rt_can_msg *msg);
void CanTransNull(canDev_e canDev, struct rt_can_msg *msg);

void RecordId(CanRecordID_t *CanRecord, uint32_t id);

#endif
