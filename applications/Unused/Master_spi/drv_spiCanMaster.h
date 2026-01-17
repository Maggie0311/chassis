#ifndef _DRV_SPICANMASTER_H
#define _DRV_SPICANMASTER_H

#include "drv_SPICAN.h"

#include "drv_can_com.h"
#include "drv_motor.h"

typedef enum SPI_CAN_E
{
    SPI_CAN1,
    SPI_CAN2,
    SPI_CAN_1_2,
    SPI_CAN_ALL,

} SPI_CAN_e;

typedef enum SPI_CAN_CallBack
{
    rx_chasMotor,
    spiCanrxAll,

} SPI_CAN_CallBack_e;

typedef void (*SpicanRx_fun_p)(SPICAN_MsgOnTransfer_t* Msg);

// 0为CAN1,1为CAN2
CanRecordID_t *SPI_CAN_getCanID(uint8_t motor);
/**
 * @brief 添加从机CAN的回调
 * @param {SPI_CAN_CallBack_e} motorType 3508或者6020
 * @param {SpicanRx_fun_p} fun 参数为can的8字节的内容
 */
void AttachSpiCanCallBack(SPI_CAN_CallBack_e motorType, SpicanRx_fun_p fun);
rt_err_t DrvSpiCanInit(void);

rt_err_t SPI_CANSTransPIDOut(SPI_CAN_e type, Motor_t *motor);
rt_err_t SPI_CANSTrans(SPI_CAN_e type, SPICAN_MsgOnTransfer_t *Msg);
rt_err_t SPI_CANSTransNULL(SPI_CAN_e type, SPICAN_MsgOnTransfer_t *Msg);
rt_err_t SPI_CANThreadTX_Init(void);

#endif
