#include "drv_can_com.h"
#include "stdio.h"
#include "stdarg.h"

// CAN 设备句柄
extern rt_device_t can1_dev;
extern rt_device_t can2_dev;

canRx_fun_p canRx_fun[rx_All];
CanRecordID_t CanRecordID[Can_all];
uint32_t *check[Can_all];
uint16_t temp_id = 0;
CanRecordID_t *getCanID(canRxType_e canType)
{
    return &CanRecordID[canType];
}

/***
 * @brief    根据can1接收到的ID进入不同的处理函数
 * @param    msg can1报文
 ***/
void can1_rec(struct rt_can_msg *msg)
{
    RecordId(&CanRecordID[0], msg->id);
    temp_id = msg->id;
    switch (msg->id)
    {
        // 底盘电机
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 1
    case RUN_RF_MOTOR_ID: 
    case ANGLE_RF_MOTOR_ID:
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LF_CAN_DEVICE_NAME == 1
    case RUN_LF_MOTOR_ID:
    case ANGLE_LF_MOTOR_ID:
    #endif

    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LB_CAN_DEVICE_NAME == 1
    case RUN_LB_MOTOR_ID:
    case ANGLE_LB_MOTOR_ID:
    #endif
    #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RB_CAN_DEVICE_NAME == 1
    case RUN_RB_MOTOR_ID:
    case ANGLE_RB_MOTOR_ID:
    #endif
        #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 1 && LF_CAN_DEVICE_NAME == 1 && LB_CAN_DEVICE_NAME == 1 && RB_CAN_DEVICE_NAME == 1
            if (NULL != canRx_fun[rx_chasMotor])
                canRx_fun[rx_chasMotor](msg);
            return;
        #endif
#if IMU_CHASSIS_CAN_DEVICE_NAME == 1
    case CHASSIS_IMU_ANGLE_RX: // 底盘IMU
        if (NULL != canRx_fun[rx_chasIMU_elur])
            canRx_fun[rx_chasIMU_elur](msg);
        return;
    case CHASSIS_IMU_ACCEL_RX:
        if (NULL != canRx_fun[rx_chasIMU_acc])
            canRx_fun[rx_chasIMU_acc](msg);
        return;
    case CHASSIS_IMU_W_RX:
        if (NULL != canRx_fun[rx_chasIMU_gyro])
            canRx_fun[rx_chasIMU_gyro](msg);
        return;
    case CHASSIS_IMU_Quaternion_RX:
        if (NULL != canRx_fun[rx_chasIMU_Quaternion])
            canRx_fun[rx_chasIMU_Quaternion](msg);
        return;
#endif
#if SCPR_CAN_DEVICE_NAME == 1
    case SCPR_RX:   //超电
        if (NULL != canRx_fun[rx_scpr])
            canRx_fun[rx_scpr](msg);
        return ;
#endif
#if GIMBAL_CAN_DEVICE_NAME == 1
    case YAW_MOTOR_RXID:    //云台电机
    case PITCH_MOTOR_RXID:
        if (NULL != canRx_fun[rx_gimMotor]) // 云台电机
            canRx_fun[rx_gimMotor](msg);
        return;
    case GIMBAL_CCTRL_RX:   //云台控制数据
        if (NULL != canRx_fun[rx_gimctrl]) // 云台控制数据
            canRx_fun[rx_gimctrl](msg);
        return;
    case GIMBAL_DATA_RX:    //云台杂项数据
    if (NULL != canRx_fun[rx_gimData]) // 云台杂项数据
        canRx_fun[rx_gimData](msg);
    return;
    case GIMBAL_IMU_DATA_RX: //云台IMU数据
    if (NULL != canRx_fun[rx_gimIMU]) // 云台IMU
        canRx_fun[rx_gimIMU](msg);
    return;
#endif
    default:
        temp_id = msg->id;
        return;
    }
}

/***
 * @brief    根据can2接收到的ID进入不同的处理函数
 * @param    msg can2报文
 ***/
void can2_rec(struct rt_can_msg *msg)
{

    RecordId(&CanRecordID[1], msg->id);
    switch (msg->id)
    {
        // 底盘电机
        #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 2
        case RUN_RF_MOTOR_ID: 
        case ANGLE_RF_MOTOR_ID:
        #endif
        #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LF_CAN_DEVICE_NAME == 2
        case RUN_LF_MOTOR_ID:
        case ANGLE_LF_MOTOR_ID:
        #endif
    
        #if !defined BSP_CHASSIS_USE_SPI_CAN &&  LB_CAN_DEVICE_NAME == 2
        case RUN_LB_MOTOR_ID:
        case ANGLE_LB_MOTOR_ID:
        #endif
        #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RB_CAN_DEVICE_NAME == 2
        case RUN_RB_MOTOR_ID:
        case ANGLE_RB_MOTOR_ID:
        #endif
            #if !defined BSP_CHASSIS_USE_SPI_CAN &&  RF_CAN_DEVICE_NAME == 2 && LF_CAN_DEVICE_NAME == 2 && LB_CAN_DEVICE_NAME == 2 && RB_CAN_DEVICE_NAME == 2
                if (NULL != canRx_fun[rx_chasMotor])
                    canRx_fun[rx_chasMotor](msg);
                return;
            #endif
    #if IMU_CHASSIS_CAN_DEVICE_NAME == 2
        case CHASSIS_IMU_ANGLE_RX: // 底盘IMU
            if (NULL != canRx_fun[rx_chasIMU_elur])
                canRx_fun[rx_chasIMU_elur](msg);
            return;
        case CHASSIS_IMU_ACCEL_RX:
            if (NULL != canRx_fun[rx_chasIMU_acc])
                canRx_fun[rx_chasIMU_acc](msg);
            return;
        case CHASSIS_IMU_W_RX:
            if (NULL != canRx_fun[rx_chasIMU_gyro])
                canRx_fun[rx_chasIMU_gyro](msg);
            return;
        case CHASSIS_IMU_Quaternion_RX:
            if (NULL != canRx_fun[rx_chasIMU_Quaternion])
                canRx_fun[rx_chasIMU_Quaternion](msg);
            return;
    #endif
    #if SCPR_CAN_DEVICE_NAME == 2
        case SCPR_RX:   //超电
            if (NULL != canRx_fun[rx_scpr])
                canRx_fun[rx_scpr](msg);
            return ;
    #endif
    #if GIMBAL_CAN_DEVICE_NAME == 2
        case YAW_MOTOR_RXID:    //云台电机
        case PITCH_MOTOR_RXID:
            if (NULL != canRx_fun[rx_gimMotor]) // 云台电机
                canRx_fun[rx_gimMotor](msg);
            return;
        case GIMBAL_CCTRL_RX:   //云台控制数据
            if (NULL != canRx_fun[rx_gimctrl]) // 云台控制数据
                canRx_fun[rx_gimctrl](msg);
            return;
        case GIMBAL_DATA_RX:    //云台杂项数据
        if (NULL != canRx_fun[rx_gimData]) // 云台杂项数据
            canRx_fun[rx_gimData](msg);
        return;
        case GIMBAL_IMU_DATA_RX: //云台IMU数据
        if (NULL != canRx_fun[rx_gimIMU]) // 云台IMU
            canRx_fun[rx_gimIMU](msg);
        return;
    #endif
        default:
            temp_id = msg->id;
            return;
    }
}

rt_err_t CanTrans(canDev_e canDev, struct rt_can_msg *msg)
{
    uint32_t sendNum = 0;
    switch (canDev)
    {
    case USED_CAN1:
        if (can1_dev)
        {
            sendNum = rt_device_write(can1_dev, 0, msg, sizeof(struct rt_can_msg));
            if (sendNum == 0)
                return RT_ERROR;
        }
        else
            return RT_ERROR;
        break;
    case USED_CAN2:
        if (can2_dev)
        {
            sendNum = rt_device_write(can2_dev, 0, msg, sizeof(struct rt_can_msg));
            if (sendNum == 0)
                return RT_ERROR;
        }
        else
            return RT_ERROR;
		break;
    case USED_CAN_1_2:
        if (can1_dev && can2_dev)
        {
            sendNum = rt_device_write(can1_dev, 0, msg, sizeof(struct rt_can_msg));
            if (sendNum == 0)
                return RT_ERROR;
            sendNum = rt_device_write(can2_dev, 0, msg, sizeof(struct rt_can_msg));
            if (sendNum == 0)
                return RT_ERROR;
        }
        else
            return RT_ERROR;
    default:
        break;
    }
    return RT_EOK;
}

void CanTransNull(canDev_e canDev, struct rt_can_msg *msg)
{
    struct rt_can_msg tmsgs = *msg;
    uint32_t sendNum = 0;

    for (int i = 0; i < 8; i++)
        tmsgs.data[i] = 0;

    switch (canDev)
    {
    case USED_CAN1:
        if (can1_dev)
        {
            rt_device_write(can1_dev, 0, &tmsgs, sizeof(struct rt_can_msg));
            break;
        }
    case USED_CAN2:
        if (can2_dev)
        {
            rt_device_write(can2_dev, 0, &tmsgs, sizeof(struct rt_can_msg));
            break;
        }
    case USED_CAN_1_2:
        if (can1_dev && can2_dev)
        {
            sendNum = rt_device_write(can1_dev, 0, msg, sizeof(struct rt_can_msg));
            sendNum = rt_device_write(can2_dev, 0, msg, sizeof(struct rt_can_msg));
            if (sendNum == 0)
            break;
        }
    default:
        break;
    }
}

/**
 * @brief 注册can接受函数
 * @param {canRxFun_e} ID 目前可能的接受函数ID
 * @param {canRx_fun_p fun} 接受函数 void (*canRx_fun_p)(struct rt_can_msg *msg)
 */
void attachCanRXfun(canRxFun_e ID, canRx_fun_p fun)
{
    if (ID < rx_All)
        canRx_fun[ID] = fun;
}

/**
 * @brief 删除can接受函数
 * @param {canRxFun_e} funId 目前可能的接受函数ID
 */
void deleteCanRXfun(canRxFun_e funId)
{
    if (funId < rx_All)
        canRx_fun[funId] = NULL;
}

void RecordId(CanRecordID_t *CanRecord, uint32_t id)
{
    uint8_t flag = 0;
    for (int i = 0; i < NUM; i++)
    {
        if (CanRecord->RXcanDataID[i] == id)
        {
            flag = 1;
            CanRecord->f_Hz[i] = (uint16_t)(1000.f / (rt_tick_get() - CanRecord->refTick[i]));
            CanRecord->refTick[i] = rt_tick_get();
        }
    }
    if (!flag)
    {
        if (CanRecord->Cannum < NUM)
            CanRecord->RXcanDataID[CanRecord->Cannum] = id;

        CanRecord->Cannum++;
    }
}
