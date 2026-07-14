#ifndef __DRV_IMU_CAN_H__
#define __DRV_IMU_CAN_H__

#include <board.h>
#include "can_com_thread.h"

/* CAN设备 */
#define IMU_CAN_DEV "can2"

/* 接收线程轮询周期，单位ms */
#define IMU_RX_PERIOD 10

/* 接收报文ID（待填写） */
#define IMU_YAW_Z_RX   0x107   // 包1: yaw角度 + z轴加速度
#define IMU_PITCH_Y_RX 0x109   // 包2: pitch角度 + y轴加速度
#define IMU_ROLL_X_RX  0x108   // 包3: roll角度 + x轴加速度

#define IMU_SPI_USE 1
/* 报文索引 */
typedef enum
{
    IMU_YAW_Z_INDEX   = 0,  // 包1: yaw_angle + z_accel
    IMU_PITCH_Y_INDEX = 1,  // 包2: pitch_angle + y_accel
    IMU_ROLL_X_INDEX  = 2,  // 包3: roll_angle + x_accel
    IMU_MSG_NUM,
} IMU_Msg_Index_e;

/**
 * @brief IMU CAN接收数据结构体
 *        三个包分别携带：
 *          包1 - yaw角度(deg) + z轴加速度(m/s²)
 *          包2 - pitch角度(deg) + y轴加速度(m/s²)
 *          包3 - roll角度(deg) + x轴加速度(m/s²)
 */
typedef struct
{
    float yaw_angle;    // yaw角度，单位deg
    float pitch_angle;  // pitch角度，单位deg
    float roll_angle;   // roll角度，单位deg
    float x_accel;      // x轴加速度，单位m/s²
    float y_accel;      // y轴加速度，单位m/s²
    float z_accel;      // z轴加速度，单位m/s²
} IMU_Can_Data_t;

extern IMU_Can_Data_t imu_can_data;

/**
 * @brief 初始化IMU CAN通信（仅接收）
 * @return RT_EOK 成功
 */
rt_err_t IMU_Can_Init(void);

/**
 * @brief 获取IMU数据是否在线（500ms内有数据）
 * @return RT_TRUE 在线
 */
rt_bool_t IMU_Can_IsOnline(void);

#endif /* __DRV_IMU_CAN_H__ */
