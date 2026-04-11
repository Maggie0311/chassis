#include "drv_IMU_Can.h"

#include <rtthread.h>
#include <board.h>

/* 三个接收报文的CAN实例 */
static CANInstance imu_can_rx[IMU_MSG_NUM];

/* 对外暴露的IMU数据 */
IMU_Can_Data_t imu_can_data;

/*---------- 接收回调 ----------*/

/**
 * @brief 包1回调：yaw角度 + z轴加速度
 *        布局：[0~3] yaw_angle(float)  [4~7] z_accel(float)
 */
static void IMU_Yaw_Z_Rx_call(CANInstance *instance)
{
    imu_can_data.yaw_angle = *(float *)(&instance->rx_buff[0]);
    imu_can_data.z_accel   = *(float *)(&instance->rx_buff[4]);
}

/**
 * @brief 包2回调：pitch角度 + y轴加速度
 *        布局：[0~3] pitch_angle(float)  [4~7] y_accel(float)
 */
static void IMU_Pitch_Y_Rx_call(CANInstance *instance)
{
    imu_can_data.pitch_angle = *(float *)(&instance->rx_buff[0]);
    imu_can_data.y_accel     = *(float *)(&instance->rx_buff[4]);
}

/**
 * @brief 包3回调：roll角度 + x轴加速度
 *        布局：[0~3] roll_angle(float)  [4~7] x_accel(float)
 */
static void IMU_Roll_X_Rx_call(CANInstance *instance)
{
    imu_can_data.roll_angle = *(float *)(&instance->rx_buff[0]);
    imu_can_data.x_accel    = *(float *)(&instance->rx_buff[4]);
}

/*---------- 初始化 ----------*/

/**
 * @brief 初始化IMU CAN通信（仅接收，使用回调解析数据）
 * @return RT_EOK 成功，RT_ERROR 失败
 */
rt_err_t IMU_Can_Init(void)
{
    int res = 0;
    CAN_Init_Config_s config;

    /* 包1: yaw_angle + z_accel */
    rt_memset(&config, 0, sizeof(config));
    config.can_device_name     = IMU_CAN_DEV;
    config.rx_id               = IMU_YAW_Z_RX;
    config.can_module_callback = IMU_Yaw_Z_Rx_call;
    res += CAN_Attach(&imu_can_rx[IMU_YAW_Z_INDEX], &config);

    /* 包2: pitch_angle + y_accel */
    rt_memset(&config, 0, sizeof(config));
    config.can_device_name     = IMU_CAN_DEV;
    config.rx_id               = IMU_PITCH_Y_RX;
    config.can_module_callback = IMU_Roll_X_Rx_call;
    res += CAN_Attach(&imu_can_rx[IMU_PITCH_Y_INDEX], &config);

    /* 包3: roll_angle + x_accel */
    rt_memset(&config, 0, sizeof(config));
    config.can_device_name     = IMU_CAN_DEV;
    config.rx_id               = IMU_ROLL_X_RX;
    config.can_module_callback = IMU_Pitch_Y_Rx_call;
    res += CAN_Attach(&imu_can_rx[IMU_ROLL_X_INDEX], &config);

    return (res == 0) ? RT_EOK : RT_ERROR;
}

/*---------- 外部接口 ----------*/

/**
 * @brief 获取IMU数据是否在线（500ms内三个包均有数据）
 * @return RT_TRUE 在线
 */
rt_bool_t IMU_Can_IsOnline(void)
{
    rt_tick_t now = rt_tick_get();
    for (int i = 0; i < IMU_MSG_NUM; i++)
    {
        if (!imu_can_rx[i].rx_time || (now - imu_can_rx[i].rx_time >= 500))
            return RT_FALSE;
    }
    return RT_TRUE;
}
