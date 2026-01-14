#include "drv_gimCtrlRx.h"
#include "drv_dataserve.h"
#include "func_MonHandling.h"
#include "fun_ctrlSrc.h"
#include "app_monitor.h"
#include "mod_motion.h"
#include "controlMsg.h"

int16_t gimRxMonId = -1; // 云台监视器id

Ctrl_data_info_t gimbal_ctrl;        // 云台控制数据
gimbal_msg_t gimbal_msg = {0};       // 云台杂项数据
imu_euler_t chassis_imu = {0, 0, 0}; // 云台算出的底盘姿态数据
uint8_t unused[8];
uint32_t gimRxDebug = 1;
rt_tick_t gimbal_msg_freshtick = 0; // can接受云台数据刷新
rt_tick_t imu_data_freshtick = 0;
static uint16_t dataNum[Gim_dataAll];    // 数据服务器NUM
static uint8_t MaxPower = SC_POWERT_LIM; // 当云台端裁认为裁判系统异常则会给出一个底盘最大功率

/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg)
{
    SWDG_FEED(gimRxMonId);
    gimbal_ctrl.vel_x = (float)MSG_READ_2BYTES(0, 1);
    gimbal_ctrl.vel_y = (float)MSG_READ_2BYTES(2, 3);
    gimbal_ctrl.mode = (Motion_mode_e)(msg->data[6]);

    // 云台跟随角外部修改
    if (gimbal_ctrl.mode == NO_FOLLOW)
    {
        gimbal_ctrl.fol_ang = 0;
        gimbal_ctrl.angvel = 0;
    }
    if (gimbal_ctrl.mode == FOLLOW_GIMBAL)
    {
        // gimbal_ctrl.fol_ang = (float)MSG_READ_2BYTES(4, 5) / 8192.0f * 360.0f; // 云台端通信跟随角单位为8192(对应360°)
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(4, 5); // 云台发送角速度单位0.1度/s，逻辑为在跟随模式下，云台直接发送YAW轴角速度值给底盘
        gimbal_ctrl.fol_ang = 0;
    }
    else if (gimbal_ctrl.mode == SLOW_GYRO || gimbal_ctrl.mode == FAST_GYRO ||gimbal_ctrl.mode == CHANGE_GYRO)
    {
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(4, 5);
        gimbal_ctrl.fol_ang = 0;
    }
    MaxPower = msg->data[7];

    Source_WriteData_s(CS_GIMBAL, &gimbal_ctrl);

    gimbal_ctrl.getYawAngel = (getYawAngelCallBack)Get_GM_ANGLE_360f;
    gimbal_ctrl.arg = GM_YAW;
    gimbal_ctrl.folIsReverse = !YAW_ANGLE_REVERSAL;                  /*当底盘在云台的顺时针方向角度为正所以跟随时反向*/
    Package_Write(dataNum[Gim_Ctrl], gimbal_ctrl); // 更新数据服务器的数据
}

/***
 * @brief    处理云台发送的各类数据报文
 * @param    msg     can2报文
 ***/
void Refresh_Gimbaldata(struct rt_can_msg *msg)
{
    SWDG_FEED(gimRxMonId);
    gimbal_msg_freshtick = rt_tick_get();

    if (gimRxDebug)
        rt_memcpy(&gimbal_msg, msg->data, sizeof(gimbal_msg) < 8 ? sizeof(gimbal_msg) : 8);
    Package_Write(dataNum[Gim_data], gimbal_msg); // 更新数据服务器的数据
    if (gimbal_msg.chassis_reset)
        Robot_Chassis_Reset(RT_TRUE);
}
// 接收云台算出的底盘姿态角度数据
void Refresh_Chassis_Gimbalcaled_IMU_Data(struct rt_can_msg *msg)
{
    chassis_imu.pitch = *((float *)&msg->data[0]);
    chassis_imu.roll = *((float *)&msg->data[4]);
    imu_data_freshtick = rt_tick_get();
}

/**
 * @brief 获得云台给底盘发送的底盘的姿态数据
 * 
 * @param imu_data_type 
 * @return float 
 */
float Get_Gim_Chassis_IMU_Data(imu_data_type_e imu_data_type)
{
    if(imu_data_type==IMU_YAW)
    {
        return chassis_imu.yaw;
    }
    else if(imu_data_type==IMU_PITCH)
    {
        return chassis_imu.pitch;
    }
    else if(imu_data_type==IMU_ROLL)
    {
        return chassis_imu.roll;
    }
}
// 当云台端裁认为裁判系统异常则会给出一个底盘最大功率
uint8_t GetGimMaxPower(void)
{
    return MaxPower;
}

rt_uint8_t GetGimCtrlDataNum(void) // 常用单独分开
{
    return dataNum[Gim_Ctrl];
}
rt_uint8_t GetGimDataNum(void)
{
    return dataNum[Gim_data];
}
uint8_t GetGimbalOffline(void)
{
    uint8_t gimbleoffline = 0;
    if(rt_tick_get() - gimbal_msg_freshtick >= 500)
    {
        gimbleoffline |= 1<<0;
    }
    return gimbleoffline;
}