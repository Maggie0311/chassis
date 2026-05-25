#ifndef __DRV_GIMBALCTRLRX_H__
#define __DRV_GIMBALCTRLRX_H__

#include <board.h>
#include "string.h"

#define MSG_READ_2BYTES(lo, hi) ((rt_int16_t)((msg->data[lo] << 8) + msg->data[hi])) // 二字节读取宏

#define Package_Request(name, data) Request_Add_Package((name), (strlen(name)), (&data), (sizeof(data)))

#ifdef CORE_USING_HERO // kconfig
#define SC_POWERT_LIM 50
#elif defined CORE_USING_INFANTRY
#define SC_POWERT_LIM 40
#else
#define SC_POWERT_LIM 30
#endif

typedef __packed struct
{
    // 工作状态数据
    unsigned ui_reset : 1;                // 重置 UI 标志位
    unsigned chassis_reset : 1;           // 底盘复位指令
    unsigned current_menu : 4;            // 当前所处菜单
    unsigned motion_mode : 3;             // 底盘模式

    unsigned if_aimbot_best_dis : 1;      // 当前是否为云台认为的最佳射击距离
    unsigned strike_mode : 1;             // 发射机构模式(高/低射频)
    unsigned magazine_status : 1;         // 弹舱盖设定状态
    unsigned heatlimit_status : 1;        // 热量状态(是否开启热量限制)
    unsigned aimbot_mode : 3;             // 瞄准模式

    unsigned self_color : 1;              // 当前己方颜色
    unsigned GimIfSuperCapOutput : 1;  // 底盘功率限制(是否开启超级电容电量自动节省)(C键)
    unsigned rub_started : 1;             // 摩擦轮已开启
    unsigned now_client_control : 1;      // 当前使用客户端模式
    unsigned capacity_close_flag : 1;     // 当前是否关闭超级电容充电(未使用)
    unsigned force_refsystem_offline : 1; // 当前是否强制裁判系统离线
    unsigned set_chassis_mode : 2;        // 当前设置的本地底盘模式

    unsigned set_ammobooster_mode : 2;    // 当前设置的本地发射机构模式
    unsigned visual_target_found : 2;     // 视觉是否找到目标（用于在UI上提示操作手是否找到目标）
    unsigned Energy_is_already : 1;       // 能量机关拟合完成标志位
    unsigned Wire_less_charge : 1;        // 无线充电标志位（1打开无线充电，0关闭无线充电）
    unsigned reserve : 4;                 // 保留位
    
    unsigned Gimbal_if_reset : 1;         // 云台是否发生了复位事件
    unsigned Key_ctrl : 1;                // ctrl 键是否按下-按下时采用高的加速度值
    // 离线模块数据
    unsigned visual_com_online : 1;      // 视觉通信是否正常
    unsigned visual_working_correct : 1; // 视觉工作是否正常
    unsigned yaw_motor_online : 1;       // Yaw 轴电机通信正常
    unsigned pitch_motor_online : 1;     // Pitch 轴电机通信正常

    unsigned right_rub_motor_online : 1; // 右侧摩擦轮电机通信正常
    unsigned left_rub_motor_online : 1;  // 左侧摩擦轮电机通信正常
    unsigned launch_motor_online : 1;    // 播弹盘电机通信正常
    unsigned strike_stuck : 1;           // 发射机构卡弹
#ifdef CORE_USING_SENTRY
    unsigned sentryMode : 1; // 哨兵模式
#endif
   
} gimbal_msg_t; // 杂项数据包

typedef enum
{
    Gim_Ctrl,
    Gim_data,
    Gim_dataAll,
} GimbalData_e;

/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg);
void Refresh_Chassis_Gimbalcaled_IMU_Data(struct rt_can_msg *msg);
float Get_Gim_Chassis_IMU_Data(imu_data_type_e imu_data_type);

#endif
