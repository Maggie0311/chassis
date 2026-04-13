#include "fun_protocolGim.h"

/***
 * @brief:   获取云台端传来的发弹模式(单发/三连发)(低射频/高射频)
 * @param:   None
 * @return:  枪管模式
 ***/
strike_mode_e Get_StrikeMode(void)
{ // UI
    return ((gimbal_mixdata_msg.strike_mode == 0) ? SINGLE_STRIKE : TRIPLE_STRIKE);
}

/***
 * @brief 返回底盘运动模式
 * @retval
 ***/
ui_motion_mode_e Get_Motion_Mode(void)
{ // powerCtrl/UI
    return (ui_motion_mode_e)gimbal_mixdata_msg.motion_mode;
}

/***
 * @brief    //返回自瞄模式
 * @retval   0: 普通自瞄
 * @retval   1: 小能量机关
 * @retval   2: 大能量机关
 * @retval   3: 静止击打前哨战
 * @retval   4: 旋转击打前哨战
 * @retval   5: 第三种击打前哨战的模式
 * @retval   6: 吊射模式
 ***/
ui_aimbot_mode_e Get_Aimbot_Mode(void)
{ // UI
    return (ui_aimbot_mode_e)gimbal_mixdata_msg.aimbot_mode;
}

/**
 * @brief 获取当前自瞄的己方颜色
 * @author fwlh
 * @return My_Color_Enum    自方颜色
 */
My_Color_Enum Get_Color_Myself(void)
{ // UI
    return (My_Color_Enum)gimbal_mixdata_msg.self_color;
}

/***
 * @brief 返回Ui重置标志位
 * @retval
 ***/
rt_uint8_t Get_UI_Reset_Flg(void)
{ // UI
    return gimbal_mixdata_msg.ui_reset;
}

// 返回弹仓状态,1:开，0:关
rt_int8_t Get_Magazine_Status(void)
{ // UI
    return gimbal_mixdata_msg.magazine_status;
}

// 热量控制开关状态
rt_uint8_t Get_HeatLimit_Status(void)
{ // UI
   return gimbal_mixdata_msg.heatlimit_status;
}

// 获取二维按键当前处于哪个菜单下
ui_option_display_e Get_UI_Option_Flag(void)
{ // UI
    return (ui_option_display_e)gimbal_mixdata_msg.current_menu;
}

// 获取云台端对超电允许放电的状态
uint8_t GetGimIfSuperCapOutput(void)
{ // powerCtrl
    return gimbal_mixdata_msg.GimIfSuperCapOutput;
}
// 获取云台端对无线充电的状态(0为不进行无线充电，1为进行无线充电)
uint8_t Get_Wireless_Charge(void)
{ // Supercap/UI
    return gimbal_mixdata_msg.Wire_less_charge;
}
// 读取当前是否需要超级电容充电, 返回 1 为不需要
rt_uint8_t Get_Charge_Cmd(void)
{
    return gimbal_mixdata_msg.capacity_close_flag;
}

// 读取云台是否强制裁判系统离线的指令
rt_uint8_t Get_Ref_Offline_Cmd(void)
{ // UI
   return gimbal_mixdata_msg.force_refsystem_offline;
}

// 读取上一次云台控制信息刷新时间
rt_tick_t Get_Gim_FreshTick(void)
{ // 自己/UI/powerCtrl
    return gimbal_mixdata_msg_freshtick;
}

// 读取云台当前是否刚刚发生复位事件
rt_uint8_t Get_Gim_Reset_Status(void)
{ // UI
    if (rt_tick_get() - gimbal_mixdata_msg_freshtick < 100)
        return (gimbal_mixdata_msg.Gimbal_if_reset);
    else
        return 0;
}

// 读取当前摩擦轮是否开启
rt_bool_t Read_Rub_Started(void)
{ // UI
    return (gimbal_mixdata_msg.rub_started ? RT_TRUE : RT_FALSE);
}

// 读取当前是否处于客户端模式, 返回真代表在客户端模式
rt_bool_t Get_Client_Status(void)
{ // UI
    return (gimbal_mixdata_msg.now_client_control ? RT_TRUE : RT_FALSE);
}

// 获取当前的本地底盘模式(0功率优先 1血量优先)
Local_Chassis_Mode_Enum Get_Local_Chassis_Type(void)
{ // UI
    return (Local_Chassis_Mode_Enum)gimbal_mixdata_msg.set_chassis_mode;
}

// 获取当前的本地枪口模式(0爆发优先 1冷却优先 2弹速优先)
Local_AmmoBooster_Mode_Enum Get_Local_AmmoBooster_Type(void)
{ // UI
    return (Local_AmmoBooster_Mode_Enum)0;
}

// 获取本地的机器人等级
// rt_uint8_t Get_Local_Robot_Level(void)
// {
//     return gimbal_mixdata_msg.set_level + 1;
// }
// 获取视觉自瞄能量机关拟合完成标志位
rt_uint8_t Get_Energy_is_already(void)
{ // UI
    return gimbal_mixdata_msg.Energy_is_already;
}
// 获取云台中的离线电机, 从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘
rt_uint8_t Get_Gimbal_Motor_Offline_State(void)
{ // UI
    rt_uint8_t OfflineMotor = 0;
    OfflineMotor |= (!gimbal_mixdata_msg.yaw_motor_online) << gimErr_yaw;
    OfflineMotor |= (!gimbal_mixdata_msg.pitch_motor_online) << gimErr_pitch;
    OfflineMotor |= (!gimbal_mixdata_msg.right_rub_motor_online) << gimErr_rRub;
    OfflineMotor |= (!gimbal_mixdata_msg.left_rub_motor_online) << gimErr_lRub;
    OfflineMotor |= (!gimbal_mixdata_msg.launch_motor_online) << gimErr_launch;
    return OfflineMotor;
}
// 获取发射机构是否存在卡弹情况
rt_bool_t Get_Gimbal_Stuck_State(void)
{ // UI
    return gimbal_mixdata_msg.strike_stuck;
}



// 得到当前云台错误状态
uint8_t getGimErrState(void)
{ // UI
    uint8_t state = 0;
    uint32_t tick = 0;
    state = Get_Gimbal_Motor_Offline_State();
    tick = Get_Gim_FreshTick();
    if (rt_tick_get() - tick > 200)
        state |= 1 << gimErr_com;
    return state;
}
/// @brief 获取当前云台是否为自瞄认为的最佳射击距离，
/// @return 如果是则返回1，否则返回0
uint8_t Get_if_aimbot_best_dis(void)
{ // UI
    return gimbal_mixdata_msg.if_aimbot_best_dis;
}

uint8_t Get_Allow_Heat_Shoot(void)
{ // UI
//    return gimbal_mixdata_msg.allow_heat_shoot;
}
/**
 * @brief 获取云台发送给底盘关于视觉是否识别到敌方的信息（0为未识别到，1为识别到）
 *
 * @return uint8_t
 */
uint8_t Get_Gim_Visual_Target_Found(void)
{ // UI
    return gimbal_mixdata_msg.visual_target_found;
}
// 获取视觉模块异常状态
Visual_Error_State_Enum Get_Visual_Working_Error_State(void)
{ // UI
    switch (gimbal_mixdata_msg.visual_com_online << 1 | gimbal_mixdata_msg.visual_working_correct)
    {
    case (1 << 1 | 1):
        return Visual_Working_EOK;
    case (1 << 1 | 0):
        return Visual_Working_Failure;
    case (0 << 1 | 1):
    case (0 << 1 | 0):
    default:
        return Visual_Com_Failure;
    }
}

uint8_t get_chassis_pitch(float *pitch)
{ // motion
    if (rt_tick_get() - imu_data_freshtick < 100)
    {
        *pitch = gimbal_imudata_msg.pitch;
        return 1;
    }
    else
        return 0;
}

uint8_t get_chassis_roll(float *roll)
{ // motion
    if (rt_tick_get() - imu_data_freshtick < 100)
    {
        *roll = gimbal_imudata_msg.roll;
        return 1;
    }
    else
        return 0;
}

imu_euler_t get_chassis_imu_data(void)
{ // UI
    if (rt_tick_get() - imu_data_freshtick < 100)
        return gimbal_imudata_msg;
    else
        return (imu_euler_t){0, 0, 0};
}

uint8_t get_Key_ctrl(void)
{ // 无
    if (rt_tick_get() - imu_data_freshtick < 100)
        return gimbal_mixdata_msg.Key_ctrl;
    else
        return 0;
}
