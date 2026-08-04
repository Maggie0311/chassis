#ifndef __FUN_PROTOCOLGIM_H__
#define __FUN_PROTOCOLGIM_H__
#include <board.h>
#include "drv_Gimbal_Com.h"

// ui选项显示索引
typedef enum
{
    MainMenu = 0,         // 当前处于主菜单
    ChassisModeMenu,      // 当前处于底盘模式菜单
    AimModeMenu,          // 当前处于自瞄模式菜单
    MiscMenu,             // 当前处于杂项菜单
    RobotResetMenu,       // 当前处于单片机复位菜单
    SCAPCtrlMenu,         // 当前处于超级电容控制菜单
    GunSpeedMenu,         // 当前处于弹速设置菜单
    PerformanceMenu,      // 当前处于机器人性能设置菜单
    PerformanceChooseMenu // 当前处于机器人性能选择菜单
} ui_option_display_e;

typedef enum
{
    UI_NO_FOLLOW = 0,     // 不跟随
    UI_FOLLOW_GIMBAL,     // 跟随
    UI_SLOW_GYRO,         // 慢陀螺
    UI_FAST_GYRO,         // 快陀螺
    UI_Change_GYRO,       // 受击打变速陀螺
    UI_VERTICAL_FOLLOW,   // 正交跟随（平步）
    UI_CHASS_STOP_RESET,  // 底盘立即停止并在一段时间后复位（平步）
    UI_FOLLOWBACK_GIMBAL, // 反向跟随
    UI_MOVE_BACK,         // 倒车
    UI_CHASS_AUTO,        // 底盘自主
    UI_JUMP,              // 跳跃
} ui_motion_mode_e;

typedef enum
{
    AIMBOT_MODE = 0,    // 普通自瞄
    AIMBUFF_CONST_MODE, // 前哨
    DANGLING_MODE,   // 吊射模式
} ui_aimbot_mode_e;

typedef enum
{
    SINGLE_STRIKE = 0x01,
    TRIPLE_STRIKE = 0x02,
} strike_mode_e;

typedef enum
{
    AmmoBooster_Unselected = 0, // 未选择
    Outbreak_Priority,          // 爆发优先
    Cooling_Priority,           // 冷却优先
    BulletSpeed_Priority,       // 弹速优先
} Local_AmmoBooster_Mode_Enum;  // 枪管类型

typedef enum
{
    Chassis_Unselected = 0, // 未选择
    Power_Priority,         // 功率优先
    Blood_Priority,         // 血量优先
    Banlance_Chassis,       // 平衡底盘

    ChassisTypeAll,
} Local_Chassis_Mode_Enum; // 底盘类型

typedef enum
{
    Visual_Working_EOK = 0, // 工作正常
    Visual_Com_Failure,     // 通信异常
    Visual_Working_Failure, // 工作异常
} Visual_Error_State_Enum;

typedef enum
{
    My_Color_Blue,
    My_Color_Red,
} My_Color_Enum; // 当前己方颜色--注意查看云台代码而不是枚举

typedef enum
{
    gimErr_yaw,
    gimErr_pitch,
    gimErr_Rub,
    gimErr_launch,
    gimErr_com,
    gimErr_Num,

} gimbalErrState_e;

// 得到当前云台错误状态
uint8_t getGimErrState(void);

/***
 * @brief 返回底盘运动模式
 * @retval
 ***/
extern ui_motion_mode_e Get_Motion_Mode(void);

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
extern ui_aimbot_mode_e Get_Aimbot_Mode(void);

/**
 * @brief 获取当前自瞄的己方颜色
 * @author fwlh
 * @return My_Color_Enum    自方颜色
 */
extern My_Color_Enum Get_Color_Myself(void);

/***
 * @brief 返回Ui重置标志位
 * @retval
 ***/
extern rt_uint8_t Get_UI_Reset_Flg(void);

// 获取二维按键当前处于哪个菜单下
//extern ui_option_display_e Get_UI_Option_Flag(void);

// 读取当前是否切换超电放电状态，这里里只是开关超电的按键状态，并不实际打开或者关闭超电
uint8_t GetGimIfSuperCapOutput(void);

// 读取上一次云台控制信息刷新时间
extern rt_tick_t Get_Gim_FreshTick(void);

// 读取云台当前是否刚刚发生复位事件
extern rt_uint8_t Get_Gim_Reset_Status(void);

// 读取当前摩擦轮是否开启
extern rt_bool_t Read_Rub_Started(void);

// 读取当前是否处于客户端模式, 返回真代表在客户端模式
extern rt_bool_t Get_Client_Status(void);

// 获取云台中的离线电机, 从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘
extern rt_uint8_t Get_Gimbal_Motor_Offline_State(void);

// 获取发射机构是否存在卡弹情况
extern rt_bool_t Get_Gimbal_Stuck_State(void);

// 获取视觉模块异常状态
extern Visual_Error_State_Enum Get_Visual_Working_Error_State(void);

//英雄上台阶标志位
extern uint8_t get_up_stairs_flag(void);
//英雄下台阶标志位
extern uint8_t get_down_stairs_flag(void);

extern uint8_t get_forward_up(void);
extern uint8_t get_back_up(void);
extern uint8_t Get_Rub_Speed(void);

// 获取云台反馈的 Yaw/Pitch 角度(单位:度, 范围 -180.00~+180.00, 仅在吊射模式有效)
extern float Get_GimbalFB_Yaw(void);
extern float Get_GimbalFB_Pitch(void);

//! 未发暂时留空
// // 获取当前的本地底盘模式(0功率优先 1血量优先)
// extern Local_Chassis_Mode_Enum Get_Local_Chassis_Type(void);
// // 获取当前的本地枪口模式(0爆发优先 1冷却优先 2弹速优先)
// extern Local_AmmoBooster_Mode_Enum Get_Local_AmmoBooster_Type(void);
// // 获取当前云台是否为自瞄认为的最佳射击距离
// uint8_t Get_if_aimbot_best_dis(void);
// // 获取云台发送给底盘关于视觉是否正常识别的标志位
// uint8_t Get_Gim_Visual_Target_Found(void);
// //视觉识别基地引导灯和大弹丸
// hero_light_bullet_t get_hero_ui_data(void);

// 未用IMU
//  uint8_t get_chassis_pitch(float *pitch);
//  uint8_t get_chassis_roll(float *roll);
//  imu_euler_t get_chassis_imu_data(void);
//  获取本地的机器人等级
//  extern rt_uint8_t Get_Local_Robot_Level(void);
#endif