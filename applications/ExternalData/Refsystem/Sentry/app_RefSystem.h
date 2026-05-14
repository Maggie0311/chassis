#ifndef __APP_REFSYSTEM_H__
#define __APP_REFSYSTEM_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "mod_RefSystem.h"
#include "func_Visual_Com.h"

/* 英雄弹速上限档位 */
#define HERO_BS_1 10
#define HERO_BS_2 16

/* 步兵弹速上限档位 */
#define INFANTRY_BS_1 15
#define INFANTRY_BS_2 18
#define INFANTRY_BS_3 30

typedef struct
{
    rt_uint8_t robot_if_hurt;     // 机器人是否受伤
    rt_uint8_t robot_hurt_armor;  // 机器人受伤的装甲板ID
    rt_uint8_t build_if_attacked; // 建筑物是否受伤，低三位代表哨兵，前哨站，基地。位数据为1：掉血，为0：不掉血
    rt_uint16_t sentry_hp;        // 哨兵血量
    rt_uint16_t outpost_hp;       // 前哨站血量
    rt_uint16_t base_hp;          // 基地血量
    uint32_t tick;

} Ref_Hurt_t;

#include "drv_GimMotor.h"
#include "SuperCap_Com_new.h"
#include "fun_protocolGim.h"
#include "robot_data.h"

// UI需要获取的数据
typedef struct UI_Used_Data_t
{
    /*------------------------------------使用----------------------------------------------------------*/
    float RemainCapacity;    // 超级电容电量
    float ScRate;            // 超级电容电流衰减比例
    float GM_YAW_360F;       // 云台编码器角度
    float GM_Pitch;          // 云台编码器角度

    strike_mode_e Strike_Mode; // 显示子弹模式,有单发与三连发

    ui_motion_mode_e Motion_Mode; // 底盘模式

    ui_aimbot_mode_e Aimbot_Mode; // 自瞄模式

    My_Color_Enum Color_Myself;   // 当前自瞄的己方颜色

    uint16_t powerbuff; //缓冲能量

    ui_option_display_e MenuOption;             // 当前所在的菜单

    Visual_Error_State_Enum Visual_Error_State; // 视觉异常状态


    uint8_t gimErrState;                                // 云台异常状态
    rt_uint8_t Gimbal_Motor_Offline;                    // 云台电机离线情况,从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘, 置 1 表示离线
    rt_uint8_t Chassis_Motor_Offline;                   // 底盘电机离线情况, 从低到高分别为：右前、左前、左后、右后、Yaw轴电机, 置 1 表示离线


    unsigned Rub_Started : 1;              // 摩擦轮已开启
    unsigned IfSuperCap_open : 1;          // 云台是否允许超级电容放电, 1 代表允许放电
    unsigned PowerRestriction_Status : 1;  // 自动电量保留状态, 1 代表开启自动电量保留（未使用）
    unsigned Capacity_Charge_Cmd : 1;      // 开启超级电容充电的指令, 1 为开启充电(0时建议标红)(未使用)
    unsigned Gimbal_Online : 1;            // 云台通信正常, 1 代表通信正常
    unsigned Super_Capacity_Online : 1;    // 超级电容控制板通信正常, 1 代表通信正常
    unsigned Chassis_Reset_Warning : 1;    // 底盘复位警示, 1 代表底盘刚刚发生了复位
    unsigned Gimbal_Reset_Warning : 1;     // 云台复位警示, 1 代表云台刚刚发生了复位, 需要告诉操作手及时设置机器人状态
    unsigned RefSystem_Offline : 1;        // 裁判系统数据离线标志, 1 代表裁判系统数据离线
    unsigned Strike_Stuck : 1;             // 发射机构卡弹标志位, 1 代表发射机构卡弹
    unsigned Robot_Hurted : 1;             // 机器人收到伤害
    unsigned UI_Reset_Flag : 1;            // 重置 UI 的标志位, 翻转时需要重置
    rt_time_t Last_UI_refresh_Time;        // 上一次 UI 刷新时间
    unsigned IfAllow_Heat_Shoot : 1;       // 英雄是否允许超热量射击, 1 代表允许
     //!云台(视觉)目前还没加些功能
    //unsigned if_aimbot_best_dis : 1;       // 当前是否为云台认为的最佳射击距离
    //hero_light_bullet_t Hero_UI_Data;     // 英雄视觉识别的基地引导灯和弹丸数据
    // unsigned visual_target_found : 1; // 视觉是否找到目标（用于在UI上提示操作手是否找到目标）

    /*------------------------------------未用-------------------*/
    // imu_euler_t Ch_IMU_Data; // 底盘IMU的数据
    // float GM_IMU_Pitch;      // 云台imu pitch
    // ext_robot_hurt_t hurtInfo; // 获取伤害状态,发送频率--当建筑受到攻击时警告
    // unsigned Self_Robot_HP;
    // Local_Chassis_Mode_Enum Local_Chassis_Mode; // 当前设置的本地底盘类型(只有裁判系统离线的时候画出来)
    // Local_AmmoBooster_Mode_Enum Local_AmmoBooster_Mode; // 当前设置的本地发射机构类型(只有裁判系统离线的时候画出来)
    // rt_uint8_t Local_Robot_Level;                       // 当前设置的本地机器人等级(只有裁判系统离线的时候画出来)
    // float Bullet_Speed;   // 弹速
    // float Ref_AmmoRemain; // 剩余弹量
    // // unsigned sentryIsErr : 1;          // 哨兵是否在启动区
    // unsigned Magazine_Status : 1; // 弹仓状态
    // unsigned Bulid_If_Hurt : 1;   // 建筑物状态
    // unsigned Client_Mode : 1;     // 当前处于客户端控制模式
    // unsigned HeatLimit_Status : 1; // 本地热量限制状态, 1 代表关闭
    // unsigned IfWireless_Charge : 1; // 是否开启无线充电, 1 代表开启无线充电，0表关闭无线充电
    // unsigned RefSystem_Forced_Offline : 1; // 裁判系统强制离线指令, 1 代表强制离线
    // unsigned Energy_is_already : 1;        // 能量机关是否拟合结束

} UI_Used_Data_t;

/*------------------------------------XXX----------------------------------------------------------*/


/**
 * @brief    获取枪口上限速度
 * @param    None
 * @return   单位 m/s
 */
rt_uint16_t Ref_Bullet_Speed_Limit(void);

/**
 * @brief    获取剩余弹量
 *		  	发送频率： 10Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
rt_uint16_t Ref_AmmoRemain(void);
/**
 * @brief 获取当前机器人是否飞坡增益
 *        根据规则：同一台机器人需在 10 秒内检测到一方场地两处飞坡增益点的场地交互模块卡，才能触发飞坡增益
 *        飞坡增益持续时间：20秒
 * @return uint8_t
 */
uint8_t RefGetIsFlySlope(void);

rt_err_t DJI_Init(void);
extern Ref_Hurt_t hurt;

#endif
