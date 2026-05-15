#ifndef _VISUAL_DATA_H_
#define _VISUAL_DATA_H_

#include <board.h>

/******************视觉发送，底盘接收*************************************** */
//控制数据，包含x和y的线速度和角速度，以及小陀螺模式，跟随角度
typedef __packed struct
{
    float x;                                           // x线速度
    float y;                                           // y线速度
    float w;                                           // 角速度
    uint8_t is_tyro;                                   // 小陀螺模式
    uint16_t virt_yaw;                                 // 跟随虚拟角度【0~360】单位：0.1度
}Ctrl_info_t;

/******************底盘发送，视觉接收*************************************** */
typedef __packed struct 
{
    float vel_x;
    float vel_y;
} Robot_wheel_odometry_t;//底盘正解算后的x\y方向速度!

typedef __packed struct
{
    uint8_t game_type : 4;              // 比赛类型
    uint8_t game_progress : 4;          // 当前比赛阶段
    uint16_t stage_remain_time;         // 当前阶段剩余时间
    uint8_t dart_target : 4;            // 飞镖目标 默认/前哨 = 0 基地 = 1/2/3/4
    uint8_t small_energy_state : 2;     // 小能量机关状态 0为默认 1为激活 2为正在激活
    uint8_t big_energy_state : 2;       // 大能量机关状态 0为默认 1为激活 2为正在激活
}Game_info_t; // 比赛状态数据!

typedef __packed struct
{
    uint16_t robot_1_HP;  // 己方 英雄机器人血量，未上场以及罚下血量为 0
    uint16_t robot_2_HP;   // 己方 工程机器人血量
    uint16_t robot_3_HP;   // 己方 步兵机器人血量
    uint16_t robot_4_HP;   // 己方 步兵机器人血量
    uint16_t reserve;      // 保留位
    uint16_t robot_7_HP;   // 己方 哨兵机器人血量
    uint16_t outpost_HP;   // 己方 前哨站血量
    uint16_t base_HP;      // 己方 基地血量
}Game_robot_survivors_t; // 比赛机器人血量数据!

typedef __packed struct
{
    uint8_t robot_id;               // 机器人id 可用于进行数据校验
    uint8_t robot_level;            // 机器人等级
    uint16_t remain_HP;             // 剩余血量
    uint16_t max_HP;                // 血量上限
    uint16_t bullet_remaining_num_17mm; // 机器人枪口每秒冷却值
    uint16_t shooter_cooling_limit; // 机器人枪口热量上限
    uint16_t chassis_power_limit;   // 机器人底盘功率上限
} Robot_info_t; // 机器人状态数据!

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
    uint16_t coin_remaining_num;        // 剩余金币数量
    uint16_t projectile_allowance_fortress;//获取堡垒增益点提供的储备17mm弹丸允许发弹量
} Bullet_remaining_t; // 子弹剩余发射数和剩余金币数量!

/***********设置的CMDid,发送次数，发送周期********************************* */
#define CMD_ID_VISUAL_CTRL 0x01 // 视觉发送控制指令ID
#define CMD_ID_SENTRY_CMD_INFO 0x03 // 视觉发送哨兵数据ID（视觉发送给底盘）

#define CMD_ID_GAME_INFO 0x04 // 向视觉发送比赛信息
#define CMD_ID_GAME_ROBOT_survivors 0x05 // 向视觉发送机器人血量信息
#define CMD_ID_ROBOT_INFO 0x07 // 向视觉发送机器人信息（自身信息）
#define CMD_ID_Bullet_remaining 0x09 // 向视觉发送剩余子弹数和剩余金币数量
#define CMD_ID_Robot_wheel_odometry 0x0D  // 底盘向导航发送的底盘正解算信息
#define CMD_ID_Sentry_INFO 0x0E  // 向视觉发送哨兵自主决策反馈信息
#define CMD_ID_opponent_position 0x0F //  向视觉发送雷达发送的对方机器人位置信息

//200/x = f(Hz)
#define CMD_SET_GAME_INFO -1,20 // 向视觉发送比赛信息
#define CMD_SET_GAME_ROBOT_survivors -1,20// 向视觉发送机器人血量信息
#define CMD_SET_ROBOT_INFO -1,20 // 向视觉发送机器人信息（自身信息）
#define CMD_SET_Bullet_remaining -1,20 // 向视觉发送剩余子弹数和剩余金币数量
#define CMD_SET_Robot_wheel_odometry -1,1 // 给视觉发送地盘姿态正解算后的x、y方向速度数据数据
#define CMD_SET_Sentry_INFO -1,50 // 向视觉发送哨兵自主决策反馈信息（裁判系统以1Hz频率反馈）
#define CMD_SET_opponent_position -1,5 // 向视觉发送对方机器人位置信息（30Hz左右频率反馈）

// #define CMD_ID_VISUAL_RADAR 0x02 // 视觉发送雷达数据ID
// #define CMD_ID_EVENT_DATA 0x06 // 向视觉发送场地事件信息
// #define CMD_ID_ROBOT_POSITION 0x08 // 向视觉发送本机器人位置信息（自身位置信息）
// #define CMD_ID_Ground_robot_position 0x0A // 向视觉发送地面机器人位置信息（自家机器人信息）
// #define CMD_ID_Sentry_info 0x0B // 给视觉发送哨兵自主决策反馈数据（裁判系统反馈信息）
// #define CMD_ID_Robot_command_map 0x0C // 给视觉发送云台手发送给哨兵的点位信息（云台手命令哨兵前往的位置）

//设置 发送次数（有限次就写正整数，无限次发送写-1） 和 每几个周期发送一次
// #define CMD_SET_VISUAL_CTRL -1,2 // 视觉发送控制指令
// #define CMD_SET_VISUAL_RADAR -1,2 // 视觉发送雷达数据ID
// #define CMD_SET_SENTRY_CMD_INFO -1,2 // 视觉发送哨兵数据ID（视觉发送给底盘）
// #define CMD_SET_GAME_INFO -1,10 // 向视觉发送比赛信息
// #define CMD_SET_GAME_ROBOT_survivors -1,100// 向视觉发送机器人血量信息
// #define CMD_SET_EVENT_DATA -1,500 // 向视觉发送场地事件信息
// #define CMD_SET_ROBOT_INFO -1,100 // 向视觉发送机器人信息（自身信息）
// #define CMD_SET_ROBOT_POSITION -1,500 // 向视觉发送本机器人位置信息（自身位置信息）
// #define CMD_SET_Bullet_remaining -1,200 // 向视觉发送剩余子弹数和剩余金币数量
// #define CMD_SET_Ground_robot_position -1,200 // 向视觉发送地面机器人位置信息（自家机器人信息）
// #define CMD_SET_Sentry_info -1,100 // 给视觉发送哨兵自主决策反馈数据（裁判系统反馈信息）
// #define CMD_SET_Robot_command_map -1,20 // 给视觉发送云台手发送给哨兵的点位信息（云台手命令哨兵前往的位置）
// #define CMD_SET_Robot_wheel_odometry -1,10 // 给视觉发送地盘姿态正解算后的x、y方向速度数据数据

// #define CMD_SET_VISUAL_CTRL -1,20 // 视觉发送控制指令
// #define CMD_SET_VISUAL_RADAR -1,20 // 视觉发送雷达数据ID
// #define CMD_SET_SENTRY_CMD_INFO -1,20 // 视觉发送哨兵数据ID（视觉发送给底盘）

// #define CMD_SET_EVENT_DATA -1,20 // 向视觉发送场地事件信息

// #define CMD_SET_ROBOT_POSITION -1,20 // 向视觉发送本机器人位置信息（自身位置信息）
// #define CMD_SET_Ground_robot_position -1,20 // 向视觉发送地面机器人位置信息（自家机器人信息）
// #define CMD_SET_Sentry_info -1,20 // 给视觉发送哨兵自主决策反馈数据（裁判系统反馈信息）
// #define CMD_SET_Robot_command_map -1,20 // 给视觉发送云台手发送给哨兵的点位信息（云台手命令哨兵前往的位置）
// #define CMD_SET_Robot_wheel_odometry -1,1 // 给视觉发送地盘姿态正解算后的x、y方向速度数据数据

#endif
