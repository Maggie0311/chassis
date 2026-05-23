#ifndef __DRV_GIMBAL_COM_H__
#define __DRV_GIMBAL_COM_H__

#include "robot_data.h"
#include "can_com_thread.h"

/*Tx*/
#define Gim_Tx_PERIOD 10//发送间隔（ms）
typedef void (*Gimbal_TransDataGet_ptr)(CANInstance *instance);//获取需要发送的数据的函数指针

#define GIMBAL_SHOOT_TX 0x105 // 裁判系统枪管信息ID
// #define GIMBAL_MIXDATA_TX 0x106  // 向云台发送各类数据ID

typedef enum
{
    Shooter_msg_Index,      //枪管信息
    // Mixed_msg_Index,        //其他信息
    Msg_num_Tx,             //报文种数
} GimCom_Tx_index_e;   //发送报文索引

typedef __packed struct
{
    // 枪口状态数据
    uint16_t cooling_heat;     // 枪口热量
    uint16_t cooling_rate;     // 枪口每秒冷却值
    uint16_t cooling_limit;    // 枪口热量上限
    uint16_t speed_limit;      // 枪口速度上限
    //uint16_t cooling_buff;     // 枪口冷却具体值（直接值，值为 x 表示热量冷却增加 x/s）（26改）
    //!字节超了
} Gim_Reftrans1_t;        // 发送给云台的枪口数据

// //!云台还未修改，暂时保持老版
// typedef __packed struct
// {
//     // 工作状态数据
//     unsigned isStartGame : 1;   // 是否开始比赛(1为开始，0为未开始)
//     unsigned hurtID : 3;        // 装甲板ID，未受攻击时为FF
//     unsigned tempColor : 1;     // 红蓝方
//     unsigned ourOutpost : 1;    // 己方前哨战是否存活
//     unsigned enermyOutpost : 1; // 敌方前哨战是否存活
//     unsigned speed : 16;        // 弹速
//     //unsigned SentryHP : 16;     // 哨兵的血量
//     unsigned remaining_bullet_num:16;//剩余发弹量
//     uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
//     uint16_t target_robot_ID; // 要作用的目标机器人 ID           当发送位置信息时，该项为 0
//     unsigned reserved : 1;//保留位
// } Gim_Reftrans2_t;        // 发送给云台的裁判系统数据



/*Rx*/
#define MSG_READ_2BYTES(low, high) ((rt_int16_t)(((low) << 8) | high)) // 二字节读取宏

#define GIMBAL_CTRL_RX 0x100    // 云台控制底盘相关ID（rx_gimctrl）
#define GIMBAL_DATA_RX 0x401     // 从云台接收的各类数据ID（rx_gimData）
// #define GIMBAL_IMU_DATA_RX 0x402 // 从云台接收的IMU数据ID（rx_gimIMU）
#define GIMBAL_UI_LIGHT_DATA_RX 0x403 // 从云台接收的UI引导灯数据ID（rx_gimUI）
#define GIMBAL_UI_BULLET_DATA_RX 0x404 // 从云台接收的UI弹丸数据ID（rx_gimBullet）
typedef enum
{
    Ctrl_msg_Index,    // 底盘控制
    mixData_msg_Index, // 杂项数据
    Imu_msg_Index,     // IMU数据
#if defined HREO_UI_SPECIAL
    Ui_light_msg_Index,  // UI引导灯数据
    Ui_bullet_msg_Index, // UI弹丸数据
#endif
    Msg_num_Rx,      // 报文种数
} GimCom_Rx_index_e; // 接收报文索引

/* 角度编解码: -180.00~+180.00°, 0.01°/LSB, 16bit uint */
#define CAN_U16_TO_ANGLE(raw) (((float)(raw) / 100.0f) - 180.0f)

typedef __packed struct
{
    // ---- Word 0: 控制区 bit[0:31] ----
    unsigned chassis_reset  : 1;   // bit0:  底盘复位指令
    unsigned supercap_switch : 1;  // bit1:  超电开关(C键翻转)
    unsigned step_up        : 1;   // bit2:  上台阶(Ctrl+Z翻转)
    unsigned step_down      : 1;   // bit3:  下台阶(Ctrl+X翻转)
    unsigned lift_front     : 1;   // bit4:  前抬升(Ctrl+R翻转)
    unsigned lift_rear      : 1;   // bit5:  后抬升(Ctrl+F翻转)
    unsigned yaw_angle      : 16;  // bit6-21:  Yaw反馈角(0.01°/LSB)
    unsigned motion_mode    : 4;   // bit22-25: 底盘运动模式 ← 位置变了!
    unsigned ctrl_rsv       : 6;   // bit26-31: 控制区保留位

    // ---- Word 1: UI区 bit[32:63] ----
    unsigned ui_reset              : 1;   // bit32: UI重置标志
    unsigned aimbot_mode           : 3;   // bit33-35: 瞄准模式
    unsigned self_color            : 1;   // bit36: 己方颜色
    unsigned visual_com_online     : 1;   // bit37: 视觉通信在线
    unsigned visual_working_correct : 1;  // bit38: 视觉工作正常
    unsigned yaw_motor_online      : 1;   // bit39: Yaw电机在线
    unsigned pitch_motor_online    : 1;   // bit40: Pitch电机在线
    unsigned rub_started           : 1;   // bit41: 摩擦轮已开启
    unsigned rub_comm_error        : 1;   // bit42: 摩擦轮通信异常
    unsigned launch_motor_online   : 1;   // bit43: 拨弹盘电机在线
    unsigned strike_stuck          : 1;   // bit44: 发射机构卡弹
    unsigned allow_overheat_flag   : 1;   // bit45: 允许超热量标志(B键翻转)
    unsigned now_client_control    : 1;   // bit46: 客户端控制模式
    unsigned gimbal_reset          : 1;   // bit47: 云台复位状态
    unsigned pitch_angle           : 16;  // bit48-63: Pitch反馈角(0.01°/LSB)
} Gim_mixData_Rx_t; // 原 Gim_mixData_Rx_t, 协议升级

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} imu_euler_t;

typedef struct
{
    uint16_t Ui_Guide_Light_Circle_x;     // 英雄吊射基地引导灯坐标（视觉发送以左上角为原点，向右是x，向下是y)
    uint16_t Ui_Guide_Light_Circle_y;     // 英雄吊射基地引导灯
    uint8_t Ui_Guide_Light_Circle_radius; // 英雄吊射基地引导灯半径
    rt_tick_t Ui_Guide_Light_Circle_freshtick; // 英雄吊射基地引导灯数据刷新时间
    uint16_t Ui_Bullet_Circle_x;          // 英雄大弹丸坐标（视觉发送以左上角为原点，向右是x，向下是y)
    uint16_t Ui_Bullet_Circle_y;          // 英雄大弹丸
    uint8_t Ui_Bullet_Circle_radius;      // 英雄大弹丸半径
    rt_tick_t Ui_Bullet_Circle_freshtick; // 英雄大弹丸数据刷新时间
} hero_light_bullet_t;

extern Gim_mixData_Rx_t gimbal_mixdata_msg;
extern rt_tick_t gimbal_mixdata_msg_freshtick;
extern imu_euler_t gimbal_imudata_msg;
extern hero_light_bullet_t gimbal_ui_data_msg;
extern rt_tick_t imu_data_freshtick;
extern rt_tick_t gimbal_ui_data_msg_freshtick;

//云台通信初始化
rt_err_t Gimbal_Com_Init(void);

#endif
