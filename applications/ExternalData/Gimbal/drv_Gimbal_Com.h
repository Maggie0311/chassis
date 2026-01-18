#ifndef __DRV_GIMBAL_COM_H__
#define __DRV_GIMBAL_COM_H__

#include "robot_data.h"
#include "can_com_thread.h"

/*Tx*/
#define Gim_Tx_PERIOD 10//发送间隔（ms）
typedef void (*Gimbal_TransDataGet_ptr)(CANInstance *instance);//获取需要发送的数据的函数指针

#define GIMBAL_SHOOT_TX 0x105 // 裁判系统枪管信息ID
#define GIMBAL_MIXDATA_TX 0x106  // 向云台发送各类数据ID

typedef enum
{
    Shooter_msg_Index,      //枪管信息
    Mixed_msg_Index,        //其他信息
    Msg_num_Tx,             //报文种数
} GimCom_Tx_index_e;   //发送报文索引

typedef __packed struct
{
    // 枪口状态数据
    uint16_t cooling_heat;     // 枪口热量
    uint16_t cooling_rate;     // 枪口每秒冷却值
    uint16_t cooling_limit;    // 枪口热量上限
    uint16_t speed_limit;      // 枪口速度上限
    uint16_t cooling_buff;     // 枪口冷却具体值（直接值，值为 x 表示热量冷却增加 x/s）（26改）
    //!字节超了
} Gim_Reftrans1_t;        // 发送给云台的枪口数据

typedef __packed struct
{
    // 杂项数据
    unsigned isStartGame : 1;   // 是否开始比赛（1为开始，0为未开始）
    unsigned hurtID : 3;        // 装甲板ID，未受攻击时为FF
    unsigned tempColor : 1;     // 红蓝方（0为红方，1为蓝方）
    unsigned ourOutpost : 1;    // 己方前哨战是否存活（1为存活，0为未存活）
    //! 新赛季敌方前哨战是否存活已删除
#ifdef CORE_USING_SENTRY
    unsigned SentryIsFire : 1;//？
#endif
    unsigned speed : 16;        // 弹速
    uint16_t SentryHP : 16;     //! 哨兵的血量 把unsigned改成uint16_t
//!:烧饼超字节了，往上挪了挪
    uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
    uint16_t target_robot_ID; // 要作用的目标机器人 ID           当发送位置信息时，该项为 0
} Gim_Reftrans2_t;        // 发送给云台的裁判系统数据

/*Rx*/
#define GIMBAL_CCTRL_RX 0x100    // 云台控制底盘相关ID（rx_gimctrl）
#define GIMBAL_DATA_RX 0x401     // 从云台接收的各类数据ID（rx_gimData）
#define GIMBAL_IMU_DATA_RX 0x402 // 从云台接收的IMU数据ID（rx_gimIMU）

typedef enum
{
    Msg_num_Rx,                //报文种数
} GimCom_Rx_index_e;   //接收报文索引




//云台通信初始化
rt_err_t Gimbal_Com_Init(void)

#endif
