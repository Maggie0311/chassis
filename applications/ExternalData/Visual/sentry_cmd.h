// 哨兵向裁判系统通讯发送功能头文件
#ifndef _SENTRY_CMD_H_
#define _SENTRY_CMD_H_

#include "app_RefSystem.h"
#include "visual_data.h"

// 一次发送裁判系统的数据最大长度    本身一次发送最大长度加上帧头帧尾等
#define MAX_Sentry_Data_BUFF_SIZE (Robot_Interact_DataLen + 15) //(5+2+6+Robot_Interact_DataLen+2)

#define VISUAL_DATA_NUM_MAX 6

typedef __packed struct
{
    float x; // 地图坐标x
    float y; // 地图坐标y
} radar_cardata_t;

typedef __packed struct
{
    uint8_t receive_flag;
    visual_data_t radarData;
    rt_tick_t tick;
} radar_RxData_t;

typedef __packed struct
{
    uint8_t send_flag;
    radar_cardata_t radar_cardata[VISUAL_DATA_NUM_MAX];
} radar_TxData_t;

typedef __packed struct
{
    FrameHeader_t txFrameHeader; // 帧头
    uint16_t CmdID;              // 机器人间通信为0x301

    ext_student_interactive_header_data_t dataFrameHeader;
    uint8_t send_data[Robot_Interact_DataLen]; // 发送数据

    uint16_t FrameTail; // CRC16校验

} Interact_Data_t; // 用于0x301发送的结构体

typedef enum
{
    Sentry_Data_Type_Cmd = 0, // 哨兵自主决策指令
    Sentry_Data_Type_Radar,   // 哨兵与雷达间通信
} Sentry_Data_Type_e;

// 哨兵自主决策，雷达通讯线程初始化
rt_err_t Sentry_Data_Send_Init(void);

// 解析雷达站数据
void protocolRadarData(const Frame_t *const frame);

// 获取云台手是否有效按下雷达键值("R")
uint8_t get_radar_commd_keyboard(void);

// 获取雷达站数据
uint8_t get_Radar_Data(void);

#endif
