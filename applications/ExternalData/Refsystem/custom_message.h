#ifndef _USER_308_H_
#define _USER_308_H_

#include "app_RefSystem.h"
#include "robot_message.h"

#define CUSTOM_DATA_NUM (30)
#define CUSTOM_MESSAGE_LINE_NUM_MAX (32)

typedef __packed struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
} custom_header_t;

typedef __packed struct
{
    FrameHeader_t txFrameHeader; // 帧头
    uint16_t CmdID;              // 机器人间通信为0x301

    custom_header_t dataFrameHeader;
    uint8_t custom_data[CUSTOM_DATA_NUM]; // 发送数据

    uint16_t FrameTail; // CRC16校验

} Custom_Data_t; // 用于0x308发送的结构体-43字节

typedef enum
{
    Message_Type_SingleLine = 0,     // 单行消息
    Message_Type_MultiLine,          // 多行消息
    Message_Type_Montage_SingleLine, // 拼接的单行消息
    Message_Type_Montage_MultiLine,  // 拼接的多行消息
} Message_Type_e;

typedef __packed struct
{
    // uint16_t sender_id;
    // uint16_t receiver_id;
    Message_Type_e message_type;                                                   // 消息类型
    uint8_t send_line_num;                                                         // 当前发送行数(0:发送完成或未开始发送，1:发送第一行，2:发送第二行...)
    uint8_t message_line_num;                                                      // 发送消息行数
    uint8_t init_line_num;                                                         // 初始化消息行数
    uint8_t message_data_utf8[CUSTOM_MESSAGE_LINE_NUM_MAX][CUSTOM_DATA_NUM];       // 消息源码utf-8内容
    uint16_t message_data_utf16[CUSTOM_MESSAGE_LINE_NUM_MAX][CUSTOM_DATA_NUM / 2]; // 消息解码utf-16内容

    uint8_t message_montage_order[CUSTOM_MESSAGE_LINE_NUM_MAX]; // 拼接顺序码组

    uint8_t cnt; // 8位用户定制代码消息数据自定义统计计数器
} Message_t;

typedef enum
{
    custom_message_timer_eve = 1 << 1,      // 每周期检查是否可以发送
    custom_message_dataReady_eve = 1 << 2,  // 检查是否需要发送
    custom_message_sendReady_eve = 1 << 3,  // 检查上一次是否发送完成(即一行消息发送完成)
    custom_message_sendFinish_eve = 1 << 4, // 检查整个消息是否发送完成(即多行消息全部发送完毕)
} Custom_Message_threadsyn_e;

// user-308通讯线程初始化
rt_err_t Custom_Message_Data_Send_Init(void);
rt_err_t Custom_Message_event_Init(void);
rt_err_t Custom_Message_waitForSend(void);
rt_err_t Custom_Message_sendEvent(Custom_Message_threadsyn_e event);
rt_err_t Custom_Message_waitEvent(Custom_Message_threadsyn_e event);
uint8_t message_cat_block(Message_List_e index, uint8_t det, uint8_t src);
uint8_t message_cat_single(Message_List_e index);
uint8_t message_cat_multi(Message_List_e index);

#endif
