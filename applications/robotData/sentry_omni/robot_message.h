#ifndef _ROBOT_MESSAGE_H_
#define _ROBOT_MESSAGE_H_

typedef enum
{
    realvel_single_dyn = 0, // y轴速度,float数据转utf-16，动态变化
    scap_state_single,      // 超级电容状态
    chassis_state, // 底盘状态
    gimbal_state,  // 云台状态
    referee_state, // 裁判系统状态
    
    message_num, // 消息总数
} Message_List_e;

void Select_Send_Message(void);

#endif
