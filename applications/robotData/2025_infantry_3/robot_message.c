#include "robot_message.h"
#include "custom_message.h"
#include "drv_utf.h"
#include "fun_powerCtrl.h"
#include "mod_motion.h"
#include "SuperCap_Com.h"
#include "protect.h"
extern uint8_t power_ctrl_close;

Message_t message[message_num] = {
    [realvel_single_dyn] =
        {
            .message_type = Message_Type_Montage_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .message_data_utf8[1] = "",
            .message_data_utf8[2] = "",
            .message_data_utf8[3] = "",
            .message_data_utf8[4] = "|",
            .init_line_num = 5,
        },
    [scap_state_single] =
        {
            .message_type = Message_Type_Montage_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .message_data_utf8[1] = "超电",
            .message_data_utf8[2] = "打开",
            .message_data_utf8[3] = "关闭",
            .message_data_utf8[4] = ":",
            .message_data_utf8[5] = "异常",
            // .message_data_utf8[6] = "放电",
            // .message_data_utf8[7] = "等待",
            .message_data_utf8[6] = "掉线",
            .message_data_utf8[7] = "|",
            .init_line_num = 8,
        },
    [chassis_state]=
        {
            .message_type = Message_Type_Montage_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .message_data_utf8[1] = "底盘",
            .message_data_utf8[2] = "正常",
            .message_data_utf8[3] = "左前",
            .message_data_utf8[4] = "右前",
            .message_data_utf8[5] = "左后",
            .message_data_utf8[6] = "右后",
            .message_data_utf8[7] = "掉线",
            .message_data_utf8[8] = ":",
            .message_data_utf8[9] = "|",
            .init_line_num = 10,
        },
    [gimbal_state]=
        {
            .message_type = Message_Type_Montage_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .message_data_utf8[1] = "云台",
            .message_data_utf8[2] = "正常",
            .message_data_utf8[3] = "控制",
            .message_data_utf8[4] = "YAW",
            .message_data_utf8[5] = "PITCH",
            .message_data_utf8[6] = "掉线",
            .message_data_utf8[7] = ":",
            .init_line_num = 8,
        },
    [referee_state]=
        {
            .message_type = Message_Type_Montage_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .message_data_utf8[1] = "裁判系统",
            .message_data_utf8[2] = "正常",
            .message_data_utf8[3] = "掉线",
            .message_data_utf8[4] = ":",
            .init_line_num = 5,
        },
};

Message_List_e Now_Message = (Message_List_e)0;

void Select_Send_Message(void)
{
    static Message_List_e Message_Select = (Message_List_e)0;
    /****用户从此编写消息选择的代码****/
    static Message_List_e i = (Message_List_e)0;
    if (i == message_num)
        i = (Message_List_e)0;
    Message_Select = i++; // 选择的消息放置在Message_Select中
    /****用户从此编写消息选择的代码****/
    /****用户从此编写所选消息数据更新的代码****/
    uint8_t order_index_temp = 0;
    switch (Message_Select)
    {
    // case realvel_single_dyn:
    //     float_to_utf16(getRealVel().vel.y, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[1], CUSTOM_DATA_NUM / 2);
    //     message[realvel_single_dyn].message_montage_order[order_index_temp++] = 1;
    //     message[realvel_single_dyn].message_montage_order[order_index_temp++] = 4;
    //     float_to_utf16(getMaxVely_in_a_move(), 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[2], CUSTOM_DATA_NUM / 2);
    //     message[realvel_single_dyn].message_montage_order[order_index_temp++] = 2;
    //     // float_to_utf16(getRealVel().vel.x, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[2], CUSTOM_DATA_NUM / 2);
    //     // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 2;
    //     // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 4;
    //     // float_to_utf16(getRealVel().angvel, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[3], CUSTOM_DATA_NUM / 2);
    //     // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 3;
    //     message[realvel_single_dyn].message_montage_order[order_index_temp++] = 0;
    //     break;
    case scap_state_single:
        message[scap_state_single].message_montage_order[order_index_temp++] = 1; // 超电
        message[scap_state_single].message_montage_order[order_index_temp++] = 4;     // ：
        if (power_ctrl_close)
            message[scap_state_single].message_montage_order[order_index_temp++] = 3; // 关闭
        else
            message[scap_state_single].message_montage_order[order_index_temp++] = 2; // 打开
        if(get_SuperCap_if_abnormal()==1)
        {
            message[scap_state_single].message_montage_order[order_index_temp++] = 7; // |
            message[scap_state_single].message_montage_order[order_index_temp++] = 5; // 异常
        }
        if(Read_Super_Capacity_Online()==RT_FALSE)
        {
            message[scap_state_single].message_montage_order[order_index_temp++] = 7; // |
            message[scap_state_single].message_montage_order[order_index_temp++] = 6; // 掉线
        }
        message[scap_state_single].message_montage_order[order_index_temp++] = 0; // 结束拼接
        break;
    case chassis_state:
        message[chassis_state].message_montage_order[order_index_temp++] = 1; // 底盘
        message[chassis_state].message_montage_order[order_index_temp++] = 8; // ：
        if ((GetMotorOffline()&1<<0) > 0)
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 4; // 右前轮
        }
        if ((GetMotorOffline()&1<<1) > 0)
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 3; // 左前轮
        }
        if ((GetMotorOffline()&1<<2) > 0)
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 5; // 左后轮
        }
        if ((GetMotorOffline()&1<<3) > 0)
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 6; // 右后轮
        }
        if (GetMotorOffline()>0)
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 7; // 掉线
        }
        else 
        {
            message[chassis_state].message_montage_order[order_index_temp++] = 2; // 正常
        }
    case gimbal_state:
        message[gimbal_state].message_montage_order[order_index_temp++] = 1; // 云台
        message[gimbal_state].message_montage_order[order_index_temp++] = 7; // ：
        if ((GetGimbalOffline() & 1<<0) > 0)
        {
            message[gimbal_state].message_montage_order[order_index_temp++] = 2; // 控制
        }
        if ((GetGimMotorOffline() & 1<<0) > 0)
        {
            message[gimbal_state].message_montage_order[order_index_temp++] = 4; // YAW
        }
        if ((GetGimMotorOffline() & 1<<1) > 0)
        {
            message[gimbal_state].message_montage_order[order_index_temp++] = 5; // PITCH
        }
        if(GetGimbalOffline()>0||GetGimMotorOffline()>0)
        {
            message[gimbal_state].message_montage_order[order_index_temp++] = 6; // 掉线
        }
        else 
        {
            message[gimbal_state].message_montage_order[order_index_temp++] = 3; // 正常
        }
        message[gimbal_state].message_montage_order[order_index_temp++] = 0; // 结束拼接
        break;
    case referee_state:
        message[referee_state].message_montage_order[order_index_temp++] = 1; // 裁判系统
        message[referee_state].message_montage_order[order_index_temp++] = 4; // ：
        if(!ISRefValid())
        {
            message[referee_state].message_montage_order[order_index_temp++] = 3; // 掉线
        }
        else 
        {
            message[referee_state].message_montage_order[order_index_temp++] = 2; // 正常
        }
    default:
        break;
    }
    /****用户从此编写所选消息数据更新的代码****/
    Custom_Message_waitEvent(custom_message_sendFinish_eve);
    Now_Message = Message_Select;
}
