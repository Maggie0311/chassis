#include "robot_message.h"
#include "custom_message.h"
#include "drv_utf.h"

#include "mod_motion.h"
#include "SuperCap_Com_new.h"
#include "robot_control.h"
#include "robot_control.h"

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
            .message_data_utf8[5] = "充电",
            .message_data_utf8[6] = "放电",
            .message_data_utf8[7] = "等待",
            .message_data_utf8[8] = "掉线",
            .init_line_num = 9,
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
    case realvel_single_dyn:
        float_to_utf16(Get_realmotion().vel.y, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[1], CUSTOM_DATA_NUM / 2);
        message[realvel_single_dyn].message_montage_order[order_index_temp++] = 1;
        message[realvel_single_dyn].message_montage_order[order_index_temp++] = 4;
        float_to_utf16(getMaxVely_in_a_move(), 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[2], CUSTOM_DATA_NUM / 2);
        message[realvel_single_dyn].message_montage_order[order_index_temp++] = 2;
        // float_to_utf16(getRealVel().vel.x, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[2], CUSTOM_DATA_NUM / 2);
        // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 2;
        // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 4;
        // float_to_utf16(getRealVel().angvel, 0, (uint16_t *)message[realvel_single_dyn].message_data_utf16[3], CUSTOM_DATA_NUM / 2);
        // message[realvel_single_dyn].message_montage_order[order_index_temp++] = 3;
        message[realvel_single_dyn].message_montage_order[order_index_temp++] = 0;
        break;
    case scap_state_single:
        message[scap_state_single].message_montage_order[order_index_temp++] = 1; // 超电
        if (power_ctrl_close)
            message[scap_state_single].message_montage_order[order_index_temp++] = 3; // 关闭
        else
            message[scap_state_single].message_montage_order[order_index_temp++] = 2; // 打开
        message[scap_state_single].message_montage_order[order_index_temp++] = 4;     // ：

        switch (Get_SuprCapState())
        {
        case CAP_STATE_CHARGE:
            message[scap_state_single].message_montage_order[order_index_temp++] = 5; // 充电
            break;
        case CAP_STATE_DISCHARGE:
            message[scap_state_single].message_montage_order[order_index_temp++] = 6; // 放电
            break;
        case CAP_STATE_NO_WORK:
            message[scap_state_single].message_montage_order[order_index_temp++] = 7; // 等待
            break;
        case CAP_STATE_ERROR:
            message[scap_state_single].message_montage_order[order_index_temp++] = 8; // 掉线
            break;
        default:
            message[scap_state_single].message_montage_order[order_index_temp++] = 7; // 等待
            break;
        }
        message[scap_state_single].message_montage_order[order_index_temp++] = 0; // 结束拼接
        break;
    default:
        break;
    }
    /****用户从此编写所选消息数据更新的代码****/
    Custom_Message_waitEvent(custom_message_sendFinish_eve);
    Now_Message = Message_Select;
}
