#include "robot_message.h"
#include "custom_message.h"
#include "drv_utf.h"

#include "mod_motion.h"
#include "SuperCap_Com_new.h"

extern uint8_t power_ctrl_close;

Message_t message[message_num] = {
    [realvel_single_dyn] =
        {
            .message_type = Message_Type_SingleLine,
            .message_line_num = 1,
            .message_data_utf8[0] = "",
            .init_line_num = 1,
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

    /****用户从此编写所选消息数据更新的代码****/
    Custom_Message_waitEvent(custom_message_sendFinish_eve);
    Now_Message = Message_Select;
}
