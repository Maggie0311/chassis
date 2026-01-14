#define _CRT_SECURE_NO_WARNINGS
#include "drv_send_UI.h"
#include "drv_CRC.h"

#if defined _WIN32
#include "drv_txt_operation.h"
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define rt_tick_get     clock       //rt_tick_get()函数是stm32嵌入式RTT操作系统调用的毫秒级时间读取函数，clock()是windows中time.h库中包含的毫秒级时间读取函数
///   该函数是用来发送数据到某个地方（为了调试，我写了一个文件操作的函数，使其写入txt文件中，模拟串口发送出去），如果是在linux系统中，建议改为自己的串口发送函数
///   在windows系统由于没有串口（目前不会），建议只修改写入的文件名称即可（记得需要引用我写的drv_txt_operation.h来对txt文件进行操作）
///   在linux系统中，需要重写为使用自己的串口驱动发送数据，但是建议还是使用该函数名称和原型，方便后续全平台兼容
///   在电控使用的stm32嵌入式平台上，如果使用rtt操作系统，无需定义该函数，在rtt中自带该函数
/// <param name="serial"></param>串口设备名称（设备句柄，用来查找和使用设备）
/// <param name="start_postion"></param>需要发送的内容相较于缓冲区地址的偏移字节数量
/// <param name="buffer"></param>需要发送的缓冲区地址
/// <param name="size"></param>需要发送的内容的字节数
/// <returns></returns>返回成功输出的字节数
int rt_device_write(rt_device_t serial, int start_postion, char* buffer, int size)
{
    //append_to_file("test.txt", buffer + start_postion, size);
    static int count = 0;
    count++;
    printf("写入%d次\n", count);
}
#elif defined __linux__
#include "drv_txt_operation.h"
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define rt_tick_get     clock       //rt_tick_get()函数是stm32嵌入式RTT操作系统调用的毫秒级时间读取函数，clock()是windows中time.h库中包含的毫秒级时间读取函数

///   该函数是用来发送数据到某个地方（为了调试，我写了一个文件操作的函数，使其写入txt文件中，模拟串口发送出去），如果是在linux系统中，建议改为自己的串口发送函数
///   在windows系统由于没有串口（目前不会），建议只修改写入的文件名称即可（记得需要引用我写的drv_txt_operation.h来对txt文件进行操作）
///   在linux系统中，需要重写为使用自己的串口驱动发送数据，但是建议还是使用该函数名称和原型，方便后续全平台兼容
///   在电控使用的stm32嵌入式平台上，如果使用rtt操作系统，无需定义该函数，在rtt中自带该函数
/// <param name="serial"></param>串口设备名称（设备句柄，用来查找和使用设备）
/// <param name="start_postion"></param>需要发送的内容相较于缓冲区地址的偏移字节数量
/// <param name="buffer"></param>需要发送的缓冲区地址
/// <param name="size"></param>需要发送的内容的字节数
/// <returns></returns>返回成功输出的字节数
int rt_device_write(rt_device_t serial, int start_postion, char* buffer, int size)
{

}

#elif defined __arm__   

#endif


// unsigned short REF_ROBO_ID;   // 该机器人ID
// unsigned short REF_CLIENT_ID; // 该客户端ID


//该文件中主要实现的功能是：底层的发送UI数据包给裁判系统
rt_device_t serial_UI;
char UI_tx_buff[150];			//这里设置150字节缓冲区用来放置最后实际发送的数据，发送至串口,大小建议至少在130以上，该数据包至少需要装得下7个图形元素

#define UI_OBJECT_NUM_MAX 150     //这里的设置UI对象的最大个数（包括图形个数和字符串个数总和），需要保证注册的元素个数不超过该值，如果超过了，需要适当改大一些

UI_Object object[UI_OBJECT_NUM_MAX];    //设置实际UI元素保存的位置
int UI_graph_index[UI_OBJECT_NUM_MAX];  //用于记录需要发送的UI图形元素的下标索引
int UI_string_index[UI_OBJECT_NUM_MAX]; //用于记录需要发送的UI字符串元素的下标索引

UI_Trans_Ctrl_t  UI_trans_ctrl_info[UI_OBJECT_NUM_MAX];

unsigned int UI_run_time = 0;



void Drv_Set_Operation_auto(UI_Object* object);

/// <summary>
///             该函数用来将图形元素打包发送至串口发送缓冲区中，注意参数remain_pocket_num是剩余发送图形元素个数，需要在外部修改其值
/// </summary>
/// <param name="pocket"></param>        需要打包的源图形元素
/// <param name="tx_buff"></param>       发送缓冲区
/// <param name="remain_pocket_num"></param>需要发送的剩余图形元素个数
/// <returns></returns>
void Drv_Add_Object_To_Buff(void *object, char* tx_buff,int remain_pocket_num)
{
    int trans_num=0;
    static unsigned char now_pocket_num = 0;//用于记录现在已经拼包的数量（最大为7个，最后一次发送时，可能不足7个，特殊处理）
    static uint16_t CommandID[8] = { 0x0000,0x0101,0x0102,0x0103,0x0103,0x0103, 0x0104, 0x0104 };//这里用这个数据来查找所需要的发送子命令ID，用所拼起来的包的数量查找
    static UI_Tx_Buffer_Header pocket_head = { .SOF = 0xA5,.DataLength = 0,.Seq = 1,.Crc8 = 0,.CommandId = 0x0301,.ContentId = 0,.ReceiverId = 0,.SenderId = 0 };
    if (object == NULL || tx_buff == NULL|| remain_pocket_num < 0)
    {
        return ;    //如果实体为空，返回表示跳过该元素（可能存在元素未注册就使用的情况）；如果发送缓冲区为空，返回表示跳过所有元素，即都不发送；如果发送元素小于0，返回表示参数有误
    }               //这样解释的根据是外部每调用一次该函数，就应该将剩余图形元素个数自减1，表示发送（写入缓冲区中），如果失败，就会跳过该元素
    if (remain_pocket_num > 0)//这里表示图形元素还没有完全放进来，一边将图形元素全部放入缓冲区，一边判断是否满7个，如果满7个，就直接打包发送
    {
        UI_Object* object_add = (UI_Object*)object;
        Drv_Set_Operation_auto(object_add);         //自动修改操作类型
        //需要先将包一个一个放在缓冲区中（预留包头的位置）
        memcpy(tx_buff + sizeof(UI_Tx_Buffer_Header) + now_pocket_num * sizeof(UI_Object), object_add, sizeof(UI_Object));
        now_pocket_num++;
        if (now_pocket_num >= 7)        //如果数据包中图形元素达到7个，那么它将会被发送
        {
            pocket_head.DataLength = 6 + now_pocket_num * sizeof(UI_Object);//设置帧头中数据长度,6是指ContentId，ReceiverId,SenderId长度之和
            Append_CRC8_Check_Sum((char*)&pocket_head, 5);                  //设置帧头中CRC8校验值
            pocket_head.ContentId = CommandID[now_pocket_num];              //设置发送子命令ID
            pocket_head.ReceiverId = REF_CLIENT_ID;                         //设置接收者ID
            pocket_head.SenderId = REF_ROBO_ID;                             //设置发送者ID
            memcpy(tx_buff, &pocket_head, sizeof(pocket_head));             //将帧头（和子命令部分）写入缓冲区
            Append_CRC16_Check_Sum(tx_buff, sizeof(UI_Tx_Buffer_Header) + now_pocket_num * sizeof(UI_Object) + 2);//设置CRC16校验  13+7*15+2(帧头5字节+2字节CMDID+6子命令ID中的内容+图形元素大小15字节*图形元素个数+帧尾2字节CRC16校验)
            trans_num = rt_device_write(serial_UI, 0, tx_buff, sizeof(UI_Tx_Buffer_Header) + now_pocket_num * sizeof(UI_Object) + 2);//将数据发送
            rt_thread_delay(UI_SEND_INTERVAL_TIME);
            now_pocket_num = 0;                                             //每次发送结束后，重新设置缓冲区保存图形元素的个数
        }
    }
    else if(remain_pocket_num == 0 )//这里还剩0个图形元素个数，表示本次循环结束，需要将最后的图形元素发送出去，这里对刚好不满7个的特殊处理
    {
        if (now_pocket_num == 0)
        {
            return;                 //如果刚好本次循环结束，所有需要在本次循环中发送图形元素个数刚好是7的倍数，则会发生该情况，否则会被按照不满7个图形元素的情况处理
        }
        else
        {
            pocket_head.DataLength = 6 + now_pocket_num * sizeof(UI_Object);//设置帧头中数据长度,6是指ContentId，ReceiverId,SenderId长度之和
            Append_CRC8_Check_Sum((char*)&pocket_head, 5);                  //设置帧头中CRC8校验值
            pocket_head.ContentId = CommandID[now_pocket_num];              //设置发送子命令ID
            pocket_head.ReceiverId = REF_CLIENT_ID;                         //设置接收者ID
            pocket_head.SenderId = REF_ROBO_ID;                             //设置发送者ID
            memcpy(tx_buff, &pocket_head, sizeof(pocket_head));             //将帧头（和子命令部分）写入缓冲区
            Append_CRC16_Check_Sum(tx_buff, sizeof(UI_Tx_Buffer_Header) + now_pocket_num * sizeof(UI_Object) + 2);//设置CRC16校验  13+7*15+2(帧头5字节+2字节CMDID+6子命令ID中的内容+图形元素大小15字节*图形元素个数+帧尾2字节CRC16校验)
            trans_num = rt_device_write(serial_UI, 0, tx_buff, sizeof(UI_Tx_Buffer_Header) + now_pocket_num * sizeof(UI_Object) + 2);//将数据发送
            rt_thread_delay(UI_SEND_INTERVAL_TIME); 
            now_pocket_num = 0;                                             //每次发送结束后，重新设置缓冲区保存图形元素的个数
        }
    }
}

void Drv_Add_String_To_Buff(void* object, char* tx_buff)
{
    int trans_num=0;
    static UI_Tx_Buffer_Header pocket_head = { .SOF = 0xA5,.DataLength = 0,.Seq = 1,.Crc8 = 0,.CommandId = 0x0301,.ContentId = 0,.ReceiverId = 0,.SenderId = 0 };
    unsigned char string_length = 0;
    if (object == NULL || tx_buff == NULL )
    {
        return ;    //如果实体为空，返回表示跳过该元素（可能存在元素未注册就使用的情况）；如果发送缓冲区为空，返回表示跳过所有元素，即都不发送
    }               //这样解释的根据是外部每调用一次该函数，就应该将剩余字符元素个数自减1，表示发送（写入缓冲区中），如果失败，就会跳过该元素
    UI_Object* object_add = (UI_Object*)object;
    Drv_Set_Operation_auto(object_add);         //自动修改操作类型
    if (object_add->detailDword3.strVal == NULL)
    {
        return;     //如果字符串元素指向的字符串地址为空，则没有正确初始化注册或者字符串地址异常，则返回，跳过该元素
    }
    pocket_head.DataLength = 6 + sizeof(UI_Object) + 30;            //设置帧头中数据长度,6是指ContentId，ReceiverId,SenderId长度之和,30指字符串的固定发送长度（建议查看裁判系统串口协议附录）
    Append_CRC8_Check_Sum((char*)&pocket_head, 5);                  //设置帧头中CRC8校验值
    pocket_head.ContentId = 0x0110;                                 //设置发送子命令ID，字符串为0x0110
    pocket_head.ReceiverId = REF_CLIENT_ID;                         //设置接收者ID
    pocket_head.SenderId = REF_ROBO_ID;                             //设置发送者ID
    memcpy(tx_buff, &pocket_head, sizeof(pocket_head));             //将帧头（和子命令部分）写入缓冲区13字节
    memcpy(tx_buff+sizeof(pocket_head), object_add, sizeof(UI_Object));                 //将元素内容写入缓冲区
    memset(tx_buff + sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Object), 0, 30);//设置实际字符串放置位置缓冲区的内容为0，保证发送的内容不是乱码
    string_length = object_add->detailDword1.detailB >= 30 ? 30 : object_add->detailDword1.detailB; //此处判断发送字符串长度是否超过最大字符串长度（固定长度为30字节）
    strncpy(tx_buff + sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Object), object_add->detailDword3.strVal, string_length);//将字符串内容复制到缓冲区中
    Append_CRC16_Check_Sum(tx_buff, sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Object) + 30 + 2);//设置CRC16校验  13+30+2(帧头5字节+2字节CMDID+6子命令ID中的内容+字符串配置大小15字节+字符串固定发送长度+帧尾2字节CRC16校验)
    trans_num = rt_device_write(serial_UI, 0, tx_buff, sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Object)+ 30 + 2);//将数据发送
    rt_thread_delay(UI_SEND_INTERVAL_TIME);
}

/// <summary>
///             该函数用来底层实现打包和发送数据包
/// </summary>
/// <param name="graph_num"></param>    本次循环需要发送图形元素个数
/// <param name="string_num"></param>   本次循环需要发送字符元素个数
void Drv_Send_UI_Object(int graph_num ,int string_num)
{
    for (int i = graph_num,j=0;i >= 0;i--,j++)
    {
        Drv_Add_Object_To_Buff(&object[UI_graph_index[j]], UI_tx_buff, i);
    }
    for (int i = string_num,j=0;i>0;i--,j++)
    {
        Drv_Add_String_To_Buff(&object[UI_string_index[j]],UI_tx_buff);
    }
}

unsigned char Attach_UI_Object(UI_Object_Type_e object_type, UI_Object_Color_e color, UI_TransType_t TransType, unsigned int send_period, CallBack_fun_p fun)
{
    static int UI_object_index = 0;//这里用来记录分配给每个元素的裁判系统索引

    if (UiStr < object_type)    //如果注册元素类型超过字符串，则返回0，表示注册失败
    {
        return 0;
    }
    //首先设置元素相关内容
    object[UI_object_index].detailDword1.type = object_type;                        //设置注册元素类型
    object[UI_object_index].refereeHandle[0] = (UI_object_index+1)&0xFF;                //设置元素裁判系统索引
    object[UI_object_index].refereeHandle[1] = ((UI_object_index+1) & 0xFF00) >> 8;     //设置元素裁判系统索引
    object[UI_object_index].refereeHandle[2] = ((UI_object_index+1) & 0xFF0000) >> 16;  //设置元素裁判系统索引
    object[UI_object_index].detailDword1.layer = Ui_layer_0;                        //设置元素所在图层（这里先固定写入0号图层）
    object[UI_object_index].detailDword1.color = color;                             //设置元素颜色
    //其次设置UI刷新控制器
    UI_trans_ctrl_info[UI_object_index].TransType = TransType;                      //设置元素发送类型（永久发送和有限次发送）
    UI_trans_ctrl_info[UI_object_index].send_period = send_period;                  //设置发送频率为多少次发送一次
    UI_trans_ctrl_info[UI_object_index].call_back_fun = fun;                        //设置发送回调函数，在检索每次循环需要发送的元素时，如果该元素在本次循环需要发送，
                                                                                    //将元素索引存放在UI_graph_index和UI_string_index数组中，然后调用回调函数来更新其值
    UI_object_index++;
    return 1;
}

/// <summary>
///             该函数用来将原本注册过的函数元素将其修改发送信息（发送次数和发送频率），前提是在注册阶段注册过回调函数（否则无法找到对应元素）
/// </summary>
/// <param name="TransType"></param>设置发送类型
/// <param name="send_period"></param>设置发送频率
/// <param name="fun"></param>查找的对应元素的回调函数指针
/// <returns></returns>如果找到对应元素并修改了发送信息，则返回1表示设置成功，返回0，表示未找到，设置失败
unsigned char Modify_UI_Object_info(UI_TransType_t TransType, unsigned int send_period, CallBack_fun_p fun)
{
    unsigned char ret=0;
    //首先通过回调函数来查找到UI元素对象
    for (int i = 0;i < UI_OBJECT_NUM_MAX;i++)
    {
        if (UI_trans_ctrl_info[i].call_back_fun == NULL && UI_trans_ctrl_info[i].send_period==0 && UI_trans_ctrl_info[i].TransType==0)
        {
            break;                       //如果没有找到回调函数且所有发送相关类型信息都没有设置过的话，那说明后续没有注册的元素信息了，直接退出
        }
        else if (fun == UI_trans_ctrl_info[i].call_back_fun)    //如果找到对应的回调函数，则说明可以找到对应的元素，设置其发送信息,退出循环
        {
            UI_trans_ctrl_info[i].TransType = TransType;
            UI_trans_ctrl_info[i].send_period = send_period;
            ret = 1;
            break;
        }
        else                        //如果没有找到，则继续向后遍历
        {
            continue;
        }
    }
    return ret;
}


/// <summary>
///                 该函数用来查找并发送需要发送的UI元素（包括字符串和图形元素），调用一次就遍历一次所有需要发送的元素
/// </summary>
void Drv_Find_Send_UI_Object()
{
    int index_object = 0;           //UI元素下标索引
    int index_graph = 0;            //图形元素下标索引(同时也记录该次需要发送的图形元素个数)
    int index_string = 0;           //字符串元素下标索引（同时也记录该次需要发送的字符串元素个数）
    //int graph_num = 0, string_num = 0;//图形元素个数和字符串元素个数
    while(object[index_object].refereeHandle[0]!=0 || object[index_object].refereeHandle[1] != 0 || object[index_object].refereeHandle[2] != 0 )//有一个不为0则说明该元素已经被注册过
    {
        if (UI_trans_ctrl_info[index_object].TransType == TRANS_FOREVER)//如果设置该元素为永久发送
        {
            if (UI_trans_ctrl_info[index_object].send_period == 0 || (UI_trans_ctrl_info[index_object].send_period > 0 && (UI_run_time % UI_trans_ctrl_info[index_object].send_period) == 0))
            {       //如果发送周期为0或者发送周期大于0且运行次数刚好能整除运行周期数,说明本次周期需要发送该元素
                if (object[index_object].detailDword1.type != UiStr)  //如果UI类型为非字符串类型，那么就设置其UI元素下标到图形元素下标索引数组中
                {
                    UI_graph_index[index_graph] = index_object;//记录该次需要发送的元素下标索引
                    if (UI_trans_ctrl_info[index_object].call_back_fun != NULL)
                    {
                        UI_trans_ctrl_info[index_object].call_back_fun(&object[index_object]);//如果设置的更新回调函数不为空，则调用更新函数将其数据相关内容更新
                    }
                    index_graph++;                              //更新图形元素数组下标索引
                }
                else
                {
                    UI_string_index[index_string] = index_object;//记录该次需要发送的元素下标索引
                    if (UI_trans_ctrl_info[index_object].call_back_fun != NULL)
                    {
                        UI_trans_ctrl_info[index_object].call_back_fun(&object[index_object]);//如果设置的更新回调函数不为空，则调用更新函数将其数据相关内容更新
                    }
                    index_string++;                              //更新字符串元素数组下标索引
                }
            }
            
        }
        else if (UI_trans_ctrl_info[index_object].TransType > 0)//如果设置该元素为有限次发送
        {
            if (UI_trans_ctrl_info[index_object].send_period == 0 || (UI_trans_ctrl_info[index_object].send_period > 0 && (UI_run_time % UI_trans_ctrl_info[index_object].send_period) == 0))
            {       //如果发送周期为0或者发送周期大于0且运行次数刚好能整除运行周期数,说明本次周期需要发送该元素
                if (object[index_object].detailDword1.type != UiStr)  //如果UI类型为非字符串类型，那么就设置其UI元素下标到图形元素下标索引数组中
                {
                    UI_graph_index[index_graph] = index_object;//记录该次需要发送的元素下标索引
                    if (UI_trans_ctrl_info[index_object].call_back_fun != NULL)
                    {
                        UI_trans_ctrl_info[index_object].call_back_fun(&object[index_object]);//如果设置的更新回调函数不为空，则调用更新函数将其数据相关内容更新
                    }
                    index_graph++;                              //更新图形元素数组下标索引
                }
                else
                {
                    UI_string_index[index_string] = index_object;//记录该次需要发送的元素下标索引
                    if (UI_trans_ctrl_info[index_object].call_back_fun != NULL)
                    {
                        UI_trans_ctrl_info[index_object].call_back_fun(&object[index_object]);//如果设置的更新回调函数不为空，则调用更新函数将其数据相关内容更新
                    }
                    index_string++;                              //更新图形元素数组下标索引
                }
            }
            UI_trans_ctrl_info[index_object].TransType--;       //将发送值减1
        }
        index_object++;//更新UI元素下标索引
    
    }
    Drv_Send_UI_Object(index_graph, index_string);
    UI_run_time++;
}

void Drv_Clear_Layer(UI_Layer_Delete_e delete_type, UI_Layer_e layer, char* tx_buff)
{
    int trans_num=0;
    static UI_Tx_Buffer_Header pocket_head = { .SOF = 0xA5,.DataLength = 0,.Seq = 1,.Crc8 = 0,.CommandId = 0x0301,.ContentId = 0,.ReceiverId = 0,.SenderId = 0 };
    if (delete_type > UI_Layer_Delete_ALL || layer > Ui_layer_9)
    {
        return;    //如果删除操作非法，所删除的图层非法，返回表示删除失败
    }               
    UI_Layer_Delete delete_info = { .delete_type = delete_type,.layer = layer };

    pocket_head.DataLength = 6 + sizeof(UI_Layer_Delete);           //设置帧头中数据长度,6是指ContentId，ReceiverId,SenderId长度之和+删除操作数据长度（建议查看裁判系统串口协议附录）
    Append_CRC8_Check_Sum((char*)&pocket_head, 5);                  //设置帧头中CRC8校验值
    pocket_head.ContentId = 0x0100;                                 //设置发送子命令ID，删除图层为0x0100
    pocket_head.ReceiverId = REF_CLIENT_ID;                         //设置接收者ID
    pocket_head.SenderId = REF_ROBO_ID;                             //设置发送者ID
    memcpy(tx_buff, &pocket_head, sizeof(pocket_head));             //将帧头（和子命令部分）写入缓冲区13字节
    memcpy(tx_buff+sizeof(pocket_head), &delete_info, sizeof(delete_info));             //将删除数据写入缓冲区2字节
    Append_CRC16_Check_Sum(tx_buff, sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Layer_Delete) + 2);//设置CRC16校验  13+30+2(帧头5字节+2字节CMDID+6子命令ID中的内容+字符串配置大小15字节+字符串固定发送长度+帧尾2字节CRC16校验)
    trans_num = rt_device_write(serial_UI, 0, tx_buff, sizeof(UI_Tx_Buffer_Header) + sizeof(UI_Layer_Delete) + 2);//将数据发送
    rt_thread_delay(UI_SEND_INTERVAL_TIME);
}

/// <summary>
///             该函数用来UI复位时将图层删除
/// </summary>
/// <param name="delete_type"></param>删除图层操作类型
/// <param name="layer"></param>        删除的图层
void Clear_Layer(UI_Layer_Delete_e delete_type, UI_Layer_e layer)
{
    Drv_Clear_Layer(delete_type, layer, UI_tx_buff);
}

/// <summary>
///             该函数用来设置运行计数器的值，在UI复位中，可以设置该值为0，会在下一次运行时，将会发送所有该发送的元素
/// </summary>
/// <param name="set_value"></param>
void Drv_Set_run_time(unsigned int set_value)
{
    UI_run_time = set_value;
}


/// <summary>
///             该函数用来在底层发送前设置其操作类型，注意是在发送前调用并修改的
/// </summary>
/// <param name="object"></param>UI元素对象指针
void Drv_Set_Operation_auto(UI_Object* object)
{
    if (object == NULL)                                     //元素对象为空，直接返回
    {
        return;
    }
    if (object->detailDword1.operation == UI_NULL)          //如果刚初始化，或者被删除，手动设置为UI_NULL，在发送时将自动设置成创建操作
    {
        object->detailDword1.operation = UI_CREATE;
    }
    else if (object->detailDword1.operation == UI_CREATE)   //如果上次操作是创建模式，则在发送时将自动设置为修改模式
    {
        object->detailDword1.operation = UI_MODIFY;
    }

}

/**
 * @brief 该函数用来获取某个UI元素是否已经被创建（主要是用于那些触发显示的UI元素，解决在复位情况下异常显示的问题）
 *         在刷新UI数据时，调用该函数可以检查是否某些本不应该存在的元素仍然显示在屏幕上
 * @param fun 
 * @return unsigned char 返回0表示该元素已经不存在了，返回1表示该元素存在
 */
unsigned char Drv_Get_Object_Exist(CallBack_fun_p fun)
{
    unsigned char ret = 0;
    if (fun == NULL)
    {
        return ret;
    }
    //首先通过回调函数来查找到UI元素对象
    for (int i = 0;i < UI_OBJECT_NUM_MAX;i++)
    {
        if (UI_trans_ctrl_info[i].call_back_fun == NULL && UI_trans_ctrl_info[i].send_period == 0 && UI_trans_ctrl_info[i].TransType == 0)
        {
            break;                       //如果没有找到回调函数且所有发送相关类型信息都没有设置过的话，那说明后续没有注册的元素信息了，直接退出
        }
        else if (fun == UI_trans_ctrl_info[i].call_back_fun)    //如果找到对应的回调函数，则说明可以找到对应的元素，设置其发送信息,退出循环
        {
            if (object[i].detailDword1.operation == UI_CREATE || object[i].detailDword1.operation == UI_MODIFY)  //在发送时的逻辑为，如果设置操作类型为空，那么在底层发送前会自动设置为创建，所以此处通过判断是否为创建来判断该元素是否存在
            {
                ret=1; //如果操作类型为创建，则表示该元素存在
            }
            else if (object[i].detailDword1.operation == UI_DELDTE || object[i].detailDword1.operation == UI_NULL)                    //如果设置为创建，则设置操作类型为空，在底层发送前会自动设置为创建
            {
                ret = 0;    //如果操作类型为删除或者空，则表示该元素不存在
            }
            break;
        }
        else                        //如果没有找到，则继续向后遍历
        {
            continue;
        }
    }
    return ret;
}

/// <summary>
///                 该函数用来设置某个具体的UI元素的“可见性”，设置不可见表示下次运行将该元素删除，设置为可见表示下次运行将该元素显示（创建）
/// </summary>      该函数主要是用来设置像菜单那种需要在某些时候显示，在某些时候消失的UI元素
/// <param name="operation"></param>    操作类型，删除或者创建    删除：即不可见      创建：即可见
/// <param name="fun"></param>          函数指针：注册UI元素时设置的回调函数，用来寻找需要修改的对应UI元素
/// <returns></returns>                 返回0失败，返回1成功
unsigned char Drv_Set_Object_Visualable(UI_Operation_e operation, CallBack_fun_p fun)
{
    unsigned char ret = 0;
    if (operation > UI_DELDTE || fun == NULL)
    {
        return ret;
    }
    //首先通过回调函数来查找到UI元素对象
    for (int i = 0;i < UI_OBJECT_NUM_MAX;i++)
    {
        if (UI_trans_ctrl_info[i].call_back_fun == NULL && UI_trans_ctrl_info[i].send_period == 0 && UI_trans_ctrl_info[i].TransType == 0)
        {
            break;                       //如果没有找到回调函数且所有发送相关类型信息都没有设置过的话，那说明后续没有注册的元素信息了，直接退出
        }
        else if (fun == UI_trans_ctrl_info[i].call_back_fun)    //如果找到对应的回调函数，则说明可以找到对应的元素，设置其发送信息,退出循环
        {
            if (operation == UI_DELDTE)                         //如果设置为删除，则设置操作类型为删除
            {
                object[i].detailDword1.operation = UI_DELDTE;
                Modify_UI_Object_info(TRANS_ONCE, UI_trans_ctrl_info[i].send_period, fun);
            }
            else if (operation == UI_CREATE)                    //如果设置为创建，则设置操作类型为空，在底层发送前会自动设置为创建
            {
                object[i].detailDword1.operation = UI_NULL;
                Modify_UI_Object_info(TRANS_ONCE, UI_trans_ctrl_info[i].send_period, fun);
            }
            else if (operation == UI_MODIFY)                    //如果设置为修改，则设置操作类型为修改
            {
                object[i].detailDword1.operation = UI_MODIFY;
                Modify_UI_Object_info(TRANS_ONCE, UI_trans_ctrl_info[i].send_period, fun);
            }
            ret = 1;
            break;
        }
        else                        //如果没有找到，则继续向后遍历
        {
            continue;
        }
    }
    return ret;
}

/// <summary>
///                 该函数用来在UI复位，删除全部图层后调用，将UI元素的操作类型设置为初始化时的空操作，在实际下次发送时会在底层自动修改为创建操作
/// </summary>      并且设置有限次发送的发送次数为1，保证能够刷新后显示出来
/// <returns></returns> 返回成功设置操作类型的UI元素个数
unsigned char Drv_Set_Visualable_All()
{
    unsigned char ret = 0;
    //首先通过回调函数来查找到UI元素对象
    for (int i = 0;i < UI_OBJECT_NUM_MAX;i++)
    {
        
        if (object[i].refereeHandle[0] == 0 && object[i].refereeHandle[1] == 0 && object[i].refereeHandle[2] == 0)
        {
            break;                       //如果裁判系统索引为0，那说明后续没有注册的元素信息了，直接退出
        }
        else    //如果裁判系统索引不为0，则说明该元素存在，设置其操作类型为空操作，在底层发送前会自行判断如果是空操作，就会设置成创建操作
        {
            object[i].detailDword1.operation = UI_NULL;
            if(UI_trans_ctrl_info[i].TransType!=TRANS_FOREVER && UI_trans_ctrl_info[i].send_period > 0) //如果该元素不是永久发送的，并且发送周期大于0，则设置其发送周期为1次（这种元素是固定发送几次的，显示在屏幕上不再变化的）
            {                                                                                           //如果是触发式发送的元素，需要在注册阶段将发送频率设置为0，表示其为触发式发送元素，区别于单次发送的元素
                Modify_UI_Object_info(TRANS_ONCE, UI_trans_ctrl_info[i].send_period, UI_trans_ctrl_info[i].call_back_fun);
            }
            ret++;
        }
    }
    return ret;
}


