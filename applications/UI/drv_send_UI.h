#ifndef __DRV_SEND_UI_H__
#define __DRV_SEND_UI_H__

//定义UI刷新频率和时间
#define UI_REFRESH_RATE (25) // UI刷新频率(裁判系统的刷新频率上限为30HZ，超过该刷新率会导致UI丢失卡顿的情况)
#define UI_SEND_INTERVAL_TIME (float)(1000.f / UI_REFRESH_RATE) // UI发送间隔时间(单位毫秒)



// 定义通用数据类型
#ifdef _WIN32
#define rt_uint8_t unsigned char
#define rt_uint16_t unsigned short
#define rt_uint32_t unsigned int 
#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned int 
#elif defined __arm__
#include <rtthread.h>
#endif 


//定义串口发送设备和底层驱动接口
#if defined   __arm__
#include <rtthread.h>
#include <rtdevice.h>
#elif defined _WIN32
#define rt_device_t   void*          //  该函数是stm32 RTT操作系统特有的结构体，主要是用来传递设备（串口）指针，来定向到输出和输入的数据流，
                                    //  建议在看懂数据输出位置后，重新设置该类型和其调用的函数

#elif defined __linux__
#define rt_device_t   void*          //  该函数是stm32 RTT操作系统特有的结构体，主要是用来传递设备（串口）指针，来定向到输出和输入的数据流
                                    //  建议在看懂数据输出位置后，重新设置该类型和其调用的函数
#endif 


// 统一支持所有主流编译器  设置结构体字节对齐类型为1字节对齐
#if defined(__GNUC__) || defined(__clang__)
    // GCC/Clang 编译器
#define HEAD_PACK 
#define TAIL_PACK __attribute__((packed))
#elif defined(__ARMCC_VERSION) || defined(__CC_ARM)
    // ARM Compiler 5/6
#define HEAD_PACK __packed
#define TAIL_PACK 
#pragma anon_unions
#elif defined(_MSC_VER)
    // MSVC 编译器
#define HEAD_PACK __pragma(pack(push, 1))
#define TAIL_PACK __pragma(pack(pop))
#else
    // 其他编译器
#warning "Unsupported compiler - struct packing not guaranteed"
#define HEAD_PACK
#define TAIL_PACK
#endif


typedef HEAD_PACK struct UI_Tx_Buffer_Header
{
    // 包头
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t Crc8;
    // 命令ID
    uint16_t CommandId;
    // 数据段包结构的头
    uint16_t ContentId;
    uint16_t SenderId;
    uint16_t ReceiverId;
} TAIL_PACK UI_Tx_Buffer_Header;

typedef HEAD_PACK struct UI_Object
{
    // 裁判系统三字节索引
    uint8_t refereeHandle[3];

    // 下面4字节把官方的3比特操作类型改成了3个脏比特
    HEAD_PACK union _DetailDword1Internals {
        uint32_t dw;
        HEAD_PACK struct
        {
            uint32_t operation : 3; ///< [内存中未使用] 图形操作
            uint32_t type : 3;      ///< [UiObjectType] 图形类型;
            uint32_t layer : 4;     ///< 图层；0～9为合法；非法图层会被强制设为0
            uint32_t color : 4;   ///< [UiObjectColor] 颜色；非法颜色会被强制设为UiTeam
            uint32_t detailA : 9; ///< （圆弧）起始角度/（文本）字体大小
            uint32_t detailB : 9; ///< （圆弧）终止角度/（字符串）长度
        } TAIL_PACK;
    } detailDword1 TAIL_PACK;

    // 下面4字节与官方一致
    HEAD_PACK union _DetailDword2Internals {
        uint32_t dw;
        HEAD_PACK struct
        {
            uint32_t width : 10; // 宽度
            uint32_t x : 11;     // 第一个x坐标
            uint32_t y : 11;     // 第一个y坐标
        } TAIL_PACK;
    } detailDword2 TAIL_PACK;

    // 下面4字节与官方一致
    HEAD_PACK union _DetailDword3Internals {
        uint32_t dw;
        HEAD_PACK struct
        {
            uint32_t radius : 10; // 正圆半径
            uint32_t reserved : 22;
        } circle TAIL_PACK; // 正圆使用的DetailDword3
        HEAD_PACK struct
        {
            uint32_t reserved : 10;
            uint32_t x2 : 11; // 第二个x坐标
            uint32_t y2 : 11; // 第二个y坐标
        } line TAIL_PACK;     // 直线和矩形使用的DetailDword3
        HEAD_PACK struct
        {
            uint32_t reserved : 10;
            uint32_t xSemiaxis : 11; // x半轴长度
            uint32_t ySemiaxis : 11; // y半轴长度
        } ellipse TAIL_PACK;
        uint32_t intVal;    ///< 整型数
        uint32_t floatVal;  ///< 浮点数
        char* strVal; ///< 字符串指针
    } detailDword3 TAIL_PACK;
} TAIL_PACK UI_Object;

typedef HEAD_PACK struct
{
    uint8_t delete_type;
    uint8_t layer;
}TAIL_PACK UI_Layer_Delete;
//ui对象类型枚举
typedef enum UiObjectType {
    UiLine,        // 直线
    UiRect,        // 矩形
    UiCircle,      // 圆形
    UiEllipse,     // 椭圆
    UiArc,         // 弧形
    UiFloat,       // 浮点数
    UiInt,         // 整数
    UiStr,         // 字符串
} UI_Object_Type_e;

//ui图层枚举
typedef enum UiLayer {
    Ui_layer_0,
    Ui_layer_1,
    Ui_layer_2,
    Ui_layer_3,
    Ui_layer_4,
    Ui_layer_5,
    Ui_layer_6,
    Ui_layer_7,
    Ui_layer_8,
    Ui_layer_9,
} UI_Layer_e;
//ui颜色枚举
typedef enum UiObjectColor {
    UiTeam,     // 团队颜色
    UiYellow,   // 黄色
    UiGreen,    // 绿色
    UiOrange,   // 橙色
    UiMagenta,  // 紫色
    UiPink,     // 粉色
    UiCyan,     // 青色
    UiBlack,    // 黑色
    UiWhite,    // 白色
} UI_Object_Color_e;

typedef enum {
    UI_Layer_NULL,
    UI_Layer_Delete_One,
    UI_Layer_Delete_ALL
}UI_Layer_Delete_e;

typedef enum {
    UI_NULL,
    UI_CREATE,
    UI_MODIFY,
    UI_DELDTE
}UI_Operation_e;

typedef void (*CallBack_fun_p)(UI_Object*);//运行UI前的回调函数指针类型，用来修改图形或者字符串相关内容（设置颜色，图层等等）

typedef enum {
    TRANS_FOREVER = -1, // 永久发送
    TRANS_NULL = 0,     //初始注册阶段不进行发送（常用于触发式发送）
    TRANS_ONCE = 1,     // 单次发送，可以发送多次，发送的次数强制类型转换即可
    // 这里写为枚举是为了清楚显示永久发送写-1即可（用枚举表示） 
} UI_TransType_t;


typedef struct {

    UI_TransType_t TransType; // 发送类型
    unsigned int send_period; // 发送周期(每轮询几次发送一次)
    CallBack_fun_p call_back_fun;   //发送回调函数（在回调函数中修改图形或者字符串相关内容）
}UI_Trans_Ctrl_t;


void Drv_Send_UI_Object(int graph_num, int string_num);
unsigned char Attach_UI_Object(UI_Object_Type_e object_type, UI_Object_Color_e color, UI_TransType_t TransType, unsigned int send_period, CallBack_fun_p fun);
void Drv_Find_Send_UI_Object();
void Drv_Set_run_time(unsigned int set_value);
void Clear_Layer(UI_Layer_Delete_e delete_type, UI_Layer_e layer);
unsigned char Drv_Get_Object_Exist(CallBack_fun_p fun);
unsigned char Drv_Set_Object_Visualable(UI_Operation_e operation, CallBack_fun_p fun);
unsigned char Drv_Set_Visualable_All();


/*******************************外部数据************************************/
extern rt_device_t serial_UI;
extern rt_uint16_t REF_ROBO_ID;   // 该机器人ID
extern rt_uint16_t REF_CLIENT_ID; // 该客户端ID

#endif
