#ifndef __SUPERCAP_COM_H__
#define __SUPERCAP_COM_H__
#include <board.h>

#include "can_com_thread.h"

#define SC_PERIOD 10 // 超级电容线程轮询周期，单位ms
#define VCAP_DATA_NUM 10

/*超电通信ID*/
#define SCPR_RX 0x096 // 从超级电容端接收
#define SCPR_TX 0x095 // 向超级电容端发送

/**
 * @brief 超电通信接收数据结构体
 * 
 */
typedef struct
{
    float cap;        // 剩余电容量,0.0~1.0。1.0为满电
    uint16_t V_cap;   // 超级电容电压  mV
    uint16_t voltage; // 电池电压单位mV
    uint16_t current; // 电池总电流mA
    float power_cup;    //接收数据后计算得出的功率W（使用电池电压乘以电池电流大小，注意单位W）
    uint8_t recvAck;
    volatile uint8_t isStartSend;
} Scpr_ComRx_t;

/**
 * @brief 超电发送数据结构体
 * 
 */
typedef struct
{
    uint8_t Whether_To_Output; /*是否工作，优先级最高*/
    uint8_t isdataRef;         /*数据刷新置1，收到超级电容应对置0 每次数据更新置1*/
    uint16_t isFlySlope;       /*飞坡标志*/
    uint8_t isOverPower;       /*是否超功率扣血，优先级第二*/
    uint16_t chassis_powerBuf; /*功率缓冲*/
    uint16_t chassis_powerLim; /*功率上限*/
} Scpr_ComTx_t;

/**
 * @brief 超电对象结构体
 * 
 */
typedef struct {
    CANInstance SC_com_instance;
    Scpr_ComRx_t scpr_com_rx;
    Scpr_ComTx_t scpr_com_tx;
}Scpr_Object;

/**
 * @brief 超电状态
 * 
 */
typedef enum
{
    CAP_STATE_ERROR = 0, // 超电状态无法获取
    CAP_STATE_CHARGE,    // 充电
    CAP_STATE_DISCHARGE, // 放电
    CAP_STATE_NO_WORK    // 等待充电/不在工作/进入保护(详细保护类型可参考超电代码)
} SupCap_State_e;

//获取上次接收通信时间
rt_tick_t Get_SuprCap_recTick(void);
/**
 * @brief    初始化与超级电容端通信
 * @return   true:初始化成功	false:初始化失败
 */
rt_err_t Scpr_Com_Init(void);
/**
 * @brief 获取超级电容控制板当前是否在线
 * @author fwlh
 * @return rt_bool_t        超级电容控制板在线返回真
 */
rt_bool_t Read_Super_Capacity_Online(void);
// 获得超电电压V
rt_err_t Get_SuprCapVoltage(float *V_cap);
// 获得超级电容充放电状态
SupCap_State_e Get_SuprCapState(void);
// 获得超级电容测得底盘总电流ma，电压mv，还有超级电容电压V
rt_bool_t getSuprCapMearData(float *volatge, float *current, float *V_Cap);
/***
 * @brief    获得超级电容剩余电量
 * @param    None
 * @return   剩余电容量,0~100。100为满电
 ***/
rt_bool_t Get_RemainCapcity(float *cap);
//获取超级电容测量底盘功率
rt_bool_t Get_SuprCapPower(float *power);
//获取超电是否输出
uint8_t Get_if_supercap_output(void);

/**
 * @brief 设置超电输出模式（由 fun_powerCtrl 调用，替代 extern power_ctrl_close）
 * @param mode  0=只充不放(关闭放电)  1=正常充放  2=只放不充
 */
void Set_SuprCap_OutputMode(uint8_t mode);

extern Scpr_ComRx_t scpr_comRx;
#endif
