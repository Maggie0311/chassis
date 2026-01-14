#ifndef __SUPERCAP_COM_H__
#define __SUPERCAP_COM_H__
#include <board.h>

#define SC_PERIOD 10 // 超级电容线程轮询周期，单位ms

typedef struct
{
    uint8_t powerRefFlag; /*接受到标志位*/
    uint8_t Whether_To_Charge;
    uint8_t maxPower;
    uint16_t chassis_powerBuf;
    uint16_t chassis_powerLim;
    uint32_t heatDataTick;

} scpr_ctrlCmd_t;

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
extern rt_bool_t Read_Super_Capacity_Online(void);

/***
 * @brief    获得超级电容剩余电量
 * @param    None
 * @return   剩余电容量,0~100。100为满电
 ***/
float Get_RemainCapcity(void);

/***
 * @brief    获得超级电容电压
 * @param    None
 * @return   电压大小(V),当通讯超时时返回0。
 ***/
float Get_Cap_Voltage(void);

#endif
