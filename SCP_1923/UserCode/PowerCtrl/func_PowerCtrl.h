#ifndef __FUNC_POWERCTRL_H__
#define __FUNC_POWERCTRL_H__

#include "main.h"

#include "pid.h"

// 最高功率超限
#define POWER_OUT_LIMIT 160

// 最高缓冲能量（飞坡）
#define BUFFER_ENERGY_FLY_MAX 250.0f
// 最高缓冲能量（不飞坡）
#define BUFFER_ENERGY_MAX 60.0f
// 最高功率设定值
#define POWER_SET_MAX 200.0f
// 最低保守功率设定值
#define POWER_SET_LOW 44.0f
// 当与裁判系统断联后，将功率上限定为原来的固定比例
#define POWER_K_SET 0.85f
// 裁判系统断联超时时间(s)
#define OVER_TIME 0.8f

#define BUFFER_ENERGY_SET 45.0f // 闭环到的缓冲能量值
#define BUFFER_ENERGY_TOLERATE 10

typedef struct
{
    Time_Struct_t Receive_time; // 接收到有效数据的时间
    float Power_Limit;          // 通过通信得到的裁判系统功率的设定值
    float Buffer_Energy;        // 通过通信得到的当前的缓冲能量剩余值
    float Local_Buffer_Energy;  // 本地积分计算得到的缓冲能量数值
    uint8_t Work_state;         // 是否充放电
    uint8_t ACK_Flag;           // 应答标志位
    pid_t Buffer_Energy_PID;    // 缓冲能量环 pid 的句柄
} Energy_Control_Structure_s;   // 实际用于计算的数据结构体

/**
 * @brief PowerOutage事件检测轮询函数（10hz）
 * @author dty
 */
void PowerOutage_Routine(void);

/**
 * @brief 功率控制过程的初始化
 * @author fwlh
 */
void Power_Control_Init(void);
/**
 * @brief 功率控制循环函数
 *
 */
void Power_Ctrl_Routine(void);

/**
 * @brief 电量计算及显示
 * @author dty
 */
void Battery_Cal(void);

#endif /* __FUNC_POWERCTRL_H__ */
