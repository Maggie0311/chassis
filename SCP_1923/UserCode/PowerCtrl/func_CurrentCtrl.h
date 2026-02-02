#ifndef __FUNC_CURRENTCTRL_H__
#define __FUNC_CURRENTCTRL_H__

#include "main.h"

// 允许设定的最大充电电流
#define MAX_CHARGE_CURRENT_PERMITTED 8.f
// 允许设定的最大放电电流
#define MAX_DISCHARGE_CURRENT_PERMITTED -8.f
// 最高电容电压
// #define CTRLSET_UCAP_MAX 27.5f//(50/11)
#define CTRLSET_UCAP_MAX 29.f // 新的29V电容组
// #define CTRLSET_UCAP_MAX 23.4f//9个
// #define CTRLSET_UCAP_MAX 24.f
// 最低电容电压
#define CTRLSET_UCAP_MIN 10.0f
// 滞回量（按照能量计算需要的滞回量）
#define HYSTERESIS_PARAMENT_OV 1.0f
#define HYSTERESIS_PARAMENT_UV 3.0f
#define HYSTERESIS_PARAMENT_UMIN 10.0f
#define HYSTERESIS_PARAMENT_UMAX 24.0f
// 由电流引起的电压增加量
#define U_FIX 0.2f

// 关闭PWM输出的类型
// 充电保护
#define CHARGE_CAP_OV 0x001
// 放电保护
#define CHARGE_CAP_UV 0x002
// 过压、欠压、过流等保护
#define SAFETYPROTECT 0x004
// 底盘控制
#define CHASSISCTRL 0x008
// 断电保护
#define POWEROUTAGEPROTECT 0x010
// fault保护
#define FAULTPROTECT 0x020
// 用于关闭输出
#define STOP 0x040
// hardfault保护
#define HARDFAULTPROTECT 0x080
// 所有失能
#define ALL_PWMDISABLE 0xfff

typedef enum
{
    CurrentCtrl_Mode_Buck,
    CurrentCtrl_Mode_Buck_Boost,
    CurrentCtrl_Mode_Boost,
} CurrentCtrl_Mode_e;

typedef struct
{
    CurrentCtrl_Mode_e Mode;   // 目前H桥的模式
    pid_t CurrentPID;          // 电流环 PID 闭环结构体
    float Accl_Now;            // 最终使用的电流变化率
    float I_SetReq;            // 外部的电流设定值
    uint32_t Close;            // 关闭标志位
    uint8_t ChassisEnableFlag; // 底盘是否允许工作
} Current_Ctrl_s;

/**
 * @brief 修改底盘决定工作的标志位
 *
 * @param Flag 底盘是否工作标志位 1 工作 0 不工作
 */
void Chassis_EnableSet(uint8_t Flag);

/**
 * @brief 查询目前PWM输出是否关闭
 * @author dty
 * @param type 要查询的保护类型
 * @return uint8_t 1：关闭；0：未关闭
 */
uint8_t IS_PWM_Disable(uint32_t type);

/**
 * @brief 按照设定值进行电流环计算
 * @param PWMCount 时刻
 */
void CurrentCtrl_Routine(uint32_t PWMCount);

/**
 * @brief 修改电流环设定值，对其进行限幅
 * @param NewSet 电流设定值
 */
void Current_FreshSet(float NewSet);

/**
 * @brief 电流环初始化
 */
void CurrentCtrl_Init(void);

/**
 * @brief PWM输出控制（上下桥完全关断）
 * @author dty
 * @param type 类型
 * @param state 0：关闭；1：打开。
 */
void PWM_Output_Ctrl(uint32_t type, uint8_t state);

/**
 * @brief 读取 Current_Ctrl 变量
 * @return Current_Ctrl_s*
 */
Current_Ctrl_s *Current_CtrlRead(void);

#endif /* __FUNC_CURRENTCTRL_H__ */
