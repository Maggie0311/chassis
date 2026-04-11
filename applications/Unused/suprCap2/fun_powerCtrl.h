#ifndef __FUN_POWERCTRL_H__
#define __FUN_POWERCTRL_H__

#include <board.h>
#include "drv_wheel.h"
#include "Wheel_data.h" //电机功率参数在此h文件末

#define Power_Ctrl_Debug (0) // 功率控制debug

// 分配等级功能开关：0:不使用，1:全部使用，2：只使用限制不使用增益。（不建议只使用加速度增益效果所以没留接口）
#define IF_USED_ALLO_LEVEL (2)
#define One_Level_Persent (0.01f) // 一分配等级对应的调整功率比例
#define Gain_Trigger_Power (15)   // 触发加速度增益的冗余功率大小

#define POWER_REMANI (100)           // 初始化时功率
#define POWER_REMANI_FLYSLOPE (250) // 飞坡增益后缓冲上限
#define RefPowerBuf (60)            // 底盘初始缓冲能量（单位j）

/*电机功率计算公式的参数*/
#define K_Conservative (1.0f) // 功率限制的保守系数(设为1时不作保守限制)
/****电机功率计算数据****/
#ifdef WHEEL_MOTOR_3508
#define K_IA2Tor_out (0.3f)              // 输出轴电流扭矩转换比（N·m/A）
#define K_Vcut REDUCT_RATIO              // 减速比
#define I_Count_Lim (16384)              // 反馈电流数据最大绝对值
#define I_Lim (20.f)                     // 反馈电流数据最大绝对值对应的最大绝对扭矩电流
#define K_IC2IA (I_Lim / I_Count_Lim)    // 反馈电流数据(COUNT)转换为安培(A)的转换比
#define K_IA2Tor (K_IA2Tor_out / K_Vcut) // 扭矩电流（A）转换为转子转矩(N·m)的转换比
#define K_IC2Tor (K_IC2IA * K_IA2Tor)    // 反馈电流数据(COUNT)转换为转子转矩(N·m)的转换比
#endif

// 超级电容最多输出电流--有一定余量
#define Supr_Cap_MaxOutputCurrent (8.f)
// 预留能量控制响应时间 一般大于200ms 单位ms
#define Supr_Cap_DelayTime (600)
// 预留能量，单位j
#define PowerCtrlReamin (35)
// 裁判系统功率反馈延时时间  单位ms
#define RefPowerData_RefTime (500)
/*************************获取到的功率数据结构体******************************/
/*裁判系统数据结构体*/
typedef struct
{
    uint8_t isValid;         // 裁判系统数据是否可信
    rt_uint16_t IsOverPower; // 裁判系统得到是否有超功率
    uint8_t IsFlySlope;      // 是否发送飞坡

    rt_tick_t refresh_tick; // 裁判系统数据更新时刻

    float Voltage;            // 均为国际单位
    float Current;            // 均为国际单位
    float Power;              // 均为国际单位
    float lastPower;          // 均为国际单位
    rt_uint16_t PowerBuff;    // 裁判系统得到的当前剩余缓冲能量单位 j
    rt_uint16_t Power_Maxlim; // 裁判系统通信最大功率限制 单位w

    uint16_t powerBuffOverCnts; // 缓冲耗尽计数(tick)
    uint16_t powerOverCnts;

} Power_info_Ref_t;

/*超级电容数据结构体*/
typedef struct
{
    /**获取到的数据（有效时可用）（无效时置为0,0,24V,0）**/
    uint8_t isValid; // 超级电容数据是否可信*
    float superCap;  // 超级电容当前电量0-100*
    float V_Cap;     // 超级电容组电压V
    float Voltage;   // 超级电容测量电池电压单位V
    float Current;   // 超级电容测量电池输出总电流A***二代电容没用到
    /**本程序要计算的数据**/
    float SupplyCurrt; // 超级电容当前可以放电电流 单位A
    float Power;       // 超级电容测量底盘总功率单位W***二代电容没用到
    float Power_last;
} Power_info_Cap_t;

typedef struct
{
    /*裁判系统数据*/
    Power_info_Ref_t Ref;
    /*超级电容数据*/
    Power_info_Cap_t Cap;
} Power_info_t;

/*************************功率控制的结构体******************************/
typedef struct
{
    // 分配等级功能：1.强制功率限制。2.功率有剩余时基于功率分配的加速度增益规划（根据pid的err大小和方向与当前输出等数据之间的关系）
    float Power_Allo_level;  // 当前功率分配等级（小于0时为强制限制功率）（置0时为正常功率分配和限制）（大于0时分配功率获得加速度增益）
    float Power_Maxlim_sum;  // 当前时刻允许的底盘最大瞬时功率（包括静态功率）
    float Power_Maxlim;      // 当前时刻允许的电机最大瞬时功率（去除静态功率）
    uint8_t power_allo_flag; // 功率分配标志，用于查看功率分配状态，0:未超功率无限制，1:普通超功率限制，2:强制功率限制
    float k_power_allo;      // 功率分配系数
    float nowRemain;         // 当前程序预留能量
} PowerCtrl_t;

typedef struct
{
    float power_now;         // 电机当前功率（暂时没有用到）
    float initial_power_set; // 电机进行功率分配前的预输出功率（最初的功率）
    float allo_power_set;    // 电机进行功率分配后的预输出功率（分配的功率）
    float gain_allo_level;   // 功率增益等级（没错，浮点型，其实是个比率，为了保持功能与名字对齐）
    float k_gain_power_allo; // 增益功率分配系数
    float tor_now;           // 电机当前转子转矩
    float initial_tor_set;   // 电机进行功率分配前的预输出转子转矩
    float allo_tor_set;      // 电机进行功率分配后的预输出转子转矩
    float I_now;             // 电机反馈的当前转矩电流（A）
    float I_set;             // 电机预输出的转矩电流（A）
    float V_now;             // 电机反馈的当前转子速度（rpm）
    float V_last;            // 上一时刻电机反馈的当前转子速度（rpm）
    float K1;                // 电机功率计算公式的参数
    float K2;                // 电机功率计算公式的参数
    float C;                 // 电机功率计算公式的参数
} Motor_Power_t;
/*****************************超级电容管理结构体**************************/
typedef struct
{
    rt_bool_t if_hysteresis;    // 是否在低电量保护
    rt_bool_t if_SC_protect;    // 是否开启超级电容低电量保护
    rt_bool_t if_ReserveEnergy; // 是否预留超级电容能量

    float bound_TL_Low; // 允许使用预留电量时电量判断区间下限
    float bound_TH_Low; // 允许使用预留电量时电量判断区间上限

    float bound_TL_High; // 不允许使用预留电量时电量判断区间下限
    float bound_TH_High; // 不允许使用预留电量时电量判断区间上限

} Scpr_manage_t;

float getScRate(void);
void Power_Ctrl(Scpr_manage_t *whectrl, Motor_t *wheel);
void powerCtrlInit(Scpr_manage_t *Wheels_manage);
void ScprDeInit(Scpr_manage_t *Wheels_manage);

#endif
