#ifndef __FUN_POWERCTRL_H__
#define __FUN_POWERCTRL_H__
#include <board.h>
#include "drv_wheel.h"
#include "SuperCap_Com_new.h"
#include "RLS.h"
#include "app_RefSystem.h"
#include "fun_powerCtrl.h"
#include "drv_utils.h"
#include "func_sd_master.h"
#include "fun_protocolGim.h"
#include "RLS.h"
#include  "SuperCap_Com_new.h"
#include "mod_motion.h"
#include "app_RefSystem.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "LK_MF_MG.h"
#define Power_Ctrl_Debug (1) // 功率控制debug

// 分配等级功能开关：0:不使用，1:全部使用，2：只使用限制不使用增益。（不建议只使用加速度增益效果所以没留接口）
#define IF_USED_ALLO_LEVEL (2)
#define One_Level_Persent (0.01f) // 一分配等级对应的调整功率比例
#define Gain_Trigger_Power (15)   // 触发加速度增益的冗余功率大小

#define K_Conservative (1.0f) // 功率限制的保守系数(设为1时不作保守限制)

#define POWER_REMANI (40)           // 初始化时功率
#define POWER_MAX (80)             //允许底盘在任何情况下的最大功率（包括有超电的情况下）
#define POWER_REMANI_FLYSLOPE (250) // 飞坡增益后能量缓冲上限
#define RefPowerBuf (60)            // 底盘初始缓冲能量（单位j）
#define N 3
/*电机功率计算公式的参数*/
/****电机功率计算数据****/

//TODO 舵向电机功率模型参数计算需要补充测量

#ifdef WHEEL_MOTOR_3508
#define K_IA2Tor_out_RUN (0.3f)                        // 输出轴电流扭矩转换比（N·m/A）
#define K_Vcut_RUN REDUCT_RATIO_RUN                   // 减速比
#define I_Count_Lim_RUN (16384)                        // 反馈电流数据最大绝对值
#define I_Lim_RUN (20.f)                               // 反馈电流数据最大绝对值对应的最大绝对扭矩电流
#define K_IC2IA_RUN (I_Lim_RUN / I_Count_Lim_RUN)    // 反馈电流数据(COUNT)转换为安培(A)的转换比
#define K_IA2Tor_RUN (K_IA2Tor_out_RUN / K_Vcut_RUN) // 扭矩电流（A）转换为转子转矩(N·m)的转换比
#define K_IC2Tor_RUN (K_IC2IA_RUN * K_IA2Tor_RUN)    // 反馈电流数据(COUNT)转换为转子转矩(N·m)的转换比
#endif

#ifdef STEER_MOTOR_6020
#define K_IA2Tor_out_ANGLE (0.741f)                        // 输出轴电流扭矩转换比（N·m/A）
#define K_Vcut_ANGLE REDUCT_RATIO_ANGLE                        // 减速比
#define I_Count_Lim_ANGLE (16384)                        // 反馈电流数据最大绝对值
#define I_Lim_ANGLE (3.f)                               // 反馈电流数据最大绝对值对应的最大绝对扭矩电流
#define K_IC2IA_ANGLE (I_Lim_ANGLE / I_Count_Lim_ANGLE)    // 反馈电流数据(COUNT)转换为安培(A)的转换比
#define K_IA2Tor_ANGLE (K_IA2Tor_out_ANGLE / K_Vcut_ANGLE) // 扭矩电流（A）转换为转子转矩(N·m)的转换比
#define K_IC2Tor_ANGLE (K_IC2IA_ANGLE * K_IA2Tor_ANGLE)    // 反馈电流数据(COUNT)转换为转子转矩(N·m)的转换比
#endif


#ifdef STEER_MOTOR_LK_6015
#define K_IA2Tor_out_ANGLE (0.62f)                        // 输出轴电流扭矩转换比（ N·m/A）
#define K_Vcut_ANGLE 1                       // 减速比
#define I_Count_Lim_ANGLE (850)                        // 反馈电流数据最大绝对值
#define I_Lim_ANGLE (1.1f)                               // 反馈电流数据最大绝对值对应的最大绝对扭矩电流
#define K_IC2IA_ANGLE (I_Lim_ANGLE / I_Count_Lim_ANGLE)    // 反馈电流数据(COUNT)转换为安培(A)的转换比
#define K_IA2Tor_ANGLE (K_IA2Tor_out_ANGLE / K_Vcut_ANGLE) // 扭矩电流（A）转换为转子转矩(N·m)的转换比
//#define K_IC2Tor_ANGLE (K_IC2IA_ANGLE * K_IA2Tor_ANGLE)    // 反馈电流数据(COUNT)转换为转子转矩(N·m)的转换比
#define K_IC2Tor_ANGLE  0.000673
#endif

// 超级电容最多输出电流--有一定余量
#define Supr_Cap_MaxOutputCurrent (8.f)
// 预留能量控制响应时间 一般大于200ms 单位ms
#define Supr_Cap_DelayTime (600)
// 预留能量，单位j
#define PowerCtrlReamin (35)
// 裁判系统功率反馈延时时间  单位ms
#define RefPowerData_RefTime (500)
// 强制预留能量，单位j
#define PowerBuffReamin (15)
/*************************获取到的功率数据结构体******************************/
/*裁判系统数据结构体*/
typedef struct
{
    uint8_t isValid;         // 裁判系统数据是否可信
    rt_uint16_t IsOverPower; // 裁判系统得到是否有超功率
    uint8_t IsFlySlope;      // 是否发送飞坡
    rt_tick_t refresh_tick; // 裁判系统数据更新时刻
    rt_uint16_t PowerBuff;    // 裁判系统得到的当前剩余缓冲能量单位 j
    rt_uint16_t Power_Maxlim; // 裁判系统通信最大功率限制 单位w
    float power_now;            //前期拟合功率模型参数时，需要获得裁判系统当前功率数据，单位:W
    uint16_t powerBuffOverCnts; // 缓冲耗尽计数
    uint16_t powerOverCnts;     // 超功率扣血计数

} Power_info_Ref_t;

/*超级电容数据结构体*/
typedef struct
{
    /**获取到的数据（有效时可用）（无效时置为0,0,24V,0,0）**/
    uint8_t isValid;       // 超级电容数据是否可信*
    float superCap;        // 超级电容当前电量0-100*
    float V_Cap;           // 超级电容组电压V
    float Voltage;         // 超级电容测量电池电压单位V*
    float Current;         // 超级电容测量电池输出总电流A*
    SupCap_State_e Cap_State; // 超级电容充放电状态*//此项根据底盘与超电两次通讯的电压数据来获取
    /**本程序要计算的数据**/
    float SupplyCurrt; // 超级电容当前可以放电电流 单位A
    float Power;       // 超级电容测量底盘总功率单位W
    float Power_last;
} Power_info_Cap_t;

typedef struct
{
    /*裁判系统数据*/
    Power_info_Ref_t Ref;
    /*超级电容数据*/
    Power_info_Cap_t Cap;
} Power_info_t;


typedef struct
{
    float  power_set_run;
    float  power_set_steer;
    float  power_set_sum_temp;  // 用于查看计算的电机预输出总功率
    float  run_power;        //轮向电机预输出功率
    float  steer_power;     //舵向电机预输出功率
    float  power_set_sum;   //预输出总功率
    float  run_ratio[4];    //各电机输出功率的占比
    float  angle_ratio[4];    //各电机输出功率的占比
    float  run_err_sum;
    float  run_err_sq_sum;
    float angle_err_sum;
    

} Power_Set_t;
/*************************功率控制的结构体******************************/


typedef struct
{
    // 分配等级功能：1.强制功率限制。2.功率有剩余时基于功率分配的加速度增益规划（根据pid的err大小和方向与当前输出等数据之间的关系）
    float Power_Allo_level;  // 当前功率分配等级（小于0时为强制限制功率）（置0时为正常功率分配和限制）（大于0时分配功率获得加速度增益）
    float Power_Maxlim_sum;  // 当前时刻允许的底盘最大瞬时功率（包括静态功率）
    float Power_Maxlim;      // 当前时刻允许的电机最大瞬时功率（去除静态功率）
    uint8_t power_allo_flag; // 功率分配标志，用于查看功率分配状态，0:未超功率无限制，1:普通超功率限制，2:强制功率限制
    float k_power_allo_steer;      //  舵向电机功率分配系数
    float k_power_allo_wheel;   //  轮向电机功率分配系数
    float k_power_allo_wheel_up;   //  坡上方位置的轮向电机功率分配系数
    float k_power_allo_wheel_down;   //  坡下方位置的轮向电机功率分配系数
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
	float V_last;
} Motor_Power_t;

extern PowerCtrl_t PowerCtrl;
extern Power_info_t PowerInfo;
extern RLS_Estimator RunRLS;


float getScRate(void);
void Power_Ctrl(void);
void powerCtrlInit(void);
float get_k_power_allo();
uint8_t get_SuperCap_if_abnormal(void);
uint8_t get_Super_Cap_if_open(void);
void Power_re_allo(void);
void get_true_RLS(void);
void get_pre_power(void);
void Power_Motor_Attach(MotorInstance *motor_instance);
float get_K_IC2Tor(MotorInstance *motor_instance);
float get_power_speed_max(void);
#endif
