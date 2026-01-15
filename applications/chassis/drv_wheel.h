#ifndef __DRV_WHEEL_H__
#define __DRV_WHEEL_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ThreadCtrl.h"
#include "app_monitor.h"

#include "Wheel_data.h"
#include "controlMsg.h"
#include "drv_motor.h"

#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

/**
 * @brief 更新日期：2025年4月      更新内容：设计通用底盘，合并之前的全向麦轮和舵轮
 * @author Li Jichen
 * 
 */


/*------------------------------------设定轮组类型----------------------------------------------------------*/
// #define RF_USING_OMNI_WHEEL
#if defined (RF_USING_MECANUM_WHEEL)
#define RF_WHEEL_KIND   (MECANUM_WHEEL)
#elif defined (RF_USING_OMNI_WHEEL)
#define RF_WHEEL_KIND   (OMNI_WHEEL)
#elif defined (RF_USING_STEER_WHEEL)
#define RF_WHEEL_KIND   (STEER_WHEEL)
#else
    #error "未设定右前轮组类型"
#endif 

// #define LF_USING_OMNI_WHEEL
#if defined (LF_USING_MECANUM_WHEEL)
#define LF_WHEEL_KIND   (MECANUM_WHEEL)
#elif defined (LF_USING_OMNI_WHEEL)
#define LF_WHEEL_KIND   (OMNI_WHEEL)
#elif defined (LF_USING_STEER_WHEEL)
#define LF_WHEEL_KIND   (STEER_WHEEL)
#else
#error "未设定左前轮组类型"
#endif 

// #define LB_USING_OMNI_WHEEL
#if defined (LB_USING_MECANUM_WHEEL)
#define LB_WHEEL_KIND   (MECANUM_WHEEL)
#elif defined (LB_USING_OMNI_WHEEL)
#define LB_WHEEL_KIND   (OMNI_WHEEL)
#elif defined (LB_USING_STEER_WHEEL)
#define LB_WHEEL_KIND   (STEER_WHEEL)
#else
#error "未设定左后轮组类型"
#endif 

// #define RB_USING_OMNI_WHEEL
#if defined (RB_USING_MECANUM_WHEEL)
#define RB_WHEEL_KIND   (MECANUM_WHEEL)
#elif defined (RB_USING_OMNI_WHEEL)
#define RB_WHEEL_KIND   (OMNI_WHEEL)
#elif defined (RB_USING_STEER_WHEEL)
#define RB_WHEEL_KIND   (STEER_WHEEL)
#else
#error "未设定右后轮组类型"
#endif 
/*------------------------------------设定轮组类型 END----------------------------------------------------------*/

/*------------------------------------PID参数相关宏----------------------------------------------------------*/
// 对i的特殊处理
#define ANGLE_PID_I_THREADHOLD 200
#define RUN_PID_I_THREADHOLD 1000.f
#define I_CTRL_K (0.2f)

// pid控制周期 单位ms
#define PID_CTRL_IN_PERIOD (1)    // 内环运行周期 小于外环
#define PID_CTRL_EX_PERIOD (2)    // 内环运行周期
#define MOTOR_OFFLINE_TIME (500) // 电机离线的阈值,大于这个值则认为电机离线

// 轮向电机pid参数
#define SPE_PID_FILTER_K (0.15f)
#define SPE_PID_PARAMETER 15, 0.1, 0.03, 3000, 7000, -7000      // 速度环pid

// 舵向电机pid参数
#define AGN_PID_FILTER_K (1)                                    /*1为不滤波*/
#define ANGLE_SPE_PID_PARAMETER 150, 0.4, 0, 400, 7000, -7000   // 角速度环pid
#define ANGLE_ANG_PID_PARAMETER 0.3, 0, 0, 0, 600, -600         // 角度环pid
/*------------------------------------PID参数相关宏  END----------------------------------------------------------*/
/*------------------------------------位操作和常量定义宏----------------------------------------------------------*/

#define M_SETBIT(x, y) ((x) |= (1 << y))  // 将X的第Y位置1 标号从0开始
#define M_CLRBIT(x, y) ((x) &= ~(1 << y)) // 将X的第Y位清0 标号从0开始
#define M_READBIT(x, y) ((x) & (1 << y))  // 读取某一位 标号从0开始
#define RPM2RAD_K ((float)((PI) / 30.f))  // rpm转角速度
#define RAD2RPM_K ((float)(30.0f / PI))   // 角速度转rpm
#define VCONVERT2W_k ((float)(1.f / WHEEL_RADIUS)) // 车轮速度转电机输出轴角速度 速度的单位为mm/s 角速度单位为rad/s
#define WCONVERT2V_k ((float)WHEEL_RADIUS)         // 电机输出轴角速度转车轮速度 速度的单位为mm/s 角速度单位为rad/s
/*------------------------------------位操作和常量定义宏 END----------------------------------------------------------*/

/*车轮方位（右前，左前，左后，右后）*/
typedef enum
{
    WHEEL_RF,
    WHEEL_LF,
    WHEEL_LB,
    WHEEL_RB,

    WHEELS_GROUP_NUM, // 车轮组数（四组）
} Wheel_Group_e;

/*车轮轮组电机位置（轮向电机，舵向电机）*/
typedef enum
{
    WHEEL_RUN,      // 轮向电机
    WHEEL_ANGLE,    // 舵向电机

    WHEEL_NUM = 2,  //每组电机个数（两个/一个，但至少存在轮向电机）
} Wheel_Local_e;

/*轮组种类（麦克纳姆轮，全向轮，舵轮）*/
typedef enum
{
    MECANUM_WHEEL,      //麦克纳姆轮
    OMNI_WHEEL,         //全向轮
    STEER_WHEEL,        //舵轮
    WHEEL_KINDS_NUM,    //轮组种类
}Wheel_Kind_e;

//新增轮组种类
typedef struct
{
    Motor_t motor;
    Wheel_Kind_e wheel_kind;
    
} Wheel_t;

/*电机闭环方式*/
typedef enum
{
    MECHANICAL_CALI,
    LOOP_CTRL, // 闭环
    NO_CTRL,   // 无闭环
} Crtl_e;

//电机堵转结构体
typedef struct
{
    rt_uint8_t isStuck;
    rt_uint8_t stuckCounts;
    rt_uint8_t His_stuck; // 是否出现过堵转
    rt_uint32_t lastTick;
    rt_uint32_t stuckTick;
    float lastSet;
} MotorStuck_t;

typedef struct
{
    float Kp_origin;
    float P_range;
    float P_max;
    float P_min;
    float err_max;
    float lim_up_origin;
    float lim_down_origin;
} PID_Change_t;

/*车轮基本信息结构体*/
typedef struct
{
    rt_uint8_t Wheel_Enabled;        // 写 0 可以将底盘电机失能
    rt_uint8_t offLine[4];           // 主要直观，可以去掉
    rt_uint8_t Wheels_Motor_Offline; // 用于记录底盘电机离线情况, 从低到高分别为：右前、左前、左后、右后
    Crtl_e Crtl_mode;
} Wheels_info_t;


extern Wheel_t wheel[WHEELS_GROUP_NUM][WHEEL_NUM];       //轮组数据

/*外部得到堵转标志位*/
uint8_t getMotorIsStack(Wheel_Group_e group, Wheel_Local_e wheel_type);
/*外部清除堵转标志位*/
uint8_t clcStackFlag(Wheel_Group_e group, Wheel_Local_e wheel_type);
//底盘电机初始化
rt_err_t Wheels_Init(void);

//获取舵向电机当前角度零点转化后的值
float getNowFixAngle(Wheel_Group_e group);
//设置舵向电机目标角度，相对于车体的角度，范围(-1800,1800]
void Wheel_Angle_Set(Wheel_Group_e group, float angle);
//设置轮向电机目标速度，单位mm/s
void Wheel_Speed_Set(Wheel_Group_e group, float speed);


//pid_t *Wheel_Read_Angele_PID(Wheel_Group_e group,Wheel_Local_e wheel_type);
/**
 * @brief 读取底盘电机当前的离线情况
 * 从低位到高位顺序为舵向电机的RF-LF-LB-RB，轮向电机的RF-LF-LB-RB 对应位置 1 代表离线
 * @return rt_uint8_t
 */
rt_uint8_t Read_Wheels_Offline_State(void);
/**
 * @brief 使能/失能底盘电机  Wheel_Local_e可选舵向电机还是轮向电机
 * 
 * @param wheel_type 
 * @param EN 
 */
void Wheel_Enable(Wheel_Local_e wheel_type, rt_uint8_t EN);

#endif
