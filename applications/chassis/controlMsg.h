#ifndef _CONTROLMSG_H_
#define _CONTROLMSG_H_

#include "fun_ctrlSrc.h"
#include "stdint.h"

// 数据源默认优先级（越大越高）
#define SE_MONITOR_LEVEL 4
#define SE_VISUAL_LEVEL 3
#define SE_TEST_LEVEL 2
#define SE_GIMBAL_LEVEL 1

// 数据源默认是否使用
#define SE_MONITOR_IF_USED 0
#define SE_GIMBAL_IF_USED 1
#define SE_TEST_IF_USED 0 // 底盘单独控制模式时需要设置为1（单独接入遥控器控制底盘）

#ifdef CORE_USING_SENTRY
#define SE_VISUAL_IF_USED 0 // 初始化时默认不使用，需要云台遥控器给信号是否进入哨兵模式
#else
#define SE_VISUAL_IF_USED 0
#endif

/*数据源*/
typedef enum
{
    CS_No = 0,  // 当前无数据源控制
    CS_GIMBAL,  // 云台
    CS_TEST,    // 测试模块
    CS_Visual,  // 视觉
    CS_MONITOR, // 监视器
    SOURCE_NUM, // 数据源数量
} Ctrl_source_e;
extern Ctrl_source_t ctrl_source[SOURCE_NUM];
// todo 这里也有包含向量的部分，需要检查一下
/*二维矢量*/
typedef struct
{
    float x;
    float y;
} Vector2_t;

typedef struct
{
    float x;
    float y;
    float z;
    float A;        // 矢量大小
    float xy_theta; // 二维矢量偏角,范围为(-180°,180°]，0°为坐标系的y轴，角度值增大方向为z轴旋转正方向
} Vector3_t;

// 运动三个矢量VX VY W
typedef struct
{
    Vector2_t vel; // 平移速度矢量，单位mm/s
    float angvel;  // 角速度，单位0.1°/s

} Mot_base_t;

/*运动模式*/
typedef enum
{
#ifdef CORE_USING_BALANCING_INFANTRY
    WHEEL_LEGGED_FOLDED, // 轮腿抱紧（平步）
    NO_FOLLOW,           // 不跟随
#else
    NO_FOLLOW = 0, // 不跟随
#endif
    FOLLOW_GIMBAL,     // 跟随
    SLOW_GYRO,         // 慢陀螺
    FAST_GYRO,         // 快陀螺
    CHANGE_GYRO,        // 变速陀螺
    CHASS_STOP_RESET,  // 底盘立即停止并在一段时间后复位（平步）
    FOLLOWBACK_GIMBAL, // 反向跟随
    MOVE_BACK,         // 倒车
    CHASS_AUTO,        // 底盘自主
    JUMP,              // 跳跃
    NOOOOO,
    MOTION_MODE_NUM,
} Motion_mode_e;

typedef struct
{
    Motion_mode_e motion_mode;          // 底盘模式
    Mot_base_t xyw;                     // xy平移速度矢量,单位mm/s; 自转角速度,单位0.1°/s
    Vector2_t pos;                      // 偏心坐标点，单位(mm,mm)
    float dot_angvel;                   // 绕点旋转角速度,单位0.1°/s
    float fol_angle;                    // 跟随目标角度,单位°，范围(0,360°](俯视图下，底盘车头在云台枪管的逆时针方位 >0)
    float (*getYawAngel)(uint16_t arg); // 跟随角时yaw的角度,单位°，范围[-180,180)
    uint16_t arg;                       // 得到角度的参数
    uint8_t folIsReverse;               // 跟随时候，坐标系转化时角度是否取反
} Ctrl_data_t;

typedef float (*getYawAngelCallBack)(uint16_t arg); // 跟随角时yaw的角度,单位°，范围[-180,180)

/*控制输入数据,--注意和Ctrl_data_t统一*/
typedef struct
{
    Motion_mode_e mode;              // 运动模式
    float vel_x;                     // 控制端所给数据单位:mm/s
    float vel_y;                     // 控制端所给数据单位:mm/s
    float angvel;                    // 控制端所给数据单位:0.1°/s
    float pos_x;                     // 偏心坐标点，单位(mm,mm)
    float pos_y;                     // 偏心坐标点，单位(mm,mm)
    float dot_av;                    // 绕点旋转角速度,单位0.1°/s
    float fol_ang;                   // 跟随角,单位°，范围(0,360°](俯视图下，底盘车头在云台枪管的逆时针方位 >0)
    float getYawAngel; // 跟随角时yaw的角度,单位°，范围[-180,180)
   // uint16_t arg;                    // 跟随得到角度的参数
   // uint8_t folIsReverse;            // 跟随时候，坐标系转化时角度是否取反

} Ctrl_data_info_t;

typedef union
{
    Ctrl_data_t data;
    Ctrl_data_info_t info;

} Ctrl_data_u;

/*初始化数据源结构体 安排数据源优先级等*/
void ctrlSrcInit_s(void);

// 获得本文件的控制数据
Ctrl_source_t *getNowCtrlSrc_s(void);

uint8_t Source_WriteData_s(Ctrl_source_e cs, void *ctrlData);

// 设置数据源是否有效
void setSourceIsValid_s(Ctrl_source_e cs, uint8_t isvalid);

uint8_t getNowCtrlSrcIndex(void);

rt_tick_t getCtrlSrcTick(Ctrl_source_e cs);

#endif
