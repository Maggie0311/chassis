#ifndef __MOD_MOTION_H__
#define __MOD_MOTION_H__

#include "controlMsg.h"
#include "drv_SetPlanning.h"

/*
 * 注释缩写指代：
 * Mot->Motion;         incre->increment;      fol->follow;             Vel->velocity             pos->position
 * Chass->chassis;      Gim->Gimbal;           Gc,GC->Gimbal Chassis    MotPack->Motion Module Pack运动模块包
 * Eccentric,offset,off_set->偏心,偏离中心;     angvel->角速度;           MotModule->运动模块       av->angle velocity
 * VxyW,xyw->速度矢量的xy分量和角速度w;          Pxy->位置坐标点(x,y)      ExMotMod->External Motion Module
 */

// 输入限幅参数 底盘坐标系为前进为y轴，右手为x轴的右手坐标系
#define M_MAX_ROTATE_AC 9000  // 底盘最大自旋角速度，单位0.1°/s
#define M_MAX_POSITION_Y 2000 // 旋转轴坐标最大值，单位mm --暂时去除未用
#define M_MAX_POSITION_X 2000 // --暂时去除未用

#define GYRO_ANGEL_MINLIM (6.0f) // 陀螺模式下上坡角度阈值，单位度，超过此角度时不论功率如何都开始限制旋转保证平移性

// 英雄抬升机构电机目标角度
#define Screw_Motor_UP_Target (480.0f)   // TODO
#define Screw_Motor_Down_Target (0.0f) // TODO
#define Screw_Motor_ZeroOffset (0.0f)  // TODO

/*------------------------------------模块宏开关----------------------------------------------------------*/
#define SMOOTH_VELOCITY_ENABLE 1 // 使能速度平滑
#define SMOOTH_ANGVEL_ENABLE 0   // 使能角速度平滑
/*------------------------------------模块宏开关---------------------------------------------------------*/
extern Mot_base_t Speed_xyw;
extern Mot_base_t Force_xyw;
extern SecondOrder_Set_Follow_t set_planner_data[];
typedef enum
{

    WHEEL_XSPEED, // 底盘速度的平滑
    WHEEL_YSPEED,

    WHEEL_ACSPEED, // 底盘角速度的平滑
    SMOOTH_NUM,

} Smooth_Acc_e;

/*公转运动录制结构体,创建时成员值为0即可*/
typedef struct
{
    Vector2_t last_pos; // 上次绕点坐标
    float last_angvel;  // 上次角速度
    float incre_theta;

} Mot_Record_t; // 实现绕任意的运动的关键结构体

typedef __packed struct
{
    float lim_level;
    uint8_t lim_max;
    uint8_t lim_down;
    float lim_k;
    float lim_out;
    float lim_I;
} v_lim_t;

Mot_base_t MotPack_Only_Chass(Mot_base_t VxyW);

/**
 * @brief 小陀螺运动 当 VxyW.angvel == 0时,为不跟随的底盘全向移动运动
 * @param {Mot_base_t} VxyW VxyW 平移速度矢量，单位mm/s. 自转角速度，单位0.1°/s
 * @param {float} getYawAngel 得到坐标系旋转角度  (-180.0°,180.0°]
 * @param {uint8_t} isReverse 进行坐标系变化时是角度否需要，当底盘逆时针旋转90度时角度为正则isReverse=0不需要反向
 * @return 返回控制坐标系下VxyW，旋转yawAngle后坐标系下的VxyW
 */
Mot_base_t MotPack_Small_Top(Ctrl_data_t *const ctrlMsg);

/**
 * @brief    底盘跟随云台运动,即所有控制方向都是以云台坐标系而言的 正前方为云台枪管方向
 * @note     输入的VxyW.angvel没有使用 该函数不改变也不限制输入速度单位
 * @param    VxyW.vel    云台方向的平移速度矢量，单位mm/s
 * @param    fol_angle   跟随角,在俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下,
 *           云台枪管正方向和在底盘车头重合时为0度. 跟随角θ表示底盘车头和云台枪管期望夹角 >0表示底盘在云台逆时针方向
 * @param    fol_angle 范围：[0-360°]单位度
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Follow_Gim(const Ctrl_data_t *const ctrlMsg, uint8_t trunin_mode);

int Set_Planner_init(void);
void Set_Plan_Vxyw_Process(Ctrl_data_t ctrl);

/**
 * @brief 偏心小陀螺运动
 * @param {Ctrl_data_t} *ctrlMsg 控制数据 【底盘xy速度矢量，单位mm/s 底盘自转角速度大小，单位0.1°/s】
 * @param {Vector2_t} *pos 偏心坐标点，单位(mm,mm)
 */
Mot_base_t MotPack_Offset_SmallTop(const Ctrl_data_t *const ctrlMsg, const Vector2_t *const pos);

uint8_t gyro_at_powerlim(Mot_base_t *const VxyW, Ctrl_data_t *const ctrlMsg);
Vector2_t MotModule_Transform(Vector2_t speed, float theta);

// 抬升机构状态机
Mot_base_t UP_Machine_SM(Machine_Status *status, float motor_speed[4]);
Mot_base_t Down_Machine_SM(Machine_Status *status, float motor_speed[4]);

#endif
