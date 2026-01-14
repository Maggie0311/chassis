#ifndef __FUNC_MODETRIGGER_H__
#define __FUNC_MODETRIGGER_H__
#include "controlMsg.h"
#include <rtdef.h>
#include "Wheel_data.h"
#include "drv_AccClamp.h"

/*模式触发结构体*/
typedef struct
{
    void (*Entry_Mode)(void);
    void (*Exit_Mode)(void);

} Mode_trigger_t;

typedef enum Mode_trigger_e
{
    Entry_Mode_e,
    Exit_Mode_e,

} Mode_trigger_e;

/*平滑类型*/
typedef enum
{

    WHEEL_XSPEED, // 底盘速度的平滑
    WHEEL_YSPEED,

    WHEEL_ACSPEED, // 底盘角速度的平滑
    SMOOTH_NUM,

} Smooth_Acc_e;

extern Smooth_t Smooth_Acc[SMOOTH_NUM];

Vector2_t SmoothAcc_VXY(Vector2_t Vxy_set, Vector2_t Vxy_real);
float SmoothAcc_w(float angvel_set, float angvel_real);
Vector2_t ExMotMod_Get_xyDelta(void);
/**
 * @brief    模式触发调度
 * @param    now_mode    当前模式
 */
void ModeTrig_Scheduler(Motion_mode_e now_mode);
// void func_trigInit(void);
/**
 * @brief : 设置回调函数
 * @param {Motion_mode_e} Motion_mode
 * @param {Mode_trigger_e} Mode_trigger
 */
void Set_Mode_trigger(Motion_mode_e Motion_mode, Mode_trigger_e Mode_trigger, void (*CallBackFun)(void));
#endif
