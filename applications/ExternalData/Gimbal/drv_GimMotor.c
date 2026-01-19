#include "drv_GimMotor.h"
#include "HCanID_data.h"
#include "WHeel_data.h"
#include "drv_utils.h"
#include "robot_control.h"
#include "drv_motor_new.h"
#include "DJI_6020.h"
      // yaw轴数据
#define YAW_MOTOR 0
#define PITCH_MOTOR 1

MotorInstance *Gim_Motor_Pointer[2];
uint8_t gim_motor_num=0;
void Gim_Motor_Attach(MotorInstance *motor_instance)
{
        Gim_Motor_Pointer[gim_motor_num++] = motor_instance; // 注册电机实例指针
}


/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_YAW_ANGLE_360f(void)
{

    return Gim_Motor_Pointer[YAW_MOTOR]->Motor_obj.angle;
}

float Get_PITCH_ANGLE_360f(void)
{

    return Gim_Motor_Pointer[PITCH_MOTOR]->Motor_obj.angle;

}
/***
 * @brief    获取云台电机角速度数据 
 *           角速度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角速度,向左转速度为正就是正常的情况
 * @param    kind 云台yaw or pitch电机
 * @return   单位：°/s
 *  
 * */
// 根据传入的电机类型，返回对应的电机转速或角度
float Get_GM_Omega(MotorInstance *motor_instance)
{
      return motor_instance->Motor_obj.speed*60.f;
}


/***
 * @brief    获取云台电机速度数据  yaw轴电机速度为正表示：在俯视图底盘坐标系下云台枪管逆时针旋转
 * @param    kind 云台yaw or pitch电机 pitch轴电机速度为正表示：云台枪管抬升
 * @return   单位：°/s
 ***/
float Get_GM_Speed(MotorInstance *motor_instance)
{
    return motor_instance->Motor_obj.speed*60.f;
}

/***
 * @brief    获取云台枪管正方向相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void)
{
    return (Get_GM_Speed(Gim_Motor_Pointer[YAW_MOTOR]) * 10.0f);
}

/**
 * @brief 获取云台 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
rt_bool_t Get_Yaw_Motor_Offline_State(void)
{
    return (((Gim_Motor_Pointer[YAW_MOTOR]->Motor_obj.can_instance.rx_time && (rt_tick_get() - Gim_Motor_Pointer[YAW_MOTOR]->Motor_obj.can_instance.rx_time > 200))) ? RT_TRUE : RT_FALSE);
}

/**
 * @brief 获取云台 pitch 轴电机的离线状态
 * @return rt_bool_t        返回真代表 pitch 轴电机离线(200ms)
 */
rt_bool_t Get_Pitch_Motor_Offline_State(void)
{
    return (((Gim_Motor_Pointer[PITCH_MOTOR]->Motor_obj.can_instance.rx_time && (rt_tick_get() - Gim_Motor_Pointer[PITCH_MOTOR]->Motor_obj.can_instance.rx_time > 200))) ? RT_TRUE : RT_FALSE);
}


uint8_t GetGimMotorOffline(void)
{
    uint8_t isoffline = 0;
    if(rt_tick_get()-Gim_Motor_Pointer[YAW_MOTOR]->Motor_obj.can_instance.rx_time  > 500)
    {
        isoffline|= 1<<0;
    }
    if(rt_tick_get()-Gim_Motor_Pointer[PITCH_MOTOR]->Motor_obj.can_instance.rx_time > 500)
    {
        isoffline|= 1<<1;
    }
    return isoffline;
}