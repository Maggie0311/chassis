#include "drv_GimMotor.h"
#include "robot_control.h"

      // yaw轴数据

#define GIM_MOTOR_MAXNUM 5

MotorInstance *Gim_Motor_Pointer[GIM_MOTOR_MAXNUM];
uint8_t gim_motor_num=0;

/**
 * @brief 注册云台电机
 * 
 * @param motor_instance 电机实例
 * @return int 1为错误，0为无错误
 */
int Gim_Motor_Attach(MotorInstance *motor_instance)
{
    if(motor_instance == RT_NULL)
        return 1;
    else
        Gim_Motor_Pointer[gim_motor_num++] = motor_instance; // 注册电机实例指针
    return 0;
}

/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @return   数据范围(0°~360°]
 ***/
float Get_YAW_ANGLE_360f(void)
{
#ifdef  BIG_YAW
    return Gim_Motor_Pointer[BIG_YAW]->Motor_obj.angle ;
#else
    return Gim_Motor_Pointer[YAW_MOTOR]->Motor_obj.angle;
#endif
}

/**
 * @brief 获取云台pitch电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 * 
 * @return 数据范围(0°~360°] 
 */
float Get_PITCH_ANGLE_360f(void)
{
    return Gim_Motor_Pointer[PITCH_MOTOR]->Motor_obj.angle;
}

/***
 * @brief    获取云台电机速度数据  yaw轴电机速度为正表示：在俯视图底盘坐标系下云台枪管逆时针旋转
 * @param    kind 云台yaw or pitch电机 pitch轴电机速度为正表示：云台枪管抬升
 * @return   单位：°/s
 ***/
float Get_GM_Speed(MotorInstance *motor_instance)
{
    return motor_instance->Motor_obj.speed*6.f;
}

/***
 * @brief    获取云台枪管正方向相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void)
{
#ifdef  BIG_YAW
    return ((Get_GM_Speed(Gim_Motor_Pointer[YAW_MOTOR])+Get_GM_Speed(Gim_Motor_Pointer[BIG_YAW])) * 10.0f);//TODO:
#else
    return (Get_GM_Speed(Gim_Motor_Pointer[YAW_MOTOR]) * 10.0f);
#endif
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

//todo 该函数待处理
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
