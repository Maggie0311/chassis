#ifndef __DRV_GIMMOTOR_H__
#define __DRV_GIMMOTOR_H__
#include <rtthread.h>
#include <rtdevice.h>

#define YAW_FILTER_K (0.85f)

#ifdef YAW_MOTOR_6020
#define YAW_COUNT 8192
#endif
#ifdef YAW_MOTOR_DM
#define YAW_COUNT 65536
#endif

typedef struct
{
    rt_uint32_t angle;      // 转子角度，大小范围[0~8191]6020[0~65535]DM
    rt_int16_t speed;       // 转子转速，单位rpm
    rt_int16_t current;     // 实际转矩电流
    rt_uint8_t temperature; // 电机的实际温度

    rt_int32_t fix_angle;      // 修正过后的角度，大小范围(-4096~4096]6020(-32768~32768]DM
    rt_int32_t filterFixangle; // 滤波修正过后的角度，大小范围(-4096~4096]6020(-32768~32768]DM
    rt_tick_t FreshTick;       // 数据更新的时刻
    float speed_convers;       // 电机反馈报文转换为RPM的速度
} Motordata_t;

typedef enum
{
    GM_YAW,
    GM_PITCH,

} GimMotor_e;

/***
 * @brief 更新云台电机数据，并转化为底盘坐标系下可用的数据
 * @note  在俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，
          云台和底盘正方向重合yaw轴电机角度数据转为0，yaw轴电机逆时针旋转,其速度转化为正
          修正后角度数据范围(-8192/2,8192/2] 速度范围不变
 * @note  pitch轴编码器数值：云台枪管往上抬 >0，云台枪管水平时 =0.
 * @param message 云台yaw or pitch电机报文
 ***/
void Refresh_GimbalMotor(struct rt_can_msg *message);
uint8_t GetGimMotorOffline(void);

/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿后的数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-4096,4096]
 ***/
rt_int16_t Get_GM_8192(GimMotor_e kind);

float Get_GMFilter_360f(GimMotor_e kind);
/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_GM_ANGLE_360f(GimMotor_e kind);
/***
 * @brief    获取云台电机角速度数据 
 *           角速度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角速度,向左转速度为正就是正常的情况
 * @param    kind 云台yaw or pitch电机
 * @return   单位：°/s
 *  
 * */
// 根据传入的电机类型，返回对应的电机转速或角度
float Get_GM_Omega(GimMotor_e kind);
/***
 * @brief    获取云台电机速度数据  yaw轴电机速度为正表示：在俯视图底盘坐标系下云台枪管逆时针旋转
 * @param    kind 云台yaw or pitch电机 pitch轴电机速度为正表示：云台枪管抬升
 * @return   单位：°/s
 ***/
float Get_GM_Speed(GimMotor_e kind);

/***
 * @brief    获取云台枪管正方向相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void);

/**
 * @brief 获取底盘 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
extern rt_bool_t Get_Yaw_Motor_Offline_State(void);
rt_bool_t Get_Pitch_Motor_Offline_State(void);
#endif
