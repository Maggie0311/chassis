#include "drv_GimMotor.h"
#include "HCanID_data.h"
#include "WHeel_data.h"
#include "drv_utils.h"

static Motordata_t pitch_data; // pitch轴当前数据（以水平为0度
Motordata_t yaw_data;          // yaw轴数据

/**
 * @brief 将整数转换为浮点数，用于DM电机的反馈报文解算
 * 
 * @param x_int 需要转换的整数
 * @param x_min 转换的最大值
 * @param x_max 转换的最小值
 * @param bits  整数的位数
 * @return float 最后的返回值为该整数所转换的浮点数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    // 计算范围
    float span = x_max - x_min;
    // 计算偏移量
    float offset = x_min;
    // 将整数转换为浮点数
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief    赋值电机反馈信息(3508/6020)
 * @param    message     接收到的数据帧指针
 * @param    motordata   云台电机结构体
 */
static void Assign_Motor_Data(Motordata_t *motordata, struct rt_can_msg *message)
{
    motordata->angle = (message->data[0] << 8) + message->data[1];   // 转子角度
    motordata->speed = (message->data[2] << 8) + message->data[3];   // 转子转速
    motordata->current = (message->data[4] << 8) + message->data[5]; // 实际电流
    motordata->temperature = message->data[6];                       // 温度
    motordata->FreshTick = rt_tick_get();

    motordata->speed_convers = motordata->speed;    //此处为了适配和DM一样的逻辑，单位为RPM
}

/**
 * @brief 判断云台电机是否离线
 * 
 * @return uint8_t 
 */
uint8_t GetGimMotorOffline(void)
{
    uint8_t isoffline = 0;
    if(rt_tick_get()-yaw_data.FreshTick > 500)
    {
        isoffline|= 1<<0;
    }
    if(rt_tick_get()-pitch_data.FreshTick > 500)
    {
        isoffline|= 1<<1;
    }
    return isoffline;
}
#ifdef YAW_MOTOR_DM
#define DM_V_MAX 45.0f // 达妙电机的最大转速为45.0rad/s
/**
 * @brief    赋值电机反馈信息(DM)
 * @param    message     接收到的数据帧指针
 * @param    motordata   云台电机结构体
 */
static void Assign_Motor_Data_DM(Motordata_t *motordata, struct rt_can_msg *message)
{
    motordata->angle = (rt_uint16_t)((message->data[1] << 8) | message->data[2]);            // 转子角度
    motordata->speed = (rt_uint16_t)((message->data[3] << 4) | (message->data[4] >> 4));     // 转子转速
    motordata->current = (rt_uint16_t)(((message->data[4] & 0x0F) << 8) | message->data[5]); // 实际电流
    // motordata->angle -= (1 << 15);
    // motordata->speed -= (1 << 11);
    // motordata->current -= (1 << 11);
    motordata->temperature = message->data[7]; // 温度
    motordata->FreshTick = rt_tick_get();

    motordata->speed_convers=RADPS2RPM_f(uint_to_float(motordata->speed, -DM_V_MAX, DM_V_MAX, 12)); // 转速范围[-V_MAX,V_MAX]，达妙转换后的浮点数值是弧度制,speed_convers单位为RPM
}
#endif
/**
 * @brief 俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，底盘静止
 *        云台和底盘正方向重合yaw轴电机角度数据转为0，yaw轴电机逆时针旋转,其速度转化为正
 *        修正后角度数据范围(-8192/2,8192/2] 速度范围不变
 * @param {Motordata_t} *motordata 云台电机结构体
 * @param {rt_uint16_t} zero_pot 编码器零点---即枪管正前方和底盘正前方重合时编码器数据
 * @param {rt_bool_t} if_reversal 数据源是否取反
 */
static void Fix_GimbalMotor(Motordata_t *motordata, rt_uint16_t zero_pot, rt_bool_t if_reversal)
{
    /*数据范围(-8192/2,8192/2] -------2^13=8192 负数右移不改变符号位*/
    /*转换零点，数据范围(-8192/2,8192/2]*/
    motordata->fix_angle = utils_Circle_Shortest_Dis(zero_pot, motordata->angle, 0, YAW_COUNT);

    /*对yaw angle的增大方向进行矫正*/
    if (if_reversal == 1)
    {
        if (motordata->fix_angle != (rt_int32_t)(YAW_COUNT / 2))
            motordata->fix_angle = -motordata->fix_angle;

        motordata->speed_convers = -motordata->speed_convers;
    }
}

/***
 * @brief CAN回调-更新云台电机数据，并转化为底盘坐标系下可用的数据
 * @note  在俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，
          云台和底盘正方向重合yaw轴电机角度数据转为0，yaw轴电机逆时针旋转,其速度转化为正
          修正后角度数据范围(-8192/2,8192/2](3508/6020)|(-65536/2,65536/2](DM) 速度范围不变
 * @note  pitch轴编码器数值：云台枪管往上抬 >0，云台枪管水平时 =0.
 * @param message 云台yaw or pitch电机报文
 ***/
void Refresh_GimbalMotor(struct rt_can_msg *message)
{
    switch (message->id)
    {
    case YAW_MOTOR_RXID:
#ifdef YAW_MOTOR_6020
        Assign_Motor_Data(&yaw_data, message);
        Fix_GimbalMotor(&yaw_data, YAW_ZERO_ANGLE, YAW_ANGLE_REVERSAL);
#endif
#ifdef YAW_MOTOR_DM
        Assign_Motor_Data_DM(&yaw_data, message);
        Fix_GimbalMotor(&yaw_data, YAW_ZERO_ANGLE, YAW_ANGLE_REVERSAL);
#endif
        break;

    case PITCH_MOTOR_RXID:
        Assign_Motor_Data(&pitch_data, message);
        Fix_GimbalMotor(&pitch_data, PITCH_ZERO_ANGLE, PITCH_ANGLE_REVERSAL);
        break;

    default:
        break;
    }
}

/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿后的数据
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-4096,4096]
 ***/
rt_int16_t Get_GM_8192(GimMotor_e kind)
{
    if (kind == GM_YAW)
        return yaw_data.fix_angle;
    else if (kind == GM_PITCH)
        return pitch_data.fix_angle;

    return 0;
}

/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_GM_ANGLE_360f(GimMotor_e kind)
{
#ifdef YAW_MOTOR_6020
    if (kind == GM_YAW)
        return (float)yaw_data.fix_angle / 8192.0f * 360.0f;
#endif
#ifdef YAW_MOTOR_DM
    if (kind == GM_YAW)
        return (float)yaw_data.fix_angle / 65536.0f * 360.0f;
#endif
    else if (kind == GM_PITCH)
        return (float)pitch_data.fix_angle / 8192.0f * 360.0f;

    return 0;
}
/***
 * @brief    获取云台电机角速度数据 
 *           角速度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角速度,向左转速度为正就是正常的情况
 * @param    kind 云台yaw or pitch电机
 * @return   单位：°/s
 *  
 * */
// 根据传入的电机类型，返回对应的电机转速或角度
float Get_GM_Omega(GimMotor_e kind)
{
    // 如果是6020电机，且电机类型为GM_YAW，则返回yaw_data的速度
#ifdef YAW_MOTOR_6020
    if (kind == GM_YAW)
    {
        if (YAW_ANGLE_REVERSAL)
            return -(float)yaw_data.speed_convers * 60.0f; // 单位：RPM————>°/s
        else 
            return (float)yaw_data.speed_convers * 60.0f; // 单位：RPM————>°/s
    }
#endif
    // 如果是DM电机，且电机类型为GM_YAW，则返回yaw_data的固定角度除以65536.0f再乘以360.0f
#ifdef YAW_MOTOR_DM
    if (kind == GM_YAW)
    {
        if (YAW_ANGLE_REVERSAL)
            return -(float)yaw_data.speed_convers * 60.0f; // 单位：RPM————>°/s
        else
            return (float)yaw_data.speed_convers * 60.0f; // 单位：RPM————>°/s
    }
#endif
    // 如果电机类型为GM_PITCH，则返回pitch_data的固定角度除以8192.0f再乘以360.0f
    else if (kind == GM_PITCH)
        return (float)pitch_data.speed_convers * 60.0f; // 单位：RPM————>°/s
    // 如果以上条件都不满足，则返回0
    return 0;
}
/***
 * @brief    获取云台电机角度数据 已经将电机零位补偿 当云台正方向和底盘正方向重合时为0
 *           角度为正表示在俯视图底盘坐标系下云台枪管的逆时针旋转的角度
 * @param    kind 云台yaw or pitch电机
 * @return   数据范围(-180.0°,180.0°]
 ***/
float Get_GMFilter_360f(GimMotor_e kind)
{
    if (kind == GM_YAW)
    {
        yaw_data.filterFixangle = UTILS_LP_FAST(yaw_data.filterFixangle, yaw_data.fix_angle, YAW_FILTER_K);
        if (abs(yaw_data.filterFixangle) < 30)
        {
            return 0;
        }
        else
        {
            return (float)yaw_data.filterFixangle / 8192.0f * 360.0f;
        }
    }
    else if (kind == GM_PITCH)
        return (float)pitch_data.fix_angle / 8192.0f * 360.0f;

    return 0;
}

/***
 * @brief    获取云台电机速度数据  yaw轴电机速度为正表示：在俯视图底盘坐标系下云台枪管逆时针旋转
 * @param    kind 云台yaw or pitch电机 pitch轴电机速度为正表示：云台枪管抬升
 * @return   单位：°/s
 ***/
float Get_GM_Speed(GimMotor_e kind)
{
    if (kind == GM_YAW)
        return (float)yaw_data.speed_convers * 60.f;
    else if (kind == GM_PITCH)
        return (float)pitch_data.speed_convers * 60.f;

    return 0;
}

/***
 * @brief    获取云台枪管正方向相对于底盘的自转速度; 云台相对于底盘逆时针旋转时，>0
 * @return   单位：0.1°/s
 ***/
float Get_GMtoCH_Speed(void)
{
    return (Get_GM_Speed(GM_YAW) * 10.0f);
}

/**
 * @brief 获取云台 Yaw 轴电机的离线状态
 * @author fwlh
 * @return rt_bool_t        返回真代表 Yaw 轴电机离线(200ms)
 */
rt_bool_t Get_Yaw_Motor_Offline_State(void)
{
    return (((yaw_data.FreshTick && (rt_tick_get() - yaw_data.FreshTick > 200))) ? RT_TRUE : RT_FALSE);
}

/**
 * @brief 获取云台 pitch 轴电机的离线状态
 * @return rt_bool_t        返回真代表 pitch 轴电机离线(200ms)
 */
rt_bool_t Get_Pitch_Motor_Offline_State(void)
{
    return (((pitch_data.FreshTick && (rt_tick_get() - pitch_data.FreshTick > 200))) ? RT_TRUE : RT_FALSE);
}
