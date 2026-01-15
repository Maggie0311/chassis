#include "drv_motionSolve.h"
#include "Wheel_data.h"
#include "drv_utils.h"
#include "drv_wheel.h"
#include "arm_math.h"
#include "drv_Vector.h"


static float Wheel_Azimuth[WHEELS_GROUP_NUM] = {RF_Wheel_Azimuth, LF_Wheel_Azimuth, LB_Wheel_Azimuth, RB_Wheel_Azimuth};
static float encodeData[WHEELS_GROUP_NUM][WHEEL_NUM];

//********************************逆解算部分************************************** */
/// <summary>麦轮解算，单个轮组
///
/// </summary>
/// <param name="target_xyw"></param>
/// <param name="resolve_data"></param>
static void Mecan_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float *resolve_data)
{
    static float k_w[WHEELS_GROUP_NUM] = {(VEHICLE_LONG + VEHICLE_WIDTH) / 20.0 * DAGTORAD, (VEHICLE_LONG + VEHICLE_WIDTH) / 20.0 * DAGTORAD, (VEHICLE_LONG + VEHICLE_WIDTH) / 20.0 * DAGTORAD, (VEHICLE_LONG + VEHICLE_WIDTH) / 20.0 * DAGTORAD};
    /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
    if (wheel_group == WHEEL_RF)
    {
        resolve_data[WHEEL_RUN] = (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * k_w[WHEEL_RF];
    }
    else if (wheel_group == WHEEL_LF)
    {
        resolve_data[WHEEL_RUN] = (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * k_w[WHEEL_LF];
    }
    else if (wheel_group == WHEEL_LB)
    {
        resolve_data[WHEEL_RUN] = (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * k_w[WHEEL_LB];
    }
    else if (wheel_group == WHEEL_RB)
    {
        resolve_data[WHEEL_RUN] = (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * k_w[WHEEL_RB];
    }
    else
    {
        return;
    }
}

/// <summary>全向轮解算，单个轮组
///
/// </summary>
/// <param name="target_xyw"></param>
/// <param name="resolve_data"></param>
static void Omni_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float *resolve_data)
{
    // 角速度单位为0.1°/s，所以角速度系数要除以10
    static float k_w = VEHICLE_DIAMETER / 20.f * DAGTORAD;
    static float root2_2 = ROOT2 / 2;
    /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
    if (wheel_group == WHEEL_RF)
    {
        resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * k_w;
    }
    else if (wheel_group == WHEEL_LF)
    {
        resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * k_w;
    }
    else if (wheel_group == WHEEL_LB)
    {
        resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * k_w;
    }
    else if (wheel_group == WHEEL_RB)
    {
        resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * k_w;
    }
    else
    {
        return;
    }
}

// 舵轮解算
#define DIR_ANGLE_LIM (900.f) // 单次最大打角理论上应该为90度，单位0.1°,>0

static float range_1800(float angel)
{
    float tm = angel;
    while (tm > 1800)
        tm -= 3600;
    while (tm < -1800)
        tm += 3600;
    return tm;
}
/*两个输入的角度单位均为0.1°/s 范围为-1800-1800*/
static VectorXYA_Str StWheel_Dir_Limit(VectorXYA_Str output, float nowangel)
{
    float now, target, delta_angle1;
    now = range_1800(nowangel);
    target = range_1800(output.theta);
    delta_angle1 = range_1800(target - now); // 限制为180度以内

    //  单次最大打角理论上应该为90度，单位0.1°,>0
    if (delta_angle1 > DIR_ANGLE_LIM)
    {
        output.A = -output.A;
        output.theta -= 1800.0f;
    }
    else if (delta_angle1 < -DIR_ANGLE_LIM)
    {
        output.A = -output.A;
        output.theta += 1800.0f;
    }
    return output;
}
/// <summary>
/// 舵轮解算改进版,要求底盘轮系所在位置必须为正方形底盘
/// </summary>
static void SteerWheel_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float *resolve_data)
{
    // 容易理解的系数，对计算不友好
    // static float k_w = ROOT2 * VEHICLE_LONG / 20.f * DAGTORAD;
    // static float root2_2 = ROOT2 / 2.f;
    // 将系数合在一起后的系数，方便计算
    static float k_w = VEHICLE_LONG / 20.f * DAGTORAD;
    static float angle_temp[WHEELS_GROUP_NUM] = {0, 0, 0, 0}; // 保存舵向电机的角度数据
    VectorXYA_Str temp_v = {0};
    /*输出单位为mm/s 输出顺序RF-LF-LB-RB  先计算每个轮组的x和y方向的速度分量*/
    if (wheel_group == WHEEL_RF)
    {
        temp_v.xy.x = (target_xyw.vel.x - target_xyw.angvel * k_w);
        temp_v.xy.y = (target_xyw.vel.y + target_xyw.angvel * k_w);
    }
    else if (wheel_group == WHEEL_LF)
    {
        temp_v.xy.x = (target_xyw.vel.x - target_xyw.angvel * k_w);
        temp_v.xy.y = (target_xyw.vel.y - target_xyw.angvel * k_w);
    }
    else if (wheel_group == WHEEL_LB)
    {
        temp_v.xy.x = (target_xyw.vel.x + target_xyw.angvel * k_w);
        temp_v.xy.y = (target_xyw.vel.y - target_xyw.angvel * k_w);
    }
    else if (wheel_group == WHEEL_RB)
    {
        temp_v.xy.x = (target_xyw.vel.x + target_xyw.angvel * k_w);
        temp_v.xy.y = (target_xyw.vel.y + target_xyw.angvel * k_w);
    }
    else
    {
        return;
    }
    // 后计算轮向速度大小和舵向角度
    Vector2D_Mod(&temp_v.xy, &temp_v.A);
    temp_v.theta = atan2f(-temp_v.xy.x, temp_v.xy.y) * RADTODAG * 10;
    // 判断舵向角度是否大于90°，设置最优转动方向和速度方向
    // wheel_3v = StWheel_Dir_Limit(wheel_3v, getNowAngel[i] = getNowFixAngle((Wheel_local_e)i) * 10);
    temp_v = StWheel_Dir_Limit(temp_v, 0);
    // 后续优化处理
    // 速度大于10mm/s时，刷新角度，否则保持原角度，目的是解决停车的时候舵轮还归零的情况
    if (temp_v.A >= 10.0 || temp_v.A <= -10.0)
    {
        angle_temp[wheel_group] = temp_v.theta;
    }
    // 速度太小，认为停车，设置角度为之前的保留的角度，速度设置为0
    if (temp_v.A <= 1.0 && temp_v.A >= -1.0)
    {
        temp_v.A = 0;
        temp_v.theta = angle_temp[wheel_group];
    }
    else
    {
        temp_v.theta = angle_temp[wheel_group];
    }
    // 将解算结果设置到解算数据中
    resolve_data[WHEEL_RUN] = temp_v.A;
    resolve_data[WHEEL_ANGLE] = temp_v.theta;
}
/**
 * @brief 底盘运动解算函数，输入底盘的速度和角速度，输出各个电机的速度
 *      * 该函数会根据底盘的轮系类型进行不同的解算
 *      * 向右为x轴正方向，向前为y轴正方向，底盘角速度为正表示底盘逆时针旋转
 * @param Vxyw 速度和角速度，单位mm/s和0.1°/s
 */
void Motion_Resolve(Mot_base_t *Vxyw, float motorSetData[4][2])
{
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
        {
            Mecan_Resolve(*Vxyw, group, motorSetData[group]);
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            Omni_Resolve(*Vxyw, group, motorSetData[group]);
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            SteerWheel_Resolve(*Vxyw, group, motorSetData[group]);
        }
        else
            return;
    }
}


/**
 * @brief 全向轮的力解算，使用雅可比矩阵，解算后的结果作为电机的动力前馈
 * 
 * @param tarForce 
 * @param wheel_group 
 * @param resolve_data 
 */
static void Omni_Force_Resolve(Mot_base_t tarForce, Wheel_Group_e wheel_group, float *resolve_data)
{
    static float root2_4 = ROOT2 / 4.f;
    static float k_w = VEHICLE_DIAMETER / 2000.f;
    static float inv_4kw = 1.f / (4.f * (VEHICLE_DIAMETER / 2000.f));
    if (wheel_group == WHEEL_RF)
    {
        resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y - tarForce.vel.x) + tarForce.angvel * inv_4kw;
    }
    else if (wheel_group == WHEEL_LF)
    {
        resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y + tarForce.vel.x) - tarForce.angvel * inv_4kw;
    }
    else if (wheel_group == WHEEL_LB)
    {
        resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y - tarForce.vel.x) - tarForce.angvel * inv_4kw;
    }
    else if (wheel_group == WHEEL_RB)
    {
        resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y + tarForce.vel.x) + tarForce.angvel * inv_4kw;
    }
    else
    {
        return;
    }
}
// TODO 舵轮和全向轮还没有解算的具体实现:
void Force_Resolve(Mot_base_t *Fxyw, float motorSetForce[4][2])
{
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {

        // if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
        // {
        //     Mecan_Resolve(*Vxyw, group, motorSetForce[group]);
        // }
        // else
        if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            Omni_Force_Resolve(*Fxyw, group, motorSetForce[group]);
        }
        // else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        // {
        //     SteerWheel_Resolve(*Vxyw, group, motorSetForce[group]);
        // }
        else
            return;
    }
}

//********************************逆解算部分************************************** */

//********************************正解算部分************************************** */

/// <summary>
/// 获取底盘目前各个电机状态，轮向电机速度，单位mm/s，舵向电机角度，单位0.1°/s，范围[-1800,+1800],向前为0，向左为负，向右为正
/// </summary>
/// <param name=""></param>
void GetEncode(void)
{
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL || wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            // 单位mm / s      计算过程：快转子rpm->快转子rad / s->输出轴rad / s->轮子mm / s
            encodeData[group][WHEEL_RUN] = RPMTORAD / REDUCT_RATIO_RUN * WCONVERT2V_k * Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor);
            // encodeData[group][WHEEL_RUN] = motorSetData[group][WHEEL_RUN];
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            encodeData[group][WHEEL_RUN] = RPMTORAD / REDUCT_RATIO_ANGLE * WCONVERT2V_k * Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor);
            // encodeData[group][WHEEL_RUN] = motorSetData[group][WHEEL_RUN];
            // 舵向电机角度单位0.1°/s，范围[-1800,+1800],向前为0
            encodeData[group][WHEEL_ANGLE] = 10.f * getNowFixAngle(group);
            // encodeData[group][WHEEL_ANGLE] = motorSetData[group][WHEEL_ANGLE] / 10.f;
        }
        else
            return;
    }
}

/**
 * @brief 麦轮/全向轮底盘正运动解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方
 * @param {坐标系}底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
 * @param {float} encoderData[4] 编码器得到的电机的数据，快转子的转速(rpm)
 * @param {Mot_base_t} *real_xyw 解算出来的车身的速度
 * @return real_xyw->vy 单位m/s real_xyw->vx 单位mm/s  real_xyw->angvel单位0.1°/s
 */
void fromEncoderGetV(Mot_base_t *real_xyw)
{
    // 各个轮组解算的常量系数，按照不同的种类进行划分     该系数考虑距离，角度转弧度，角速度0.1度/s的因素，最终将轮向电机速度直接除以改系数得到角速度值
    static float coefficient_OMNI = VEHICLE_DIAMETER / 20.f * DAGTORAD;
    static float coefficient_MECANUM = (VEHICLE_LONG + VEHICLE_WIDTH) / 20.f * DAGTORAD;
    static float coefficient_STEER = VEHICLE_DIAMETER / 20.f * DAGTORAD;
    // 三个方向速度的符号，顺序为[3]:y,x,w,[WHEELS_GROUP_NUM]:右前,左前，左后，右后，适用于全向轮和麦克纳姆轮，不适用于舵轮
    static int sign[3][WHEELS_GROUP_NUM] = {{1, 1, 1, 1}, {-1, 1, -1, 1}, {1, -1, -1, 1}};
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
        {
            // 单位mm/s
            real_xyw->vel.y += encodeData[group][WHEEL_RUN] / 4.0f * sign[0][group];
            real_xyw->vel.x += encodeData[group][WHEEL_RUN] / 4.0f * sign[1][group];
            // 单位rad/s ->0.1°/s
            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f / coefficient_MECANUM * sign[2][group];
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            // 单位mm/s
            real_xyw->vel.y += encodeData[group][WHEEL_RUN] * ROOT2 / 4.0f * sign[0][group];
            real_xyw->vel.x += encodeData[group][WHEEL_RUN] * ROOT2 / 4.0f * sign[1][group];
            // 单位rad/s ->0.1°/s
            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f / coefficient_OMNI * sign[2][group];
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            // 单位mm/s
            real_xyw->vel.y += encodeData[group][WHEEL_RUN] / 4.0f * cosf(encodeData[group][WHEEL_ANGLE] * DAGTORAD);
            real_xyw->vel.x += encodeData[group][WHEEL_RUN] / 4.0f * sinf(-encodeData[group][WHEEL_ANGLE] * DAGTORAD);
            // 单位rad/s ->0.1°/s
            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f * sinf((encodeData[group][WHEEL_ANGLE] - Wheel_Azimuth[group]) * DAGTORAD) / coefficient_STEER;
        }
        else
            return;
    }
}
/**
 * @brief 获得底盘实际运动状态
 *
 * @param real_xyw 返回底盘实际运动状态，x,y,w的单位分别为mm/s，mm/s，0.1°/s
 */
void Get_real_motion(Mot_base_t *real_xyw)
{
    GetEncode();
    fromEncoderGetV(real_xyw);
}
//********************************正解算部分************************************** */


