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

//********************************逆解算部分************************************** */
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
// TODO 舵轮和全向轮还没有逆解算的具体实现:

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

// void base_data_init(void)
// {
//     /* 初始化底盘数据 */
// #ifdef USING_MECANUM_WHEEL
//     float cos_theta_0 = 0; // 单位：弧度
//     arm_atan2_f32(VEHICLE_LONG, VEHICLE_WIDTH, &cos_theta_0);
//     cos_theta_0 = cosf(cos_theta_0 - 45.0f * DAGTORAD);
//     coefficient = PI / 1800.0f * sqrtf(VEHICLE_WIDTH * VEHICLE_WIDTH + VEHICLE_LONG * VEHICLE_LONG) / 2.0f / cos_theta_0;
//     /*PI / 1800.0f为单位换算，将0.1°/s转化为rad/s*/
// #elif defined USING_OMNI_WHEEL
//     coefficient = PI / 1800.0f * (float)(VEHICLE_DIAMETER / 2.0f);
// #endif
// }
// #if defined(USING_MECANUM_WHEEL) || defined(USING_OMNI_WHEEL)
// /**
//  * @brief 麦轮/全向底盘逆运动解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方 输出顺序RF-LF-LB-RB
//  * @param {Mot_base_t} target_xyw 底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
//  * @param {单位说明} 输入单位：平动速度mm/s,w为0.1°/s，输出单位output[]：车轮(或者说轮毂)的线速度mm/s
//  * @param {float} output 底盘四个车轮(或者说轮毂)的期望线速度
//  */
// void MecanOmni_Resolve(Mot_base_t target_xyw, float output[])
// {
//     /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
//     output[0] = ROOT2 * (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * coefficient;
//     output[1] = ROOT2 * (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * coefficient;
//     output[2] = ROOT2 * (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * coefficient;
//     output[3] = ROOT2 * (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * coefficient;
// }

// void MecanOmni_Resolve_2(Mot_base_t target_xyw, float output[])
// {
//     static float k_w[4]={(VEHICLE_LONG+VEHICLE_WIDTH)/2.0,(VEHICLE_LONG+VEHICLE_WIDTH)/2.0,(VEHICLE_LONG+VEHICLE_WIDTH)/2.0,(VEHICLE_LONG+VEHICLE_WIDTH)/2.0};
//     k_w/10*DAGTORAD
//     /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
//     output[0] = (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * coefficient;
//     output[1] = (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * coefficient;
//     output[2] = (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * coefficient;
//     output[3] = (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * coefficient;
// }

// /**
//  * @brief 麦轮/全向轮底盘正运动解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方
//  * @param {坐标系}底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
//  * @param {float} encoderData[4] 编码器得到的电机的数据，快转子的转速(rpm)
//  * @param {Mot_base_t} *real_xyw 解算出来的车身的速度
//  * @return real_xyw->vy 单位m/s real_xyw->vx 单位mm/s  real_xyw->angvel单位0.1°/s
//  */
// void fromEncoderGetV(const float* const encoderData, Mot_base_t *real_xyw)
// {
//     float data_temp[4];

//     for (int i = 0; i < 4; i++)
//     {
//         // 单位mm/s      计算过程：快转子rpm -> 快转子rad/s -> 输出轴rad/s -> 轮子mm/s
//         data_temp[i] = encoderData[i] * RPMTORAD / REDUCT_RATIO * WCONVERT2V_k;
//     }
//     // 单位mm/s
//     real_xyw->vel.y = (data_temp[WHEEL_RF] + data_temp[WHEEL_LF] + data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / ROOT2 / 4.0f;
//     real_xyw->vel.x = (-data_temp[WHEEL_RF] + data_temp[WHEEL_LF] - data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / ROOT2 / 4.0f;
//     // 单位rad/s ->0.1°/s
//     real_xyw->angvel = (data_temp[WHEEL_RF] - data_temp[WHEEL_LF] - data_temp[WHEEL_LB] + data_temp[WHEEL_RB]) / 4.0f / coefficient;
// }
// #endif  // USING_MECANUM_WHEEL

// #ifdef USING_STEER_WHEEL

// typedef struct
// {
//     VectorXY_Str pos; // 轮组位置，单位(mm,mm)
//     float now_dir;   // 方向舵当前的角度值，单位0.1°
// } StWheel_info_t;

// /* 返回的转向角范围为(-180°,180°]，0°为车体坐标系的y轴，角度值增大方向为z轴旋转正方向*/
// static float Get_Vector3_theta(VectorXY_Str *temp)
// {
//     if (temp->x || temp->y)
//         return atan2f(-temp->x, temp->y) * RADTODAG;
//     else
//     return 0;
// }
// /*舵轮解算时需要的参数 单位mm,每个轮组相对于中心的位置，右前为正*/
// static const StWheel_info_t steerPos[WHEELS_NUM] =
//     {
//         [WHEEL_RF] = {{VEHICLE_WIDTH / 2, VEHICLE_LONG / 2}, 0},
//         [WHEEL_LF] = {{-VEHICLE_WIDTH / 2, VEHICLE_LONG / 2}, 0},
//         [WHEEL_LB] = {{-VEHICLE_WIDTH / 2, -VEHICLE_LONG / 2}, 0},
//         [WHEEL_RB] = {{VEHICLE_WIDTH / 2, -VEHICLE_LONG / 2}, 0},
// };

// static float range_1800(float angel)
// {
//     float tm = angel;
//     while (tm > 1800)
//         tm -= 3600;
//     while (tm < -1800)
//         tm += 3600;
//     return tm;
// }

// /*判断输入是否太小*/
// static rt_bool_t StWheel_If_Brake(Mot_base_t VxyW)
// {
//     if (fabsf(VxyW.vel.x) < 0.0001f && fabsf(VxyW.vel.y) < 0.0001f && fabsf(VxyW.angvel) < 0.0001f)
//         return RT_TRUE;
//     else
//         return RT_FALSE;
// }

// /**
//  * @brief 计算一个舵轮组的速度矢量
//  * @param {float} angvel 输入单位为0.1°/s
//  * @param {m_Vector2_t} pos 单位mm
//  * @return {*} 输出速度单位为mm/s
//  */

// /// @brief 计算一个舵轮组的速度矢量
// /// @param angvel 输入单位为0.1°/s
// /// @param pos 单位mm
// /// @return 输出速度单位为mm/s
// VectorXY_Str StWheel_Cross(float angvel, VectorXY_Str pos)
// {
//     VectorXYZ_Str vector_pos;
//     VectorXYZ_Str vector_v;
//     VectorXYZ_Str vector_w = {.x = 0, .y = 0, .z = angvel};
//     VectorXY_Str vector_out;
//     Vector_2To3_Get(&pos, 1,&vector_pos);                   //将每个轮组位置量转换为三维向量
//     Vector3D_CrossProd(&vector_w, &vector_pos,&vector_v);   //计算该轮组由自转角速度投射的速度矢量
//     Vector_3To2_Get(&vector_v, PI / 1800.0f,&vector_out);   // 进行单位换算
//     return vector_out;
// }
// // typedef struct{
// //     VectorXY_Str xy;
// //     float A;        // 矢量大小
// //     float xy_theta; // 二维矢量偏角,范围为(-180°,180°]，0°为坐标系的y轴，角度值增大方向为z轴旋转正方向

// // }m_Vector3_t;

// /*两个输入的角度单位均为0.1°/s 范围为-1800-1800*/
// static VectorXYA_Str StWheel_Dir_Limit(VectorXYA_Str output, float nowangel)
// {
//     float now, target, delta_angle1;
//     now = range_1800(nowangel);
//     target = range_1800(output.theta);
//     delta_angle1 = range_1800(target - now); // 限制为180度以内

//     //  单次最大打角理论上应该为90度，单位0.1°,>0
//     if (fabsf(delta_angle1) > DIR_ANGLE_LIM)
//     {
//         output.A = -output.A;
//         output.theta += 1800.0f;
//     }
//     return output;
// }

// /*debug*/
// float getNowAngel[4];
// float resolveData1[4];
// float resolveData2[4];
// float fixData1[4];
// float fixData2[4];
// Mot_base_t SteerWheeljscope;
// float angle_temp[4]={0};
// /**
//  * @brief 舵轮运动逆解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方 先是角度舵轮,后是驱动轮，四个同类型轮子的输出顺序RF-LF-LB-RB
//  * @param {SteerWheel_t} target_Vxyw 底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
//  * @param {单位说明} 输入单位：平动速度mm/s,角度w为0.1°/s
//  * @param {float} output[0][] 舵电机的的期望角度 当为0时即为正前方 单位0.1°
//  *@param {float} output[1][] output[][0] 底盘四个车轮(或者说轮毂)的期望线速度 单位mm/s
//  */
// void SteerWheel_Resolve(Mot_base_t target_Vxyw, float output[][4])
// {
//     VectorXYA_Str wheel_3v;

//     // 计算得到每个轮组对应的速度矢量
//     for (int i = 0; i < WHEELS_NUM; i++)
//     {
//         //debug
//         //SteerWheeljscope = target_Vxyw;
//         /*计算该轮组由自转角速度投射的速度矢量 单位mm/s */
//         VectorXY_Str wheel_2v = StWheel_Cross(target_Vxyw.angvel, steerPos[i].pos);

//         /*两种速度矢量叠加*/
//         VectorXY_Str temp_xy;
//         Vector2D_Add(&wheel_2v, (VectorXY_Str *)&target_Vxyw.vel,&temp_xy);

//         /*得到合速度矢量的角度 转向角范围为(-180°,180°] 逆时针转动为正*/
//         wheel_3v.xy = temp_xy;
//         wheel_3v.theta=Get_Vector3_theta(&temp_xy);
//         Vector2D_Mod(&wheel_3v.xy,&wheel_3v.A);

//         /*单位转化°->0.1°*/
//         wheel_3v.theta = 10.0f*wheel_3v.theta;

//         //debug
//         //resolveData1[i] = wheel_3v.xy_theta;
//         //resolveData2[i] = wheel_3v.A;

//         wheel_3v = StWheel_Dir_Limit(wheel_3v, getNowAngel[i] = getTargetFixAngle((Wheel_local_e)i) * 10);

//         //debug
//         // fixData1[i] = wheel_3v.xy_theta;
//         // fixData2[i] = wheel_3v.A;
//         if(wheel_3v.A>=10.0||wheel_3v.A<=-10.0)//速度大于10mm/s时，刷新角度，否则保持原角度
//         {
//             angle_temp[i] = wheel_3v.theta;
//         }

//         // 放在里面便于debug
//         //todo
//         // if (StWheel_If_Brake(target_Vxyw) == RT_TRUE)
//         // {
//         //     output[STEER_WHEEL_ANGLE][i] = angle_temp[i];
//         //     output[STEER_WHEEL_RUN][i] = 0;
//         // }
//         if (wheel_3v.A<=1.0&&wheel_3v.A>=-1.0&&StWheel_If_Brake(target_Vxyw) == RT_TRUE)
//         {
//             output[STEER_WHEEL_ANGLE][i] = angle_temp[i];
//             output[STEER_WHEEL_RUN][i] = 0;
//         }
//         else
//         {
//             output[STEER_WHEEL_ANGLE][i] = angle_temp[i];
//             output[STEER_WHEEL_RUN][i] = wheel_3v.A;
//         }
//     }
// }

// // 得到纯解算的角度和速度
// void getRawResolveData(float output[][4])
// {
//     for (int i = 0; i < WHEELS_NUM; i++)
//     {
//         output[0][i] = resolveData1[i];
//         output[1][i] = resolveData2[i];
//     }
// }
// /**
//  * @brief 舵轮正运动学解算 已知四个车轮的舵角和当前轮子的角速度，以此计算底盘的速度和角速度
//  * @param {float} theta 四个轮子的角度 单位弧度 顺序为RF-LF-LB-RB
//  * @param {float} w 四个轮子的角速度 单位rad/s 顺序为RF-LF-LB-RB
//  * @param {SteerWheel_t} *motion 解算出来的vx,vy,w 速度单位为mm/s 角速度单位为0.1°/s
//  */
// void steer_calVelocity(float theta[4], float w[4], SteerWheel_t *motion)
// {
//     float R = sqrtf(VEHICLE_LONG * VEHICLE_LONG + VEHICLE_WIDTH * VEHICLE_WIDTH) / 2000.f; // 计算底盘的半径

//     float A[4][2] = {{cosf(theta[0]), sinf(theta[0])},
//                      {cosf(theta[1]), sinf(theta[1])},
//                      {cosf(theta[2]), sinf(theta[2])},
//                      {cosf(theta[3]), sinf(theta[3])}}; // 构建A矩阵

//     float B[4] = {WHEEL_RADIUS * w[0] / 1000.f, WHEEL_RADIUS * w[1] / 1000.f,
//                   WHEEL_RADIUS * w[2] / 1000.f, WHEEL_RADIUS * w[3] / 1000.f}; // 构建B矩阵

//     float AT[2][4] = {{cosf(theta[0]), cosf(theta[1]),
//                        cosf(theta[2]), cosf(theta[3])},
//                       {sinf(theta[0]), sinf(theta[1]),
//                        sinf(theta[2]), sinf(theta[3])}}; // A矩阵的转置

//     float ATA[2][2] = {{0, 0}, {0, 0}}; // A矩阵的转置和A矩阵的乘积
//     float ATB[2] = {0, 0};              // A矩阵的转置和B矩阵的乘积

//     // 计算A矩阵的转置和A矩阵的乘积
//     for (int i = 0; i < 2; i++)
//     {
//         for (int j = 0; j < 4; j++)
//         {
//             ATA[i][0] += AT[i][j] * A[j][0];
//             ATA[i][1] += AT[i][j] * A[j][1];
//         }
//     }

//     // 计算A矩阵的转置和B矩阵的乘积
//     for (int i = 0; i < 2; i++)
//     {
//         for (int j = 0; j < 4; j++)
//         {
//             ATB[i] += AT[i][j] * B[j];
//         }
//     }

//     // 计算底盘的速度和角速度 速度单位mm/s 角速度单位0.1°/s
//     motion->vel.x = (ATB[0] * ATA[1][1] - ATB[1] * ATA[0][1]) /
//                     (ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]) * 1000.f;
//     motion->vel.y = (ATB[0] * ATA[1][0] - ATB[1] * ATA[0][0]) /
//                     (ATA[0][1] * ATA[1][0] - ATA[0][0] * ATA[1][1]) * 1000.f;
//     motion->angvel = (WHEEL_RADIUS / (4 * R)) * (w[0] - w[1] + w[2] - w[3]) * RADTODAG * 10.f;
// }

// //舵轮底盘正运动学解算
// void fromEncoderGetV( float*  encoderData_temp, Mot_base_t *real_xyw)
// {
//     //用于记录底盘四个轮子的方位角（单位：rad）
//     static float Wheel_Azimuth[4]={RF_Wheel_Azimuth,LF_Wheel_Azimuth,LB_Wheel_Azimuth,RB_Wheel_Azimuth};
//     //TODO 底盘四个轮子的方位和顺序待定（需要实际调试查看效果）
//     float **encoderData=&encoderData_temp;
//     //底盘正运动学解算算法实现部分，单位为 mm/s 和 0.1°/s
//     for(int i=0;i<4;i++)
//     {
//         real_xyw->vel.x+=encoderData[STEER_WHEEL_RUN][i] * arm_cos_f32(encoderData[STEER_WHEEL_ANGLE][i])*WHEEL_RADIUS/4.0f;
//         real_xyw->vel.y+=encoderData[STEER_WHEEL_RUN][i] * arm_sin_f32(encoderData[STEER_WHEEL_ANGLE][i])*WHEEL_RADIUS/4.0f;
//         real_xyw->angvel+=RAD2DEG_f(encoderData[STEER_WHEEL_RUN][i] * arm_sin_f32(encoderData[STEER_WHEEL_ANGLE][i]-Wheel_Azimuth[i])*WHEEL_RADIUS/Wheel_To_Core_Distance/4.0f)*10;//先转化为弧度计算，再转化为角度计算，最后乘10转化为0.1°/s
//     }
// }

// #endif
