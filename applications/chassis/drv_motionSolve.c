#include "drv_motionSolve.h"
#include <arm_math.h>
#include "drv_Vector.h"

static float encodeData[4];
Mot_base_t real_xyw; // 正解算后的底盘真实运动情况

//********************************逆解算部分（全向）************************************** */
#ifdef USING_OMNI_RESOLVE
J4_3 J_Omni_Inv_45 = {0}; // 逆解算雅可比矩阵 (4x3)：轮速 = J_inv * 底盘速度(电机轴向为45度)
J3_4 J_Omni_Fwd_45 = {0}; // 正解算雅可比矩阵 (3x4)：底盘速度 = J_fwd * 轮速
J4_3 J_Omni_Inv_60 = {0}; // 逆解算雅可比矩阵 (4x3)：轮速 = J_inv * 底盘速度(电机轴向为60度)
J3_4 J_Omni_Fwd_60 = {0}; // 正解算雅可比矩阵 (3x4)：底盘速度 = J_fwd * 轮速

// 新改的逆解算（中科大）--全向  底盘的正方向为逆时针  电机顺序右前--左前-左后-右后
void Omni_Inverse_Kinematics_45_test(Mot_base_t target_xyw, float resolve_data[4])
{
    // 使用的单位均为标准单位
    float L = VEHICLE_LONG / 1000.0f;              // 转换为米
    float W = VEHICLE_WIDTH / 1000.0f;             // 转换为米
    float s = WHEEL_RADIUS / 1000.0f;              // mm -> m
    float r = VEHICLE_DIAMETER / 2;                // 转换为m
    float vx = target_xyw.vel.x / 1000.0f;         // mm/s -> m/s
    float vy = target_xyw.vel.y / 1000.0f;         // mm/s -> m/s
    float w = target_xyw.angvel * 0.1f * DEGTORAD; // °/s -> rad/s
    float root2_2 = ROOT2 / 2;
    resolve_data[0] = (root2_2 * vx + root2_2 * vy + w * r) / s;
    resolve_data[1] = (-root2_2 * vx + root2_2 * vy + w * r) / s;
    resolve_data[2] = (-root2_2 * vx - root2_2 * vy + w * r) / s;
    resolve_data[3] = (root2_2 * vx - root2_2 * vy + w * r) / s; // 单位是rad/s

    // 转换为电机的rpm
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = resolve_data[i] * RADTORPM;
    }
}

//

// 全向轮逆解算函数（现在已经改成了向前为x轴，向左为y轴）
//  输入：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
//  输出：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
void Omni_Inverse_Kinematics_45(Mot_base_t target_xyw, float resolve_data[4])
{
    /*
    雅可比矩阵形式：
    [v_RF]   [J11 J12 J13] [vx]
    [v_LF] = [J21 J22 J23] * [vy]
    [v_LB]   [J31 J32 J33] [ω]
    [v_RB]   [J41 J42 J43]

    输出单位为mm/s，顺序：RF-LF-LB-RB
    */
    static int if_init_J = 0;
    if (!if_init_J)
    {
        // 初始化雅可比矩阵
        float L = VEHICLE_LONG / 1000.0f;  // 转换为米
        float W = VEHICLE_WIDTH / 1000.0f; // 转换为米
        float r = WHEEL_RADIUS / 1000.0f;  // mm -> m

        // float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        // float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        // float alpha_deg[4] = {120.0f, 60.0f, 120.0f, 60.0f}; // 135 45 135 45
        // for (int i = 0; i < 4; i++)
        // {
        //     float a = alpha_deg[i] * DEGTORAD;
        //     J_Omni_Inv_45.data[i][0] = cosf(a) / r;                            // vx系数      -6.25
        //     J_Omni_Inv_45.data[i][1] = sinf(a) / r;                            // vy系数      10.625
        //     J_Omni_Inv_45.data[i][2] = (-cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        // }
        // if_init_J = 1;

        float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        float alpha_deg[4] = {45.0f, 135.0f, 45.0f, 135.0f}; // 135 45 135 45
        for (int i = 0; i < 4; i++)
        {
            float a = alpha_deg[i] * DEGTORAD;
            J_Omni_Inv_45.data[i][0] = sin(a) / r;                            // vx系数      -6.25
            J_Omni_Inv_45.data[i][1] = cos(a) / r;                            // vy系数      10.625
            J_Omni_Inv_45.data[i][2] = (cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        }
        if_init_J = 1;
    }
    // 将目标速度从mm/s转换为m/s（内部计算用国际单位）
    float vx = target_xyw.vel.x / 1000.0f;         // mm/s -> m/s
    float vy = target_xyw.vel.y / 1000.0f;         // mm/s -> m/s
    float ω = target_xyw.angvel * 0.1f * DEGTORAD; // °/s -> rad/s

    // 使用雅可比矩阵计算轮子角速度（rad/s）
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = (J_Omni_Inv_45.data[i][0] * vx +
                           J_Omni_Inv_45.data[i][1] * vy +
                           J_Omni_Inv_45.data[i][2] * ω);
    }
    // 转换为rpm
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = resolve_data[i] * RADTORPM;
    }
}

// 全向轮逆解算函数  （现在已经改成了向前为x轴，向左为y轴）
// 输入：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
// 输出：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
void Omni_Inverse_Kinematics_60(Mot_base_t target_xyw, float resolve_data[4])
{
    /*
    雅可比矩阵形式：
    [v_RF]   [J11 J12 J13] [vx]
    [v_LF] = [J21 J22 J23] * [vy]
    [v_LB]   [J31 J32 J33] [ω]
    [v_RB]   [J41 J42 J43]

    输出单位为mm/s，顺序：RF-LF-LB-RB
    */
    static int if_init_J = 0;
    if (!if_init_J)
    {
        // 初始化雅可比矩阵
        float L = VEHICLE_LONG / 1000.0f;  // 转换为米
        float W = VEHICLE_WIDTH / 1000.0f; // 转换为米
        float r = WHEEL_RADIUS / 1000.0f;  // mm -> m

        // float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        // float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        // float alpha_deg[4] = {120.0f, 60.0f, 120.0f, 60.0f}; // 135 45 135 45
        // for (int i = 0; i < 4; i++)
        // {
        //     float a = alpha_deg[i] * DEGTORAD;
        //     J_Omni_Inv_60.data[i][0] = cosf(a) / r;                            // vx系数      -6.25
        //     J_Omni_Inv_60.data[i][1] = sinf(a) / r;                            // vy系数      10.625
        //     J_Omni_Inv_60.data[i][2] = (-cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        // }
        // if_init_J = 1;

        float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        float alpha_deg[4] = {30.0f, 150.0f, 30.0f, 150.0f}; // 135 45 135 45
        for (int i = 0; i < 4; i++)
        {
            float a = alpha_deg[i] * DEGTORAD;
            J_Omni_Inv_60.data[i][0] = sin(a) / r;                            // vx系数      -6.25
            J_Omni_Inv_60.data[i][1] = cos(a) / r;                            // vy系数      10.625
            J_Omni_Inv_60.data[i][2] = (cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        }
        if_init_J = 1;
    }
    // 将目标速度从mm/s转换为m/s（内部计算用国际单位）
    float vx = target_xyw.vel.x / 1000.0f;         // mm/s -> m/s
    float vy = target_xyw.vel.y / 1000.0f;         // mm/s -> m/s
    float ω = target_xyw.angvel * 0.1f * DEGTORAD; // °/s -> rad/s

    // 使用雅可比矩阵计算轮子角速度（rad/s）
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = (J_Omni_Inv_60.data[i][0] * vx +
                           J_Omni_Inv_60.data[i][1] * vy +
                           J_Omni_Inv_60.data[i][2] * ω);
    }
    // 转换为rpm
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = resolve_data[i] * RADTORPM;
    }
}
//********************************正解算部分（全向）************************************** */

void Omni_Forward_Kinematics(float speed[4])
{
    real_xyw.vel.x = 0.0f;
    real_xyw.vel.y = 0.0f;
    real_xyw.angvel = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        // 单位mm / s      计算过程：快转子rpm->快转子rad / s->输出轴rad / s->轮子mm / s
        encodeData[i] = RPMTORAD * WHEEL_RADIUS * speed[i];
    }
    // 各个轮组解算的常量系数，按照不同的种类进行划分     该系数考虑距离，角度转弧度，角速度0.1度/s的因素，最终将轮向电机速度直接除以改系数得到角速度值
    static float coefficient_OMNI = VEHICLE_DIAMETER / 2; // 底盘半径 mm
    // 三个方向速度的符号，顺序为[3]:y,x,w,[WHEELS_GROUP_NUM]:右前,左前，左后，右后，适用于全向轮和麦克纳姆轮，不适用于舵轮
    // static int sign[3][4] = {{1, 1, 1, 1}, {-1, 1, -1, 1}, {1, -1, -1, 1}};
    static int sign[3][4] = {{1, 1, 1, 1}, {1, -1, 1, -1}, {1, -1, -1, 1}};
    for (int i = 0; i < 4; i++)
    {
        // 单位mm/s
        real_xyw.vel.x += encodeData[i] * ROOT2 / 4.0f * sign[0][i]; // 底盘的速度单位是mm
        real_xyw.vel.y += encodeData[i] * ROOT2 / 4.0f * sign[1][i];
        // 单位rad/s ->0.1°/s
        real_xyw.angvel += encodeData[i] / 4.0f / coefficient_OMNI * sign[2][i] * RADTODEG * 10.0f;
    }
}

// 新的正解算 中科大
void Omni_Forward_Kinematics_test(float speed[4])
{
    real_xyw.vel.x = 0.0f;
    real_xyw.vel.y = 0.0f;
    real_xyw.angvel = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        // 单位mm / s      计算过程：快转子rpm->快转子rad / s->输出轴rad / s->轮子mm / s
        encodeData[i] = RPMTORAD * WHEEL_RADIUS * speed[i];
    }
    static float coefficient_OMNI = VEHICLE_DIAMETER / 2; // 底盘半径 mm
    static int sign[3][4] = {{1, -1, -1, 1}, {1, 1, -1, -1}, {1, 1, 1, 1}};

    for (int i = 0; i < 4; i++)
    {
        // 单位mm/s
        real_xyw.vel.x += encodeData[i] * ROOT2 / 4.0f * sign[0][i]; // 底盘的速度单位是mm
        real_xyw.vel.y += encodeData[i] * ROOT2 / 4.0f * sign[1][i];
        // 单位rad/s ->0.1°/s
        real_xyw.angvel += encodeData[i] / 4.0f / coefficient_OMNI * sign[2][i] * RADTODEG * 10.0f;
    }
}

#endif

//********************************逆解算部分（舵轮）************************************** */
#define USING_STEER_RESOLVE

// 舵轮正逆解算
#ifdef USING_STEER_RESOLVE

// 舵轮逆解算
void Steer_Inverse_Kinematics_test(Mot_base_t target_xyw, float resolve_data_run[4], float resolve_data_steer[4])
{
    // 使用的单位均为标准单位 
    static float s = WHEEL_RADIUS;
    float vx = target_xyw.vel.x;         // mm/s -> m/s
    float vy = target_xyw.vel.y;         // mm/s -> m/s
    float w = target_xyw.angvel * 0.1f * DEGTORAD; // °/s -> rad/s

    static float rx[4]={VEHICLE_LONG/2,VEHICLE_LONG/2,-VEHICLE_LONG/2,-VEHICLE_LONG/2};
		static float ry[4]={-VEHICLE_WIDTH/2,VEHICLE_WIDTH/2,VEHICLE_WIDTH/2,-VEHICLE_WIDTH/2};
    static float wx[4]={0,0,0,0},wy[4]={0,0,0,0};
    static float angle_steer_last[4] = {0};
    static float delta[4];
    for (int i = 0; i < 4; i++)
    {
        // wx[i] = (vx - root2_2 * r*w) / s; // rad/s
        // wy[i] = (vy + root2_2 * r*w) / s;
        
        wx[i] = (vx - ry[i]*w) / s; // rad/s
        wy[i] = (vy + rx[i]*w) / s;
        //resolve_data_steer[i] = atan2f(wy[i], wx[i]) * RADTODEG; // 舵向电机设定的角度（单位是°）返回的角度是-180°到180°
        resolve_data_steer[i] = atan2f(wy[i], wx[i]) * RADTODEG; // 舵向电机设定的角度（单位是°）返回的角度是-180°到180°

        delta[i] = resolve_data_steer[i] - angle_steer_last[i];
        if (fabsf(delta[i]) <= 90.0f)
        {
            resolve_data_run[i] = hypot(wx[i], wy[i]) * RADTORPM; // 取模，得到轮向电机速度（单位为rpm)
        }
        else if (delta[i] >= 90.0f)
        {
            resolve_data_run[i] = -hypot(wx[i], wy[i]) * RADTORPM;  // 取模，得到轮向电机速度（单位为rpm)
            resolve_data_steer[i] = resolve_data_steer[i] - 180.0f; // 角度
        }
        else if (delta[i] <= -90.0f)
        {
            resolve_data_run[i] = -hypot(wx[i], wy[i]) * RADTORPM;  // 取模，得到轮向电机速度（单位为rpm)
            resolve_data_steer[i] = 180.0f + resolve_data_steer[i]; // 角度
        }
        angle_steer_last[i] = resolve_data_steer[i]; // 记录保存上次舵向设定值
    }
}

// 舵轮正解算

void Steer_Forward_Kinematics_test(float wheel_rpm[4], Mot_base_t *chassis_vel)
{
}

// // 舵轮解算
// #define DIR_ANGLE_LIM (900.f) // 单次最大打角理论上应该为90度，单位0.1°,>0

// static float range_1800(float angel)
// {
//     float tm = angel;
//     while (tm > 1800)
//         tm -= 3600;
//     while (tm < -1800)
//         tm += 3600;
//     return tm;
// }
// /*两个输入的角度单位均为0.1°/s 范围为-1800-1800*/
// static VectorXYA_Str StWheel_Dir_Limit(VectorXYA_Str output, float nowangel)
// {
//     float now, target, delta_angle1;
//     now = range_1800(nowangel);
//     target = range_1800(output.theta);
//     delta_angle1 = range_1800(target - now); // 限制为180度以内

//     //  单次最大打角理论上应该为90度，单位0.1°,>0
//     if (delta_angle1 > DIR_ANGLE_LIM)
//     {
//         output.A = -output.A;
//         output.theta -= 1800.0f;
//     }
//     else if (delta_angle1 < -DIR_ANGLE_LIM)
//     {
//         output.A = -output.A;
//         output.theta += 1800.0f;
//     }
//     return output;
// }
// /// <summary>
// /// 舵轮解算改进版,要求底盘轮系所在位置必须为正方形底盘
// /// </summary>
// static void SteerWheel_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float *resolve_data)
// {
//     // 容易理解的系数，对计算不友好
//     // static float k_w = ROOT2 * VEHICLE_LONG / 20.f * DEGTORAD;
//     // static float root2_2 = ROOT2 / 2.f;
//     // 将系数合在一起后的系数，方便计算
//     static float k_w = VEHICLE_LONG / 20.f * DEGTORAD;
//     static float angle_temp[WHEELS_GROUP_NUM] = {0, 0, 0, 0}; // 保存舵向电机的角度数据
//     VectorXYA_Str temp_v = {0};
//     /*输出单位为mm/s 输出顺序RF-LF-LB-RB  先计算每个轮组的x和y方向的速度分量*/
//     if (wheel_group == WHEEL_RF)
//     {
//         temp_v.xy.x = (target_xyw.vel.x - target_xyw.angvel * k_w);
//         temp_v.xy.y = (target_xyw.vel.y + target_xyw.angvel * k_w);
//     }
//     else if (wheel_group == WHEEL_LF)
//     {
//         temp_v.xy.x = (target_xyw.vel.x - target_xyw.angvel * k_w);
//         temp_v.xy.y = (target_xyw.vel.y - target_xyw.angvel * k_w);
//     }
//     else if (wheel_group == WHEEL_LB)
//     {
//         temp_v.xy.x = (target_xyw.vel.x + target_xyw.angvel * k_w);
//         temp_v.xy.y = (target_xyw.vel.y - target_xyw.angvel * k_w);
//     }
//     else if (wheel_group == WHEEL_RB)
//     {
//         temp_v.xy.x = (target_xyw.vel.x + target_xyw.angvel * k_w);
//         temp_v.xy.y = (target_xyw.vel.y + target_xyw.angvel * k_w);
//     }
//     else
//     {
//         return;
//     }
//     // 后计算轮向速度大小和舵向角度
//     Vector2D_Mod(&temp_v.xy, &temp_v.A);
//     temp_v.theta = atan2f(-temp_v.xy.x, temp_v.xy.y) * RADTODAG * 10;
//     // 判断舵向角度是否大于90°，设置最优转动方向和速度方向
//     // wheel_3v = StWheel_Dir_Limit(wheel_3v, getNowAngel[i] = getNowFixAngle((Wheel_local_e)i) * 10);
//     temp_v = StWheel_Dir_Limit(temp_v, 0);
//     // 后续优化处理
//     // 速度大于10mm/s时，刷新角度，否则保持原角度，目的是解决停车的时候舵轮还归零的情况
//     if (temp_v.A >= 10.0 || temp_v.A <= -10.0)
//     {
//         angle_temp[wheel_group] = temp_v.theta;
//     }
//     // 速度太小，认为停车，设置角度为之前的保留的角度，速度设置为0
//     if (temp_v.A <= 1.0 && temp_v.A >= -1.0)
//     {
//         temp_v.A = 0;
//         temp_v.theta = angle_temp[wheel_group];
//     }
//     else
//     {
//         temp_v.theta = angle_temp[wheel_group];
//     }
//     // 将解算结果设置到解算数据中
//     resolve_data[WHEEL_RUN] = temp_v.A;
//     resolve_data[WHEEL_ANGLE] = temp_v.theta;
// }
// /**
//  * @brief 底盘运动解算函数，输入底盘的速度和角速度，输出各个电机的速度
//  *      * 该函数会根据底盘的轮系类型进行不同的解算
//  *      * 向右为x轴正方向，向前为y轴正方向，底盘角速度为正表示底盘逆时针旋转
//  * @param Vxyw 速度和角速度，单位mm/s和0.1°/s
//  */
// void Motion_Resolve(Mot_base_t *Vxyw, float motorSetData[4][2])
// {
//     for (int group = 0; group < WHEELS_GROUP_NUM; group++)
//     {
//         if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
//         {
//             Mecan_Resolve(*Vxyw, group, motorSetData[group]);
//         }
//         else if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
//         {
//             Omni_Resolve(*Vxyw, group, motorSetData[group]);
//         }
//         else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
//         {
//             SteerWheel_Resolve(*Vxyw, group, motorSetData[group]);
//         }
//         else
//             return;
//     }
// }
#endif

//********************************逆解算部分（麦轮）************************************** */

// todo:舵轮解算没有改

#ifdef USING_MECANUM_RESOLVE

void Mecanum_Forward_Kinematics_test(float wheel_rpm[4], Mot_base_t *chassis_vel)
{
}
//********************************正解算部分（麦轮）************************************** */

void Mecanum_Inverse_Kinematics_test(Mot_base_t target_xyw, float resolve_data[4])
{
}
#endif
// 返回正解算后底盘的运动状态
Mot_base_t Get_realmotion(void)
{
    return real_xyw;
}
