#include "drv_motionSolve.h"
#include <arm_math.h>
#include "drv_Vector.h"

// static float Wheel_Azimuth[WHEELS_GROUP_NUM] = {RF_Wheel_Azimuth, LF_Wheel_Azimuth, LB_Wheel_Azimuth, RB_Wheel_Azimuth};

//********************************逆解算部分************************************** */
// #ifdef USING_MECANUM_RESOLVE
J4_3 J_Mecanum_Inv = {0}; // 逆解算雅可比矩阵 (4x3)：轮速 = J_inv * 底盘速度
J3_4 J_Mecanum_Fwd = {0}; // 正解算雅可比矩阵 (3x4)：底盘速度 = J_fwd * 轮速

// 麦克纳姆轮正解算函数
// 输入：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
// 输出：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
void Mecanum_Forward_Kinematics(float wheel_rpm[4], Mot_base_t *chassis_vel)
{
    if (chassis_vel == NULL)
        return;
    // 初始化雅可比矩阵
    // 计算正解算雅可比矩阵 (3x4) ==========
    // 正解算：底盘速度 = J_fwd * 轮子线速度
    // J_fwd 是 J_inv 的伪逆矩阵

    // 对于麦克纳姆轮，我们可以直接推导出解析解
    // 根据运动学方程：
    // v_RF = (vx - vy) + ω * k
    // v_LF = (vx + vy) - ω * k
    // v_LB = (vx - vy) - ω * k
    // v_RB = (vx + vy) + ω * k

    // 解这个方程组得到正解算公式：
    // vx = (v_LF + v_RB + v_RF + v_LB) / 4
    // vy = (v_LF + v_RB - v_RF - v_LB) / 4
    // ω = (v_RF - v_LF + v_RB - v_LB) / (4k)

    // 计算伪逆矩阵（简化方法）
    // 对于超定系统，伪逆 = (J^T * J)^(-1) * J^T
    static int if_init_J = 0;
    if (!if_init_J)
    {
        // 方法1：直接使用推导的解析公式
        float L = VEHICLE_LONG / 1000.0f;  // 转换为米
        float W = VEHICLE_WIDTH / 1000.0f; // 转换为米
        float r = WHEEL_RADIUS / 1000.0f;  // mm -> m
        float alpha = 45.0f * DEGTORAD;    // 麦克纳姆轮辊子角度

        // 计算基础系数
        float base_k = (L + W) / (2.0f * 1.41421356f);
        float k = base_k; // 旋转半径

        // 正解算矩阵 (3x4)
        J_Mecanum_Fwd.data[0][0] = 1.0f / 4.0f; // vx 对 RF 轮的系数
        J_Mecanum_Fwd.data[0][1] = 1.0f / 4.0f; // vx 对 LF 轮的系数
        J_Mecanum_Fwd.data[0][2] = 1.0f / 4.0f; // vx 对 LB 轮的系数
        J_Mecanum_Fwd.data[0][3] = 1.0f / 4.0f; // vx 对 RB 轮的系数

        J_Mecanum_Fwd.data[1][0] = -1.0f / 4.0f; // vy 对 RF 轮的系数
        J_Mecanum_Fwd.data[1][1] = 1.0f / 4.0f;  // vy 对 LF 轮的系数
        J_Mecanum_Fwd.data[1][2] = -1.0f / 4.0f; // vy 对 LB 轮的系数
        J_Mecanum_Fwd.data[1][3] = 1.0f / 4.0f;  // vy 对 RB 轮的系数

        J_Mecanum_Fwd.data[2][0] = 1.0f / (4.0f * k);  // ω 对 RF 轮的系数
        J_Mecanum_Fwd.data[2][1] = -1.0f / (4.0f * k); // ω 对 LF 轮的系数
        J_Mecanum_Fwd.data[2][2] = -1.0f / (4.0f * k); // ω 对 LB 轮的系数
        J_Mecanum_Fwd.data[2][3] = 1.0f / (4.0f * k);  // ω 对 RB 轮的系数
        if_init_J = 1;
    }
    // 将RPM转换为线速度(m/s)
    float wheel_speed_mps[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        // RPM -> mm/s -> m/s
        float speed_mmps = wheel_rpm[i] * RPMTORAD * WHEEL_RADIUS; // mm/s
        wheel_speed_mps[i] = speed_mmps / 1000.0f;                 // m/s
    }
    // 使用正解算雅可比矩阵计算底盘速度
    // [vx]   [J11 J12 J13 J14] [v_RF]
    // [vy] = [J21 J22 J23 J24] * [v_LF]
    // [ω]    [J31 J32 J33 J34] [v_LB]
    //                            [v_RB]
    float vx_mps = 0, vy_mps = 0, omega_radps = 0;

    // 方法1：使用矩阵乘法
    for (int i = 0; i < 4; i++)
    {
        vx_mps += J_Mecanum_Fwd.data[0][i] * wheel_speed_mps[i];
        vy_mps += J_Mecanum_Fwd.data[1][i] * wheel_speed_mps[i];
        omega_radps += J_Mecanum_Fwd.data[2][i] * wheel_speed_mps[i];
    }
    // 单位转换：m/s -> mm/s, rad/s -> °/s
    chassis_vel->vel.x = vx_mps * 1000.0f;
    chassis_vel->vel.y = vy_mps * 1000.0f;
    chassis_vel->angvel = omega_radps * RADTODEG;
}

// 麦克纳姆轮正解算函数
//  输入：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
//  输出：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
void Omni_Forward_Kinematics_120(float wheel_rpm[4], Mot_base_t *chassis_velocity)
{
    /*
    对于120°/60° X型麦克纳姆轮，解析解：
    vx = r * (-0.5*ω1 + 0.5*ω2 - 0.5*ω3 + 0.5*ω4)
    vy = r * (√3/2*ω1 + √3/2*ω2 + √3/2*ω3 + √3/2*ω4)
    ω = k * (ω1 - ω2 + ω3 - ω4)

    其中k = r / (L/2 + W/2)
    */

    static float sqrt3_2 = ROOT3 / 2; // √3/2
    static float k_omega = 0.0f;

    static int if_init = 0;
    if (!if_init)
    {
        float L = VEHICLE_LONG / 1000.0f;
        float W = VEHICLE_WIDTH / 1000.0f;
        float r = WHEEL_RADIUS / 1000.0f;
        k_omega = r / ((L + W) * 0.5f);
        if_init = 1;
    }

    // RPM转换为rad/s
    float omega[4];
    for (int i = 0; i < 4; i++)
    {
        omega[i] = wheel_rpm[i] * RPMTORAD;
    }

    // 计算底盘速度（m/s, rad/s）
    float vx = (omega[1] + omega[3] - omega[0] - omega[2]) * 0.5f * (WHEEL_RADIUS / 1000.0f);
    float vy = (omega[0] + omega[1] + omega[2] + omega[3]) * 0.5f * sqrt3_2 * (WHEEL_RADIUS / 1000.0f);
    float w = (omega[0] - omega[1] + omega[2] - omega[3]) * k_omega;

    // 单位转换
    chassis_velocity->vel.x = vx * 1000.0f;         // m/s -> mm/s
    chassis_velocity->vel.y = vy * 1000.0f;         // m/s -> mm/s
    chassis_velocity->angvel = w / DEGTORAD / 0.1f; // rad/s -> 0.1°/s
}

// 全轮逆解算函数
// 输入：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
// 输出：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
void Omni_Inverse_Kinematics_120(Mot_base_t target_xyw, float resolve_data[4])
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
        //     J_Mecanum_Inv.data[i][0] = cosf(a) / r;                            // vx系数      -6.25
        //     J_Mecanum_Inv.data[i][1] = sinf(a) / r;                            // vy系数      10.625
        //     J_Mecanum_Inv.data[i][2] = (-cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        // }
        // if_init_J = 1;

        float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        float alpha_deg[4] = {30.0f, 150.0f, 30.0f, 150.0f}; // 135 45 135 45
        for (int i = 0; i < 4; i++)
        {
            float a = alpha_deg[i] * DEGTORAD;
            J_Mecanum_Inv.data[i][0] = sin(a) / r;                            // vx系数      -6.25
            J_Mecanum_Inv.data[i][1] = cos(a) / r;                            // vy系数      10.625
            J_Mecanum_Inv.data[i][2] = (cosf(a) * y[i] - sinf(a) * x[i]) / r; // ω系数       0.94
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
        resolve_data[i] = (J_Mecanum_Inv.data[i][0] * vx +
                           J_Mecanum_Inv.data[i][1] * vy +
                           J_Mecanum_Inv.data[i][2] * ω);
    }
    // 转换为rpm
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = resolve_data[i] * RADTORPM;
    }
}

// 麦克纳姆轮正解算函数
//  输入：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
//  输出：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
// 麦克纳姆轮正解算函数（45°标准型）
// 输入：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
// 输出：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
void Omni_Forward_Kinematics_45(float wheel_rpm[4], Mot_base_t *chassis_velocity)
{
    /*
    对于45°/135°标准X型麦克纳姆轮，解析解：
    vx = (r/√2) * (ω1 - ω2 - ω3 + ω4)
    vy = (r/√2) * (ω1 + ω2 - ω3 - ω4)
    ω = (r/(L+W)) * (ω1 - ω2 + ω3 - ω4)

    其中：
    - r: 轮子半径 (m)
    - L: 车长 (m)
    - W: 车宽 (m)
    - ω1-ω4: 轮子角速度 (rad/s)，顺序：RF-LF-LB-RB
    */

    static float root2_2 = ROOT2 / 2; // √2/2
    static float k_omega = 0.0f;

    static int if_init = 0;
    if (!if_init)
    {
        float L = VEHICLE_LONG / 1000.0f;
        float W = VEHICLE_WIDTH / 1000.0f;
        float r = WHEEL_RADIUS / 1000.0f;
        k_omega = r / (L + W); // 注意：这里分母是(L+W)，不是(L+W)/2
        if_init = 1;
    }

    // RPM转换为rad/s
    float omega[4];
    for (int i = 0; i < 4; i++)
    {
        omega[i] = wheel_rpm[i] * RPMTORAD; // 假设RPMTORAD已定义
    }

    // 计算底盘速度（m/s, rad/s）
    float r_m = WHEEL_RADIUS / 1000.0f;

    // 使用标准麦克纳姆轮公式
    float vx = (omega[0] - omega[1] - omega[2] + omega[3]) * r_m * root2_2;
    float vy = (omega[0] + omega[1] - omega[2] - omega[3]) * r_m * root2_2;
    float w = (omega[0] - omega[1] + omega[2] - omega[3]) * k_omega;

    // 单位转换
    chassis_velocity->vel.x = vx * 1000.0f;         // m/s -> mm/s
    chassis_velocity->vel.y = vy * 1000.0f;         // m/s -> mm/s
    chassis_velocity->angvel = w / DEGTORAD / 0.1f; // rad/s -> 0.1°/s
}
// 全轮逆解算函数
// 输入：底盘运动速度 (vx, vy单位: mm/s, ω单位: °/s)
// 输出：四个轮子的转速 (RPM)，顺序：RF-LF-LB-RB
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
        //     J_Mecanum_Inv.data[i][0] = cosf(a) / r;                            // vx系数      -6.25
        //     J_Mecanum_Inv.data[i][1] = sinf(a) / r;                            // vy系数      10.625
        //     J_Mecanum_Inv.data[i][2] = (-cosf(a) * y[i] + sinf(a) * x[i]) / r; // ω系数       0.94
        // }
        // if_init_J = 1;

        float x[4] = {L / 2, -L / 2, -L / 2, L / 2};
        float y[4] = {W / 2, W / 2, -W / 2, -W / 2};
        float alpha_deg[4] = {45.0f, 135.0f, 45.0f, 135.0f}; // 135 45 135 45
        for (int i = 0; i < 4; i++)
        {
            float a = alpha_deg[i] * DEGTORAD;
            J_Mecanum_Inv.data[i][0] = sin(a) / r;                            // vx系数      -6.25
            J_Mecanum_Inv.data[i][1] = cos(a) / r;                            // vy系数      10.625
            J_Mecanum_Inv.data[i][2] = (cosf(a) * y[i] - sinf(a) * x[i]) / r; // ω系数       0.94
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
        resolve_data[i] = (J_Mecanum_Inv.data[i][0] * vx +
                           J_Mecanum_Inv.data[i][1] * vy +
                           J_Mecanum_Inv.data[i][2] * ω);
    }
    // 转换为rpm
    for (int i = 0; i < 4; i++)
    {
        resolve_data[i] = resolve_data[i] * RADTORPM;
    }
}
// /// <summary>全向轮解算，单个轮组(旧)
// ///
// /// </summary>
// /// <param name="target_xyw"></param>
// /// <param name="resolve_data"></param>
// void Omni_Resolve(Mot_base_t target_xyw, float *resolve_data)
// {
//     // 角速度单位为0.1°/s，所以角速度系数要除以10
//     static float k_w = VEHICLE_DIAMETER / 20.f * DEGTORAD;
//     static float root2_2 = ROOT2 / 2;
//     /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
//     resolve_data[0] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * k_w;
//     resolve_data[1] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * k_w;
//     resolve_data[2] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * k_w;
//     resolve_data[3] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * k_w;
// }
// #ifdef USING_OMNI_RESOLVE

// /// <summary>全向轮解算，单个轮组
// ///
// /// </summary>
// /// <param name="target_xyw"></param>
// /// <param name="resolve_data"></param>
// static void Omni_Resolve(Mot_base_t target_xyw, Wheel_Group_e wheel_group, float *resolve_data)
// {
//     // 角速度单位为0.1°/s，所以角速度系数要除以10
//     static float k_w = VEHICLE_DIAMETER / 20.f * DEGTORAD;
//   static float root2_2 = ROOT2 / 2;
//     /*输出单位为mm/s 输出顺序RF-LF-LB-RB*/
//     if (wheel_group == WHEEL_RF)
//     {
//         resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) + target_xyw.angvel * k_w;
//     }
//     else if (wheel_group == WHEEL_LF)
//     {
//         resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) - target_xyw.angvel * k_w;
//     }
//     else if (wheel_group == WHEEL_LB)
//     {
//         resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y - target_xyw.vel.x) - target_xyw.angvel * k_w;
//     }
//     else if (wheel_group == WHEEL_RB)
//     {
//         resolve_data[WHEEL_RUN] = root2_2 * (target_xyw.vel.y + target_xyw.vel.x) + target_xyw.angvel * k_w;
//     }
//     else
//     {
//         return;
//     }
// }

void Omni_Forward_Kinematics(float wheel_rpm[4], Mot_base_t *chassis_vel)
{
}

void Omni_Inverse_Kinematics(Mot_base_t target_xyw, float resolve_data[4])
{
}

// 舵轮正逆解算
#ifdef USING_STEER_RESOLVE

void Steer_Forward_Kinematics(float wheel_rpm[4], Mot_base_t *chassis_vel)
{
}
void Steer_Inverse_Kinematics(Mot_base_t target_xyw, float resolve_data[4])
{
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
    // static float k_w = ROOT2 * VEHICLE_LONG / 20.f * DEGTORAD;
    // static float root2_2 = ROOT2 / 2.f;
    // 将系数合在一起后的系数，方便计算
    static float k_w = VEHICLE_LONG / 20.f * DEGTORAD;
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
#endif
//********************************逆解算部分************************************** */
/**
 * @brief 全向轮的力解算，使用雅可比矩阵，解算后的结果作为电机的动力前馈
 *
 * @param tarForce
 * @param wheel_group
 * @param resolve_data
 */
// static void Omni_Force_Resolve(Mot_base_t tarForce, Wheel_Group_e wheel_group, float *resolve_data)
//{
//     static float root2_4 = ROOT2 / 4.f;
//     static float k_w = VEHICLE_DIAMETER / 2000.f;
//     static float inv_4kw = 1.f / (4.f * (VEHICLE_DIAMETER / 2000.f));
//     if (wheel_group == WHEEL_RF)
//     {
//         resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y - tarForce.vel.x) + tarForce.angvel * inv_4kw;
//     }
//     else if (wheel_group == WHEEL_LF)
//     {
//         resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y + tarForce.vel.x) - tarForce.angvel * inv_4kw;
//     }
//     else if (wheel_group == WHEEL_LB)
//     {
//         resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y - tarForce.vel.x) - tarForce.angvel * inv_4kw;
//     }
//     else if (wheel_group == WHEEL_RB)
//     {
//         resolve_data[WHEEL_RUN] = root2_4 * (tarForce.vel.y + tarForce.vel.x) + tarForce.angvel * inv_4kw;
//     }
//     else
//     {
//         return;
//     }
// }
//  TODO 舵轮和全向轮还没有逆解算的具体实现:

// void Force_Resolve(Mot_base_t *Fxyw, float motorSetForce[4][2])
//{
//     for (int group = 0; group < WHEELS_GROUP_NUM; group++)
//     {

//        // if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
//        // {
//        //     Mecan_Resolve(*Vxyw, group, motorSetForce[group]);
//        // }
//        // else
//        if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
//        {
//            Omni_Force_Resolve(*Fxyw, group, motorSetForce[group]);
//        }
//        // else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
//        // {
//        //     SteerWheel_Resolve(*Vxyw, group, motorSetForce[group]);
//        // }
//        else
//            return;
//    }
//}

//********************************正解算部分************************************** */

// //全向轮正解算
// //获取底盘目前各个电机状态，轮向电机速度，单位mm/s，舵向电机角度，单位0.1°/s，范围[-1800,+1800],向前为0，向左为负，向右为正
// void GetEncode(void)
// {
//    for (int group = 0; group < 4; group++)
//    {
//            // 单位mm / s      计算过程：快转子rpm->快转子rad / s->输出轴rad / s->轮子mm / s
//            encodeData[group] = RPMTORAD * WCONVERT2V_k * Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor);
//            // encodeData[group][WHEEL_RUN] = motorSetData[group][WHEEL_RUN];
//    }
// }

// /**
// * @brief 麦轮/全向轮底盘正运动解算 此控制坐标系为x轴正方向为右手方向，y轴正方向为正前方
// * @param {坐标系}底盘角速度为正 表示底盘逆时针旋转 输入，输出为正表示车轮带着车身向前运动
// * @param {float} encoderData[4] 编码器得到的电机的数据，快转子的转速(rpm)
// * @param {Mot_base_t} *real_xyw 解算出来的车身的速度
// * @return real_xyw->vy 单位m/s real_xyw->vx 单位mm/s  real_xyw->angvel单位0.1°/s
// */
// void fromEncoderGetV(Mot_base_t *real_xyw)
// {
//    // 各个轮组解算的常量系数，按照不同的种类进行划分     该系数考虑距离，角度转弧度，角速度0.1度/s的因素，最终将轮向电机速度直接除以改系数得到角速度值
//    static float coefficient_OMNI = VEHICLE_DIAMETER / 20.f * DEGTORAD;
//    static float coefficient_MECANUM = (VEHICLE_LONG + VEHICLE_WIDTH) / 20.f * DEGTORAD;
//    static float coefficient_STEER = VEHICLE_DIAMETER / 20.f * DEGTORAD;
//    // 三个方向速度的符号，顺序为[3]:y,x,w,[WHEELS_GROUP_NUM]:右前,左前，左后，右后，适用于全向轮和麦克纳姆轮，不适用于舵轮
//    static int sign[3][WHEELS_GROUP_NUM] = {{1, 1, 1, 1}, {-1, 1, -1, 1}, {1, -1, -1, 1}};
//    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
//    {
//        if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL)
//        {
//            // 单位mm/s
//            real_xyw->vel.y += encodeData[group][WHEEL_RUN] / 4.0f * sign[0][group];
//            real_xyw->vel.x += encodeData[group][WHEEL_RUN] / 4.0f * sign[1][group];
//            // 单位rad/s ->0.1°/s
//            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f / coefficient_MECANUM * sign[2][group];
//        }
//        else if (wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
//        {
//            // 单位mm/s
//            real_xyw->vel.y += encodeData[group][WHEEL_RUN] * ROOT2 / 4.0f * sign[0][group];
//            real_xyw->vel.x += encodeData[group][WHEEL_RUN] * ROOT2 / 4.0f * sign[1][group];
//            // 单位rad/s ->0.1°/s
//            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f / coefficient_OMNI * sign[2][group];
//        }
//        else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
//        {
//            // 单位mm/s
//            real_xyw->vel.y += encodeData[group][WHEEL_RUN] / 4.0f * cosf(encodeData[group][WHEEL_ANGLE] * DEGTORAD);
//            real_xyw->vel.x += encodeData[group][WHEEL_RUN] / 4.0f * sinf(-encodeData[group][WHEEL_ANGLE] * DEGTORAD);
//            // 单位rad/s ->0.1°/s
//            real_xyw->angvel += encodeData[group][WHEEL_RUN] / 4.0f * sinf((encodeData[group][WHEEL_ANGLE] - Wheel_Azimuth[group]) * DEGTORAD) / coefficient_STEER;
//        }
//        else
//            return;
//    }
// }
// /**
// * @brief 获得底盘实际运动状态
// *
// * @param real_xyw 返回底盘实际运动状态，x,y,w的单位分别为mm/s，mm/s，0.1°/s
// */
// void Get_real_motion(Mot_base_t *real_xyw)
// {
//    GetEncode();
//    fromEncoderGetV(real_xyw);
// }
//********************************正解算部分************************************** */
