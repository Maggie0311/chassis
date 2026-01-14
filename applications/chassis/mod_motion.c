#include "pid.h"
#include "drv_utils.h"
// #include "func_ModeTrigger.h"
#include "drv_Vector.h"
#include "drv_Setplanning3D.h"
#include "drv_GimMotor.h"
#include "fun_powerCtrl.h"
#include "arm_math.h"
#include "drv_gimCtrlRx.h"
// #include "drv_AccClamp.h"
#include "fun_protocolGim.h"
#include "func_smooth_acc.h"
#include "mod_motion.h"
#include "Wheel_data.h"
#include <arm_math.h>

#define POWER_LIMIT_TEST 0
// 25赛季修改跟随方法,这里不再使用以前复杂的计算办法,改为简单的pid跟随,既节省能量,又优化了跟随办法,也减少了复杂的计算方式
// 现在采用两段式PID计算办法,当误差较大时,使用较大的PID参数,当误差较小时,使用较小的PID参数,在每个机器人单独文件中有该PID参数
pid_t angfol_pid = PID_INIT(20, 0, 10, 0, 2000, -2000);
v_lim_t v_angle_lim = {0.f, 50, 30, 0.3f, 0.f, 0.f};
SmoothRateLimiter smooth_data[SMOOTH_NUM] = {0};
SecondOrder_Set_Follow_t set_planner_data[SMOOTH_NUM] = {0};
#ifndef ANGFOL_KP_MIN
#define ANGFOL_KP_MIN 33
#endif

static uint8_t fol_isReverse = 0; // 从控制坐标系转化到底盘坐标系
static uint8_t flag_reg = 0;      // isFirst, turn_reg, isIn_1, isIn_2 , trunin_mode_last, trunin
/*底盘四个车轮(或者说轮毂)的期望线速度 单位mm/s 输出顺序RF-LF-LB-RB*/
float motorSetData[WHEELS_GROUP_NUM][WHEEL_NUM]; // 经过解算后的数据

// debug
Mot_base_t SmoothSpeed;    /*处理过后控制使用设定值*/
Mot_base_t Setmotion;      /*未处理原始设定值*/
Mot_base_t real_xyw;       /*反解算的实际速度*/
float debug_yaw_angle = 0; /*跟随角debug*/
float debug_fol_error = 0;
uint8_t isSetValid[2] = {1, 1}; /*debug*/
SetPlanning_Str motionSetPlan[3];

static float range_angle_180(float angle)
{
    while (angle >= 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

/**
 * @brief   输入幅度限制模块1
 * @param   VxyW   速度矢量和角速度矢量
 * @return  限制后Mot_base_t数据
 */
static Mot_base_t MotModule_Clamp_Base(Mot_base_t VxyW)
{
    Mot_base_t temp =
        {
            .vel.y = utils_clamp(VxyW.vel.y, -M_MAX_YSPEED, M_MAX_YSPEED),
            .vel.x = utils_clamp(VxyW.vel.x, -M_MAX_XSPEED, M_MAX_XSPEED),
            .angvel = utils_clamp(VxyW.angvel, -M_MAX_ROTATE_AC, M_MAX_ROTATE_AC)};
    return temp;
}

/**
 * @brief 矢量坐标转换模块 右手坐标系 y轴为正前方
 * @param {Vector2_t} speed 速度矢量,不改变单位；推荐单位：mm/s
 * @param {float} theta 两个坐标系的夹角 输入单位°，范围(-180°~180°]
 * @param {uint8_t} isReverse 当存在云台时和云台角度匹配即可
 */
static Vector2_t MotModule_Transform(Vector2_t speed, float theta, uint8_t isReverse)
{
    Vector2_t chassis_speed;
    Vector2_t gimSpeed = speed;
    theta = theta / 180.0f * PI; // 转化为弧度
    if (isReverse)
    {
        chassis_speed.x = gimSpeed.x * cosf(theta) - gimSpeed.y * sinf(theta); // x=x'cosθ-y'sinθ
        chassis_speed.y = gimSpeed.x * sinf(theta) + gimSpeed.y * cosf(theta); // y=y'cosθ+x'sinθ
    }
    else
    {
        chassis_speed.x = gimSpeed.x * cosf(theta) + gimSpeed.y * sinf(theta);  // x'=ysinθ+xcosθ
        chassis_speed.y = -gimSpeed.x * sinf(theta) + gimSpeed.y * cosf(theta); // y'=ycosθ-xsinθ
    }
    return chassis_speed;
}

/**
 * @brief  跟随角pid计算模块
 * @param {float} set_angle 设定角度值，单位° 范围(-180~180°]
 * @param {float} now_a ngle 当前yaw角度值，单位° 范围(-180~180°]
 * @param {rt_bool_t} WNeedReverse 当认为云台逆时针转动角度为正增大时则需要reverse,
 *        解释当set=0,now=30,则底盘需要逆时针转动,0-30=-30,因此需要reverse
 * @return {*}角速度：单位0.1°/s
 */
static float MotModule_FollowPid(float set_angle, float now_angle, rt_bool_t WNeedReverse, uint8_t trunin_mode, Mot_base_t *const VxyW)
{
    static uint8_t conuts = 0;
    static PID_Change_t PID_folang_Change;
    float error = 0;

    if (READBIT(flag_reg, 0) == 0) //(!isFirst)
    {
        PID_folang_Change.err_max = 90.0f;
        PID_folang_Change.Kp_origin = angfol_pid.kp;
        PID_folang_Change.P_min = ANGFOL_KP_MIN;
        PID_folang_Change.P_range = 2 * (PID_folang_Change.Kp_origin - PID_folang_Change.P_min);
        PID_folang_Change.P_max = PID_folang_Change.P_min + PID_folang_Change.P_range;
        PID_folang_Change.lim_up_origin = angfol_pid.out_limit_up;
        PID_folang_Change.lim_down_origin = angfol_pid.out_limit_down;
        SETBIT(flag_reg, 0); // isFirst = 1;
    }
#if IF_USE_FOLLOW_BACK == 1
    static uint16_t counts_1 = 0;
    static float set_angle_last = 0;
    float err_tmp = 0;
    if (FLOAT_ISEQUAL(set_angle, 0))
    {
        error = utils_Circle_Shortest_Dis(now_angle, set_angle, -180.0f, 180.0f);
        err_tmp = utils_Circle_Shortest_Dis(now_angle, 180, -180.0f, 180.0f);
        if (fabsf(fabsf(error) - fabsf(err_tmp)) > 45.f)
            error = (fabsf(err_tmp) < fabsf(error)) ? (SETBIT(flag_reg, 1), err_tmp) : (CLRBIT(flag_reg, 1), error);
        else
            error = READBIT(flag_reg, 1) ? err_tmp : error;
    }
    else if (FLOAT_ISEQUAL(set_angle, 180.f))
    {
        error = utils_Circle_Shortest_Dis(now_angle, set_angle, -180.0f, 180.0f);
        err_tmp = utils_Circle_Shortest_Dis(now_angle, 0, -180.0f, 180.0f);
        if (fabsf(fabsf(error) - fabsf(err_tmp)) > 45.f)
            error = (fabsf(err_tmp) < fabsf(error)) ? (SETBIT(flag_reg, 1), err_tmp) : (CLRBIT(flag_reg, 1), error);
        else
            error = READBIT(flag_reg, 1) ? err_tmp : error;
    }
    else if (!FLOAT_ISEQUAL(set_angle_last, set_angle) || READBIT(flag_reg, 2) != 0) // isIn_1
    {
        set_angle_last = set_angle;
        if (counts_1++ < 2000)
            SETBIT(flag_reg, 2); // isIn_1 = 1;
        else
        {
            CLRBIT(flag_reg, 2); // isIn_1 = 0;
            counts_1 = 0;
        }
        error = utils_Circle_Shortest_Dis(now_angle, set_angle, -180.0f, 180.0f);
    }
    else /*将error变换到设定值左右的(-180.0°,180.0°]*/
#endif
        error = utils_Circle_Shortest_Dis(now_angle, set_angle, -180.0f, 180.0f);

    debug_fol_error = error;
    if (trunin_mode == 2 && !READBIT(flag_reg, 4))
    {
        SETBIT(flag_reg, 5);
    }
    if (trunin_mode == 2)
    {
        SETBIT(flag_reg, 4);
        if (READBIT(flag_reg, 5))
        {
            float V_angle_turn_in = 0;
            if (RT_TRUE == WNeedReverse)
                V_angle_turn_in = -VxyW->angvel;
            else
                V_angle_turn_in = VxyW->angvel;
            if (V_angle_turn_in > 0 && ((fabsf(error) > 60.0f) || error < -20.0f))
            {
                angfol_pid.out_limit_up = utils_clamp(V_angle_turn_in, 1500, 3000); // V_angle_turn_in > 2000 ? 2000 : V_angle_turn_in;
                error = 60.f;
            }
            else if (V_angle_turn_in <= 0 && ((fabsf(error) > 60.0f) || error > 20.0f))
            {
                angfol_pid.out_limit_down = utils_clamp(V_angle_turn_in, -3000, -1500); // V_angle_turn_in < -2000 ? -2000 : V_angle_turn_in;
                error = -60.f;
            }
            else
            {
                if (fabsf(error) < 20.0f)
                {
                    CLRBIT(flag_reg, 5);
                    angfol_pid.out_limit_up = PID_folang_Change.lim_up_origin;
                    angfol_pid.out_limit_down = PID_folang_Change.lim_down_origin;
                }
            }
        }
    }
    else
    {
        CLRBIT(flag_reg, 4);
        angfol_pid.out_limit_up = PID_folang_Change.lim_up_origin;
        angfol_pid.out_limit_down = PID_folang_Change.lim_down_origin;
    }

    // 是否进入死区/滞回
    if (fabsf(VxyW->vel.x) > 1.0f || fabsf(VxyW->vel.y) > 1.0f)
    {
        conuts = 0;
        CLRBIT(flag_reg, 3); // isIn_2 = 0;
    }
    else if (fabsf(error) < 0.8f && !READBIT(flag_reg, 3)) // 0.8f//(YAW_COUNT / 3600.f)指0.1度;---isIn_2
    {
        if (conuts < 20)
            conuts++;
        else
        {
            SETBIT(flag_reg, 3); // isIn_2 = 1;
            conuts = 0;
        }
    }
    else if (fabsf(error) > 1.5f && READBIT(flag_reg, 3))
    {
        if (conuts < 3)
            conuts += fabsf(error);
        else
        {
            CLRBIT(flag_reg, 3); // isIn_2 = 0;
            conuts = 0;
        }
    }
    if (READBIT(flag_reg, 3)) // isIn_2
        error = 0;

    /*pid计算*/
    angfol_pid.kp = (fabsf(error) / PID_folang_Change.err_max) * PID_folang_Change.P_range + PID_folang_Change.P_min;
    PID_Calculate(&angfol_pid, error);

    if (RT_TRUE == WNeedReverse)
        return -angfol_pid.out;
    else
        return angfol_pid.out;
}

/**
 * @brief 偏心运动模块（将绕某点旋转转化为底盘中心的速度矢量）
 * @param {float} motion 传入自旋速度大小，单位 0.1°/s，正方向俯视图逆时针。
 * @param {float} pos 偏心底盘中心位置
 * @return 中心的速度矢量,不改变传入的速度矢量单位
 */
static void MotModule_Eccentric(Mot_base_t *motion, const Vector2_t *const pos)
{
    VectorXYZ_Str v, r, w;
    r.x = -pos->x;
    r.y = -pos->y;
    r.z = 0;

    w.x = 0;
    w.y = 0;
    w.z = DEG2RAD_f(motion->angvel / 10);

    /*υ = ω x r，计算底盘中心的速度矢量*/
    Vector3D_CrossProd(&w, &r, &v);
    motion->vel.x += v.x;
    motion->vel.y += v.y;
}

/**
 * @brief 偏心小陀螺运动
 * @param {Ctrl_data_t} *ctrlMsg 控制数据 【底盘xy速度矢量，单位mm/s 底盘自转角速度大小，单位0.1°/s】
 * @param {Vector2_t} *pos 偏心坐标点，单位(mm,mm)
 */
Mot_base_t MotPack_Offset_SmallTop(const Ctrl_data_t *const ctrlMsg, const Vector2_t *const pos)
{
    Mot_base_t vxyW_input = ctrlMsg->xyw;
    Mot_base_t vxyW_output = ctrlMsg->xyw;
    float yaw_angle = 0;
    Vector2_t posTemp = *pos;

    /*输入限制*/
    utils_clamp(posTemp.x, -M_MAX_POSITION_X, M_MAX_POSITION_X);
    utils_clamp(posTemp.y, -M_MAX_POSITION_Y, M_MAX_POSITION_Y);

    if (ctrlMsg->getYawAngel)
    {
        yaw_angle = ctrlMsg->getYawAngel(ctrlMsg->arg);
    }
    /*本文件暂存*/
    fol_isReverse = ctrlMsg->folIsReverse;

    /*将云台的速度矢量转化到底盘坐标系上*/
    vxyW_input.vel = MotModule_Transform(vxyW_input.vel, yaw_angle, fol_isReverse);

    /* 叠加偏心运动 */
    MotModule_Eccentric(&vxyW_input, &posTemp);

    /*输出限制*/
    vxyW_output.vel.x = utils_clamp(vxyW_input.vel.x, -M_MAX_XSPEED, M_MAX_XSPEED);
    vxyW_output.vel.y = utils_clamp(vxyW_input.vel.y, -M_MAX_YSPEED, M_MAX_YSPEED);
    vxyW_output.angvel = utils_clamp(vxyW_input.angvel, -M_MAX_POSITION_X, M_MAX_POSITION_X);

    return vxyW_output;
}

/**
 * @brief    只有底盘运动
 * @param    VxyW.vel   底盘xy速度矢量，单位mm/s
 * @param    VxyW.angvel   底盘自转角速度大小，单位0.1°/s
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Only_Chass(Mot_base_t VxyW)
{
    // 输入幅度限制模块
    VxyW = MotModule_Clamp_Base(VxyW);

    return VxyW;
}

// 当旋转较快时执行yaw轴电机角度补偿
void yawAngleOffset(const float angvel, float *const yawAngel)
{
    // float deltaT = 0;
    // float temp = 0;
    // if (!FLOAT_ISEQUAL(angvel, 0))
    // {
    //     if (fabsf(angvel) > 100)
    //     {
    //         deltaT = (fabsf(angvel) / 1000.f) * (K_YAW_DeltaT);
    //     }
    // }
    // /*补偿执行滞后等（单位已统一），根据角度正方向和转速正方向知补偿项为负号*/
    // temp = *yawAngel - (angvel / 10.f) * (deltaT / 1000.f);
    // temp = range_angle_180(temp);
    // *yawAngel = temp;
}

// 跟随时，当功率受限时降低平移保证转向
v_lim_t v_vel_lim = {0.f, 65, 50, 1.0f, 0.f, 0.f};
uint8_t follow_at_powerlim(Mot_base_t *const VxyW)
{
    uint8_t ret = 0;
    v_vel_lim.lim_I = 0;
    if (PowerCtrl.power_allo_flag != 0 && fabsf(VxyW->angvel) > 1000.0f) // && PowerCtrl.Power_Maxlim_sum - PowerInfo.Ref.Power_Maxlim < 100.f)
    {
        // if(PowerCtrl.k_power_allo)
        v_vel_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_vel_lim.lim_k);
        if (PowerCtrl.power_allo_flag == 2)
            v_vel_lim.lim_I = v_vel_lim.lim_max * fabsf(PowerCtrl.Power_Allo_level) / 100.f;
    }
    else
    {
        if (v_vel_lim.lim_out > 5.0f)
            v_vel_lim.lim_out -= 5.0f;
        else
            v_vel_lim.lim_out = 0;
    }
    if (fabsf(VxyW->angvel) > 450.0f)
    {
        v_vel_lim.lim_out += 0.1f * fabsf(VxyW->angvel) / 450.f;
    }
    v_vel_lim.lim_out = utils_clamp(v_vel_lim.lim_out, 0.f, v_vel_lim.lim_max);
    v_vel_lim.lim_level = v_vel_lim.lim_out + v_vel_lim.lim_I;
    if (v_vel_lim.lim_level >= v_vel_lim.lim_max - 0.001f)
    {
        v_vel_lim.lim_level = v_vel_lim.lim_max;
        ret = 1;
    }
    VxyW->vel.x *= (1.0f - v_vel_lim.lim_level / 100.f);
    VxyW->vel.y *= (1.0f - v_vel_lim.lim_level / 100.f);
    return ret;
}

/**
 * @brief    底盘跟随云台运动,即所有控制方向都是以云台坐标系而言的 正前方为云台枪管方向
 * @note     输入的VxyW.angvel没有使用 该函数不改变也不限制输入速度单位
 * @param    VxyW.vel    云台方向的平移速度矢量，单位mm/s
 * @param    fol_angle   跟随角,在俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下,
 *           云台枪管正方向和在底盘车头重合时为0度. 跟随角θ表示底盘车头和云台枪管期望夹角 >0表示底盘在云台逆时针方向
 * @param    fol_angle 范围：[0-360°]单位度
 * @return   输出到Resolve函数的VxyW
 */
float yaw_angle = 0;
float anglevel_last = 0; // 记录上次角速度滤波后的值
Mot_base_t MotPack_Follow_Gim(const Ctrl_data_t *const ctrlMsg, uint8_t trunin_mode)
{
    fol_isReverse = ctrlMsg->folIsReverse; /*本文件暂存*/
    float fol_angle = ctrlMsg->fol_angle;
    float yaw_Omega = 0;
    static Mot_base_t last_speed = {0};
    /*输入限制*/
    Mot_base_t VxyW = MotModule_Clamp_Base(ctrlMsg->xyw);

    /*确保角度为0-360*/
    utils_norm_circle_number(&fol_angle, 0, 360.f);
    if (fabsf(ctrlMsg->xyw.angvel) >= 300)
    {
        anglevel_last = ctrlMsg->xyw.angvel;
    }
    else
    {
        anglevel_last = 0;
    }
    // anglevel_last = UTILS_LP_FAST(anglevel_last, ctrlMsg->xyw.angvel, 0.2f); // 对云台角速度进行低通滤波
    if (ctrlMsg->getYawAngel)
    {
        /*获取云台和底盘的夹角 数据范围(-180.0°,180.0°]*/ // 得到当前控制坐标系和底盘坐标系角度
        yaw_angle = ctrlMsg->getYawAngel(ctrlMsg->arg);
        if (fabs(yaw_angle) <= 0.5f) // 跟随角死区
        {
            yaw_angle = 0;
        }
        if (fabs(yaw_angle) >= 15.f) // 跟随角较大时，修改跟随角PID为较大参数
        {
            ExMotMod_Modify_Apid(FOLLOW_PID_2);
        }
        else // 跟随角较小时，修改跟随角PID为较小参数
        {
            ExMotMod_Modify_Apid(FOLLOW_PID_1);
        }
    }

    /*将fol_angle转化为(-180.0°,180.0°]度*/
    fol_angle = range_angle_180(fol_angle);

    /*计算跟随时的动态角速度大小 fol_angle即表示pid的target*/
    // VxyW.angvel = MotModule_FollowPid(fol_angle, yaw_angle, fol_isReverse, trunin_mode, &VxyW);
    PID_Calculate(&angfol_pid, yaw_angle);
    VxyW.angvel = anglevel_last + angfol_pid.out;
    // VxyW.angvel=0;
    /*跟随时，当功率受限时降低平移保证转向*/
#ifdef POWER_LIMIT_TEST == 0
    follow_at_powerlim(&VxyW);
#endif
    // yaw_Omega = Get_GM_Omega(ctrlMsg->arg); // 这里的arg是云台yaw或者pitch轴电机的枚举，在云台接受控制数据处定义
    // yawAngleOffset(yaw_Omega, &yaw_angle);  // 当旋转较快时执行yaw轴电机角度补偿

    /*将云台的控制速度矢量转化到底盘坐标系上*/
    // VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle, fol_isReverse);
    return VxyW;
}

/**
 * @brief 判断是否由非跟随模式转化为跟随模式
 *
 * @param fixCtrl 控制数据源
 * @return uint8_t 返回1表示由非跟随模式转化为跟随模式
 */
uint8_t move_at_no_follow(Ctrl_data_t *const fixCtrl)
{
    static rt_tick_t no_follow_tick = 0; // 未跟随模式时间，如果控制端不控制（判断x和y同时很小的时候）,在0.1s后转化为非跟随模式
    uint8_t ret = 0;
    float yaw_angle = 0;
    if (fabsf(fixCtrl->xyw.vel.x) > 1.0f || fabsf(fixCtrl->xyw.vel.y) > 1.0f) // 有控制输入（控制端在控制）
    {
        ret = 1; // 由非跟随模式转化为跟随模式
        no_follow_tick = rt_tick_get();
    }
    else if (rt_tick_get() - no_follow_tick > 100) // 控制源无控制0.1s后转化为非跟随模式
    {
        ret = 0;
    }
    else // 控制源无控制0.1s内仍然为跟随模式
    {
        ret = 1;
    }
    return ret;
}

// 陀螺时，当功率受限时降低陀螺转速保证平移性
uint8_t gyro_at_powerlim(Mot_base_t *const VxyW, Ctrl_data_t *const ctrlMsg)
{
    static uint8_t ret_0 = 0;
    v_angle_lim.lim_I = 0;
    if ((fabsf(VxyW->vel.y) > 80.0f || fabsf(VxyW->vel.x) > 80.0f) && PowerCtrl.Power_Maxlim_sum - PowerInfo.Ref.Power_Maxlim < 180.f)
    {
        if (PowerCtrl.power_allo_flag == 1)
            v_angle_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_angle_lim.lim_k);
        else if (PowerCtrl.power_allo_flag == 2)
        {
            v_angle_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_angle_lim.lim_k);
            v_angle_lim.lim_I = v_angle_lim.lim_max * fabsf(PowerCtrl.Power_Allo_level) / 100.f;
        }
    }
    else
    {
        if (v_angle_lim.lim_out > 1.0f)
            v_angle_lim.lim_out -= 1.0f;
        else
            v_angle_lim.lim_out = 0;
    }
    // debug
    //    if (fabsf(VxyW->vel.x) > 500.0f)
    //        v_angle_lim.lim_out += 0.5f * (fabsf(VxyW->vel.x) / 1000.f + 9);
    //    if (fabsf(VxyW->vel.y) > 500.0f)
    //        v_angle_lim.lim_out += 0.5f * (fabsf(VxyW->vel.y) / 1000.f + 9);

    float pitch = 0, roll = 0;
    if ((fabsf(VxyW->vel.y) > 500.0f && (get_chassis_pitch(&pitch)) ? fabsf(pitch) > GYRO_ANGEL_MINLIM : 0))
        if (VxyW->vel.y * pitch > 0) // 此判断实现只针对上坡
            v_angle_lim.lim_out += 0.2f * (fabsf(VxyW->vel.y) / 1000.f + (fabsf(pitch) - GYRO_ANGEL_MINLIM));
    if ((fabsf(VxyW->vel.x) > 500.0f && (get_chassis_roll(&roll)) ? fabsf(roll) > GYRO_ANGEL_MINLIM : 0))
        if (VxyW->vel.x * roll > 0) // 此判断实现只针对上坡
            v_angle_lim.lim_out += 0.2f * (fabsf(VxyW->vel.x) / 1000.f + (fabsf(roll) - GYRO_ANGEL_MINLIM));

    v_angle_lim.lim_out = utils_clamp(v_angle_lim.lim_out, 0, v_angle_lim.lim_max);
    v_angle_lim.lim_level = v_angle_lim.lim_out + v_angle_lim.lim_I;
    if (v_angle_lim.lim_level >= v_angle_lim.lim_max - 0.001f)
    {
        v_angle_lim.lim_level = v_angle_lim.lim_max;
        ret_0 = 2;
    }
    else if (v_angle_lim.lim_level < v_angle_lim.lim_down)
        ret_0 = 0;
    VxyW->angvel *= (1.0f - v_angle_lim.lim_level / 100.f);
    return ret_0;
}

// 功率限制控制器结构体
typedef struct
{
    float K_in;          // 当前输入缩放因子(平滑后)
    float K_in_raw;      // 原始计算的缩放因子(平滑前)
    float K_out_last;    // 上一次输出端限制比例
    float max_power;     // 系统最大允许功率(W)
    float min_K;         // 最小缩放因子(0.01~0.1)
    float smooth_factor; // 平滑因子(0.05~0.2)

    // PID控制器参数
    pid_t pid_controller;
} power_input_limiter_t;

// 全局实例初始化
power_input_limiter_t input_limiter = {
    .K_in = 1.0f,
    .K_in_raw = 1.0f,
    .K_out_last = 1.0f,
    .max_power = 200.0f, // 根据实际系统调整
    .min_K = 0.05f,
    .smooth_factor = 0.1f,
    .pid_controller = PID_INIT(0.8f, 0.05f, 0.02f, 0.2f, 0.1f, -0.1f) // 调整PID限幅
};

float update_input_limiter(float K_out)
{
    /* 1. 计算控制误差 */
    float error = 1.0f - K_out;

    /* 2. 使用PID计算缩放因子调整量 */
    PID_Calculate(&input_limiter.pid_controller, error);

    /* 3. 计算原始K_in */
    input_limiter.K_in_raw = input_limiter.K_in - input_limiter.pid_controller.out;

    /* 4. 限制原始K_in范围 */
    if (input_limiter.K_in_raw > 1.0f)
        input_limiter.K_in_raw = 1.0f;
    else if (input_limiter.K_in_raw < input_limiter.min_K)
        input_limiter.K_in_raw = input_limiter.min_K;

    /* 5. 应用正确的平滑滤波: 当前值 = 平滑因子×新值 + (1-平滑因子)×旧值 */
    input_limiter.K_in = input_limiter.smooth_factor * input_limiter.K_in_raw +
                         (1 - input_limiter.smooth_factor) * input_limiter.K_in;

    input_limiter.K_out_last = K_out;
    return input_limiter.K_in;
}

/**
 * @brief 应用输入端功率限制
 * @param power_limit_k 当前实际功率限制系数
 *                      计算得出实际在控制端限制的比例
 */
void apply_input_power_limit(float power_limit_k)
{

    power_limit_k = (power_limit_k > 1) ? 1 : power_limit_k;
    /* 2. 更新缩放因子 */
    update_input_limiter(power_limit_k);
}

Mot_base_t MotPack_Power_Limit(Ctrl_data_t *const ctrlMsg)
{

    Mot_base_t VxyW = ctrlMsg->xyw;
    apply_input_power_limit(PowerCtrl.k_power_allo_wheel); // 这里没有兼容除全向和麦轮以外的底盘
    if (ctrlMsg->motion_mode == FOLLOW_GIMBAL || ctrlMsg->motion_mode == NO_FOLLOW)
    {
        VxyW.vel.x *= input_limiter.K_in;
        VxyW.vel.y *= input_limiter.K_in;
    }
    else if (ctrlMsg->motion_mode == SLOW_GYRO || ctrlMsg->motion_mode == FAST_GYRO)
    {
        // VxyW.vel.x *= input_limiter.K_in;
        // VxyW.vel.y *= input_limiter.K_in;
        VxyW.angvel *= input_limiter.K_in;
    }
    return VxyW;
}

uint8_t if_turn_to_change = 0;
void gyro_at_hurt(Mot_base_t *const VxyW, Ctrl_data_t *const ctrlMsg)
{
    static rt_uint32_t last_time = 0;
    static float A = 0.5f; // 增加或者减小的幅度为50%
    static float T = 4.f;  // 周期为2s
    static rt_uint32_t now_time = 0;
    float temp = 0;
    if (ctrlMsg->motion_mode == CHANGE_GYRO)
    {
        if (Ref_Get_Robot_If_Hurt() && rt_tick_get() - last_time > T * 1000)
        {
            Ref_Robot_Reset_Hurt();
            last_time = rt_tick_get();
            if_turn_to_change = 1;
        }
        if (rt_tick_get() - last_time < T * 1000)
        {
            now_time = rt_tick_get();
            temp = fabsf(arm_sin_f32((now_time - last_time) / 1000.f * 2 * PI / T));
            VxyW->angvel = (1 + A * temp) * ctrlMsg->xyw.angvel;
            if_turn_to_change = 0;
        }
    }
}

/**
 * @brief 小陀螺运动 当 VxyW.angvel == 0时,为不跟随的底盘全向移动运动
 * @param {Mot_base_t} VxyW VxyW 平移速度矢量，单位mm/s. 自转角速度，单位0.1°/s
 * @param {float} getYawAngel 得到坐标系旋转角度  (-180.0°,180.0°],云台在底盘正方向左边时为正
 * @param {uint8_t} isReverse 进行坐标系变化时是角度否需要，当底盘逆时针旋转90度时角度为正则isReverse=0不需要反向
 * @return 返回控制坐标系下VxyW，旋转yawAngle后坐标系下的VxyW
 */
Mot_base_t MotPack_Small_Top(Ctrl_data_t *const ctrlMsg)
{
    /*输入限制*/
    Mot_base_t VxyW = MotModule_Clamp_Base(ctrlMsg->xyw);
    static Mot_base_t last_speed = {0};

    // 如果控制量太小，则将输出置0
    if (VxyW.vel.x <= 1 && VxyW.vel.x >= -1)
    {
        VxyW.vel.x = 0;
    }
    if (VxyW.vel.y <= 1 && VxyW.vel.y >= -1)
    {
        VxyW.vel.y = 0;
    }
    if (VxyW.angvel <= 1 && VxyW.angvel >= -1)
    {
        VxyW.angvel = 0;
    }

    float angle = 0;
    float yaw_Omega = 0;
    uint8_t if_turn_to_follow = 0;

    if (ctrlMsg->motion_mode == NO_FOLLOW)
        if_turn_to_follow = move_at_no_follow(ctrlMsg);
    if (if_turn_to_follow)
    {
        ctrlMsg->motion_mode = FOLLOW_GIMBAL;
        return MotPack_Follow_Gim(ctrlMsg, if_turn_to_follow);
    }
    else
        CLRBIT(flag_reg, 4);
    if (ctrlMsg->motion_mode == CHANGE_GYRO)
    {
        gyro_at_hurt(&VxyW, ctrlMsg); // 陀螺受伤时，开启变速小陀螺
    }
#ifdef POWER_LIMIT_TEST == 0
    gyro_at_powerlim(&VxyW, ctrlMsg); // 功率受限时限制陀螺保证平移性
#endif
    if (ctrlMsg->getYawAngel)
    {
        // 得到当前控制坐标系和底盘坐标系角度
        angle = ctrlMsg->getYawAngel(ctrlMsg->arg);
    }
    yawAngleOffset(VxyW.angvel, &angle); // 当小陀螺较快时执行yaw轴电机角度补偿

    fol_isReverse = ctrlMsg->folIsReverse; /*本文件暂存*/

    return VxyW;
}

/**
 * @brief    修改跟随的角度pid参数,暂时用在test调试上
 * @param    kp-ki-kd    pid参数
 */
void ExMotMod_Modify_Apid(float kp, float ki, float kd, float i_lim, float lim_up, float lim_down)
{
    angfol_pid.kp = kp;
    angfol_pid.ki = ki;
    angfol_pid.kd = kd;
}

/*新修改的设定值规划函数初始化 */
int Set_Planner_init(void)
{
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_XSPEED], ACC_MAX_X, DOUBLE_ACC_MAX_X, PLAN_PERIOD);
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_YSPEED], ACC_MAX_Y, DOUBLE_ACC_MAX_Y, PLAN_PERIOD);
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_ACSPEED], ACC_MAX_W, DOUBLE_ACC_MAX_W, PLAN_PERIOD);
    return 1;
}

int MotSmooth_Vxyw_init(void)
{
    SmoothRateLimiter_init(&smooth_data[WHEEL_XSPEED], ACC_MAX_X, Accel_factor_X, DECEL_FACTOR_X);
    SmoothRateLimiter_init(&smooth_data[WHEEL_YSPEED], ACC_MAX_Y, Accel_factor_Y, DECEL_FACTOR_Y);
    SmoothRateLimiter_init(&smooth_data[WHEEL_ACSPEED], ACC_MAX_W, Accel_factor_W, DECEL_FACTOR_W);
    return 1;
}

/*新修改的设定值规划s型曲线的速度趋势 并进行云台到底盘的坐标系转换*/
Mot_base_t Set_Plan_Vxyw_Process(Ctrl_data_t ctrl)
{
    Mot_base_t ret_xyw = {0};
    Mot_base_t plan_xyw = {0};
    float angle = 0;
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_XSPEED], ctrl.xyw.vel.x, 0.0f); // 依据控制源发来的控制量，对线速度和角速度做规划
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_YSPEED], ctrl.xyw.vel.y, 0.0f);
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_ACSPEED], ctrl.xyw.angvel, 0.0f);
    plan_xyw.vel.x = set_planner_data[WHEEL_XSPEED].p;
    plan_xyw.vel.y = set_planner_data[WHEEL_YSPEED].p;
    plan_xyw.angvel = set_planner_data[WHEEL_ACSPEED].p;

    if (ctrl.getYawAngel)
    {
        // 得到当前控制坐标系和底盘坐标系角度
        angle = ctrl.getYawAngel(ctrl.arg);
    }
    ret_xyw.vel = MotModule_Transform(plan_xyw.vel, angle, ctrl.folIsReverse); // 对线速度进行坐标系转换
    ret_xyw.angvel = plan_xyw.angvel;

    // ret_xyw.vel = MotModule_Transform(ctrl.xyw.vel, angle, ctrl.folIsReverse);
    // ret_xyw.angvel = ctrl.xyw.angvel; // 角速度不参与平滑规划
    return ret_xyw;
}
/**
 * @brief    平滑速度规划函数
 * @param    speed 需要平滑的速度
 * @return   返回平滑后的速度
 */
Mot_base_t MotSmooth_Vxyw_Process(Ctrl_data_t ctrl)
{
    Mot_base_t ret_xyw = {0};
    Mot_base_t smooth_xyw = {0};

    float angle = 0;
    if (ctrl.motion_mode == FOLLOW_GIMBAL) // 跟随模式下，自转也参与加速度规划，这样就不需要管模式切换时的角速度阶跃问题
    {                                      // 但是需要保证角速度跟随效果不错，还可以平滑控制效果，使底盘控制效果更加丝滑
        smooth_xyw.vel.x = DynamicSmooth_func(&smooth_data[WHEEL_XSPEED], ctrl.xyw.vel.x);
        smooth_xyw.vel.y = DynamicSmooth_func(&smooth_data[WHEEL_YSPEED], ctrl.xyw.vel.y);
        smooth_xyw.angvel = DynamicSmooth_func(&smooth_data[WHEEL_ACSPEED], ctrl.xyw.angvel);
    }
    else
    {
        smooth_xyw.vel.x = DynamicSmooth_func(&smooth_data[WHEEL_XSPEED], ctrl.xyw.vel.x);
        smooth_xyw.vel.y = DynamicSmooth_func(&smooth_data[WHEEL_YSPEED], ctrl.xyw.vel.y);
        smooth_xyw.angvel = DynamicSmooth_func(&smooth_data[WHEEL_ACSPEED], ctrl.xyw.angvel);
    }
    if (ctrl.getYawAngel)
    {
        // 得到当前控制坐标系和底盘坐标系角度
        angle = ctrl.getYawAngel(ctrl.arg);
    }
    ret_xyw.vel = MotModule_Transform(smooth_xyw.vel, angle, ctrl.folIsReverse);
    ret_xyw.angvel = smooth_xyw.angvel; // 角速度不参与平滑规划
    return ret_xyw;
}

// 统一的Mot_base_t类型数据输出接口
void ExMotMod_Output(const Ctrl_data_t *const ctrlData)
{
    Mot_base_t tarSpeed = ctrlData->xyw;
    static Mot_base_t last_speed = {0};
    Setmotion = tarSpeed; // 保存直接控制数据用于debug

    if (1 == isSetValid[0])
    {
        /*将底盘的设定运动转化到底盘各电机的运动上 即轮子运动解算*/
        Motion_Resolve(&tarSpeed, motorSetData);
    }
    if (1 == isSetValid[1])
    {
        /*将计算出来的转速设定值写入底盘控制线程*/
        for (int group = 0; group < (int)WHEELS_GROUP_NUM; group++)
        {
            Wheel_Speed_Set((Wheel_Group_e)group, motorSetData[group][WHEEL_RUN]);
            if (wheel[group]->wheel_kind == STEER_WHEEL)
            {
                Wheel_Angle_Set((Wheel_Group_e)group, motorSetData[group][WHEEL_ANGLE]);
            }
        }
    }
}

// 得到实时的控制速度
Mot_base_t getSetMotion(void)
{
    return SmoothSpeed;
}

Mot_base_t getRealVel(void)
{
    return real_xyw;
}

float max_vely = 0;
void refreshMaxVely_in_a_move(void)
{
    static uint8_t is_clear = 1;
    if (fabsf(real_xyw.vel.y) > 120)
    {
        if (is_clear)
        {
            max_vely = 0;
            is_clear = 0;
        }
        if (fabsf(real_xyw.vel.y) > max_vely)
            max_vely = fabsf(real_xyw.vel.y);
    }
    else
        is_clear = 1;
}

float getMaxVely_in_a_move(void)
{
    return max_vely;
}
