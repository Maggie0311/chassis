#include "mod_motion.h"
#include "pid_new.h"
#include "drv_utils.h"
#include "drv_Vector.h"
#include "app_RefSystem.h"
#include "robot_data.h"
#include <arm_math.h>
#include "drv_GimMotor.h"
#include "fun_powerCtrl.h"
#include "robot_control.h"
#include "drv_IMU_Can.h"
#include "drv_motionSolve.h"

#ifdef BSP_USING_VOFA
    #include "vofa.h"
#endif

extern PIDInstance follow_pid_fast, follow_pid_slow; // 跟随角pid实例

SecondOrder_Set_Follow_t set_planner_data[SMOOTH_NUM] = {0}; // 设定值规划
Mot_base_t Force_xyw = {0};
Mot_base_t Speed_xyw = {0};

Mot_base_t SmoothSpeed; /*处理过后控制使用设定值*/
Mot_base_t Setmotion;   /*未处理原始设定值*/

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
            .vel.y = utils_clamp(VxyW.vel.y, -VEL_MAX_Y, VEL_MAX_Y),
            .vel.x = utils_clamp(VxyW.vel.x, -VEL_MAX_X, VEL_MAX_X),
            .angvel = utils_clamp(VxyW.angvel, -M_MAX_ROTATE_AC, M_MAX_ROTATE_AC)};
    return temp;
}

/**
 * @brief 矢量坐标转换模块 右手坐标系 x轴为正前方
 * @param {Vector2_t} speed 速度矢量,不改变单位；推荐单位：mm/s
 * @param {float} theta 两个坐标系的夹角 输入单位°，范围(0°~360°]
 */
Vector2_t MotModule_Transform(Vector2_t speed, float theta)
{
    Vector2_t chassis_speed;
    Vector2_t gimSpeed = speed;
    theta = theta / 180.0f * PI;                                                         // 转化为弧度
    chassis_speed.x = gimSpeed.x * arm_cos_f32(theta) - gimSpeed.y * arm_sin_f32(theta); // x'=ysinθ+xcosθ
    chassis_speed.y = gimSpeed.x * arm_sin_f32(theta) + gimSpeed.y * arm_cos_f32(theta); // y'=ycosθ-xsinθ
    return chassis_speed;
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

    // if (ctrlMsg->getYawAngel)
    // {
    yaw_angle = ctrlMsg->getYawAngel;
    // }
    /*本文件暂存*/
    // fol_isReverse = ctrlMsg->folIsReverse;

    /*将云台的速度矢量转化到底盘坐标系上*/
    vxyW_input.vel = MotModule_Transform(vxyW_input.vel, yaw_angle);

    /* 叠加偏心运动 */
    MotModule_Eccentric(&vxyW_input, &posTemp);

    /*输出限制*/
    vxyW_output.vel.x = utils_clamp(vxyW_input.vel.x, -VEL_MAX_X, VEL_MAX_X);
    vxyW_output.vel.y = utils_clamp(vxyW_input.vel.y, -VEL_MAX_Y, VEL_MAX_X);
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
    float temp = 0;

    if (fabsf(angvel) > 6000)
        temp = YAW_OFFSET_FAST;
    else if (fabsf(angvel) > 2000)
    {
        // temp = (fabsf(angvel) - d_YAW) / K_YAW;
        temp = YAW_OFFSET_SLOW;
    }
    
    /*补偿执行滞后等（单位已统一），根据角度正方向和转速正方向知补偿项为负号*/
    temp = range_angle_180(temp);
    if (angvel > 0)
        *yawAngel -= temp;
    else
        *yawAngel += temp;
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
Mot_base_t MotPack_Follow_Gim(const Ctrl_data_t * ctrlMsg, uint8_t trunin_mode)
{
    // fol_isReverse = ctrlMsg->folIsReverse; /*本文件暂存*/
    static float anglevel_last = 0; // 记录上次角速度滤波后的值
    float fol_angle = ctrlMsg->fol_angle;
    float yaw_angle = 0;
    float yaw_Omega = 0;
    // static PIDInstance *pid; 新普布跟随我用的是kuai'man
    static Mot_base_t last_speed = {0};

    //     /* ---- 速度闭环 PID（卡尔曼融合速度做反馈） ---- */
    // PIDCalculate(&fix_speed_x, fused_speed_xy.x, ctrlMsg->xyw.vel.x);
    // PIDCalculate(&fix_speed_y, fused_speed_xy.y, ctrlMsg->xyw.vel.y);

    // /* ---- 新息前馈（零延迟打滑响应） ---- */
    // float ff_x = Innovation_Feedforward(kalman_innovation_xy.x);
    // float ff_y = Innovation_Feedforward(kalman_innovation_xy.y);

    

    // ctrlMsg->xyw.vel.x += (ff_x + fix_speed_x.Output);
    // ctrlMsg->xyw.vel.y +=( ff_y + fix_speed_y.Output);
    /*输入限制*/
    Mot_base_t VxyW = MotModule_Clamp_Base(ctrlMsg->xyw);
    //VxyW.vel.x+=(ff_x + fix_speed_x.Output);
    //VxyW.vel.y+=(ff_y + fix_speed_y.Output);

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

    yaw_angle = ctrlMsg->getYawAngel;

    // if (fabs(yaw_angle) >= 15.f) // 跟随角较大时，修改跟随角PID为较大参数
    // {
    //     pid = &follow_pid_fast;
    // }
    // else // 跟随角较小时，修改跟随角PID为较小参数
    // {
    //     pid = &follow_pid_slow;
    // }

    /*将fol_angle转化为(-180.0°,180.0°]度*/
    fol_angle = range_angle_180(fol_angle);
    yaw_angle = range_angle_180(yaw_angle);

    /*计算跟随时的动态角速度大小 fol_angle即表示pid的target*/
    // PID_Calculate(&angfol_pid, yaw_angle);

    // 新的pid计算，yaw_angle本身即为误差，所以“反馈值”参数写0
    // VxyW.angvel = anglevel_last + PIDCalculate(pid, 0, yaw_angle);
    float direction = 0;//跟随方向
    if (fabsf(yaw_angle) > 90)// 当云台角度大于90度时，修改跟随方向
    {
        if (yaw_angle > 0)
            direction=180;
        else
            direction=-180;
    }
    //direction += fol_angle;// 加入跟随角
    VxyW.angvel = anglevel_last + PIDCalculate(&follow_pid_fast, direction, yaw_angle);

    /*跟随时，当功率受限时降低平移保证转向*/
    // yaw_Omega = Get_GM_Omega(ctrlMsg->arg); // 这里的arg是云台yaw或者pitch轴电机的枚举，在云台接受控制数据处定义
    // yawAngleOffset(yaw_Omega, &yaw_angle);  // 当旋转较快时执行yaw轴电机角度补偿

    /*将云台的控制速度矢量转化到底盘坐标系上*/
    // VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);
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

/**
 * @brief 受伤时变速小陀螺
 *
 * @param VxyW
 * @param ctrlMsg
 */
void gyro_at_hurt(Mot_base_t *const VxyW, Ctrl_data_t *const ctrlMsg)
{
    static uint8_t if_turn_to_change = 0;
    static rt_uint32_t last_time = 0;
    static float A = 0.5f; // 增加或者减小的幅度为50%
    static float T = 4.f;  // 周期为2s
    static rt_uint32_t now_time = 0;
    float temp = 0;
    if (ctrlMsg->motion_mode == CHANGE_GYRO)
    {
        if (hurt.robot_if_hurt && rt_tick_get() - last_time > T * 1000)
        {
            hurt.robot_if_hurt = 0;
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

    uint8_t if_turn_to_follow = 0;

    if (ctrlMsg->motion_mode == NO_FOLLOW)
        if_turn_to_follow = move_at_no_follow(ctrlMsg);
    if (if_turn_to_follow)
    {
        ctrlMsg->motion_mode = FOLLOW_GIMBAL;
        return MotPack_Follow_Gim(ctrlMsg, if_turn_to_follow);
    }

    if (ctrlMsg->motion_mode == CHANGE_GYRO)
    {
        gyro_at_hurt(&VxyW, ctrlMsg); // 陀螺受伤时，开启变速小陀螺
    }
    // fol_isReverse = ctrlMsg->folIsReverse; /*本文件暂存*/

    // 陀螺角度补偿
    ctrlMsg->getYawAngel = Get_YAW_ANGLE_360f();
    yawAngleOffset(real_xyw.angvel, &ctrlMsg->getYawAngel);

    return VxyW;
}

/*新修改的设定值规划函数初始化 */
int Set_Planner_init(void)
{
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_XSPEED], ACC_MAX_X, DOUBLE_ACC_MAX_X, PLAN_PERIOD);
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_YSPEED], ACC_MAX_Y, DOUBLE_ACC_MAX_Y, PLAN_PERIOD);
    SecondOrder_Set_Follow_init(&set_planner_data[WHEEL_ACSPEED], ACC_MAX_W, DOUBLE_ACC_MAX_W, PLAN_PERIOD);
    return 1;
}

/* ---- 新息前馈补偿参数 ---- */
// innovation = 正解算速度 - 卡尔曼预测速度
// 打滑时正解算虚高 → innovation 为正大值 → 前馈输出负补偿，压低设定值
#define INNOV_FF_GAIN       (-0.1f)  // 前馈增益（负号：innovation正=正解算虚高→需要减速补偿）
#define INNOV_DEADZONE      (30.f)   // 前馈死区(mm/s)，小新息不响应，避免正常行驶时抖动

/**
 * @brief   新息前馈补偿计算
 *          带死区的线性前馈，零延迟响应打滑
 */
static float Innovation_Feedforward(float innovation)
{
    if (fabsf(innovation) < INNOV_DEADZONE)
        return 0.0f;
    // 超过死区的部分做线性前馈
    float sign = (innovation > 0.0f) ? 1.0f : -1.0f;
    return INNOV_FF_GAIN * (innovation - sign * INNOV_DEADZONE);
}

#include "vofa.h"
// float data_vofa[4]={0.0f,0.0f,0.0f,0.0f};
extern PIDInstance fix_speed_x, fix_speed_y; // 速度闭环pid实例
/*新修改的设定值规划s型曲线的速度趋势 并进行云台到底盘的坐标系转换*/
void Set_Plan_Vxyw_Process(Ctrl_data_t ctrl)
{
    Mot_base_t plan_xyw = {0};
    Mot_base_t plan_acc_xyw = {0};
    float angle = 0;
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_XSPEED], ctrl.xyw.vel.x, 0.0f); // 依据控制源发来的控制量，对线速度和角速度做规划
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_YSPEED], ctrl.xyw.vel.y, 0.0f);
    SecondOrder_Set_Follow_Cal(&set_planner_data[WHEEL_ACSPEED], ctrl.xyw.angvel, 0.0f);
    plan_xyw.vel.x = set_planner_data[WHEEL_XSPEED].p;
    plan_xyw.vel.y = set_planner_data[WHEEL_YSPEED].p;
    plan_xyw.angvel = set_planner_data[WHEEL_ACSPEED].p;
    plan_acc_xyw.vel.x = set_planner_data[WHEEL_XSPEED].v; // 云台发来的控制速度进行规划后的加速度
    plan_acc_xyw.vel.y = set_planner_data[WHEEL_YSPEED].v;
    plan_acc_xyw.angvel = set_planner_data[WHEEL_ACSPEED].v;

//     data_vofa[0] = ctrl.xyw.vel.x;
//     data_vofa[1] = plan_xyw.vel.x;
//     data_vofa[2] = plan_acc_xyw.vel.x;
//     data_vofa[3] = set_planner_data[WHEEL_XSPEED].a;
//  VOFA_Trans_Justfloat(4, data_vofa);
//   PIDCalculate(&fix_speed_x, fused_speed_xy.x, plan_xyw.vel.x);
//    PIDCalculate(&fix_speed_y, fused_speed_xy.y, plan_xyw.vel.y);


//    /* ---- 新息前馈（零延迟打滑响应） ---- */
//    float ff_x = Innovation_Feedforward(kalman_innovation_xy.x);
//    float ff_y = Innovation_Feedforward(kalman_innovation_xy.y);

//    data_vofa[0] = Speed_xyw.vel.x;
//    data_vofa[1] = fused_speed_xy.x;
//    data_vofa[2] = plan_xyw.vel.x;
//    VOFA_Trans_Justfloat(sizeof(data_vofa), data_vofa);

//    if(ctrl.motion_mode == FOLLOW_GIMBAL||ctrl.motion_mode == NO_FOLLOW)
//    {

//        plan_xyw.vel.x+=(ff_x + fix_speed_x.Output);
//        plan_xyw.vel.y+=(ff_y + fix_speed_y.Output);
//    }

  // plan_xyw.vel.x+= fix_speed_x.Output;
   //plan_xyw.vel.y+= fix_speed_y.Output;

    // 得到当前控制坐标系和底盘坐标系角度
    angle = ctrl.getYawAngel;

    Speed_xyw.vel = MotModule_Transform(plan_xyw.vel, angle); // 对线速度进行坐标系转换
    Speed_xyw.angvel = plan_xyw.angvel;
    Force_xyw.vel = MotModule_Transform(plan_acc_xyw.vel, angle); // 对线加速度进行坐标系转换
    Force_xyw.angvel = plan_acc_xyw.angvel;
    // Speed_xyw.vel = MotModule_Transform(ctrl.xyw.vel, angle);//控制量直接进行转换，不用平滑处理
    // Speed_xyw.angvel = ctrl.xyw.angvel; // 角速度不参与平滑规划
}

// float update_input_limiter(float K_out)
// {
//     /* 1. 计算控制误差 */
//     float error = 1.0f - K_out;

//     /* 2. 使用PID计算缩放因子调整量 */
//     PID_Calculate(&input_limiter.pid_controller, error);

//     /* 3. 计算原始K_in */
//     input_limiter.K_in_raw = input_limiter.K_in - input_limiter.pid_controller.out;

//     /* 4. 限制原始K_in范围 */
//     if (input_limiter.K_in_raw > 1.0f)
//         input_limiter.K_in_raw = 1.0f;
//     else if (input_limiter.K_in_raw < input_limiter.min_K)
//         input_limiter.K_in_raw = input_limiter.min_K;

//     /* 5. 应用正确的平滑滤波: 当前值 = 平滑因子×新值 + (1-平滑因子)×旧值 */
//     input_limiter.K_in = input_limiter.smooth_factor * input_limiter.K_in_raw +
//                          (1 - input_limiter.smooth_factor) * input_limiter.K_in;

//     input_limiter.K_out_last = K_out;
//     return input_limiter.K_in;
// }

// // 功率限制控制器结构体
// typedef struct
// {
//     float K_in;          // 当前输入缩放因子(平滑后)
//     float K_in_raw;      // 原始计算的缩放因子(平滑前)
//     float K_out_last;    // 上一次输出端限制比例
//     float max_power;     // 系统最大允许功率(W)
//     float min_K;         // 最小缩放因子(0.01~0.1)
//     float smooth_factor; // 平滑因子(0.05~0.2)

//     // PID控制器参数
//     pid_t pid_controller;
// } power_input_limiter_t;

// // 全局实例初始化
// power_input_limiter_t input_limiter = {
//     .K_in = 1.0f,
//     .K_in_raw = 1.0f,
//     .K_out_last = 1.0f,
//     .max_power = 200.0f, // 根据实际系统调整
//     .min_K = 0.05f,
//     .smooth_factor = 0.1f,
//     .pid_controller = PID_INIT(0.8f, 0.05f, 0.02f, 0.2f, 0.1f, -0.1f) // 调整PID限幅
// };

// // 陀螺时，当功率受限时降低陀螺转速保证平移性
// uint8_t gyro_at_powerlim(Mot_base_t *const VxyW, Ctrl_data_t *const ctrlMsg)
// {
//     static uint8_t ret_0 = 0;
//     v_angle_lim.lim_I = 0;
//     if ((fabsf(VxyW->vel.y) > 80.0f || fabsf(VxyW->vel.x) > 80.0f) && PowerCtrl.Power_Maxlim_sum - PowerInfo.Ref.Power_Maxlim < 180.f)
//     {
//         if (PowerCtrl.power_allo_flag == 1)
//             v_angle_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_angle_lim.lim_k);
//         else if (PowerCtrl.power_allo_flag == 2)
//         {
//             v_angle_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_angle_lim.lim_k);
//             v_angle_lim.lim_I = v_angle_lim.lim_max * fabsf(PowerCtrl.Power_Allo_level) / 100.f;
//         }
//     }
//     else
//     {
//         if (v_angle_lim.lim_out > 1.0f)
//             v_angle_lim.lim_out -= 1.0f;
//         else
//             v_angle_lim.lim_out = 0;
//     }
//     // debug
//     //    if (fabsf(VxyW->vel.x) > 500.0f)
//     //        v_angle_lim.lim_out += 0.5f * (fabsf(VxyW->vel.x) / 1000.f + 9);
//     //    if (fabsf(VxyW->vel.y) > 500.0f)
//     //        v_angle_lim.lim_out += 0.5f * (fabsf(VxyW->vel.y) / 1000.f + 9);

//     float pitch = 0, roll = 0;
//     if ((fabsf(VxyW->vel.y) > 500.0f && (get_chassis_pitch(&pitch)) ? fabsf(pitch) > GYRO_ANGEL_MINLIM : 0))
//         if (VxyW->vel.y * pitch > 0) // 此判断实现只针对上坡
//             v_angle_lim.lim_out += 0.2f * (fabsf(VxyW->vel.y) / 1000.f + (fabsf(pitch) - GYRO_ANGEL_MINLIM));
//     if ((fabsf(VxyW->vel.x) > 500.0f && (get_chassis_roll(&roll)) ? fabsf(roll) > GYRO_ANGEL_MINLIM : 0))
//         if (VxyW->vel.x * roll > 0) // 此判断实现只针对上坡
//             v_angle_lim.lim_out += 0.2f * (fabsf(VxyW->vel.x) / 1000.f + (fabsf(roll) - GYRO_ANGEL_MINLIM));

//     v_angle_lim.lim_out = utils_clamp(v_angle_lim.lim_out, 0, v_angle_lim.lim_max);
//     v_angle_lim.lim_level = v_angle_lim.lim_out + v_angle_lim.lim_I;
//     if (v_angle_lim.lim_level >= v_angle_lim.lim_max - 0.001f)
//     {
//         v_angle_lim.lim_level = v_angle_lim.lim_max;
//         ret_0 = 2;
//     }
//     else if (v_angle_lim.lim_level < v_angle_lim.lim_down)
//         ret_0 = 0;
//     VxyW->angvel *= (1.0f - v_angle_lim.lim_level / 100.f);
//     return ret_0;
// }

// // 跟随时，当功率受限时降低平移保证转向
// v_lim_t v_vel_lim = {0.f, 65, 50, 1.0f, 0.f, 0.f};
// uint8_t follow_at_powerlim(Mot_base_t *const VxyW)
// {
//     uint8_t ret = 0;
//     v_vel_lim.lim_I = 0;
//     if (PowerCtrl.power_allo_flag != 0 && fabsf(VxyW->angvel) > 1000.0f) // && PowerCtrl.Power_Maxlim_sum - PowerInfo.Ref.Power_Maxlim < 100.f)
//     {
//         // if(PowerCtrl.k_power_allo)
//         v_vel_lim.lim_out += fabsf((1.0f - get_k_power_allo()) * v_vel_lim.lim_k);
//         if (PowerCtrl.power_allo_flag == 2)
//             v_vel_lim.lim_I = v_vel_lim.lim_max * fabsf(PowerCtrl.Power_Allo_level) / 100.f;
//     }
//     else
//     {
//         if (v_vel_lim.lim_out > 5.0f)
//             v_vel_lim.lim_out -= 5.0f;
//         else
//             v_vel_lim.lim_out = 0;
//     }
//     if (fabsf(VxyW->angvel) > 450.0f)
//     {
//         v_vel_lim.lim_out += 0.1f * fabsf(VxyW->angvel) / 450.f;
//     }
//     v_vel_lim.lim_out = utils_clamp(v_vel_lim.lim_out, 0.f, v_vel_lim.lim_max);
//     v_vel_lim.lim_level = v_vel_lim.lim_out + v_vel_lim.lim_I;
//     if (v_vel_lim.lim_level >= v_vel_lim.lim_max - 0.001f)
//     {
//         v_vel_lim.lim_level = v_vel_lim.lim_max;
//         ret = 1;
//     }
//     VxyW->vel.x *= (1.0f - v_vel_lim.lim_level / 100.f);
//     VxyW->vel.y *= (1.0f - v_vel_lim.lim_level / 100.f);
//     return ret;
// }
