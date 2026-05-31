#include "fun_powerCtrl.h"
#include "drv_utils.h"
#include "func_sd_master.h"
#include "fun_protocolGim.h"
#include "RLS.h"
#include "SuperCap_Com_new.h"
#include "mod_motion.h"
#include "app_RefSystem.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "LK_MF_MG.h"
#include "robot_control.h"
#include "drv_motionSolve.h"
#include "drv_Custom.h"

#ifdef BSP_USING_VOFA
#include "vofa.h"
#endif

#ifdef BSP_USING_LOG
#define LOG_TAG "power_Log"
#include <ulog.h>
#endif

#define DELTASPEEDERR 4000
#define fabs(x) (x > 0 ? x : -x)
#define SIGNAL(x) (x / x)
// #define RunERRhigh 900
// #define RunERRlow 200

// #define AngleERRhigh 200
// #define AngleERRlow 50
// #define fabs(x) (x > 0 ? x : -x)
// #define SIGNAL(x) (x / x)
// #define SAFE_MIN 1e-12f // 防止除零的最小值
#define WHEEL_RUN 0
#define WHEEL_ANGLE 1


Power_info_t PowerInfo;
PowerCtrl_t PowerCtrl;
// Power_Set_t Power_Set;

// TODO 电机控制参数结构体申明
Motor_Power_t Motor_power[WHEELS_GROUP_NUM][2];

// note 修改裁判系统和超电数据有效性标志位
uint8_t PowerCtrl_Close_Flag[2] = {1, 0}; // 是否抛弃超电数据，是否抛弃裁判系统数据
uint8_t Cap_false_flag = 0;               // 底盘检测超电可能掉线标志，标记超电异常标志位（用于UI告知操作手）,1为异常，0为正常
// RLS_Estimator RunRLS;
// RLS_Estimator AngleRLS;

// float RunPhi[4], AnglePhi[4];
// float WheelInitTheta[3], SteerInitTheta[3];
// float power_speed_max = 9000;

// 传入的参数为电机组总地址，需要区分舵向电机和轮向电机，用于索引
// 以下设计均是每个部分将对应的舵向电机和轮向电机分别计算
uint8_t if_steer_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
uint8_t if_wheel_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
                                               // 0:不使用 1:使用

MotorInstance *Motor_Power_Pointer[2][WHEELS_GROUP_NUM];
// uint8_t get_over_voltage_flag=0;
uint8_t run_count = 0;
uint8_t angle_count = 0;

void Power_Motor_Attach(MotorInstance *motor_instance)
{
    if (motor_instance->Motor_obj.power_ctrl_type == POWER_CTRL_WHEEL)
    {
        if_wheel_used[run_count] = 1;
        Motor_Power_Pointer[WHEEL_RUN][run_count++] = motor_instance; // 注册电机实例指针
    }
    else if (motor_instance->Motor_obj.power_ctrl_type == POWER_CTRL_STEER)
    {
        if_steer_used[angle_count] = 1;
        Motor_Power_Pointer[WHEEL_ANGLE][angle_count++] = motor_instance; // 注册电机实例指针
    }
}

float get_K_IC2Tor(MotorInstance *motor_instance)
{
    void *measure_p = (void *)(motor_instance->Motor_obj.motor_measure);
    switch (motor_instance->Motor_obj.motor_type)
    {
    case M3508:
        DJI_3508_Motor_Measure_s *P_3508 = (DJI_3508_Motor_Measure_s *)measure_p;
        if (P_3508)
        return (P_3508->current_to_real) * (P_3508->current_to_torque) * 19.2f /(P_3508->RATIO); // 输出轴扭矩/count，与功率模型单位一致
     case GM6020:
            DJI_6020_Motor_Measure_s *P_6020 = (DJI_6020_Motor_Measure_s *)measure_p;
        if (P_6020) return (P_6020->current_to_real) * (P_6020->current_to_torque);
    case LK_ms6015:
        LK_Motor_Measure_s *LK_p = (LK_Motor_Measure_s *)measure_p;
        if (LK_p)
            return (LK_p->current_to_real) * (LK_p->current_to_torque);
    default:
        break;
    }
}

float get_Ratio(MotorInstance *motor_instance)
{
    void *measure_p = (void *)(motor_instance->Motor_obj.motor_measure);
    switch (motor_instance->Motor_obj.motor_type)
    {
    case M3508:
        DJI_3508_Motor_Measure_s *P_3508 = (DJI_3508_Motor_Measure_s *)measure_p;
        if (P_3508)
            return (P_3508->RATIO);
    case GM6020:
        DJI_6020_Motor_Measure_s *P_6020 = (DJI_6020_Motor_Measure_s *)measure_p;
        if (P_6020)
            return (P_6020->RATIO);
    case LK_ms6015:
        LK_Motor_Measure_s *LK_p = (LK_Motor_Measure_s *)measure_p;
        if (LK_p)
            return (LK_p->RATIO);
    default:
        break;
    }
}
void powerCtrlInit(void)
{
    rt_memset(&PowerInfo, 0, sizeof(PowerInfo));
    rt_memset(&PowerCtrl, 0, sizeof(PowerCtrl));
    // rt_memset(&Power_Set, 0, sizeof(Power_Set));
    rt_memset(Motor_power, 0, sizeof(Motor_power));

    PowerInfo.Cap.SupplyCurrt = Supr_Cap_MaxOutputCurrent;
    PowerInfo.Ref.PowerBuff = PowerCtrlReamin; // 防止第一次运行功率控制，但裁判系统还没有通信导致认为超电异常
    PowerCtrl.nowRemain = PowerCtrlReamin;

    for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        Motor_power[group][WHEEL_RUN].K1 = Wheel_Power_K1;
        Motor_power[group][WHEEL_RUN].K2 = Wheel_Power_K2;
        Motor_power[group][WHEEL_RUN].C = Wheel_Power_C;
        Motor_power[group][WHEEL_ANGLE].K1 = Steer_Power_K1;
        Motor_power[group][WHEEL_ANGLE].K2 = Steer_Power_K2;
        Motor_power[group][WHEEL_ANGLE].C = Steer_Power_C;
    }

    // RLS_Init(&RunRLS, WheelInitTheta, 1);
    // RLS_Init(&AngleRLS, SteerInitTheta, 0.01);
}

static float powerfilterK = 0.15f; // 从超级电容获取功率数据低通滤波系数
/**
 * @brief 一阶低通滤波
 *
 * @param Data_now  当前值
 * @param Data_Last 上次值
 * @param input     输入值
 * @param D         参数D
 * @author zqjj
 */
static void utils_Low_Pass_Filter_Float(float *Data_now, float *Data_Last, float input, float D)
{
    *Data_now = UTILS_LP_FAST(*Data_Last, input, D);
    *Data_Last = *Data_now;
}

static void getSCapDataAndInfo(void)
{
    // 超级电容数据有效性，判断超电是否离线
    PowerInfo.Cap.isValid = Read_Super_Capacity_Online();
    // 更新超级电容功率数据
    PowerInfo.Cap.isValid |= Get_RemainCapcity(&PowerInfo.Cap.superCap);  // 更新超级电容剩余电量
    PowerInfo.Cap.superCap = utils_clamp(PowerInfo.Cap.superCap, 0, 100); // 限幅
    PowerInfo.Cap.isValid |= getSuprCapMearData(&PowerInfo.Cap.Voltage, &PowerInfo.Cap.Current, &PowerInfo.Cap.V_Cap);
    PowerInfo.Cap.Cap_State = Get_SuprCapState();
    PowerInfo.Cap.isValid |= Get_SuprCapPower(&PowerInfo.Cap.Power); // 获取超级电容测得底盘功率

// #ifdef BSP_USING_VOFA
//     float data[6] = {0,0,0,0,0,0};
//     data[0] = PowerInfo.Cap.Power;
//     data[1] = real_xyw.vel.x;
//     data[2] = -real_xyw.vel.y;//为了与vofa控件方向保持一致
//     data[3] = -real_xyw.angvel;
//     data[4] = PowerInfo.Cap.superCap;
//     data[5] = PowerInfo.Ref.PowerBuff;
//     VOFA_Trans_Justfloat(6, data);
// #endif
    
    // 转化单位为国际单位
    PowerInfo.Cap.Voltage /= 1000.f;
    PowerInfo.Cap.Current /= 1000.f;
    // 对功率数据进行低通滤波，并刷新至Power_last中
    utils_Low_Pass_Filter_Float(&PowerInfo.Cap.Power, &PowerInfo.Cap.Power_last, PowerInfo.Cap.Power, powerfilterK);

    // 计算超级电容供电能力
    // 按电压分段式线性电流响应放电，//SCAP_MAX_CURRENT参考值：8A
    if (PowerInfo.Cap.V_Cap > 16)
        PowerInfo.Cap.SupplyCurrt = SCAP_MAX_CURRENT;
    else if (PowerInfo.Cap.V_Cap > 10)
        PowerInfo.Cap.SupplyCurrt = (SCAP_MAX_CURRENT - 3.0f) + 3.0f * utils_calc_ratio(10, 16, PowerInfo.Cap.V_Cap);
    else if (PowerInfo.Cap.V_Cap > 7)
        PowerInfo.Cap.SupplyCurrt = (SCAP_MAX_CURRENT - 3.0f) * utils_calc_ratio(7, 10, PowerInfo.Cap.V_Cap);
    else
        PowerInfo.Cap.SupplyCurrt = 0;
    // 超级电容离线/超级电容通讯离线
    if (PowerInfo.Cap.isValid == RT_FALSE)
    {
        PowerInfo.Cap.SupplyCurrt = 0;
        PowerInfo.Cap.Current = 0;
        PowerInfo.Cap.superCap = 0;
        PowerInfo.Cap.Voltage = 24;
        PowerInfo.Cap.Cap_State = CAP_STATE_ERROR;
    }
}
rt_uint16_t power_max_limit_test = 50;
// 得到裁判系统的关于底盘功率的数据 50Hz更新 20ms,判读数据是否合理
static void getRefDataAndInfo(void)
{
    static uint32_t overPowerTick = 0;
    rt_tick_t tick_tmp = 0;
    PowerInfo.Ref.isValid = ISRefValid(); // 数据是否有效
    if (1 == PowerInfo.Ref.isValid)
    {
        /**以下数据计算后为国际单位**/
        getRefData(memberGet_parm(PowerInfo.Ref.PowerBuff, ext_power_heat_data, chassis_power_buffer)); // 缓冲能量J
        // PowerInfo.Ref.PowerBuff = 60; // 缓冲能量J
         getRefData(memberGet_parm(PowerInfo.Ref.Power_Maxlim, ext_game_robot_status, chassis_power_limit)); // 最大功率限制W
        // PowerInfo.Ref.Power_Maxlim = 100;//power_max_limit_test; // 最大功率限制W
        getRefData(memberGet_parm(PowerInfo.Ref.power_now, ext_power_heat_data, chassis_power_reserve));
        // PowerInfo.Ref.IsOverPower = Ref_Chassis_IsOverPower(&tick_tmp); // 是否超功率及超功率时刻，串口协议1.90已经删除
        // 裁判系统功率数据更新时刻：电流，电压，功率，缓冲能量
        PowerInfo.Ref.refresh_tick = GetRefDataRefreshTick(Index_power_heat_data);

        if (PowerInfo.Ref.PowerBuff <= 0)      // 缓冲能量耗尽
            PowerInfo.Ref.powerBuffOverCnts++; // 缓冲耗尽计数

        // if (PowerInfo.Ref.IsOverPower && tick_tmp != overPowerTick) // 是否超功率且超功率时间不同，则计数
        // {                                                           // 超功率时间刷新并计数
        //     overPowerTick = tick_tmp;
        //     PowerInfo.Ref.powerOverCnts++;
        // }
        PowerInfo.Ref.IsFlySlope = RefGetIsFlySlope(); // 是否飞坡
        // 判断合理性与基本限幅,若未飞坡，则缓冲能量不多于60J，若飞坡，缓冲能量不多于250J
        if (0 == PowerInfo.Ref.IsFlySlope)
            PowerInfo.Ref.PowerBuff = PowerInfo.Ref.PowerBuff > RefPowerBuf ? RefPowerBuf : PowerInfo.Ref.PowerBuff;
        else
            PowerInfo.Ref.PowerBuff = PowerInfo.Ref.PowerBuff > POWER_REMANI_FLYSLOPE ? POWER_REMANI_FLYSLOPE : PowerInfo.Ref.PowerBuff;

        // 缓冲能量限幅
        PowerInfo.Ref.PowerBuff = utils_clamp(PowerInfo.Ref.PowerBuff, 0, POWER_REMANI_FLYSLOPE);
    }
    else
    { // 裁判系统数据无效，保守限制：
        PowerInfo.Ref.Power_Maxlim = POWER_REMANI;
        PowerInfo.Ref.PowerBuff = PowerCtrlReamin - 5;
    }
}

static void getPowerData(void)
{
    // 得到超级电容数据
    getSCapDataAndInfo();
    // 得到裁判系统数据并做基本限幅
    getRefDataAndInfo();
}
uint8_t test_wr = 0;
/**
 * @brief 超电输出功能开关
 *
 * @param GimIfSuperCapOutput    超电输出功能开关（来自云台按键）
 */
// 满电滞回保护参数（暂时注释，备用）
// #define SCAP_HYST_HIGH  90.0f   // 接近满电阈值(%)，超过此值进入只放不充
// #define SCAP_HYST_LOW   85.0f   // 滞回下限(%)，低于此值恢复正常充放
static void SCap_PowerCtrl(uint8_t GimIfSuperCapOutput)
{
    static uint8_t user_enable = 1;              // 用户设置的放电状态
    static uint8_t pre_button_state = 1;         // 前次按键状态
    static uint8_t if_used_up = 0;               // 是否电量耗尽（<3%）
    static uint8_t if_used_up_user_unchange = 0; // 耗尽后用户是否已手动切换

    // 裁判系统有效时：缓冲能量过低或超电掉线 → 强制关闭
    if (PowerInfo.Ref.isValid)
    {
        if (PowerInfo.Ref.PowerBuff <= PowerBuffReamin || PowerInfo.Cap.Cap_State == CAP_STATE_ERROR)
        {
            //Cap_false_flag = 1;
            Set_SuprCap_OutputMode(0);   // 关闭超电输出
            PowerCtrl_Close_Flag[0] = 1;
        }
    }

   
    // 云台按键边沿检测
    if (pre_button_state != GimIfSuperCapOutput)
    {
        user_enable = !user_enable;
        if_used_up_user_unchange = 0;
        pre_button_state = GimIfSuperCapOutput;
    }

    // 超电电量耗尽检测（<3% 强制关闭放电）
    if (PowerInfo.Cap.superCap <= 3.0f)
    {
        if_used_up = 1;
        if_used_up_user_unchange = 1;
        user_enable = 0;
    }
    else if (PowerInfo.Cap.superCap > 3.0f)
    {
        if_used_up = 0;
        if_used_up_user_unchange=0;
    }



    #ifdef CORE_USING_INFANTRY
    if ( Get_Wireless_Charge())//!PowerCtrl_Close_Flag[0] &&
    //if (!PowerCtrl_Close_Flag[0] && test_wr)
    {
			PowerCtrl_Close_Flag[0]=1;
        Set_SuprCap_OutputMode(2); // 只放不充（无线充电期间禁止超电充电）
    }
    // 正常工作：超电在线且未耗尽时正常充放
    else if (if_used_up == 0 && user_enable == 1&&!if_used_up_user_unchange)
    {
        Set_SuprCap_OutputMode(1); // 正常充放
        PowerCtrl_Close_Flag[0] = 0;
    }
    else
    {
        Set_SuprCap_OutputMode(0); // 电量耗尽 → 关闭
        PowerCtrl_Close_Flag[0] = 1;
		//pre_button_state=0;
    }
    #else

     if (if_used_up == 0 && user_enable == 1)
    {
        Set_SuprCap_OutputMode(1); // 正常充放
        PowerCtrl_Close_Flag[0] = 0;
    }
    else
    {
        Set_SuprCap_OutputMode(0); // 电量耗尽 → 关闭
        PowerCtrl_Close_Flag[0] = 1;
		//pre_button_state=0;
    }

    #endif

         #ifdef CORE_USING_SENTRY
                 Set_SuprCap_OutputMode(1); // 正常充放
        PowerCtrl_Close_Flag[0] = 0;
        #endif

    // ── 满电滞回控制（暂时注释，备用）──────────────────────────────────
    // 防止接近满电时反复充放电过热：
    //   superCap > SCAP_HYST_HIGH(90%) → 进入"只放不充"模式(2)
    //   superCap < SCAP_HYST_LOW(85%)  → 恢复正常充放(1)
    //   中间区间 → 维持上次模式（死区）
    //
    // if (PowerInfo.Cap.isValid && if_used_up == 0)
    // {
    //     static uint8_t hyst_mode = 1; // 1=正常充放, 2=只放不充
    //
    //     if (PowerInfo.Cap.superCap > SCAP_HYST_HIGH)
    //         hyst_mode = 2;
    //     else if (PowerInfo.Cap.superCap < SCAP_HYST_LOW)
    //         hyst_mode = 1;
    //
    //     Set_SuprCap_OutputMode(hyst_mode);
    //     PowerCtrl_Close_Flag[0] = 0;
    // }
    // else if (if_used_up == 1)
    // {
    //     Set_SuprCap_OutputMode(0);
    //     PowerCtrl_Close_Flag[0] = 1;
    // }
}

// uint8_t get_over_voltage(void)
// {
//     return get_over_voltage_flag;
// }
// 在UI界面告知操作手超电可能电容组放电或者报废异常
uint8_t get_SuperCap_if_abnormal(void)
{
    return Cap_false_flag;
}
uint8_t get_Super_Cap_if_open(void)
{
    return Get_if_supercap_output();
}


float k1_test=Wheel_Power_K1;
float k2_test = Wheel_Power_K2;
float c_test = 0;
static void get_chassis_max_power(void)
{

    PowerCtrl.Power_Allo_level = 0;
   // PowerCtrl_Close_Flag[0]=1;//关闭超电使用
    if (PowerInfo.Cap.isValid && Get_if_supercap_output() && !PowerCtrl_Close_Flag[0])
    { // 超级电容数据有效，没有异常，没有关闭数据使用
        if (PowerInfo.Ref.isValid)
        { // 当裁判系统功率有效
            // 根据功率上限，缓冲能量以及超级电容输出电流计算当前动态最多功率
            if (PowerInfo.Ref.PowerBuff < PowerCtrlReamin)
            { // 缓冲能量已经小于预留能量
                PowerCtrl.Power_Maxlim_sum = PowerInfo.Ref.Power_Maxlim +
                                             PowerInfo.Cap.SupplyCurrt * PowerInfo.Cap.Voltage;
            }
            else
            {
                PowerCtrl.Power_Maxlim_sum = PowerInfo.Ref.Power_Maxlim +
                                             PowerInfo.Cap.SupplyCurrt * PowerInfo.Cap.Voltage +
                                             (PowerInfo.Ref.PowerBuff - PowerCtrl.nowRemain) * 1000.f / (RefPowerData_RefTime + rt_tick_get() - PowerInfo.Ref.refresh_tick);
            }
#if IF_USED_ALLO_LEVEL != 0
            // 功率强制保护限制
            if (PowerInfo.Ref.PowerBuff < PowerCtrlReamin && PowerInfo.Ref.PowerBuff > PowerBuffReamin)
            {
                PowerCtrl.Power_Allo_level = -100 * (1.0f - utils_calc_ratio(PowerBuffReamin, PowerCtrlReamin, PowerInfo.Ref.PowerBuff));
            }
            else if (PowerInfo.Ref.PowerBuff <= PowerBuffReamin)
            {
                PowerCtrl.Power_Allo_level = -100;
            }
            // 此位置可以继续添加强制限制功率的条件，满足条件时只需将分配等级继续负向增大即可增强功率限制（注意和下面保持对应）
            PowerCtrl.Power_Allo_level = utils_clamp(PowerCtrl.Power_Allo_level, -100.f, 0.f);
#endif
        }
        else
        { // 裁判系统通信异常
            PowerCtrl.Power_Maxlim_sum = POWER_REMANI +
                                         PowerInfo.Cap.SupplyCurrt * PowerInfo.Cap.Voltage;
        }
    }
    else if (PowerInfo.Ref.isValid && !PowerCtrl_Close_Flag[1])
    { // 超级电容异常但裁判系统有效
        if (PowerInfo.Ref.PowerBuff < PowerCtrlReamin)
            PowerCtrl.Power_Maxlim_sum = PowerInfo.Ref.Power_Maxlim;
        else
            PowerCtrl.Power_Maxlim_sum = PowerInfo.Ref.Power_Maxlim +
                                         (PowerInfo.Ref.PowerBuff - PowerCtrl.nowRemain) * 1000.f / (RefPowerData_RefTime + rt_tick_get() - PowerInfo.Ref.refresh_tick);
#if IF_USED_ALLO_LEVEL != 0
        if (PowerInfo.Ref.PowerBuff < PowerCtrlReamin && PowerInfo.Ref.PowerBuff > PowerBuffReamin)
        {
            PowerCtrl.Power_Allo_level = -100 * (1.0f - utils_calc_ratio(PowerBuffReamin, PowerCtrlReamin, PowerInfo.Ref.PowerBuff));
        }
        else if (PowerInfo.Ref.PowerBuff <= PowerBuffReamin)
        {
            PowerCtrl.Power_Allo_level = -100;
        }
        // 此位置可以继续添加强制限制功率的条件，满足条件时只需将分配等级继续负向增大即可增强功率限制（注意和前面保持对应）
        PowerCtrl.Power_Allo_level = utils_clamp(PowerCtrl.Power_Allo_level, -100.f, 0.f);
#endif
    }
    else
    { // 全部异常
        PowerCtrl.Power_Maxlim_sum = POWER_REMANI;
    }
    // 该功率计算式子满足在分配等级为0时与正常功率限制计算等效，当分配等级功能未启用时Power_Allo_level始终为0，符合逻辑
    // 一部分固定的静态功率不属于电机可用功率：即要减去Power_C_sum
    PowerCtrl.Power_Maxlim = (PowerCtrl.Power_Maxlim_sum - Power_C_sum) * (K_Conservative + PowerCtrl.Power_Allo_level * One_Level_Persent);
    static char str[30] = {0} ;
    static int count = 0;
    count++;
    if(count==500)
    {
        count = 0;
        int i = sprintf(str, "%d", (int)PowerCtrl.Power_Maxlim_sum);
        Custom_Message_Send(str);
    }
}

#if Power_Ctrl_Debug == 1
// debug参数：
float now_power_sum;
float out_power_sum; // 此数据目前不正常，但没有问题，见计算处的注释
#endif

float power_set_sum; //  预输出总功率   // debug可参考
float power_set_sum_temp = 0; // 用于查看计算的电机预输出总功率

float steer_power = 0;
float run_power = 0;
static void Power_lim_allo_to_wheels()
{
    /*程序在运行这个函数的时候，PID的输出wheel[group][wheel_type].motor.spe.out已经根据初始化时的设置进行方向回反了，
     * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
     * 对于wheel[group][wheel_type].motor.dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
     */

    for(int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机
            
            Motor_power[group][WHEEL_RUN].K1 = k1_test;
            Motor_power[group][WHEEL_RUN].K2 = k2_test;
        }
    }



    //*电机计算相关参数更新
    // get_true_RLS();

    uint8_t group = 0, power_allo_flag_temp = 0;
    float temp_run = 0, temp_angle = 0,power_set_sum_temp = 0;

    float power_set_steer = 0, power_set_wheel = 0;
    // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
    // for (group = 0; group < WHEELS_GROUP_NUM; group++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
    // {
    //     //        轮向电机调整方向
    //     if (if_wheel_used[group] == 1)
    //        //     Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Current_pid.Output *= (Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.reverse_flag);
    //     // 舵向电机调整方向
    //     if (if_steer_used[group] == 1)
    //        //  Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output *= (Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.reverse_flag);
    // }

    //* 获得预输出功率
    // get_pre_power();
#if Power_Ctrl_Debug == 1
    float out_power_sum_temp = 0; // debug
#endif

    for (int group = 0; group < WHEELS_GROUP_NUM;group++)
    {
        if (if_steer_used[group] == 1)
        {
            // 舵向电机速度刷新至
            Motor_power[group][WHEEL_ANGLE].V_last = Motor_power[group][WHEEL_ANGLE].V_now;
            Motor_power[group][WHEEL_ANGLE].V_now = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.speed ; // 原始转子转速数据(rpm)
        }
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机速度刷新至
            Motor_power[group][WHEEL_RUN].V_last = Motor_power[group][WHEEL_RUN].V_now;
            Motor_power[group][WHEEL_RUN].V_now = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.speed ; // 原始转子转速数据(rpm)
        }
        /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
    

#if Power_Ctrl_Debug == 1
    if (if_steer_used[group] == 1)
    {
        Motor_power[group][WHEEL_ANGLE].I_now = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.current;                         // 原始扭矩电流数据
        Motor_power[group][WHEEL_ANGLE].tor_now = fabsf(Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.torque);                       // 转子扭矩计算
        Motor_power[group][WHEEL_ANGLE].power_now = fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * Motor_power[group][WHEEL_ANGLE].tor_now / 9.55f +
                                                    Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(Motor_power[group][WHEEL_ANGLE].tor_now) +
                                                    Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
                                                    Motor_power[group][WHEEL_ANGLE].C; // 电机功率模型计算公式
        if (Motor_power[group][WHEEL_ANGLE].power_now > 0)
            temp_angle += Motor_power[group][WHEEL_ANGLE].power_now;
    }
    if (if_wheel_used[group] == 1)
    {
        Motor_power[group][WHEEL_RUN].I_now = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.current;    // 原始扭矩电流数据
        Motor_power[group][WHEEL_RUN].tor_now = fabsf(Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.torque);   // 转子扭矩计算
        Motor_power[group][WHEEL_RUN].power_now = fabsf(Motor_power[group][WHEEL_RUN].V_now) * Motor_power[group][WHEEL_RUN].tor_now / 9.55f +
                                                  Motor_power[group][WHEEL_RUN].K1 * SQUARE(Motor_power[group][WHEEL_RUN].tor_now) +
                                                  Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) +
                                                  Motor_power[group][WHEEL_RUN].C; // 电机功率模型计算公式
        if (Motor_power[group][WHEEL_RUN].power_now > 0)
            temp_run += Motor_power[group][WHEEL_RUN].power_now;
    }
#endif
    if (if_steer_used[group] == 1)
    {
        // 舵向电机
        Motor_power[group][WHEEL_ANGLE].initial_tor_set = fabsf(Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.torque); // 转子扭矩计算
        // 功率分配前的预输出功率
        Motor_power[group][WHEEL_ANGLE].initial_power_set = fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * Motor_power[group][WHEEL_ANGLE].initial_tor_set / 9.55f +
                                                            Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(Motor_power[group][WHEEL_ANGLE].initial_tor_set) +
                                                            Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
                                                            Motor_power[group][WHEEL_ANGLE].C; // 电机功率模型计算公式
        power_set_steer += Motor_power[group][WHEEL_ANGLE].initial_power_set;
    }
    if (if_wheel_used[group] == 1)
    {
        // 轮向电机
        Motor_power[group][WHEEL_RUN].initial_tor_set = fabsf(Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.torque); // 转子扭矩计算
        // 功率分配前的预输出功率
        Motor_power[group][WHEEL_RUN].initial_power_set = fabsf(Motor_power[group][WHEEL_RUN].V_now) * Motor_power[group][WHEEL_RUN].initial_tor_set / 9.55f +
                                                          Motor_power[group][WHEEL_RUN].K1 * SQUARE(Motor_power[group][WHEEL_RUN].initial_tor_set) +
                                                          Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) +
                                                          Motor_power[group][WHEEL_RUN].C; // 电机功率模型计算公式
        power_set_wheel += Motor_power[group][WHEEL_RUN].initial_power_set;
    }
}
// 计算总功率大小(舵向电机功率+轮向电机功率)
steer_power = power_set_steer;
run_power = power_set_wheel;
power_set_sum_temp = power_set_steer + power_set_wheel;
#if Power_Ctrl_Debug == 1
now_power_sum = temp_run + temp_angle + Power_C_sum; // debug
#endif
power_set_sum = power_set_sum_temp; // 电机预输出的总功率（不包括固定静态功率）

if (power_set_sum > PowerCtrl.Power_Maxlim) // determine if larger than max power
{
    if (PowerCtrl.Power_Allo_level >= 0)
        power_allo_flag_temp = 1; // 正常功率限制模式标志：1
    else
        power_allo_flag_temp = 2; // 强制功率限制模式标志：2
    // 一部分固定的静态功率不属于电机可用功率：即要使用减去Power_C_sum的功率数据，PowerCtrl.Power_Maxlim和power_set_sum的值已不包括静态功率
    // 现在的分配办法是：统计舵向电机和轮向电机总的预输出功率，然后根据比例分配系数来分配功率
    PowerCtrl.k_power_allo_steer = (PowerCtrl.Power_Maxlim / power_set_sum) * (power_set_steer / power_set_sum); // 功率分配系数
    PowerCtrl.k_power_allo_wheel = (PowerCtrl.Power_Maxlim / power_set_sum) * (power_set_wheel / power_set_sum); // 功率分配系数
}
else
{
    PowerCtrl.k_power_allo_steer = 1.0f; // 全时可引用参数对外更新
    PowerCtrl.k_power_allo_wheel = 1.0f;
}
#if IF_USED_ALLO_LEVEL == 1
// 加速度增益，还未完善，先别用！！！
else if (PowerCtrl.Power_Maxlim - power_set_sum >= Gain_Trigger_Power)
{ // PID set 的控制是转子转速rpm,
    power_allo_flag_temp = 3;
    power_set_sum = Power_C_sum; // power_set_sum运行到这里也完成使命了，这里拿来统计增益功率重分配后的总功率
    for (group = 0; group < 4; group++)
    {
        Motor_power[group].gain_allo_level = fabsf(Motor_power[group].V_now - wheel[group].spe.set) / 2000.0;
        Motor_power[group].k_gain_power_allo = (K_Conservative + Motor_power[group].gain_allo_level * One_Level_Persent);
        Motor_power[group].allo_power_set = Motor_power[group].initial_power_set * Motor_power[group].k_gain_power_allo;
        power_set_sum += Motor_power[group].allo_power_set;
    }
    if (power_set_sum >= PowerCtrl.Power_Maxlim) // 保险：增益功率重分配后检查
        power_allo_flag_temp = 0;
}
#endif
PowerCtrl.power_allo_flag = power_allo_flag_temp; // 功率分配标志位
#if IF_USED_ALLO_LEVEL == 2
if (PowerCtrl.Power_Maxlim < 1.0f)
{
    for (group = 0; group < 4; group++)
    {
        if (if_steer_used[group] == 1)
        {
            Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = 0;
            Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Current_pid.Output = 0;
        }
        if (if_wheel_used[group] == 1)
        {
            Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = 0;
            Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Current_pid.Output = 0;
        }
       
    }
}
else
#endif
    if (PowerCtrl.power_allo_flag != 0)
{
    for (group = 0; group < 4;group++)
    {
#if IF_USED_ALLO_LEVEL == 1
        if (PowerCtrl.power_allo_flag == 3) // 增益情况下的功率分配计算方法不同，且由于在增益条件判断和检查时需要用到，已经计算过了，这里不再计算
            ;
        else
#endif
        {
            if (if_steer_used[group] == 1)
            {
                Motor_power[group][WHEEL_ANGLE].allo_power_set = Motor_power[group][WHEEL_ANGLE].initial_power_set * PowerCtrl.k_power_allo_steer; // 功率分配后的预输出功率
            }
            if (if_wheel_used[group] == 1)
            {
                Motor_power[group][WHEEL_RUN].allo_power_set = Motor_power[group][WHEEL_RUN].initial_power_set * PowerCtrl.k_power_allo_wheel; // 功率分配后的预输出功率
            }
        }
        // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
        if (if_steer_used[group] == 1)
        {
            // 先检查预算是否至少覆盖速度相关损耗，若不覆盖则无法输出正向扭矩
            float speed_loss = Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) + Motor_power[group][WHEEL_ANGLE].C;
            float disc = SQUARE(Motor_power[group][WHEEL_ANGLE].V_now / 9.55f) -
                         4 * Motor_power[group][WHEEL_ANGLE].K1 * (speed_loss - Motor_power[group][WHEEL_ANGLE].allo_power_set);
            if (disc < 0) disc = 0; // 防sqrtf负数
            if (SIGN_F(Motor_power[group][WHEEL_ANGLE].V_now) * SIGN_F(Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output) >= 0)
            {
                // 电机转速与输出扭矩方向相同（电动），反电动势与扭矩方向相反
                Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f +
                                                                sqrtf(disc)) /
                                                               (2 * Motor_power[group][WHEEL_ANGLE].K1);
                // 预算不足以覆盖速度损耗时，二次方程给负值 → 钳位到0
                if (Motor_power[group][WHEEL_ANGLE].allo_tor_set < 0)
                    Motor_power[group][WHEEL_ANGLE].allo_tor_set = 0;
            }
            else
            {
                // 电机转速与输出扭矩方向相反（制动），反电动势与扭矩方向相同
                Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f -
                                                                sqrtf(disc)) /
                                                               (2 * Motor_power[group][WHEEL_ANGLE].K1);
            }
            temp_angle = fabsf(Motor_power[group][WHEEL_ANGLE].allo_tor_set); // 取扭矩绝对值
            // 功率分配只应减小扭矩，绝不应放大。钳制到不超过原始请求
            if (temp_angle > Motor_power[group][WHEEL_ANGLE].initial_tor_set)
                temp_angle = Motor_power[group][WHEEL_ANGLE].initial_tor_set;
            if (Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output >= 0)
                Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = temp_angle / get_K_IC2Tor(Motor_Power_Pointer[WHEEL_ANGLE][group]);
            else
                Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = -temp_angle / get_K_IC2Tor(Motor_Power_Pointer[WHEEL_ANGLE][group]);
            // CAN驱动发送的是Current_pid.Output，必须同步更新，否则功率限制不生效
            Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Current_pid.Output =
                Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output;
        }
        if(if_wheel_used[group] == 1)
        {
            float speed_loss = Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) + Motor_power[group][WHEEL_RUN].C;
            float disc = SQUARE(Motor_power[group][WHEEL_RUN].V_now / 9.55f) -
                         4 * Motor_power[group][WHEEL_RUN].K1 * (speed_loss - Motor_power[group][WHEEL_RUN].allo_power_set);
            if (disc < 0) disc = 0;
            if (SIGN_F(Motor_power[group][WHEEL_RUN].V_now) * SIGN_F(Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output) >= 0)
            {
                // 电机转速与输出扭矩方向相同（电动），反电动势与扭矩方向相反
                Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f +
                                                              sqrtf(disc)) /
                                                             (2 * Motor_power[group][WHEEL_RUN].K1);
                if (Motor_power[group][WHEEL_RUN].allo_tor_set < 0)
                    Motor_power[group][WHEEL_RUN].allo_tor_set = 0;
            }
            else
            {
                // 电机转速与输出扭矩方向相反（制动），反电动势与扭矩方向相同
                Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f -
                                                              sqrtf(disc)) /
                                                             (2 * Motor_power[group][WHEEL_RUN].K1);
            }
            temp_run = fabsf(Motor_power[group][WHEEL_RUN].allo_tor_set); // 取扭矩绝对值
            if (temp_run > Motor_power[group][WHEEL_RUN].initial_tor_set)
                temp_run = Motor_power[group][WHEEL_RUN].initial_tor_set;
            if (Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output >= 0)
                Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = temp_run / get_K_IC2Tor(Motor_Power_Pointer[WHEEL_RUN][group]);
            else
                Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output = -temp_run / get_K_IC2Tor(Motor_Power_Pointer[WHEEL_RUN][group]);
            // CAN驱动发送的是Current_pid.Output，必须同步更新，否则功率限制不生效
            Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Current_pid.Output =
                Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Output;
        }
#if Power_Ctrl_Debug == 1
        if (if_steer_used[group] == 1)
        {
            // 注：用反解出的扭矩再计算输出总功率，但这里计算完的总功率数据是错误的，而扭矩对应的输出功率并没有错，原因待思考
            out_power_sum_temp += fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * temp_angle / 9.55f +
                                  Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(temp_angle) +
                                  Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
                                  Motor_power[group][WHEEL_ANGLE].C;
        }
        if (if_wheel_used[group] == 1)
        {
            out_power_sum_temp += fabsf(Motor_power[group][WHEEL_RUN].V_now) * temp_run / 9.55f +
                                  Motor_power[group][WHEEL_RUN].K1 * SQUARE(temp_run) +
                                  Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) +
                                  Motor_power[group][WHEEL_RUN].C;
        }
#endif
    }
#if Power_Ctrl_Debug == 1
    out_power_sum = out_power_sum_temp + Power_C_sum; // debug
#endif
}

//___________________________________
    
}

// /**
//  * @brief 设定功率超过最大值时功率再分配
//  *
//  * @param wheel
//  */
// float RunKcoe;
// float AngleKcoe; //  置信度，小于ERRlow时为0，大于ERRhigh时为1，在上下阈值之间线性变化
// void Power_re_allo(void)
// {

//     Power_Set.run_err_sum = 0;
//     Power_Set.angle_err_sum = 0;
//     Power_Set.run_err_sq_sum = 0;
//     for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++)
//     {
//         Power_Set.run_err_sum += fabsf(Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Err);
//         Power_Set.run_err_sq_sum += SQUARE(Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Err);
//         Power_Set.angle_err_sum += fabsf(Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Err);
//     }

//     if (Power_Set.run_err_sum > RunERRhigh)
//         RunKcoe = 1;
//     else if (Power_Set.run_err_sum <= RunERRlow)
//         RunKcoe = 0;
//     else
//         RunKcoe = (Power_Set.run_err_sum - RunERRlow) / (RunERRhigh - RunERRlow);

//     if (Power_Set.angle_err_sum > AngleERRhigh)
//         AngleKcoe = 1;
//     else if (Power_Set.angle_err_sum <= AngleERRlow)
//         AngleKcoe = 0;
//     else
//         AngleKcoe = (Power_Set.angle_err_sum - AngleERRlow) / (AngleERRhigh - AngleERRlow);

//     for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++)
//     {
//         // 按照误差比例的置信度分配
//         if (if_wheel_used[group] == 1)
//             Motor_power[group][WHEEL_RUN].allo_power_set =
//                 RunKcoe * (Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Speed_pid.Err) / Power_Set.run_err_sum * Power_Set.power_set_run + (1 - RunKcoe) * Power_Set.run_ratio[group] * Power_Set.power_set_run;

//         if (if_steer_used[group] == 1)
//             Motor_power[group][WHEEL_ANGLE].allo_power_set =
//                 AngleKcoe * (Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Err) / Power_Set.angle_err_sum * Power_Set.power_set_steer + (1 - AngleKcoe) * Power_Set.angle_ratio[group] * Power_Set.power_set_steer;
//     }
// }
extern DJI_Data_t DJI_ReadData; // 完整数据存放结构体

/*功率控制周期为1ms，但裁判系统数据频率只有50Hz(20ms)*/
void Power_Ctrl(void)
{
    Set_SuprCap_OutputMode(0); // 关闭超电输出
    PowerCtrl_Close_Flag[0] = 1;
    // 得到功率控制相关数据
    getPowerData();
    // 超电余量控制与异常处理
    // SCap_PowerCtrl(GetGimIfSuperCapOutput());
    // SCap_PowerCtrl(1);

    // 计算最大可输出功率与异常处理
    get_chassis_max_power();
    // 功率限制、功率再分配
    Power_lim_allo_to_wheels();//开启功率控制
#ifdef BSP_USING_VOFA
    static float data_vofa[4];
    data_vofa[0] = PowerInfo.Ref.PowerBuff;
    data_vofa[1] = DJI_ReadData.ext_power_heat_data.chassis_power_reserve;
    data_vofa[2] = PowerCtrl.Power_Maxlim_sum;
    data_vofa[3] = now_power_sum;
    VOFA_Trans_Justfloat(4, data_vofa);
#endif
}

void set_k1(int argc, char *argv[])
{
    float k1 = 0.f;
    if(argc==2)
    {
        k1 = atof(argv[1]);
        for (int i = 0; i < WHEELS_GROUP_NUM; i++)
        {
            rt_kprintf("k1 = %d\n", (int)k1*1000);
            Motor_power[i][WHEEL_RUN].K1 = k1;
            Motor_power[i][WHEEL_ANGLE].K1 = k1;
        }
    }
    else if(argc==3)
    {
        if(strcmp(argv[1],"run")==0)
        {
            k1 = atof(argv[2]);
            rt_kprintf("k1 = %d\n", (int)k1 * 1000);
            for (int i = 0; i < WHEELS_GROUP_NUM; i++)
            {
                Motor_power[i][WHEEL_RUN].K1 = k1;
            }
        }
        else if(strcmp(argv[1],"angle")==0)
        {
            k1 = atof(argv[2]);
            rt_kprintf("k1 = %d\n", (int)k1 * 1000);
            for (int i = 0; i < WHEELS_GROUP_NUM; i++)
            {
                Motor_power[i][WHEEL_ANGLE].K1 = k1;
            }
        }
    }
}
MSH_CMD_EXPORT(set_k1, "set k1");

void set_k2(int argc, char *argv[])
{
    float k2 = 0.f;
    if (argc == 2)
    {
        k2 = atof(argv[1]);
        for (int i = 0; i < WHEELS_GROUP_NUM; i++)
        {
            rt_kprintf("k2 = %d\n", (int)k2 * 10000000);
            Motor_power[i][WHEEL_RUN].K2 = k2;
            Motor_power[i][WHEEL_ANGLE].K2 = k2;
        }
    }
    else if (argc == 3)
    {
        if (strcmp(argv[1], "run") == 0)
        {
            k2 = atof(argv[2]);
            rt_kprintf("k2 = %d\n", (int)k2 * 10000000);
            for (int i = 0; i < WHEELS_GROUP_NUM; i++)
            {
                Motor_power[i][WHEEL_RUN].K2 = k2;
            }
        }
        else if (strcmp(argv[1], "angle") == 0)
        {
            k2 = atof(argv[2]);
            rt_kprintf("k2 = %d\n", (int)k2 * 10000000);
            for (int i = 0; i < WHEELS_GROUP_NUM; i++)
            {
                Motor_power[i][WHEEL_ANGLE].K2 = k2;
            }
        }
    }
}
MSH_CMD_EXPORT(set_k2, "set k2");

__weak float getScRate(void)
{
    return 0;
}

float get_k_power_allo()
{
    return PowerCtrl.k_power_allo_wheel;
}


// /**
//  * @brief  RLS参数更新
//  *
//  */
// float GYRO_FLAG = 0, mode_change_flag = 0, last_mode = 0;
// void get_true_RLS(void)
// {
//     RunRLS.now_tick = rt_tick_get();
//     if (PowerInfo.Cap.isValid)
//         RunRLS.flag = RunRLS.now_tick - Get_SuprCap_recTick();
//     // RunRLS.flag=RunRLS.now_tick-PowerInfo.Ref.refresh_tick;
//     // AngleRLS.flag=AngleRLS.now_tick-PowerInfo.Ref.refresh_tick;
//     if (Get_Motion_Mode() == SLOW_GYRO || Get_Motion_Mode() == FAST_GYRO || Get_Motion_Mode() == CHANGE_GYRO)
//         GYRO_FLAG += 1;
//     else
//         GYRO_FLAG = 0;

//     if (last_mode != Get_Motion_Mode())
//         GYRO_FLAG = 0;
//     last_mode = Get_Motion_Mode();

//     if (RunRLS.repe_speed < 100) // 防止匀速运动时过拟合
//         RunRLS.repe_flag++;
//     else
//         RunRLS.repe_flag = 0;

//     // 以超电更新的频率为RLS更新频率的依据，确保转矩与超电反馈的功率相对应
//     if (PowerInfo.Cap.Cap_State == CAP_STATE_NO_WORK && PowerInfo.Cap.Power > 13 && RunPhi[0] > 0.f && RunPhi[1] && RunRLS.flag == 1 && RunRLS.now_power > 0 && GYRO_FLAG < 100) //&&RunRLS.repe_speed>200)
//     {

//         // RLS_Update(&RunRLS, RunPhi, PowerInfo.Cap.Power - RunPhi[2] - RunRLS.theta[2]); // 只有三个变参，减去固定的一项。
//         //  RLS_Update(&RunRLS, RunPhi, PowerInfo.Ref.power_now-Power_Set.power_set_steer-RunPhi[2]-RunRLS.theta[2] );
//         if (!Check_Stability(&RunRLS))
//             Reconstruct_P(&RunRLS);
//     }
//     else if (PowerInfo.Cap.Cap_State == CAP_STATE_NO_WORK && PowerInfo.Cap.Power > 13 && RunPhi[0] && RunPhi[1] && RunRLS.flag == 1 && RunRLS.now_power < 0 && GYRO_FLAG < 100) //&&RunRLS.repe_speed>200)
//     {
//         // RLS_Update(&RunRLS, RunPhi, RunRLS.now_power -RunPhi[2] - RunRLS.theta[2]); // 只有三个变参，减去固定的一项。
//         if (!Check_Stability(&RunRLS))
//             Reconstruct_P(&RunRLS);
//     }

//     RunRLS.now_power = 0;
//     RunRLS.repe_speed = 0;
//     memset(RunPhi, 0, sizeof(RunPhi));

//     for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++)
//     {
//         RunRLS.last_spe[group] = RunRLS.now_spe[group];
//         RunRLS.now_tor = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.torque / get_Ratio(Motor_Power_Pointer[WHEEL_RUN][group]);       // Motor_Read_NowTorque(&wheel[group][WHEEL_RUN].motor)*K_IC2Tor_RUN;
//         RunRLS.now_spe[group] = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.speed * get_Ratio(Motor_Power_Pointer[WHEEL_RUN][group]); //(&wheel[group][WHEEL_RUN].motor);

//         RunRLS.repe_speed += fabsf(RunRLS.now_spe[group] - RunRLS.last_spe[group]);

//         //  RunRLS.now_power += fabsf(RunRLS.now_spe[group])*RunRLS.now_tor/9.55f +
//         //         RunRLS.theta[0] * SQUARE(RunRLS.now_tor) +
//         //         RunRLS.theta[1] * SQUARE(RunRLS.now_spe[group]) +
//         //         RunRLS.theta[2]/4.f; // 电机功率模型计算公式（速度项平方）
//         RunRLS.now_power += RunRLS.now_spe[group] * RunRLS.now_tor / 9.55f +
//                             RunRLS.theta[0] * SQUARE(RunRLS.now_tor) +
//                             RunRLS.theta[1] * fabsf(RunRLS.now_spe[group]) +
//                             RunRLS.theta[2] / 4.f; // 电机功率模型计算公式（速度项绝对值）
//                                                    // ulog_d("pow","RLS_p,cap,ex_p,:%.2f,%.2f,%.2f",RunRLS.now_power,PowerInfo.Cap.Power,Power_Set.power_set_sum);
//         RunPhi[0] += SQUARE(RunRLS.now_tor);
//         RunPhi[1] += fabsf(RunRLS.now_spe[group]);
//         RunPhi[2] += RunRLS.now_spe[group] * RunRLS.now_tor / 9.55f;
//     }

//     if (if_steer_used[0] == 1)
//     {

//         if (AngleRLS.repe_speed < 500)
//             AngleRLS.repe_flag++;
//         else
//             AngleRLS.repe_flag = 0;
//         // 以超电更新的频率为RLS更新频率的依据，确保转矩与超电反馈的功率相对应
//         if (PowerInfo.Cap.Power > 10 && AnglePhi[0] && AnglePhi[1] && RunRLS.flag == 1 && AngleRLS.now_power > 0)
//         {
//             // RLS_Update(&AngleRLS, AnglePhi, PowerInfo.Cap.Power- RunRLS.now_power-AnglePhi[2]-AngleRLS.theta[2] );    //只有三个变参，减去固定的一项。
//             if (!Check_Stability(&AngleRLS))
//                 Reconstruct_P(&AngleRLS);
//         }
//         else if (PowerInfo.Cap.Power > 10 && AnglePhi[0] && AnglePhi[1] && RunRLS.flag == 1 && AngleRLS.now_power < 0)
//         {
//             // RLS_Update(&AngleRLS, AnglePhi, AngleRLS.now_power- RunRLS.now_power-AnglePhi[2]-AngleRLS.theta[2] );    //只有三个变参，减去固定的一项。
//             if (!Check_Stability(&AngleRLS))
//                 Reconstruct_P(&AngleRLS);
//         }

//         AngleRLS.now_power = 0;
//         memset(AnglePhi, 0, sizeof(AnglePhi));
//         AngleRLS.repe_speed = 0;

//         for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++)
//         {
//             AngleRLS.last_spe[group] = AngleRLS.now_spe[group];
//             AngleRLS.now_tor = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.torque / get_Ratio(Motor_Power_Pointer[WHEEL_ANGLE][group]);       // Motor_Read_NowTorque(&wheel[group][WHEEL_ANGLE].motor)*  K_IC2Tor_ANGLE;
//             AngleRLS.now_spe[group] = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.speed * get_Ratio(Motor_Power_Pointer[WHEEL_ANGLE][group]); // Motor_Read_NowSpeed(&wheel[group][WHEEL_ANGLE].motor);

//             AngleRLS.repe_speed = fabsf(AngleRLS.now_spe[group] - AngleRLS.last_spe[group]);
//             //  AngleRLS.now_power += fabsf(AngleRLS.now_spe[group])*AngleRLS.now_tor/9.55f +
//             //         AngleRLS.theta[0] * SQUARE(AngleRLS.now_tor) +
//             //         AngleRLS.theta[1] * SQUARE(AngleRLS.now_spe[group]) +
//             //         AngleRLS.theta[2]/4.f; // 电机功率模型计算公式（速度项平方）
//             AngleRLS.now_power += AngleRLS.now_spe[group] * AngleRLS.now_tor / 9.55f +
//                                   AngleRLS.theta[0] * SQUARE(AngleRLS.now_tor) +
//                                   AngleRLS.theta[1] * fabsf(AngleRLS.now_spe[group]) +
//                                   AngleRLS.theta[2] / 4.f; // 电机功率模型计算公式（速度项绝对值）

//             AnglePhi[0] += SQUARE(AngleRLS.now_tor);
//             AnglePhi[1] += fabsf(AngleRLS.now_spe[group]);
//             AnglePhi[2] += AngleRLS.now_spe[group] * AngleRLS.now_tor / 9.55f;
//         }
//     }
// }

// /**
//  * @brief Get the pre power object获得预输出功率相关参数
//  *
//  * @param wheel_temp
//  */

// void get_pre_power(void)
// {
//     rt_memset(&Power_Set, 0, sizeof(Power_Set));
//     uint8_t group;
//     for (group = 0; group < WHEELS_GROUP_NUM; group++) // first get all the initial motor power and total motor power
//     {
//         if (if_steer_used[group] == 1)
//         {
//             // 舵向电机速度刷新至
//             Motor_power[group][WHEEL_ANGLE].V_last = Motor_power[group][WHEEL_ANGLE].V_now;
//             Motor_power[group][WHEEL_ANGLE].V_now = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.speed * get_Ratio(Motor_Power_Pointer[WHEEL_ANGLE][group]);
//         }
//         if (if_wheel_used[group] == 1)
//         {
//             // 轮向电机速度刷新至
//             Motor_power[group][WHEEL_RUN].V_last = Motor_power[group][WHEEL_RUN].V_now;
//             Motor_power[group][WHEEL_RUN].V_now = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.speed * get_Ratio(Motor_Power_Pointer[WHEEL_RUN][group]); // Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor); // 原始转子转速数据(rpm)
//         }

//         if (if_steer_used[group] == 1)
//         {
//             // 舵向电机
//             Motor_power[group][WHEEL_ANGLE].initial_tor_set = Motor_Power_Pointer[WHEEL_ANGLE][group]->Motor_obj.Motor_ctrl.Speed_pid.Output *
//                                                               get_K_IC2Tor(Motor_Power_Pointer[WHEEL_ANGLE][group]); // fabsf( wheel_temp[group][WHEEL_RUN].motor.spe.out * K_IC2Tor_RUN); // 转子扭矩计算;//(wheel_temp[group][WHEEL_ANGLE].motor.spe.out * K_IC2Tor_ANGLE); // 转子扭矩计算
//             // 功率分配前的预输出功率
//             /*     Motor_power[group][WHEEL_ANGLE].initial_power_set = fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * Motor_power[group][WHEEL_ANGLE].initial_tor_set / 9.55f +
//                                                                      Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(Motor_power[group][WHEEL_ANGLE].initial_tor_set) +
//                                                                      Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
//                                                                      Motor_power[group][WHEEL_ANGLE].C; // 电机功率模型计算公式
//              */
//             Motor_power[group][WHEEL_ANGLE].initial_power_set = Motor_power[group][WHEEL_ANGLE].V_now * Motor_power[group][WHEEL_ANGLE].initial_tor_set / 9.55f +
//                                                                 AngleRLS.theta[0] * SQUARE(Motor_power[group][WHEEL_ANGLE].initial_tor_set) +
//                                                                 AngleRLS.theta[1] * fabsf(Motor_power[group][WHEEL_ANGLE].V_now) +
//                                                                 AngleRLS.theta[2] / 4.f; // 电机功率模型计算公式

//             Power_Set.power_set_steer += Motor_power[group][WHEEL_ANGLE].initial_power_set;
//             // 构建回归向量，第一项的系数不参与RLS，故仍然时三维的
//         }
//         if (if_wheel_used[group] == 1)
//         {
//             // 轮向电机
//             Motor_power[group][WHEEL_RUN].initial_tor_set = Motor_Power_Pointer[WHEEL_RUN][group]->Motor_obj.Motor_ctrl.Current_pid.Output *
//                                                             get_K_IC2Tor(Motor_Power_Pointer[WHEEL_RUN][group]); // fabsf( wheel_temp[group][WHEEL_RUN].motor.spe.out * K_IC2Tor_RUN); // 转子扭矩计算
//             // 功率分配前的预输出功率

//             // Motor_power[group][WHEEL_RUN].initial_power_set = fabsf(Motor_power[group][WHEEL_RUN].V_now) * Motor_power[group][WHEEL_RUN].initial_tor_set / 9.55f +
//             //         RunRLS.theta[0] * SQUARE(Motor_power[group][WHEEL_RUN].initial_tor_set) +
//             //         RunRLS.theta[1] * SQUARE(Motor_power[group][WHEEL_RUN].V_now) +
//             //         RunRLS.theta[2]/4.f; // 电机功率模型计算公式
//             Motor_power[group][WHEEL_RUN].initial_power_set = Motor_power[group][WHEEL_RUN].V_now * Motor_power[group][WHEEL_RUN].initial_tor_set / 9.55f +
//                                                               RunRLS.theta[0] * SQUARE(Motor_power[group][WHEEL_RUN].initial_tor_set) +
//                                                               RunRLS.theta[1] * fabsf(Motor_power[group][WHEEL_RUN].V_now) +
//                                                               RunRLS.theta[2] / 4.f; // 电机功率模型计算公式
//             // todo
//             Power_Set.power_set_run += Motor_power[group][WHEEL_RUN].initial_power_set;
//         }
//     }
//     // 计算总功率大小(舵向电机功率+轮向电机功率)
//     Power_Set.steer_power = Power_Set.power_set_steer;
//     Power_Set.run_power = Power_Set.power_set_run;
//     Power_Set.power_set_sum_temp = Power_Set.power_set_steer + Power_Set.power_set_run;
//     Power_Set.power_set_sum = Power_Set.power_set_sum_temp; // 电机预输出的总功率（不包括固定静态功率）

//     for (group = 0; group < WHEELS_GROUP_NUM; group++)
//     {

//         Power_Set.run_ratio[group] = Motor_power[group][WHEEL_RUN].initial_power_set / Power_Set.run_power;
//         if (if_steer_used[group] == 1)
//             Power_Set.angle_ratio[group] = Motor_power[group][WHEEL_ANGLE].initial_power_set / Power_Set.steer_power; // 单个电机功率占比
//     }
// }