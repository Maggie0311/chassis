#include "fun_powerCtrl.h"
#include "app_RefSystem.h"
#include "drv_utils.h"
#include "func_sd_master.h"
#include "fun_protocolGim.h"

#define DELTASPEEDERR 4000
#define fabs(x) (x > 0 ? x : -x)
#define SIGNAL(x) (x / x)

Power_info_t PowerInfo;
PowerCtrl_t PowerCtrl;

// TODO 电机控制参数结构体申明
Motor_Power_t Motor_power[WHEELS_GROUP_NUM][WHEEL_NUM];

// note 修改裁判系统和超电数据有效性标志位
uint8_t PowerCtrl_Close_Flag[2] = {0, 0}; // 是否抛弃超电数据，是否抛弃裁判系统数据
uint8_t Cap_false_flag = 0;          // 底盘检测超电可能掉线标志，标记超电异常标志位（用于UI告知操作手）,1为异常，0为正常
extern uint8_t power_ctrl_close;
void powerCtrlInit(void)
{
    rt_memset(&PowerInfo, 0, sizeof(PowerInfo));
    rt_memset(&PowerCtrl, 0, sizeof(PowerCtrl));
    rt_memset(Motor_power, 0, sizeof(Motor_power));
    PowerInfo.Cap.SupplyCurrt = Supr_Cap_MaxOutputCurrent;
    PowerInfo.Ref.PowerBuff = PowerCtrlReamin;  //防止第一次运行功率控制，但裁判系统还没有通信导致认为超电异常
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
    PowerInfo.Cap.isValid |= Get_SuprCapPower(&PowerInfo.Cap.Power); // 获取超级电容功率
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
rt_uint16_t power_max_limit_test=50;
// 得到裁判系统的关于底盘功率的数据 50Hz更新 20ms,判读数据是否合理
static void getRefDataAndInfo(void)
{
    static uint32_t overPowerTick = 0;
    rt_tick_t tick_tmp = 0;
    PowerInfo.Ref.isValid = ISRefValid(); // 数据是否有效
    if (1 == PowerInfo.Ref.isValid)
    {
        /**以下数据计算后为国际单位**/
        getRefData(memberGet_parm(PowerInfo.Ref.PowerBuff,ext_power_heat_data,chassis_power_buffer));          // 缓冲能量J
        // PowerInfo.Ref.PowerBuff = 60; // 缓冲能量J
        getRefData(memberGet_parm(PowerInfo.Ref.Power_Maxlim,ext_game_robot_status,chassis_power_limit));    // 最大功率限制W
        // PowerInfo.Ref.Power_Maxlim = power_max_limit_test; // 最大功率限制W
        getRefData(memberGet_parm(PowerInfo.Ref.power_now,ext_power_heat_data,chassis_power_reserve));
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
/**
 * @brief 超电输出功能开关
 *
 * @param GimIfSuperCapOutput    超电输出功能开关
 */
static void SCap_PowerCtrl(uint8_t GimIfSuperCapOutput)
{
    if (PowerInfo.Ref.isValid)
    {
        // 缓冲能量很低时认为超电异常
        if (PowerInfo.Ref.PowerBuff <= PowerBuffReamin || PowerInfo.Cap.Cap_State == CAP_STATE_ERROR)
        {                                // 缓冲能量小于预留能量,关超电
            Cap_false_flag = 1;          // 标记超电功能异常
            power_ctrl_close = 1;        // 通讯关闭超电
            PowerCtrl_Close_Flag[0] = 1; // 关闭超电数据使用
        }
    }
#else
    if (PowerInfo.Ref.isValid)
    {
        // 缓冲能量很低时认为超电异常
        if (PowerInfo.Ref.PowerBuff <= PowerBuffReamin || PowerInfo.Cap.Cap_State == CAP_STATE_ERROR)
        {                                // 缓冲能量小于预留能量,关超电
            Cap_false_flag = 1;          // 标记超电功能异常
            power_ctrl_close = 1;        // 通讯关闭超电
            PowerCtrl_Close_Flag[0] = 1; // 关闭超电数据使用
        }
        // #if KEY_C_FUN == 2
        //         // 超电余量限制功能（第二部分：超电关闭后的恢复）
        //         if (PowerInfo.Ref.PowerBuff >= 50 && !Cap_false_flag)
        //         {
        //             power_ctrl_close = 0;        // 通讯打开超电
        //             PowerCtrl_Close_Flag[0] = 0; // 打开超电数据使用
        //         }
        // #endif
    }
    static uint8_t user_enable=1;       //用户设置的放电状态
    static uint8_t pre_button_state=1;  //前次按键状态（初始上电为0，一段时间后和云台通信成功后，云台发送为1）
    static uint8_t if_used_up=0;        //是否电量耗尽
    static uint8_t if_used_up_user_unchange=0;  //是否用光超电并且用户是否改变状态


    if (pre_button_state != GimIfSuperCapOutput) // 判断云台是否切换模式
    {
        user_enable= !user_enable;                                 //标记为需要切换模式
        if_used_up_user_unchange=0;
        pre_button_state=GimIfSuperCapOutput;   //更新云台端超电输出状态
    }
    if(PowerInfo.Cap.superCap<=3)                       // 超电电量小于3%时，标记超电被使用完
    {
        if_used_up=1;                                 // 标记超电被使用完
        if_used_up_user_unchange= 1;
        user_enable=0;
        // power_ctrl_close = 1;                         // 通讯关闭超电
        // PowerCtrl_Close_Flag[0] = 1;                  // 关闭超电数据使用
    }
    else if(PowerInfo.Cap.superCap>3)
    {
        if_used_up=0;                                 // 标记超电未被使用完

        // if_turn_mode=0;                               // 清除标记
        // power_ctrl_close = 0;                         // 通讯打开超电
        // PowerCtrl_Close_Flag[0] = 0;                  // 打开超电数据使用
    }
    if(if_used_up==0 && user_enable && if_used_up_user_unchange==0)   //云台端打开超电输出，且超电未被标记使用完，
    {
        power_ctrl_close = 0;        // 通讯打开超电
        PowerCtrl_Close_Flag[0] = 0; // 打开超电数据使用
    }
    else                //云台端关闭超电输出，或者超电被标记异常
    {
        power_ctrl_close = 1;        // 通讯关闭超电
        PowerCtrl_Close_Flag[0] = 1; // 关闭超电数据使用
    }
#endif
// #endif
}

//在UI界面告知操作手超电可能电容组放电或者报废异常
uint8_t get_SuperCap_if_abnormal(void)
{
    return Cap_false_flag;
}
uint8_t get_Super_Cap_if_open(void)
{
    return !power_ctrl_close;
}

static void get_chassis_max_power(void)
{

    PowerCtrl.Power_Allo_level = 0;

    if (PowerInfo.Cap.isValid && !power_ctrl_close && !PowerCtrl_Close_Flag[0])
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
}

#if Power_Ctrl_Debug == 1
// debug参数：
float now_power_sum;
float out_power_sum; // 此数据目前不正常，但没有问题，见计算处的注释
#endif

float power_set_sum; //  预输出总功率   // debug可参考

// #if defined(USING_MECANUM_WHEEL)||defined(USING_OMNI_WHEEL)
// static void Power_lim_allo_to_wheels(Motor_t *wheel)
// {
//     /*程序在运行这个函数的时候，PID的输出wheel[index].spe.out已经根据初始化时的设置进行方向回反了，
//      * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
//      * 对于wheel[index].dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
//      */
//     uint8_t index = 0, power_allo_flag_temp = 0;
//     float temp = 0, power_set_sum_temp = 0;
// #if Power_Ctrl_Debug == 1
//     float out_power_sum_temp = 0; // debug
// #endif

//     // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
//     for (index = 0; index < 4; index++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
//         if (wheel[index].dji.reverse_flag == 1)
//             wheel[index].spe.out = -wheel[index].spe.out;

//     for (index = 0; index < 4; index++) // first get all the initial motor power and total motor power
//     {
//         Motor_power[index].V_last = Motor_power[index].V_now;
//         Motor_power[index].V_now = Motor_Read_NowSpeed(&wheel[index]); // 原始转子转速数据(rpm)

//         /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
// #if Power_Ctrl_Debug == 1
//         Motor_power[index].I_now = Motor_Read_NowTorque(&wheel[index]);          // 原始扭矩电流数据
//         Motor_power[index].tor_now = fabsf(Motor_power[index].I_now * K_IC2Tor_3508); // 转子扭矩计算
//         Motor_power[index].power_now = fabsf(Motor_power[index].V_now) * Motor_power[index].tor_now / 9.55f +
//                                        Motor_power[index].K1 * SQUARE(Motor_power[index].tor_now) +
//                                        Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
//                                        Motor_power[index].C; // 电机功率模型计算公式
//         if (Motor_power[index].power_now > 0)
//             temp += Motor_power[index].power_now;
// #endif
//         Motor_power[index].initial_tor_set = fabsf(wheel[index].spe.out * K_IC2Tor_3508); // 转子扭矩计算
//         // 功率分配前的预输出功率
//         Motor_power[index].initial_power_set = fabsf(Motor_power[index].V_now) * Motor_power[index].initial_tor_set / 9.55f +
//                                                Motor_power[index].K1 * SQUARE(Motor_power[index].initial_tor_set) +
//                                                Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
//                                                Motor_power[index].C; // 电机功率模型计算公式
//         power_set_sum_temp += Motor_power[index].initial_power_set;
//     }
// #if Power_Ctrl_Debug == 1
//     now_power_sum = temp + Power_C_sum; // debug
// #endif
//     power_set_sum = power_set_sum_temp; // 电机预输出的总功率（不包括固定静态功率）

//     if (power_set_sum > PowerCtrl.Power_Maxlim) // determine if larger than max power
//     {
//         if (PowerCtrl.Power_Allo_level >= 0)
//             power_allo_flag_temp = 1; // 正常功率限制模式标志：1
//         else
//             power_allo_flag_temp = 2; // 强制功率限制模式标志：2
//         // 一部分固定的静态功率不属于电机可用功率：即要使用减去Power_C_sum的功率数据，PowerCtrl.Power_Maxlim和power_set_sum的值已不包括静态功率
//         PowerCtrl.k_power_allo = (PowerCtrl.Power_Maxlim / power_set_sum); // 功率分配系数
//     }
//     else
//         PowerCtrl.k_power_allo = 1.0f; // 全时可引用参数对外更新
// #if IF_USED_ALLO_LEVEL == 1
//     // 加速度增益，还未完善，先别用！！！
//     else if (PowerCtrl.Power_Maxlim - power_set_sum >= Gain_Trigger_Power)
//     { // PID set 的控制是转子转速rpm,
//         power_allo_flag_temp = 3;
//         power_set_sum = Power_C_sum; // power_set_sum运行到这里也完成使命了，这里拿来统计增益功率重分配后的总功率
//         for (index = 0; index < 4; index++)
//         {
//             Motor_power[index].gain_allo_level = fabsf(Motor_power[index].V_now - wheel[index].spe.set) / 2000.0;
//             Motor_power[index].k_gain_power_allo = (K_Conservative + Motor_power[index].gain_allo_level * One_Level_Persent);
//             Motor_power[index].allo_power_set = Motor_power[index].initial_power_set * Motor_power[index].k_gain_power_allo;
//             power_set_sum += Motor_power[index].allo_power_set;
//         }
//         if (power_set_sum >= PowerCtrl.Power_Maxlim) // 保险：增益功率重分配后检查
//             power_allo_flag_temp = 0;
//     }
// #endif
//     PowerCtrl.power_allo_flag = power_allo_flag_temp; // 功率分配标志位
// #if IF_USED_ALLO_LEVEL == 2
//     if (PowerCtrl.Power_Maxlim < 1.0f)
//     {
//         for (index = 0; index < 4; index++)
//             wheel[index].spe.out = 0;
//     }
//     else
// #endif
//         if (PowerCtrl.power_allo_flag != 0)
//     {
//         for (index = 0; index < 4; index++)
//         {
// #if IF_USED_ALLO_LEVEL == 1
//             if (PowerCtrl.power_allo_flag == 3) // 增益情况下的功率分配计算方法不同，且由于在增益条件判断和检查时需要用到，已经计算过了，这里不再计算
//                 ;
//             else
// #endif
//                 Motor_power[index].allo_power_set = Motor_power[index].initial_power_set * PowerCtrl.k_power_allo; // 功率分配后的预输出功率
//             // Selection of the calculation formula according to the motor acceleration and deceleration
//             // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
//             if (SIGN_F(Motor_power[index].V_now) * SIGN_F(wheel[index].spe.out) >= 0)
//             {
//                 // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
//                 // 计算完为正
//                 Motor_power[index].allo_tor_set = (-fabsf(Motor_power[index].V_now) / 9.55f +
//                                                    sqrtf(SQUARE(Motor_power[index].V_now / 9.55f) -
//                                                          4 * Motor_power[index].K1 * (Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) + Motor_power[index].C - Motor_power[index].allo_power_set))) /
//                                                   (2 * Motor_power[index].K1);
//             }
//             else
//             {
//                 // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
//                 // 计算完为负
//                 Motor_power[index].allo_tor_set = (-fabsf(Motor_power[index].V_now) / 9.55f -
//                                                    sqrtf(SQUARE(Motor_power[index].V_now / 9.55f) -
//                                                          4 * Motor_power[index].K1 * (Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) + Motor_power[index].C - Motor_power[index].allo_power_set))) /
//                                                   (2 * Motor_power[index].K1);
//             }
//             temp = fabsf(Motor_power[index].allo_tor_set); // 取扭矩绝对值
//             // 转换为输出的扭矩电流(COUNT)，并按正确的方向输出
//             if (wheel[index].spe.out >= 0)
//                 wheel[index].spe.out = temp / K_IC2Tor_3508;
//             else
//                 wheel[index].spe.out = -temp / K_IC2Tor_3508;
// #if Power_Ctrl_Debug == 1
//             // 注：用反解出的扭矩再计算输出总功率，但这里计算完的总功率数据是错误的，而扭矩对应的输出功率并没有错，原因待思考
//             out_power_sum_temp += fabsf(Motor_power[index].V_now) * temp / 9.55f +
//                                   Motor_power[index].K1 * SQUARE(temp) +
//                                   Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
//                                   Motor_power[index].C;
// #endif
//         }
// #if Power_Ctrl_Debug == 1
//         out_power_sum = out_power_sum_temp + Power_C_sum; // debug
// #endif
//     }

//     for (index = 0; index < 4; index++) // 调回正确的输出符号
//         if (wheel[index].dji.reverse_flag == 1)
//             wheel[index].spe.out = -wheel[index].spe.out;
// }

// #elif defined(USING_STEER_WHEEL)  // 舵轮底盘

// debug参数：
float power_set_sum_temp = 0; // 用于查看计算的电机预输出总功率

static void Power_lim_allo_to_wheels_test(Wheel_t wheel_temp[4][2], Wheel_Local_e wheel_type)
{
    /*程序在运行这个函数的时候，PID的输出wheel[index].spe.out已经根据初始化时的设置进行方向回反了，
     * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
     * 对于wheel[index].dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
     */
    uint8_t group = 0, power_allo_flag_temp = 0;
    float temp = 0;
    power_set_sum_temp = 0;
    float K_IC2Tor = 0;
    if (wheel_type == WHEEL_RUN)
    {
        K_IC2Tor = K_IC2Tor_RUN;
    }
    else
    {
        K_IC2Tor = K_IC2Tor_ANGLE;
    }
#if Power_Ctrl_Debug == 1
    float out_power_sum_temp = 0; // debug
#endif

    // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
    for (group = 0; group < 4; group++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
        if (wheel_temp[group][wheel_type].motor.dji.reverse_flag == 1)
            wheel_temp[group][wheel_type].motor.spe.out = -wheel_temp[group][wheel_type].motor.spe.out;

    for (group = 0; group < 4; group++) // first get all the initial motor power and total motor power
    {
        Motor_power[group][wheel_type].V_last = Motor_power[group][wheel_type].V_now;
        Motor_power[group][wheel_type].V_now = Motor_Read_NowSpeed(&wheel[group][wheel_type].motor); // 原始转子转速数据(rpm)

        /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
#if Power_Ctrl_Debug == 1
        Motor_power[group][wheel_type].I_now = Motor_Read_NowTorque(&wheel[group][wheel_type].motor);    // 原始扭矩电流数据
        Motor_power[group][wheel_type].tor_now = fabsf(Motor_power[group][wheel_type].I_now * K_IC2Tor); // 转子扭矩计算
        Motor_power[group][wheel_type].power_now = fabsf(Motor_power[group][wheel_type].V_now) * Motor_power[group][wheel_type].tor_now / 9.55f +
                                                   Motor_power[group][wheel_type].K1 * SQUARE(Motor_power[group][wheel_type].tor_now) +
                                                   Motor_power[group][wheel_type].K2 * SQUARE(Motor_power[group][wheel_type].V_now) +
                                                   Motor_power[group][wheel_type].C; // 电机功率模型计算公式
        if (Motor_power[group][wheel_type].power_now > 0)
            temp += Motor_power[group][wheel_type].power_now;
#endif
        Motor_power[group][wheel_type].initial_tor_set = fabsf(wheel[group][wheel_type].motor.spe.out * K_IC2Tor); // 转子扭矩计算
        // 功率分配前的预输出功率
        Motor_power[group][wheel_type].initial_power_set = fabsf(Motor_power[group][wheel_type].V_now) * Motor_power[group][wheel_type].initial_tor_set / 9.55f +
                                                           Motor_power[group][wheel_type].K1 * SQUARE(Motor_power[group][wheel_type].initial_tor_set) +
                                                           Motor_power[group][wheel_type].K2 * SQUARE(Motor_power[group][wheel_type].V_now) +
                                                           Motor_power[group][wheel_type].C; // 电机功率模型计算公式
        power_set_sum_temp += Motor_power[group][wheel_type].initial_power_set;
    }
#if Power_Ctrl_Debug == 1
    now_power_sum = temp + Power_C_sum; // debug
#endif
    power_set_sum = power_set_sum_temp; // 电机预输出的总功率（不包括固定静态功率）

    if (power_set_sum > PowerCtrl.Power_Maxlim) // determine if larger than max power
    {
        if (PowerCtrl.Power_Allo_level >= 0)
            power_allo_flag_temp = 1; // 正常功率限制模式标志：1
        else
            power_allo_flag_temp = 2; // 强制功率限制模式标志：2
        // 一部分固定的静态功率不属于电机可用功率：即要使用减去Power_C_sum的功率数据，PowerCtrl.Power_Maxlim和power_set_sum的值已不包括静态功率

        // note 这里借用k_power_allo_steer来计算不管是舵向电机还是轮向电机的分配系数
        PowerCtrl.k_power_allo_steer = (PowerCtrl.Power_Maxlim / power_set_sum); // 功率分配系数
    }
    else
        PowerCtrl.k_power_allo_steer = 1.0f; // 全时可引用参数对外更新
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
            wheel[group][wheel_type].motor.spe.out = 0;
    }
    else
#endif
        if (PowerCtrl.power_allo_flag != 0)
    {
        for (group = 0; group < 4; group++)
        {
#if IF_USED_ALLO_LEVEL == 1
            if (PowerCtrl.power_allo_flag == 3) // 增益情况下的功率分配计算方法不同，且由于在增益条件判断和检查时需要用到，已经计算过了，这里不再计算
                ;
            else
#endif
                Motor_power[group][wheel_type].allo_power_set = Motor_power[group][wheel_type].initial_power_set * PowerCtrl.k_power_allo_steer; // 功率分配后的预输出功率
            // Selection of the calculation formula according to the motor acceleration and deceleration
            // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
            if (SIGN_F(Motor_power[group][wheel_type].V_now) * SIGN_F(wheel[group][wheel_type].motor.spe.out) >= 0)
            {
                // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                // 计算完为正
                Motor_power[group][wheel_type].allo_tor_set = (-fabsf(Motor_power[group][wheel_type].V_now) / 9.55f +
                                                               sqrtf(SQUARE(Motor_power[group][wheel_type].V_now / 9.55f) -
                                                                     4 * Motor_power[group][wheel_type].K1 * (Motor_power[group][wheel_type].K2 * SQUARE(Motor_power[group][wheel_type].V_now) + Motor_power[group][wheel_type].C - Motor_power[group][wheel_type].allo_power_set))) /
                                                              (2 * Motor_power[group][wheel_type].K1);
            }
            else
            {
                // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                // 计算完为负
                Motor_power[group][wheel_type].allo_tor_set = (-fabsf(Motor_power[group][wheel_type].V_now) / 9.55f -
                                                               sqrtf(SQUARE(Motor_power[group][wheel_type].V_now / 9.55f) -
                                                                     4 * Motor_power[group][wheel_type].K1 * (Motor_power[group][wheel_type].K2 * SQUARE(Motor_power[group][wheel_type].V_now) + Motor_power[group][wheel_type].C - Motor_power[group][wheel_type].allo_power_set))) /
                                                              (2 * Motor_power[group][wheel_type].K1);
            }
            temp = fabsf(Motor_power[group][wheel_type].allo_tor_set); // 取扭矩绝对值
            // 转换为输出的扭矩电流(COUNT)，并按正确的方向输出
            if (wheel[group][wheel_type].motor.spe.out >= 0)
                wheel[group][wheel_type].motor.spe.out = temp / K_IC2Tor;
            else
                wheel[group][wheel_type].motor.spe.out = -temp / K_IC2Tor;
#if Power_Ctrl_Debug == 1
            // 注：用反解出的扭矩再计算输出总功率，但这里计算完的总功率数据是错误的，而扭矩对应的输出功率并没有错，原因待思考
            out_power_sum_temp += fabsf(Motor_power[group][wheel_type].V_now) * temp / 9.55f +
                                  Motor_power[group][wheel_type].K1 * SQUARE(temp) +
                                  Motor_power[group][wheel_type].K2 * SQUARE(Motor_power[group][wheel_type].V_now) +
                                  Motor_power[group][wheel_type].C;
#endif
        }
#if Power_Ctrl_Debug == 1
        out_power_sum = out_power_sum_temp + Power_C_sum; // debug
#endif
    }

    for (group = 0; group < 4; group++) // 调回正确的输出符号
        if (wheel[group][wheel_type].motor.dji.reverse_flag == 1)
            wheel[group][wheel_type].motor.spe.out = -wheel[group][wheel_type].motor.spe.out;
}

float steer_power = 0;
float run_power = 0;

static void Power_lim_allo_to_wheels(Wheel_t wheel_temp[4][2])
{
    /*程序在运行这个函数的时候，PID的输出wheel[group][wheel_type].motor.spe.out已经根据初始化时的设置进行方向回反了，
     * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
     * 对于wheel[group][wheel_type].motor.dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
     */
    // 传入的参数为电机组总地址，需要区分舵向电机和轮向电机，用于索引
    // 以下设计均是每个部分将对应的舵向电机和轮向电机分别计算
    uint8_t if_steer_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
    uint8_t if_wheel_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
    // 0:不使用 1:使用
    for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++) // 分别调试舵向电机和轮向电机时，只需要在这里修改是否使用即可，不使用就不会参与功率控制
    {
        if (wheel_temp[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL || wheel_temp[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            if_steer_used[group] = 0; // 如果底盘只接舵向电机，则配置轮向电机为0即可
            if_wheel_used[group] = 1; // 如果底盘只接轮向电机，则配置舵向电机为0即可
        } // 调试完成之后恢复到现在的配置即可
        else if (wheel_temp[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            if_steer_used[group] = 1;
            if_wheel_used[group] = 1;
        }
    }

    uint8_t group = 0, power_allo_flag_temp = 0;
    float temp_run = 0, temp_angle = 0, power_set_sum_temp = 0;
    float power_set_steer_wheel = 0, power_set_steer = 0, power_set_wheel = 0;

#if Power_Ctrl_Debug == 1
    float out_power_sum_temp = 0; // debug
#endif

    // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
    for (group = 0; group < WHEELS_GROUP_NUM; group++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
    {
        // 轮向电机调整方向
        if (if_wheel_used[group] == 1)
        {
            if (wheel_temp[group][WHEEL_RUN].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = -wheel_temp[group][WHEEL_RUN].motor.spe.out;
            }
        }
        if (if_steer_used[group] == 1)
        {
            // 舵向电机调整方向
            if (wheel_temp[group][WHEEL_ANGLE].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -wheel_temp[group][WHEEL_ANGLE].motor.spe.out;
            }
        }
    }

    for (group = 0; group < WHEELS_GROUP_NUM; group++) // first get all the initial motor power and total motor power
    {
        if (if_steer_used[group] == 1)
        {
            // 舵向电机速度刷新至
            Motor_power[group][WHEEL_ANGLE].V_last = Motor_power[group][WHEEL_ANGLE].V_now;
            Motor_power[group][WHEEL_ANGLE].V_now = Motor_Read_NowSpeed(&wheel[group][WHEEL_ANGLE].motor); // 原始转子转速数据(rpm)
        }
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机速度刷新至
            Motor_power[group][WHEEL_RUN].V_last = Motor_power[group][WHEEL_RUN].V_now;
            Motor_power[group][WHEEL_RUN].V_now = Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor); // 原始转子转速数据(rpm)
        }
        /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
#if Power_Ctrl_Debug == 1
        if (if_steer_used[group] == 1)
        {
            Motor_power[group][WHEEL_ANGLE].I_now = Motor_Read_NowTorque(&wheel[group][WHEEL_ANGLE].motor);          // 原始扭矩电流数据
            Motor_power[group][WHEEL_ANGLE].tor_now = fabsf(Motor_power[group][WHEEL_ANGLE].I_now * K_IC2Tor_ANGLE); // 转子扭矩计算
            Motor_power[group][WHEEL_ANGLE].power_now = fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * Motor_power[group][WHEEL_ANGLE].tor_now / 9.55f +
                                                        Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(Motor_power[group][WHEEL_ANGLE].tor_now) +
                                                        Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
                                                        Motor_power[group][WHEEL_ANGLE].C; // 电机功率模型计算公式
            if (Motor_power[group][WHEEL_ANGLE].power_now > 0)
                temp_angle += Motor_power[group][WHEEL_ANGLE].power_now;
        }
        if (if_wheel_used[group] == 1)
        {
            Motor_power[group][WHEEL_RUN].I_now = Motor_Read_NowTorque(&wheel[group][WHEEL_RUN].motor);        // 原始扭矩电流数据
            Motor_power[group][WHEEL_RUN].tor_now = fabsf(Motor_power[group][WHEEL_RUN].I_now * K_IC2Tor_RUN); // 转子扭矩计算
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
            Motor_power[group][WHEEL_ANGLE].initial_tor_set = fabsf(wheel_temp[group][WHEEL_ANGLE].motor.spe.out * K_IC2Tor_ANGLE); // 转子扭矩计算
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
            Motor_power[group][WHEEL_RUN].initial_tor_set = fabsf(wheel_temp[group][WHEEL_RUN].motor.spe.out * K_IC2Tor_RUN); // 转子扭矩计算
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
            if (if_wheel_used[group] == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = 0;
            }
            if (if_steer_used[group] == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = 0;
            }
        }
    }
    else
#endif
        if (PowerCtrl.power_allo_flag != 0)
    {
        for (group = 0; group < 4; group++)
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
            // Selection of the calculation formula according to the motor acceleration and deceleration
            // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
            if (if_steer_used[group] == 1)
            { // 舵向电机功率反解
                if (SIGN_F(Motor_power[group][WHEEL_ANGLE].V_now) * SIGN_F(wheel_temp[group][WHEEL_ANGLE].motor.spe.out) >= 0)
                {
                    // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                    // 计算完为正
                    Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f +
                                                                    sqrtf(SQUARE(Motor_power[group][WHEEL_ANGLE].V_now / 9.55f) -
                                                                          4 * Motor_power[group][WHEEL_ANGLE].K1 * (Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) + Motor_power[group][WHEEL_ANGLE].C - Motor_power[group][WHEEL_ANGLE].allo_power_set))) /
                                                                   (2 * Motor_power[group][WHEEL_ANGLE].K1);
                }
                else
                {
                    // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                    // 计算完为负
                    Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f -
                                                                    sqrtf(SQUARE(Motor_power[group][WHEEL_ANGLE].V_now / 9.55f) -
                                                                          4 * Motor_power[group][WHEEL_ANGLE].K1 * (Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) + Motor_power[group][WHEEL_ANGLE].C - Motor_power[group][WHEEL_ANGLE].allo_power_set))) /
                                                                   (2 * Motor_power[group][WHEEL_ANGLE].K1);
                }
                temp_angle = fabsf(Motor_power[group][WHEEL_ANGLE].allo_tor_set); // 取扭矩绝对值
                if (wheel_temp[group][WHEEL_ANGLE].motor.spe.out >= 0)
                    wheel_temp[group][WHEEL_ANGLE].motor.spe.out = temp_angle / K_IC2Tor_ANGLE;
                else
                    wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -temp_angle / K_IC2Tor_ANGLE;
            }
            if (if_wheel_used[group] == 1)
            {
                // 轮向电机功率反解
                if (SIGN_F(Motor_power[group][WHEEL_RUN].V_now) * SIGN_F(wheel_temp[group][WHEEL_RUN].motor.spe.out) >= 0)
                {
                    // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                    // 计算完为正
                    Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f +
                                                                  sqrtf(SQUARE(Motor_power[group][WHEEL_RUN].V_now / 9.55f) -
                                                                        4 * Motor_power[group][WHEEL_RUN].K1 * (Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) + Motor_power[group][WHEEL_RUN].C - Motor_power[group][WHEEL_RUN].allo_power_set))) /
                                                                 (2 * Motor_power[group][WHEEL_RUN].K1);
                }
                else
                {
                    // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                    // 计算完为负
                    Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f -
                                                                  sqrtf(SQUARE(Motor_power[group][WHEEL_RUN].V_now / 9.55f) -
                                                                        4 * Motor_power[group][WHEEL_RUN].K1 * (Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) + Motor_power[group][WHEEL_RUN].C - Motor_power[group][WHEEL_RUN].allo_power_set))) /
                                                                 (2 * Motor_power[group][WHEEL_RUN].K1);
                }
                temp_run = fabsf(Motor_power[group][WHEEL_RUN].allo_tor_set); // 取扭矩绝对值
                //             temp = fabsf(Motor_power[index].allo_tor_set); // 取扭矩绝对值
                //             // 转换为输出的扭矩电流(COUNT)，并按正确的方向输出
                if (wheel_temp[group][WHEEL_RUN].motor.spe.out >= 0)
                    wheel_temp[group][WHEEL_RUN].motor.spe.out = temp_run / K_IC2Tor_RUN;
                else
                    wheel_temp[group][WHEEL_RUN].motor.spe.out = -temp_run / K_IC2Tor_RUN;
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
    for (group = 0; group < WHEELS_GROUP_NUM; group++) // 调回正确的输出符号
    {
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机调整方向
            if (wheel_temp[group][WHEEL_RUN].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = -wheel_temp[group][WHEEL_RUN].motor.spe.out;
            }
        }
        if (if_steer_used[group] == 1)
        {
            // 舵向电机调整方向
            if (wheel_temp[group][WHEEL_ANGLE].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -wheel_temp[group][WHEEL_ANGLE].motor.spe.out;
            }
        }
    }
}
/**
 * @brief 
 * 
 * @param wheel_temp 
 * @param pitch 
 * @param yaw 
 */
static void Power_lim_allo_to_wheels_test2(Wheel_t wheel_temp[4][2], float pitch,float yaw,uint8_t is_follow_mode)
{
    /*程序在运行这个函数的时候，PID的输出wheel[group][wheel_type].motor.spe.out已经根据初始化时的设置进行方向回反了，
     * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
     * 对于wheel[group][wheel_type].motor.dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
     */
    uint8_t is_on_slpoe=0;//1:表示后轮在下，0表示在平地或者除了跟随模式以外的其他模式，不做任何处理，-1：表示前轮在下
    
    if(is_follow_mode==1)   //现在为跟随模式
    {
        if(pitch>15)  //如果底盘pitch角度太大，左右后轮在下方，需要分配更多的功率
        {
            is_on_slpoe=1;  
        }
        else if(pitch<-15)  //如果底盘pitch角度太小，左右前轮在下方，需要分配更多的功率
        {
            is_on_slpoe=-1;  
        }
    }

    // 传入的参数为电机组总地址，需要区分舵向电机和轮向电机，用于索引
    // 以下设计均是每个部分将对应的舵向电机和轮向电机分别计算
    uint8_t if_steer_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
    uint8_t if_wheel_used[WHEELS_GROUP_NUM] = {0}; // 记录电机是否被使用
    // 0:不使用 1:使用
    for (uint8_t group = 0; group < WHEELS_GROUP_NUM; group++) // 分别调试舵向电机和轮向电机时，只需要在这里修改是否使用即可，不使用就不会参与功率控制
    {
        if (wheel_temp[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL || wheel_temp[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            if_steer_used[group] = 0; // 如果底盘只接舵向电机，则配置轮向电机为0即可
            if_wheel_used[group] = 1; // 如果底盘只接轮向电机，则配置舵向电机为0即可
        } // 调试完成之后恢复到现在的配置即可
        else if (wheel_temp[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            if_steer_used[group] = 1;
            if_wheel_used[group] = 1;
        }
    }

    uint8_t group = 0, power_allo_flag_temp = 0;
    float temp_run = 0, temp_angle = 0, power_set_sum_temp = 0;
    float power_set_steer_wheel = 0, power_set_steer = 0, power_set_wheel = 0;

#if Power_Ctrl_Debug == 1
    float out_power_sum_temp = 0; // debug
#endif

    // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
    for (group = 0; group < WHEELS_GROUP_NUM; group++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
    {
        // 轮向电机调整方向
        if (if_wheel_used[group] == 1)
        {
            if (wheel_temp[group][WHEEL_RUN].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = -wheel_temp[group][WHEEL_RUN].motor.spe.out;
            }
        }
        if (if_steer_used[group] == 1)
        {
            // 舵向电机调整方向
            if (wheel_temp[group][WHEEL_ANGLE].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -wheel_temp[group][WHEEL_ANGLE].motor.spe.out;
            }
        }
    }

    for (group = 0; group < WHEELS_GROUP_NUM; group++) // first get all the initial motor power and total motor power
    {
        if (if_steer_used[group] == 1)
        {
            // 舵向电机速度刷新至
            Motor_power[group][WHEEL_ANGLE].V_last = Motor_power[group][WHEEL_ANGLE].V_now;
            Motor_power[group][WHEEL_ANGLE].V_now = Motor_Read_NowSpeed(&wheel[group][WHEEL_ANGLE].motor); // 原始转子转速数据(rpm)
        }
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机速度刷新至
            Motor_power[group][WHEEL_RUN].V_last = Motor_power[group][WHEEL_RUN].V_now;
            Motor_power[group][WHEEL_RUN].V_now = Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor); // 原始转子转速数据(rpm)
        }
        /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
#if Power_Ctrl_Debug == 1
        if (if_steer_used[group] == 1)
        {
            Motor_power[group][WHEEL_ANGLE].I_now = Motor_Read_NowTorque(&wheel[group][WHEEL_ANGLE].motor);          // 原始扭矩电流数据
            Motor_power[group][WHEEL_ANGLE].tor_now = fabsf(Motor_power[group][WHEEL_ANGLE].I_now * K_IC2Tor_ANGLE); // 转子扭矩计算
            Motor_power[group][WHEEL_ANGLE].power_now = fabsf(Motor_power[group][WHEEL_ANGLE].V_now) * Motor_power[group][WHEEL_ANGLE].tor_now / 9.55f +
                                                        Motor_power[group][WHEEL_ANGLE].K1 * SQUARE(Motor_power[group][WHEEL_ANGLE].tor_now) +
                                                        Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) +
                                                        Motor_power[group][WHEEL_ANGLE].C; // 电机功率模型计算公式
            if (Motor_power[group][WHEEL_ANGLE].power_now > 0)
                temp_angle += Motor_power[group][WHEEL_ANGLE].power_now;
        }
        if (if_wheel_used[group] == 1)
        {
            Motor_power[group][WHEEL_RUN].I_now = Motor_Read_NowTorque(&wheel[group][WHEEL_RUN].motor);        // 原始扭矩电流数据
            Motor_power[group][WHEEL_RUN].tor_now = fabsf(Motor_power[group][WHEEL_RUN].I_now * K_IC2Tor_RUN); // 转子扭矩计算
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
            Motor_power[group][WHEEL_ANGLE].initial_tor_set = fabsf(wheel_temp[group][WHEEL_ANGLE].motor.spe.out * K_IC2Tor_ANGLE); // 转子扭矩计算
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
            Motor_power[group][WHEEL_RUN].initial_tor_set = fabsf(wheel_temp[group][WHEEL_RUN].motor.spe.out * K_IC2Tor_RUN); // 转子扭矩计算
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
        if(is_on_slpoe==0)
        {
            PowerCtrl.k_power_allo_steer = (PowerCtrl.Power_Maxlim / power_set_sum) * (power_set_steer / power_set_sum); // 功率分配系数
            PowerCtrl.k_power_allo_wheel = (PowerCtrl.Power_Maxlim / power_set_sum) * (power_set_wheel / power_set_sum); // 功率分配系数
            PowerCtrl.k_power_allo_wheel_down=PowerCtrl.k_power_allo_wheel;
            PowerCtrl.k_power_allo_wheel_up=PowerCtrl.k_power_allo_wheel;//前轮和后轮不做特殊处理
        }
        else if(is_on_slpoe==1)
        {
            PowerCtrl.k_power_allo_wheel_down=PowerCtrl.Power_Maxlim*1;
            PowerCtrl.k_power_allo_wheel_up=PowerCtrl.k_power_allo_wheel;//前轮和后轮不做特殊处理

        }
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
            if (if_wheel_used[group] == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = 0;
            }
            if (if_steer_used[group] == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = 0;
            }
        }
    }
    else
#endif
        if (PowerCtrl.power_allo_flag != 0)
    {
        for (group = 0; group < 4; group++)
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
            // Selection of the calculation formula according to the motor acceleration and deceleration
            // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
            if (if_steer_used[group] == 1)
            { // 舵向电机功率反解
                if (SIGN_F(Motor_power[group][WHEEL_ANGLE].V_now) * SIGN_F(wheel_temp[group][WHEEL_ANGLE].motor.spe.out) >= 0)
                {
                    // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                    // 计算完为正
                    Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f +
                                                                    sqrtf(SQUARE(Motor_power[group][WHEEL_ANGLE].V_now / 9.55f) -
                                                                          4 * Motor_power[group][WHEEL_ANGLE].K1 * (Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) + Motor_power[group][WHEEL_ANGLE].C - Motor_power[group][WHEEL_ANGLE].allo_power_set))) /
                                                                   (2 * Motor_power[group][WHEEL_ANGLE].K1);
                }
                else
                {
                    // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                    // 计算完为负
                    Motor_power[group][WHEEL_ANGLE].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_ANGLE].V_now) / 9.55f -
                                                                    sqrtf(SQUARE(Motor_power[group][WHEEL_ANGLE].V_now / 9.55f) -
                                                                          4 * Motor_power[group][WHEEL_ANGLE].K1 * (Motor_power[group][WHEEL_ANGLE].K2 * SQUARE(Motor_power[group][WHEEL_ANGLE].V_now) + Motor_power[group][WHEEL_ANGLE].C - Motor_power[group][WHEEL_ANGLE].allo_power_set))) /
                                                                   (2 * Motor_power[group][WHEEL_ANGLE].K1);
                }
                temp_angle = fabsf(Motor_power[group][WHEEL_ANGLE].allo_tor_set); // 取扭矩绝对值
                if (wheel_temp[group][WHEEL_ANGLE].motor.spe.out >= 0)
                    wheel_temp[group][WHEEL_ANGLE].motor.spe.out = temp_angle / K_IC2Tor_ANGLE;
                else
                    wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -temp_angle / K_IC2Tor_ANGLE;
            }
            if (if_wheel_used[group] == 1)
            {
                // 轮向电机功率反解
                if (SIGN_F(Motor_power[group][WHEEL_RUN].V_now) * SIGN_F(wheel_temp[group][WHEEL_RUN].motor.spe.out) >= 0)
                {
                    // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                    // 计算完为正
                    Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f +
                                                                  sqrtf(SQUARE(Motor_power[group][WHEEL_RUN].V_now / 9.55f) -
                                                                        4 * Motor_power[group][WHEEL_RUN].K1 * (Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) + Motor_power[group][WHEEL_RUN].C - Motor_power[group][WHEEL_RUN].allo_power_set))) /
                                                                 (2 * Motor_power[group][WHEEL_RUN].K1);
                }
                else
                {
                    // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                    // 计算完为负
                    Motor_power[group][WHEEL_RUN].allo_tor_set = (-fabsf(Motor_power[group][WHEEL_RUN].V_now) / 9.55f -
                                                                  sqrtf(SQUARE(Motor_power[group][WHEEL_RUN].V_now / 9.55f) -
                                                                        4 * Motor_power[group][WHEEL_RUN].K1 * (Motor_power[group][WHEEL_RUN].K2 * SQUARE(Motor_power[group][WHEEL_RUN].V_now) + Motor_power[group][WHEEL_RUN].C - Motor_power[group][WHEEL_RUN].allo_power_set))) /
                                                                 (2 * Motor_power[group][WHEEL_RUN].K1);
                }
                temp_run = fabsf(Motor_power[group][WHEEL_RUN].allo_tor_set); // 取扭矩绝对值
                //             temp = fabsf(Motor_power[index].allo_tor_set); // 取扭矩绝对值
                //             // 转换为输出的扭矩电流(COUNT)，并按正确的方向输出
                if (wheel_temp[group][WHEEL_RUN].motor.spe.out >= 0)
                    wheel_temp[group][WHEEL_RUN].motor.spe.out = temp_run / K_IC2Tor_RUN;
                else
                    wheel_temp[group][WHEEL_RUN].motor.spe.out = -temp_run / K_IC2Tor_RUN;
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
    for (group = 0; group < WHEELS_GROUP_NUM; group++) // 调回正确的输出符号
    {
        if (if_wheel_used[group] == 1)
        {
            // 轮向电机调整方向
            if (wheel_temp[group][WHEEL_RUN].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_RUN].motor.spe.out = -wheel_temp[group][WHEEL_RUN].motor.spe.out;
            }
        }
        if (if_steer_used[group] == 1)
        {
            // 舵向电机调整方向
            if (wheel_temp[group][WHEEL_ANGLE].motor.dji.reverse_flag == 1)
            {
                wheel_temp[group][WHEEL_ANGLE].motor.spe.out = -wheel_temp[group][WHEEL_ANGLE].motor.spe.out;
            }
        }
    }
}


/**
 * @brief 速度环误差相差较大时重新分配功率
 *
 * @param wheel
 */
int Current_Sum = 0;
int ERROR_Sum = 0;
int debugout[4] = {0};
static void DeltaSpeed_PowerRalloc(Wheel_t wheel_temp[4][2])
{

    for (uint8_t i = 0; i < 4; i++)
    {
        Current_Sum += fabs(wheel[i][0].motor.spe.out);
        ERROR_Sum += fabsf(wheel[i][0].motor.spe.err);
    }
    if (ERROR_Sum > DELTASPEEDERR)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            // 按照误差比例分配
            debugout[i] = SIGNAL(wheel[i][0].motor.spe.out) * fabsf(wheel[i][0].motor.spe.err) / ERROR_Sum * wheel[i][0].motor.spe.out;
        }
    }
    ERROR_Sum = 0;
    Current_Sum = 0;
}

/*功率控制周期为1ms，但裁判系统数据频率只有50Hz(20ms)*/
void Power_Ctrl(Wheel_t wheel[4][2])
{
    // 得到功率控制相关数据
    getPowerData();
    // 超电余量控制与异常处理
    SCap_PowerCtrl(GetGimIfSuperCapOutput());
    // SCap_PowerCtrl(1);

    // 计算最大可输出功率
    get_chassis_max_power();
    // 功率限制、功率再分配
    Power_lim_allo_to_wheels(wheel);
    DeltaSpeed_PowerRalloc(wheel);
    //  Power_lim_allo_to_wheels_test(wheel,STEER_WHEEL_ANGLE);
}

__weak float getScRate(void)
{
    return 0;
}

float get_k_power_allo()
{
    return PowerCtrl.k_power_allo_wheel;
}
