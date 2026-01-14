#include "fun_powerCtrl.h"
#include "SuperCap_Com.h"
#include "drv_utils.h"
#include "app_RefSystem.h"
#include "fun_protocolGim.h"

Power_info_t PowerInfo;
PowerCtrl_t PowerCtrl;
Motor_Power_t Motor_power[4];
static uint8_t PowerCtrl_Close_Flag[2] = {0, 0};

void powerCtrlInit(Scpr_manage_t *Wheels_manage)
{
    rt_memset(&PowerInfo, 0, sizeof(PowerInfo));
    rt_memset(&PowerCtrl, 0, sizeof(PowerCtrl));
    rt_memset(Motor_power, 0, sizeof(Motor_power));
    PowerInfo.Cap.SupplyCurrt = Supr_Cap_MaxOutputCurrent;
    PowerCtrl.nowRemain = PowerCtrlReamin;
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_power[i].K1 = Wheel_Power_K1;
        Motor_power[i].K2 = Wheel_Power_K2;
        Motor_power[i].C = Wheel_Power_C;
    }
    // 预留60--超级电容电量--预留云台控制--默认开启
    Wheels_manage->bound_TH_High = RESERVE_BATTERY_TH;
    Wheels_manage->bound_TH_Low = RESERVE_BATTERY_TL;
    // 预留20
    Wheels_manage->bound_TL_High = LOW_BATTERY_TH;
    Wheels_manage->bound_TL_Low = LOW_BATTERY_TL;
    // 是否正在低电量保护
    Wheels_manage->if_hysteresis = RT_FALSE;
    Wheels_manage->if_ReserveEnergy = RT_TRUE; // 是否预留超级电容的电量
    Wheels_manage->if_SC_protect = RT_TRUE;    // 默认直接开启超级电容保护
}

static float get_SupplyCurrt(Scpr_manage_t *whectrl, float *lim_level)
{
    float SupplyCurrt = 0;
    float Scap = PowerInfo.Cap.superCap;
    whectrl->if_ReserveEnergy = (rt_bool_t)Get_PowerRestriction_Status();

    // 判断是不是需要做超级电容电量预留
    if (whectrl->if_ReserveEnergy)
    {
        // 需要做电量预留
        if (Scap >= whectrl->bound_TH_High)
        {
            whectrl->if_hysteresis = RT_FALSE;
            *lim_level = 0;
            SupplyCurrt = 16.f; // 已经关闭限制了这里的赋值也不会起作用
        }
        else if (Scap >= whectrl->bound_TH_Low)
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 1;
            SupplyCurrt = 8.f + 8.f * utils_calc_ratio(whectrl->bound_TH_Low, whectrl->bound_TH_High, Scap);
        }
        else if (Scap >= whectrl->bound_TL_High)
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 1;
            SupplyCurrt = 8.f * utils_calc_ratio(whectrl->bound_TL_High, whectrl->bound_TH_Low, Scap);
        }
        else if (Scap >= whectrl->bound_TL_Low)
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 100.f - 99.f * utils_calc_ratio(whectrl->bound_TL_Low, whectrl->bound_TL_High, Scap);
            SupplyCurrt = 0;
        }
        else
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 100.f;
            SupplyCurrt = 0;
        }
    }
    // 不需要做电量预留(不建议使用)
    else
    {
        if (Scap > whectrl->bound_TL_High)
        {
            whectrl->if_hysteresis = RT_FALSE;
            *lim_level = 0;
            SupplyCurrt = 16.0f; // 无实际效用，仅区分
        }
        else if (Scap > whectrl->bound_TL_Low)
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 100.f - 99.f * utils_calc_ratio(whectrl->bound_TL_Low, whectrl->bound_TL_High, Scap);
            SupplyCurrt = 0;
        }
        else
        {
            whectrl->if_hysteresis = RT_TRUE;
            *lim_level = 100.f;
            SupplyCurrt = 0;
        }
    }
    return SupplyCurrt;
}

static void getSCapDataAndInfo(Scpr_manage_t *whectrl, float *lim_level)
{
    PowerInfo.Cap.isValid = Read_Super_Capacity_Online();
    if (1 == PowerInfo.Cap.isValid)
    { // 当此时超级电容仍然在线时考虑超级电容提供的能量
        // 更新超级电容功率数据
        PowerInfo.Cap.Power_last = PowerInfo.Cap.Power;
        PowerInfo.Cap.superCap = Get_RemainCapcity();
        PowerInfo.Cap.superCap = utils_clamp(PowerInfo.Cap.superCap, 0, 100);
        PowerInfo.Cap.V_Cap = Get_Cap_Voltage();

        // 计算超级电容供电能力
        PowerInfo.Cap.SupplyCurrt = get_SupplyCurrt(whectrl, lim_level);
    }
    else
    { // 当一段过程后超级电容离线
        *lim_level = 1;
        PowerInfo.Cap.SupplyCurrt = 0;
        PowerInfo.Cap.superCap = 0;
        // PowerInfo.Cap.Current = 0;
        // PowerInfo.Cap.Voltage = 0;
    }
}

// 得到裁判系统的关于底盘功率的数据 50Hz更新 20ms,判读数据是否合理
static void getRefDataAndInfo(void)
{
    static uint32_t overPowerTick = 0;
    rt_tick_t tick_tmp = 0;
    uint16_t temp;
    PowerInfo.Ref.isValid = ISRefValid(); // 数据是否有效
    if (1 == PowerInfo.Ref.isValid)
    {
        // 得到功率数据，单位均为国际单位
        PowerInfo.Ref.lastPower = PowerInfo.Ref.Power;
        /**以下数据计算后为国际单位**/
        getRefData(memberGet_parm(temp,ext_power_heat_data,chassis_current_reserve));
        PowerInfo.Ref.Current = temp / 1000.f;         // 电流A
        getRefData(memberGet_parm(temp,ext_power_heat_data,chassis_volt_reserve));
        PowerInfo.Ref.Voltage = temp / 1000.f;         // 电压V

        getRefData(memberGet_parm(PowerInfo.Ref.Power,ext_power_heat_data,chassis_power_reserve));                 // 功率W
        getRefData(memberGet_parm(PowerInfo.Ref.PowerBuff,ext_power_heat_data,chassis_power_buffer));          // 缓冲能量J          
        getRefData(memberGet_parm(PowerInfo.Ref.Power_Maxlim,ext_game_robot_status,chassis_power_limit));    // 最大功率限制W
        // PowerInfo.Ref.IsOverPower = Ref_Chassis_IsOverPower(&tick_tmp); // 是否超功率及超功率时刻
        // 裁判系统功率数据更新时刻：电流，电压，功率，缓冲能量
        PowerInfo.Ref.refresh_tick = GetRefDataRefreshTick(Index_power_heat_data);

        if (PowerInfo.Ref.PowerBuff <= 0)
            PowerInfo.Ref.powerBuffOverCnts++; // 缓冲耗尽计数

        // if (PowerInfo.Ref.IsOverPower && tick_tmp != overPowerTick)
        // { // 超功率时间刷新并计数
        //     overPowerTick = tick_tmp;
        //     PowerInfo.Ref.powerOverCnts++;
        // }
        // else
        //     PowerInfo.Ref.IsOverPower = 0;

        PowerInfo.Ref.IsFlySlope = RefGetIsFlySlope(); // 是否飞坡
        // 判断合理性与基本限幅
        if (0 == PowerInfo.Ref.IsFlySlope)
            PowerInfo.Ref.PowerBuff = PowerInfo.Ref.PowerBuff > RefPowerBuf ? RefPowerBuf : PowerInfo.Ref.PowerBuff;
        else
            PowerInfo.Ref.PowerBuff = PowerInfo.Ref.PowerBuff > POWER_REMANI_FLYSLOPE ? POWER_REMANI_FLYSLOPE : PowerInfo.Ref.PowerBuff;

        // 最大功率限幅
        PowerInfo.Ref.Power_Maxlim = utils_clamp(PowerInfo.Ref.Power_Maxlim, 40, 250);
        // 缓冲能量限幅
        PowerInfo.Ref.PowerBuff = utils_clamp(PowerInfo.Ref.PowerBuff, 0, POWER_REMANI_FLYSLOPE);
    }
    else
    { // 裁判系统数据无效，保守限制：
        PowerInfo.Ref.Power_Maxlim = POWER_REMANI;
        PowerInfo.Ref.PowerBuff = 0;
    }
}

static void getPowerData(Scpr_manage_t *whectrl, float *lim_level)
{
    // 得到超级电容数据
    getSCapDataAndInfo(whectrl, lim_level);
    // 得到裁判系统数据并做基本限幅
    getRefDataAndInfo();
}

static void get_chassis_max_power(float lim_level)
{

    PowerCtrl.Power_Allo_level = 0;

    if (PowerInfo.Cap.isValid && !PowerCtrl_Close_Flag[0])
    { // 超级电容数据有效
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
            if (lim_level > 0)
            {
                PowerCtrl.Power_Allo_level = -lim_level;
            }
            if (PowerInfo.Ref.Power > PowerCtrl.Power_Maxlim_sum)
            { // 当会超功率
                PowerCtrl.Power_Allo_level -= 20;
            }
            if ((PowerInfo.Ref.Power * RefPowerData_RefTime / 1000.f) > PowerInfo.Ref.PowerBuff)
            { // 到下一次数据更新会超功率
                PowerCtrl.Power_Allo_level -= 10;
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
        if (lim_level > 0)
        {
            PowerCtrl.Power_Allo_level = -lim_level;
        }
        if (PowerInfo.Ref.Power > PowerCtrl.Power_Maxlim_sum)
        { // 如果最近得到功率会超功率--数据延迟大于20ms
            PowerCtrl.Power_Allo_level -= 20;
        }
        if ((PowerInfo.Ref.Power * RefPowerData_RefTime / 1000.f) > PowerInfo.Ref.PowerBuff)
        { // 到下一次数据更新会超功率
            PowerCtrl.Power_Allo_level -= 10;
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
    // 一部分固定的静态功率不属于电机可用功率：即要减去Wheel_Power_C_sum
    PowerCtrl.Power_Maxlim = (PowerCtrl.Power_Maxlim_sum - Wheel_Power_C_sum) * (K_Conservative + PowerCtrl.Power_Allo_level * One_Level_Persent);
}

#if Power_Ctrl_Debug == 1
// debug参数：
float now_power_sum;
float out_power_sum;
#endif

float power_set_sum; //  预输出总功率
static void Power_lim_allo_to_wheels(Motor_t *wheel)
{
    /*程序在运行这个函数的时候，PID的输出wheel[index].spe.out已经根据初始化时的设置进行方向回反了，
     * 因为已经使用过了Motor_SpeedPIDCalculate函数计算过PID了，
     * 对于wheel[index].dji.reverse_flag位置1的out输出，需要进行反向才能得到与轮子正反转和set设定值相同的方向定义
     */
    uint8_t index = 0, power_allo_flag_temp = 0;
    float temp = 0, power_set_sum_temp = 0;
#if Power_Ctrl_Debug == 1
    float out_power_sum_temp = 0; // debug
#endif

    // 符号处理思路：先调整,再计算,最后调回去；简化程序，可以避免每次使用out数据都要调整一次符号的逻辑思考浪费行为
    for (index = 0; index < 4; index++) // 调整符号到统一的计算方向，同样的，在本函数最后要将方向调回正确的输出方向
        if (wheel[index].dji.reverse_flag == 1)
            wheel[index].spe.out = -wheel[index].spe.out;

    for (index = 0; index < 4; index++) // first get all the initial motor power and total motor power
    {
        Motor_power[index].V_last = Motor_power[index].V_now;
        Motor_power[index].V_now = Motor_Read_NowSpeed(&wheel[index]); // 原始转子转速数据(rpm)

        /*对于电机模型计算的目前功率的数据暂时不需要，代码已写好放在下面了，可以作为debug的参考数据*/
#if Power_Ctrl_Debug == 1
        Motor_power[index].I_now = Motor_Read_NowTorque(&wheel[index]);          // 原始扭矩电流数据
        Motor_power[index].tor_now = fabsf(Motor_power[index].I_now * K_IC2Tor); // 转子扭矩计算
        Motor_power[index].power_now = fabsf(Motor_power[index].V_now) * Motor_power[index].tor_now / 9.55f +
                                       Motor_power[index].K1 * SQUARE(Motor_power[index].tor_now) +
                                       Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
                                       Motor_power[index].C; // 电机功率模型计算公式
        if (Motor_power[index].power_now > 0)
            temp += Motor_power[index].power_now;
#endif
        Motor_power[index].initial_tor_set = fabsf(wheel[index].spe.out * K_IC2Tor); // 转子扭矩计算
        // 功率分配前的预输出功率
        Motor_power[index].initial_power_set = fabsf(Motor_power[index].V_now) * Motor_power[index].initial_tor_set / 9.55f +
                                               Motor_power[index].K1 * SQUARE(Motor_power[index].initial_tor_set) +
                                               Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
                                               Motor_power[index].C; // 电机功率模型计算公式
        power_set_sum_temp += Motor_power[index].initial_power_set;
    }
#if Power_Ctrl_Debug == 1
    now_power_sum = temp + Wheel_Power_C_sum; // debug
#endif
    power_set_sum = power_set_sum_temp;         // 电机预输出的总功率（不包括固定静态功率）
    k_power_allo = 0;                           // 功率分配系数
    if (power_set_sum > PowerCtrl.Power_Maxlim) // determine if larger than max power
    {
        if (PowerCtrl.Power_Allo_level >= 0)
            power_allo_flag_temp = 1; // 正常功率限制模式标志：1
        else
            power_allo_flag_temp = 2; // 强制功率限制模式标志：2
        // 一部分固定的静态功率不属于电机可用功率：即要使用减去Wheel_Power_C_sum的功率数据，PowerCtrl.Power_Maxlim和power_set_sum的值已不包括静态功率
        PowerCtrl.k_power_allo = (PowerCtrl.Power_Maxlim / power_set_sum);
    }
    else
        PowerCtrl.k_power_allo = 1.0f; // 全时可引用参数对外更新
#if IF_USED_ALLO_LEVEL == 1
    else if (PowerCtrl.Power_Maxlim - power_set_sum >= Gain_Trigger_Power)
    { // PID set 的控制是转子转速rpm,
        power_allo_flag_temp = 3;
        power_set_sum = Wheel_Power_C_sum; // power_set_sum运行到这里也完成使命了，这里拿来统计增益功率重分配后的总功率
        for (index = 0; index < 4; index++)
        {
            Motor_power[index].gain_allo_level = fabsf(Motor_power[index].V_now - wheel[index].spe.set) / 2000.0;
            Motor_power[index].k_gain_power_allo = (K_Conservative + Motor_power[index].gain_allo_level * One_Level_Persent);
            Motor_power[index].allo_power_set = Motor_power[index].initial_power_set * Motor_power[index].k_gain_power_allo;
            power_set_sum += Motor_power[index].allo_power_set;
        }
        if (power_set_sum >= PowerCtrl.Power_Maxlim) // 保险：增益功率重分配后检查
            power_allo_flag_temp = 0;
    }
#endif
    PowerCtrl.power_allo_flag = power_allo_flag_temp; // 功率分配标志
    if (PowerCtrl.power_allo_flag != 0)
    {
        for (index = 0; index < 4; index++)
        {
#if IF_USED_ALLO_LEVEL == 1
            if (PowerCtrl.power_allo_flag == 3) // 增益情况下的功率分配计算方法不同，且由于在增益条件判断和检查时需要用到，已经计算过了，这里不再计算
                ;
            else
#endif
                Motor_power[index].allo_power_set = Motor_power[index].initial_power_set * PowerCtrl.k_power_allo; // 功率分配后的预输出功率

            // Selection of the calculation formula according to the motor acceleration and deceleration
            // 计算功率分配后的输出转矩（这里的公式计算完的扭矩的方向是绝对的，未调整正确输出符号的）
            if (SIGN_F(Motor_power[index].V_now) * SIGN_F(wheel[index].spe.out) >= 0)
            {
                // 电机转速与输出扭矩方向相同，此时反电动势方向与输出扭矩方向相反，相同功率下输出较小的扭矩
                // 计算完为正
                Motor_power[index].allo_tor_set = (-fabsf(Motor_power[index].V_now) / 9.55f +
                                                   sqrtf(SQUARE(Motor_power[index].V_now / 9.55f) -
                                                         4 * Motor_power[index].K1 * (Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) + Motor_power[index].C - Motor_power[index].allo_power_set))) /
                                                  (2 * Motor_power[index].K1);
            }
            else
            {
                // 电机转速与输出扭矩方向相反，此时反电动势方向与输出扭矩方向相同，相同功率下输出较大的扭矩
                // 计算完为负
                Motor_power[index].allo_tor_set = (-fabsf(Motor_power[index].V_now) / 9.55f -
                                                   sqrtf(SQUARE(Motor_power[index].V_now / 9.55f) -
                                                         4 * Motor_power[index].K1 * (Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) + Motor_power[index].C - Motor_power[index].allo_power_set))) /
                                                  (2 * Motor_power[index].K1);
            }
            temp = fabsf(Motor_power[index].allo_tor_set); // 取扭矩绝对值
            // 转换为输出的扭矩电流(COUNT)，并按正确的方向输出
            if (wheel[index].spe.out >= 0)
                wheel[index].spe.out = temp / K_IC2Tor;
            else
                wheel[index].spe.out = -temp / K_IC2Tor;
#if Power_Ctrl_Debug == 1
            // 注：用反解出的扭矩再计算输出总功率，但这里计算完的总功率数据是错误的，而扭矩对应的输出功率并没有错，原因待思考
            out_power_sum_temp += fabsf(Motor_power[index].V_now) * temp / 9.55f +
                                  Motor_power[index].K1 * SQUARE(temp) +
                                  Motor_power[index].K2 * SQUARE(Motor_power[index].V_now) +
                                  Motor_power[index].C;
#endif
        }
#if Power_Ctrl_Debug == 1
        out_power_sum = out_power_sum_temp + Wheel_Power_C_sum; // debug
#endif
    }

    for (index = 0; index < 4; index++) // 调回正确的输出符号
        if (wheel[index].dji.reverse_flag == 1)
            wheel[index].spe.out = -wheel[index].spe.out;
}

/*功率控制周期为1ms，但裁判系统数据频率只有50Hz(20ms)*/
void Power_Ctrl(Scpr_manage_t *whectrl, Motor_t *wheel)
{
    static float lim_level = 1.0f;
    // 得到功率控制相关数据
    getPowerData(whectrl, &lim_level);
    if (lim_level != 0 || Power_Ctrl_Debug)
    {
        // 计算最大可输出功率
        get_chassis_max_power(lim_level);
        // 功率限制、功率再分配
        Power_lim_allo_to_wheels(wheel);
    }
}

float getScRate(void)
{ // 已经没用了
    return 0;
}
