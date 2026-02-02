#include "func_Observer.h"

#include "drv_utils.h"

typedef struct
{
    float I_Obs;                 // 充电电流的观测量
    float E_Obs;                 // 超级电容实际电压的观测值
    float CAP_Res;               // 外部给定超级电容组的内阻
    float CAP_C;                 // 外部给定超级电容组的电容值
    float Observer_Gain;         // 观测器增益
    uint8_t init_flag;           // 本模块是否被初始化
    uint32_t Last_Observe_Tick; // 上一次运行状态观测器进行观测的时间
} V_CAP_Obs_Struct_t;               // 用于观测超级电容电压的观测器结构体
V_CAP_Obs_Struct_t Observer = 
{   .I_Obs = 0.1f,
    .E_Obs = 0.1f,
    .CAP_Res = 0.2f,
    .CAP_C = 2.8571f,
    .Observer_Gain = 1.1f,
    .init_flag = 0,
    .Last_Observe_Tick = 0
};



/**
* @brief 对电容电压进行观测的观测器
* 
* @param VCap       此时的测量电压
* @param NowTick    此时的时间刻
 */
void SCAP_Observer(float VCap, uint32_t NowTick)
{
    if (isnan(VCap))
        return;
    // 初始化以后首次运行将实际测量电压输入给观测器
    if (!Observer.init_flag)
    {
        Observer.init_flag = 1;
        Observer.E_Obs = VCap;
        Observer.Last_Observe_Tick = NowTick;
        return;
    }
    if (NowTick < Observer.Last_Observe_Tick)
        return;
    Observer.I_Obs = (VCap - Observer.E_Obs) / Observer.CAP_Res;
    Observer.E_Obs += Observer.Observer_Gain * Observer.I_Obs * 
                            (NowTick - Observer.Last_Observe_Tick) / (Observer.CAP_C * 1000);
    Observer.Last_Observe_Tick = NowTick;
}

/**
* @brief 读取此时的观测电压值
* 
* @return float 
 */
float Read_U_obs(void)
{
    return Observer.E_Obs;
}
