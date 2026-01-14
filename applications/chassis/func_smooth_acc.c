#include "func_smooth_acc.h"
#include <rtthread.h>
#include <arm_math.h>
/// <summary>
/// 缓动函数，用于生成系数
/// </summary>
/// <param name="process">缓动启动阶段，值为0 - 1 </param>
/// <returns></returns>
float Slow_anction_func1(float process)
{
	return (3 - 2 * process)* process* process;
}

/// <summary>
/// 缓动函数，用于生成系数
/// </summary>
/// <param name="process">缓动启动阶段，值为0 - 1 </param>
/// <returns></returns>
float Slow_anction_func2(float process)
{
	return process;
}
/// <summary>
/// 缓动函数，用于生成系数
/// </summary>
/// <param name="process">缓动启动阶段，值为0 - 1 </param>
/// <returns></returns>
float Slow_anction_func3(float process)
{
	return 1-(1-process)*(1-process);
}
/// <summary>
/// 插值函数，用来生成中间平滑过程的值
/// </summary>
/// <param name="time_now"></param>现在的平滑时间
/// <param name="time_all"></param>平滑总时间
/// <param name="initial_value"></param>平滑过程开始值
/// <param name="set_value"></param>平滑过程结束值
/// <returns></returns>返回最终设置的中间过程值
Set_Data_Type Interpolation_func(float time_now ,float time_all , Set_Data_Type initial_value , Set_Data_Type set_value)
{
	float process = time_now / time_all;
	static Set_Data_Type value_max = 0;
	float ret_value = 0;
	process = Slow_anction_func1(process);
	if (process > 1 || process < 0)
	{
		value_max = process = 1;
	}
	ret_value = initial_value * (1 - process) + set_value * process;

	return ret_value;
}


/**
 * @brief 初始化平滑加速数据结构体（改进版）
 *
 * @param limiter 平滑限制器指针
 * @param max_change_rate 最大允许变化率（单位/秒）
 * @param accel_factor 加速因子（>1）
 * @param decel_factor 减速因子（0-1）
 */
int SmoothRateLimiter_init(SmoothRateLimiter* limiter,
    float max_change_rate,
    float accel_factor,
    float decel_factor)
{
    if (limiter == NULL) return 0;

    limiter->now_value = 0;
    limiter->last_set_value = 0;
    limiter->max_change_rate = max_change_rate;
    limiter->last_time = rt_tick_get();
    limiter->acceleration_factor = (accel_factor > 1) ? accel_factor : 1.5f;
    limiter->deceleration_factor = (decel_factor > 0 && decel_factor < 1) ? decel_factor : 0.7f;

    return 1;
}

int SmoothRateLimiter_modify(SmoothRateLimiter* limiter,
	float max_change_rate,
	float accel_factor,
	float decel_factor)
{
	if (limiter == NULL) return 0;

	limiter->max_change_rate = max_change_rate;
	limiter->acceleration_factor = (accel_factor > 1) ? accel_factor : 1.5f;
	limiter->deceleration_factor = (decel_factor > 0 && decel_factor < 1) ? decel_factor : 0.7f;

	return 1;
}

/**
 * @brief 改进的平滑函数（基于变化率限制）
 *
 * @param limiter 平滑限制器指针
 * @param set_value 新的设定值
 * @return Set_Data_Type 平滑后的输出值
 */
Set_Data_Type DynamicSmooth_func(SmoothRateLimiter* limiter, Set_Data_Type set_value)
{
    // 获取当前时间和时间差（毫秒）
    int now_time = rt_tick_get();
    float dt = (now_time - limiter->last_time);
    limiter->last_time = now_time;

    // 跳过无效时间间隔
    //if (dt <= 0.0001f) {
    //    return limiter->now_value;
    //}

    // 计算设定值变化方向和幅度
    float delta_set = set_value - limiter->last_set_value;
    limiter->last_set_value = set_value;

    // 计算当前实际值与目标值的差距
    float error = set_value - limiter->now_value;
    float abs_error = fabsf(error);

    // 计算当前实际变化率
    float current_rate = abs_error / dt;

    // 动态调整最大变化率
    float effective_max_rate = limiter->max_change_rate;

    // 当设定值快速变化时加速跟踪
    if (abs_error > effective_max_rate * dt * 2.0f) {
        effective_max_rate *= limiter->acceleration_factor;
    }
    // 当接近目标值时减速平滑
    else if (abs_error < effective_max_rate * dt * 0.5f) {
        effective_max_rate *= limiter->deceleration_factor;
    }

    // 计算允许的最大变化量
    float max_delta = effective_max_rate * dt;

    // 应用变化率限制
    if (abs_error > max_delta) {
        limiter->now_value += (error > 0) ? max_delta : -max_delta;
    }
    else {
        limiter->now_value = set_value;
    }

    return limiter->now_value;
}