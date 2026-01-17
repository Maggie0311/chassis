#ifndef __FUNC_SMOOTH_ACC_H__
#define __FUNC_SMOOTH_ACC_H__


#define Set_Data_Type  float

typedef struct {
    Set_Data_Type now_value;        // 当前输出值
    Set_Data_Type last_set_value;   // 上一次的设定值
    float max_change_rate;          // 最大允许变化率（单位/秒）
    int last_time;            // 上一次调用的时间戳
    float acceleration_factor;      // 动态加速因子
    float deceleration_factor;      // 动态减速因子
} SmoothRateLimiter;

/*平滑类型*/
typedef enum
{

    WHEEL_XSPEED, // 底盘速度的平滑
    WHEEL_YSPEED,

    WHEEL_ACSPEED, // 底盘角速度的平滑
    SMOOTH_NUM,

} Smooth_Acc_e;

int SmoothRateLimiter_init(SmoothRateLimiter* limiter,
    float max_change_rate,
    float accel_factor,
    float decel_factor);
Set_Data_Type DynamicSmooth_func(SmoothRateLimiter* limiter, Set_Data_Type set_value);


#endif // __FUNC_SMOOTH_ACC_H__