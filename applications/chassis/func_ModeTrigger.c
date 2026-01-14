#include "func_ModeTrigger.h"
#include "drv_wheel.h"
#include "fun_powerCtrl.h"

#define TICK_PROBE_STORAGE(name) static Tick_probe_t name = {0, RT_FALSE}
/*时间探针结构体*/
typedef struct
{
    rt_tick_t last_tick;
    rt_bool_t if_init;
} Tick_probe_t;
float rtTick_probe(Tick_probe_t *record_p);

// 加速度平滑的结构体
Smooth_t Smooth_Acc[SMOOTH_NUM];
static Mode_trigger_t modetrig[MOTION_MODE_NUM];

/**
 * @brief   获取所在函数被调用的时间间隔
 * @param record_p 静态的或者全局的变量指针
 * @return  当前被调用到上一次被调用的时间间隔，单位ms，float类型
 */
float rtTick_probe(Tick_probe_t *record_p)
{
    Tick_probe_t *record = record_p;
    float period = 0;
    if (record->if_init == RT_TRUE)
    {
        rt_tick_t now_tick = rt_tick_get();
        period = (float)(now_tick - record->last_tick) / (float)RT_TICK_PER_SECOND * 1000.0f; /*时间间隔，单位ms*/
        record->last_tick = now_tick;
    }
    else /*第一次执行到,给last_tick赋初值*/
    {
        record->last_tick = rt_tick_get();
        record->if_init = RT_TRUE;
        period = 0;
    }
    return period;
}

/**
 * @brief   输入xy速度平滑模块
 * @param   Vxy xy速度矢量
 * @return  平滑后值
 */
Vector2_t SmoothAcc_VXY(Vector2_t Vxy_set, Vector2_t Vxy_real)
{
    Vector2_t Vxy_temp = Vxy_set;
    Smooth_Acc_Process(&Smooth_Acc[WHEEL_XSPEED], &Vxy_temp.x, Vxy_real.x, PowerCtrl.power_allo_flag != 0);
    Smooth_Acc_Process(&Smooth_Acc[WHEEL_YSPEED], &Vxy_temp.y, Vxy_real.y, PowerCtrl.power_allo_flag != 0);

    return Vxy_temp;
}

/**
 * @brief   输出角速度平滑模块
 * @param   angvel 角速度
 * @return  平滑后值
 */
float SmoothAcc_w(float angvel_set, float angvel_real)
{
    float angvel_temp = angvel_set;
    Smooth_Acc_Process(&Smooth_Acc[WHEEL_ACSPEED], &angvel_temp, angvel_real, PowerCtrl.power_allo_flag != 0);

    return angvel_temp;
}

/**
 * @brief   获取xy设定的距离增量
 * @param   None
 * @return  Vector2_t 距离增量，单位μm,使用时要打开SMOOTH_VELOCITY_ENABLE宏
 */
Vector2_t ExMotMod_Get_xyDelta(void)
{
    Vector2_t Delta;
    float period = 0;
    static Vector2_t last_speed = {0, 0};
    TICK_PROBE_STORAGE(store);

    // /*读取经过平滑后的速度设定值*/
    Vector2_t now_speed = {Smooth_Acc_ReadOut(&Smooth_Acc[WHEEL_XSPEED]), Smooth_Acc_ReadOut(&Smooth_Acc[WHEEL_YSPEED])};

    /*获取时间间隔*/
    period = rtTick_probe(&store);

    Delta.x = (now_speed.x + last_speed.x) * (period / 2.0f);
    Delta.y = (now_speed.y + last_speed.y) * (period / 2.0f);

    last_speed = now_speed;
    return Delta;
}

/**
 * @brief    Follow_Gimbal模式进入函数
 */
//static void Follow_Gimbal_Entry(void)
//{
//    Smooth_Acc_Modify_Speed(&Smooth_Acc[WHEEL_ACSPEED], SMOOTH_ADD_UP_AC_2, SMOOTH_ADD_DOWN_AC_2); // 改大参数
//}

/**
 * @brief    Follow_Gimbal模式退出函数
 */
//static void Follow_Gimbal_Exit(void)
//{
//    Smooth_Acc_Modify_Speed(&Smooth_Acc[WHEEL_ACSPEED], SMOOTH_ADD_UP_AC_1, SMOOTH_ADD_DOWN_AC_1); // 改小参数
//}

/**
 * @brief    模式触发调度
 * @param    now_mode    当前模式
 * @return   None
 * @author   lfp
 */
void ModeTrig_Scheduler(Motion_mode_e now_mode)
{
    static Motion_mode_e last_mode;

    // 轮询所有模式
    for (int mm = 0; mm < MOTION_MODE_NUM; mm++)
    {
        // 根据当前模式和上一次模式，判断跳变沿
        if (now_mode == mm && last_mode != mm)
        {
            if (modetrig[mm].Entry_Mode != RT_NULL)
                (*modetrig[mm].Entry_Mode)(); // 生成进入函数
        }
        else if (now_mode != mm && last_mode == mm)
        {
            if (modetrig[mm].Exit_Mode != RT_NULL)
                (*modetrig[mm].Exit_Mode)(); // 生成退出函数
        }
    }

    // 赋值上一次模式
    if (now_mode != last_mode)
        last_mode = now_mode;
}

/**
 * @brief : 设置回调函数
 * @param {Motion_mode_e} Motion_mode
 * @param {Mode_trigger_e} Mode_trigger
 * @return {*}
 */
void Set_Mode_trigger(Motion_mode_e Motion_mode, Mode_trigger_e Mode_trigger, void (*CallBackFun)(void))
{
    switch (Mode_trigger)
    {
    case Entry_Mode_e:
        modetrig[Motion_mode].Entry_Mode = CallBackFun;
        break;
    case Exit_Mode_e:
        modetrig[Motion_mode].Exit_Mode = CallBackFun;
        break;
    default:
        break;
    }
}

///**
// * @brief : 初始化模式调度切换回调函数
// */
//static void Init_Mode_trigger(void)
//{
//    for (int fori; fori < MOTION_MODE_NUM; fori++)
//    {
//        modetrig[fori].Entry_Mode = NULL;
//        modetrig[fori].Exit_Mode = NULL;
//    }
//    Set_Mode_trigger(FOLLOW_GIMBAL, Entry_Mode_e, Follow_Gimbal_Entry);
//    Set_Mode_trigger(FOLLOW_GIMBAL, Exit_Mode_e, Follow_Gimbal_Exit);
//}

//static void SmoothInit(void)
//{
//    // 初始化结构体
//    Smooth_Acc_Init(&Smooth_Acc[WHEEL_XSPEED], SMOOTH_ADD_UP, SMOOTH_ADD_DOWN, SMOOTH_START, SMOOTH_MODE);
//    Smooth_Acc_Init(&Smooth_Acc[WHEEL_YSPEED], SMOOTH_ADD_UP, SMOOTH_ADD_DOWN, SMOOTH_START, SMOOTH_MODE);
//    Smooth_Acc_Init(&Smooth_Acc[WHEEL_ACSPEED], SMOOTH_ADD_UP_AC_1, SMOOTH_ADD_DOWN_AC_1, SMOOTH_START_AC, SMOOTH_MODE_AC);
//}

// 初始化平滑和模式及切换时的基本数据
// void func_trigInit(void)
// {
//     SmoothInit();
//     Init_Mode_trigger();
// }
