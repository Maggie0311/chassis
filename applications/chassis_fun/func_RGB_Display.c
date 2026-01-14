#include "func_RGB_Display.h"
#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif
#include "app_monitor.h"
#include "ThreadCtrl.h"

#ifdef MONITOR_NEED_ALARM
#include "func_Alarm.h"
#endif

#ifdef BSP_USING_RGB_LIGHT
#include "drv_rgblight.h"


static rt_thread_t RGBDisplay_tid = RT_NULL;
static struct rt_timer task_10ms;
static struct rt_semaphore RGB_10ms_Sem; // 定时器超时函数发送的10ms信号量

typedef struct
{
    float R;
    float G;
    float B;
} Color_t;

typedef struct
{
    Color_t Color;
} CONST_Para_t;

typedef struct
{
    Color_t Color;
    int Period_ms; // 闪烁周期：亮+灭
    int Pulse_ms;  // 短灭时长
} FLASH_Para_t;

typedef struct
{
    Color_t Color;
    int Period_ms;         // 呼吸周期
    float Period_K;        // 呼吸周期对1ms的角速度 rad/1ms
    float Breath_DampRate; // 呼吸时，最暗/最亮的值
    float RGBRate_Bias;    // 呼吸时正弦曲线的均值
    float RGBRate_Amp;     // 呼吸时正弦曲线的幅度
} BREATH_Para_t;

RGBDisplay_Mode_E Mode_Now = RGB_BREATH;

// 用于记录几种模式下的设定数据
CONST_Para_t CONST_Para = {0};
FLASH_Para_t FLASH_Para = {0};
BREATH_Para_t BREATH_Para = {0};

// 设置常亮颜色
void RGBDisplay_CONST(float R, float G, float B)
{
    CONST_Para.Color.R = R;
    CONST_Para.Color.G = G;
    CONST_Para.Color.B = B;

    Mode_Now = RGB_CONST;
}

// 设置呼吸灯
void RGBDisplay_BREATH(float R, float G, float B, int Period_ms, float DampRate)
{
    BREATH_Para.Color.R = R;
    BREATH_Para.Color.G = G;
    BREATH_Para.Color.B = B;
    BREATH_Para.Breath_DampRate = DampRate;
    BREATH_Para.RGBRate_Bias = 1 - DampRate / 2;
    BREATH_Para.RGBRate_Amp = DampRate / 2;

    BREATH_Para.Period_ms = Period_ms;
    BREATH_Para.Period_K = 2 * PI / (float)Period_ms;

    Mode_Now = RGB_BREATH;
}

// 设置闪烁
void RGBDisplay_FLASH(float R, float G, float B, int Period_ms, int Pulse_ms)
{
    FLASH_Para.Color.R = R;
    FLASH_Para.Color.G = G;
    FLASH_Para.Color.B = B;
    FLASH_Para.Period_ms = Period_ms;
    FLASH_Para.Pulse_ms = Pulse_ms;

    Mode_Now = RGB_FLASH;
}

// RGB显示控制线程
static void RGBDisplay_Thread(void *parameter)
{

    int Tick_Now_Mod; // 对周期取模后的Tick 供闪烁模式使用
    float RGBRateNow; // RGB输出比例 供给呼吸灯模式使用
    CREAT_ID(id);
    ADDTOMONITOR_ID("RGBDisplay_Thread", 500, NULL, ALARM_RED, 0, id);
    SWDG_START(id);

    RGBDisplay_BREATH(0, 0, 1, 2000, 0.99f);
    while (1)
    {
        rt_sem_take(&RGB_10ms_Sem, RT_WAITING_FOREVER);

        SWDG_FEED(id);

#ifdef MONITOR_NEED_ALARM
        if (!getAlarmState())
        {
#else
        if (1)
        {
#endif
            // 根据当前设定的模式进行控制
            switch (Mode_Now)
            {
            case RGB_CONST:
                Set_RGB(CONST_Para.Color.R, CONST_Para.Color.G, CONST_Para.Color.B);
                break;
            case RGB_BREATH:
                RGBRateNow = BREATH_Para.RGBRate_Amp * sinf(rt_tick_get() * BREATH_Para.Period_K) + BREATH_Para.RGBRate_Bias;
                Set_RGB(RGBRateNow * BREATH_Para.Color.R, RGBRateNow * BREATH_Para.Color.G, RGBRateNow * BREATH_Para.Color.B);
                break;
            case RGB_FLASH:
                Tick_Now_Mod = rt_tick_get() % FLASH_Para.Period_ms;
                if (Tick_Now_Mod > FLASH_Para.Pulse_ms)
                {
                    Set_RGB(0, 0, 0);
                }
                else
                {
                    Set_RGB(FLASH_Para.Color.R, FLASH_Para.Color.G, FLASH_Para.Color.B);
                }
                break;
            }
        }
    }
}

static void task_10ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&RGB_10ms_Sem) == RT_EOK)
        ;                          // 清空多余的信号量
    rt_sem_release(&RGB_10ms_Sem); // 重新释放信号量
}

// 初始化灯效控制线程
int RGB_Display_Init(void)
{
    rt_sem_init(&RGB_10ms_Sem, "RGB_10ms_Sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化灯效线程
    RGBDisplay_tid = rt_thread_create(
        "RGBDisplay_Thread", // 线程名
        RGBDisplay_Thread,   // 线程入口
        RT_NULL,             // 入口参数无
        THREAD_STACK_RGB,    // 线程栈
        THREAD_PRIO_RGB,     // 线程优先级
        THREAD_TICK_RGB);    // 线程时间片大小

    // 线程创建失败返回false
    if (RGBDisplay_tid == RT_NULL)
    {
        return -1;
    }

    // 创建线程定时器
    rt_timer_init(&task_10ms,
                  "RGBTim",
                  task_10ms_IRQHandler,
                  RT_NULL,
                  10,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    // 线程启动失败返回false
    if (rt_thread_startup(RGBDisplay_tid) != RT_EOK)
    {
        return -1;
    }

    // 启动定时器
    if (rt_timer_start(&task_10ms) != RT_EOK)
    {
        return -1;
    }
    return 0;
}
#endif //BSP_USING_RGB_LIGHT
