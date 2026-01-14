#include "onlyChassis.h"
#include "Wheel_data.h"
#include "app_monitor.h"
#include "vofa.h"
#include "ThreadCtrl.h"
#include "drv_remote.h"
#include "drv_IMU.h"
#include "drv_utils.h"
#include "controlMsg.h"
static Synthread1_t Synthread;

Ctrl_data_info_t onlyChassis_ctrl;
uint8_t testEn = 0;

rt_err_t DeCreatThreadTime(rt_timer_t *timer, const char *name,
                           void (*timeout)(void *parameter), rt_tick_t time)
{
    // 定时器创建
    *timer = rt_timer_create(
        name, timeout, RT_NULL, time, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (*timer)
    {
        if (rt_timer_start(*timer) != RT_EOK)
            return RT_ERROR;
        else
            return RT_EOK;
    }
    else
        return RT_ERROR;
}

rt_err_t DeCreatThreadSem(rt_sem_t *sem, const char *name)
{
    rt_err_t res;
    *sem = rt_sem_create(name, 0, RT_IPC_FLAG_FIFO);
    if (*sem)
        rt_sem_trytake(*sem);
    return res;
}

static void onlyChassis_Timeout_Handler(void *parameter)
{
    rt_completion_done(&Synthread.completion);
}

Ctrl_data_info_t motionTest;
static uint16_t random(uint16_t min, uint16_t max)
{
    uint16_t value = 0;
    value = rand() % (max + 1 - min) + min;
    return value;
}

// 产生一组连续变化的控制数据
static void motionCtrlTest(void)
{
    static uint32_t time;
    static uint32_t mode;
    if (rt_tick_get() - time > random(200, 900))
    {
        if (mode & 1)
        {
            motionTest.vel_x += random(1, 80);
            motionTest.vel_y += random(1, 80);
        }
        else
        {
            motionTest.vel_x -= random(1, 80);
            motionTest.vel_y -= random(1, 80);
        }
        time = rt_tick_get();
        mode++;
    }

    motionTest.folIsReverse = 0;
    onlyChassis_ctrl.getYawAngel = NULL;
    motionTest.mode = NO_FOLLOW;
    motionTest.vel_x = utils_clamp(motionTest.vel_x, -500, 500);
    motionTest.vel_y = utils_clamp(motionTest.vel_y, -500, 500);
}

static void OnlyChassis_Thread(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("OnlyChassis_Thread", 1000, NULL, ALARM_RED, 0, id);
    SWDG_START(id);
    float wk = 1;
    setSourceIsValid_s(CS_TEST, 1);
    while (1)
    {
        // 延时
        rt_completion_wait(&Synthread.completion, RT_WAITING_FOREVER);

#ifdef CORE_USING_REMOTE

        // 左拨杆朝上控制停
        if (getRemoteData((Remote_data_e)remote_s1) == rc_up)
        {
            rt_memset(&onlyChassis_ctrl, 0, sizeof(onlyChassis_ctrl));
        }
        else
        {
            if (testEn == 0)
            {
                onlyChassis_ctrl.mode = NO_FOLLOW;

                // 根据拨杆和摇杆得到控制数据
                onlyChassis_ctrl.vel_x = getRemoteData((Remote_data_e)left_x_ch);
                onlyChassis_ctrl.vel_y = getRemoteData((Remote_data_e)left_y_ch);

                if (RC_Ctrl_Data.Remote_Data.s2 == rc_up) // 角速度变化
                    wk = 14.f;
                else if (RC_Ctrl_Data.Remote_Data.s2 == rc_down)
                    wk = 5.f;
                else if (RC_Ctrl_Data.Remote_Data.s2 ==rc_mid ) 
                    wk=1.f;
                onlyChassis_ctrl.angvel = getRemoteData((Remote_data_e)w_ch) * wk;

                // 选择控制坐标系
                if (getRemoteData((Remote_data_e)remote_s1) == rc_down)
                {
                    onlyChassis_ctrl.folIsReverse = 0;
                    onlyChassis_ctrl.getYawAngel = (getYawAngelCallBack)Get_Chassis_IMU_Data;
                    onlyChassis_ctrl.arg = IMU_YAW;
                }
                else
                {
                    onlyChassis_ctrl.getYawAngel = NULL;
                }
            }
        }
#else
        onlyChassis_ctrl.vel_x = getVofaData(vx_data);
        onlyChassis_ctrl.vel_y = getVofaData(vy_data);
        onlyChassis_ctrl.angvel = getVofaData(w_data);
#endif

        if (testEn == 1)
        {
            motionCtrlTest();
            Source_WriteData_s(CS_TEST, &motionTest);
        }
        if (testEn == 0 & getRemoteIsValid())
        {
            Source_WriteData_s(CS_TEST, &onlyChassis_ctrl);
        }
        SWDG_FEED(id);
    }
}

static void monitorStop(void)
{
    int id = -1;
    id = Find_Swdg_ID("Can1_Rx_Thread");
    Swdg_Stop(id);
    id = Find_Swdg_ID("Can2_Rx_Thread");
    Swdg_Stop(id);
    id = Find_Swdg_ID("DJI_Process_thread");
    Swdg_Stop(id);
    id = Find_Swdg_ID("Scpr_Thread");
    Swdg_Stop(id);
    id = Find_Swdg_ID("gimRx");
    Swdg_Stop(id);
    id = Find_Swdg_ID("RefUsartRx");
    Swdg_Stop(id);
}

/*以底盘坐标系直接手动控制*/
rt_err_t OnlyChassis_Init(void)
{
    rt_err_t res = RT_EOK;

    // 关闭监视器用于调试
    monitorStop();

#ifdef BSP_USING_VOFA
    vofaInit();
#endif

#ifdef CORE_USING_REMOTE
    remote_uart_init();
#endif

    /* 初始化完成量对象 */
    rt_completion_init(&Synthread.completion);

    DeCreatThreadTime(&Synthread.timer, "OnlyChassis", onlyChassis_Timeout_Handler, THREAD_TIMER_REMOTE);

    /* 初始化线程 */
    rt_thread_t thread = rt_thread_create("myTest", OnlyChassis_Thread, RT_NULL,
                                          THREAD_STACK_REMOTE, THREAD_PRIO_REMOTE, THREAD_TICK_REMOTE);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        res = RT_ERROR;

    return res;
}
