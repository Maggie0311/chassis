#include "drv_wheel.h"
#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "mod_Log.h"
#include "robot_data.h"
#include "Look.h"
#include "robot_control.h"
/*所有顺序为转向轮和驱动轮---RF-LF-LB-RB*/
static struct rt_timer Cal_Timer;   /*线程定时器*/
static struct rt_semaphore Cal_sem; /*线程信号量*/

/**
 * @brief 定时器回调,用于唤醒电机pid计算线程
 *
 * @param parameter
 */
static void Cal_IRQHandler(void *parameter) //
{
    rt_sem_release(&Cal_sem);
}


float test_angle = 0.0f,test_angle_up=0.f;
// 电机控制部分
//  车轮pid计算线程
//   该线程总的来说是根据外部(Chassis_Ctrl_Thread)计算出来的底盘设定值来进行pid计算并发送数据给电机
static void PID_Cal_Thread(void)
{

    CREAT_ID(id);
    ADDTOMONITOR_ID("PID_Cal_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    while (1)
    {
        rt_sem_take(&Cal_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        //Look_Cnt[1]++;//用于观测线程运行频率


       //Screw_Motor_Move(Forward_Screw_Motor, test_angle_up);
      //  Screw_Motor_Move(Back_Screw_Motor,test_angle);
        /*pid等电机控制计算*/
        Motor_Control_Loop();

#ifdef USE_CAPACITY_3

        Power_Ctrl();
#endif

        /*can报文发送*/
        Motor_Transmit_Loop();

        /*进行正解算*/
        Forward_Kinematics();
    }
}

/*------------------------------------电机初始化----------------------------------------------------------*/
/**
 * @brief   底盘电机初始化
 */
rt_err_t Wheels_Init(void)
{
    rt_err_t res = 0;
    rt_thread_t thread = RT_NULL;

    // 初始化电机结构体以及PID
    res = chassis_motor_init();
    if (res != RT_EOK)
        return RT_ERROR;

#ifdef USE_CAPACITY_3
    powerCtrlInit();
#endif
  //  Zero_Init();
    // 初始化信号量
    res = rt_sem_init(&Cal_sem, "PID_Cal_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    // TODO 底盘电机线程优先级修改
    //  初始化底盘线程
    thread = rt_thread_create("PID_Cal_Thread", // 线程名
                              PID_Cal_Thread,   // 线程入口
                              RT_NULL,          // 入口参数无
                              4096,             // 线程栈
                              2,                // 线程优先级
                              3);               // 线程时间片大小

    // 线程创建失败返回false
    if (thread == RT_NULL)
        return RT_ERROR;

    // 线程启动失败返回false
    if (rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    // 创建线程定时器 定时释放信号量 PID_Cal_Thread
    rt_timer_init(&Cal_Timer,
                  "Cal_Timer",
                  Cal_IRQHandler,
                  RT_NULL,
                  PID_CAL_PERIOD,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);//200Hz

    // 启动定时器
    res = rt_timer_start(&Cal_Timer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}

/**
 * @brief 读取底盘电机当前的离线情况
 * 从低位到高位顺序为舵向电机的RF-LF-LB-RB，轮向电机的RF-LF-LB-RB 对应位置 1 代表离线
 * @return rt_uint8_t
 */
rt_uint8_t Read_Wheels_Offline_State(void)
{
    //    uint8_t state[WHEEL_NUM];
    //    //state[WHEEL_RUN] = Wheels_Ctrlinfo[WHEEL_RUN].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
    //    //state[WHEEL_ANGLE] = Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
    //    return (state[WHEEL_ANGLE] << WHEELS_GROUP_NUM) & state[WHEEL_RUN];
}
