#include "app_ChassisCtrl.h"

#include "ThreadCtrl.h"
#include "app_monitor.h"
#include "mod_Log.h"

#include "mod_motion.h"
#include "drv_wheel.h"
#include "Look.h"
#include "func_MonHandling.h"
#include "drv_motionSolve.h"
#include "robot_control.h"
#include "drv_GimMotor.h"
#include "fun_protocolGim.h"
#include  "fun_powerCtrl.h"
#ifdef BSP_USING_LOG
#define LOG_TAG "Ctrl"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#endif

/*-------------------外部控制单位,xy平移速度矢量,单位mm/s; 自转角速度,单位0.1°/s------------*/

/*方便debug (Ctrl_data_info_t *)nowCtrlSrc->data*/
Ctrl_source_t *nowCtrlSrc = NULL; // 当前数据源全局变量

/*线程定时器*/
static struct rt_semaphore ChassCtrl_sem;
static void ChassCtrl_IRQHandler(void *parameter)
{
    rt_sem_release(&ChassCtrl_sem);
}
uint8_t test_stair=0,test_stair_down=0;
// debug
Ctrl_data_t debug_fixctrl;
Machine_Status status=Forward_Screw_Move_Up;
extern float motor_speed[4];

extern Mot_base_t real_xyw;
extern float test_off_x;
extern float test_off_y;


/**
 * @brief    模式状态调度 总的来说就是根据云台控制的运动方式，得到底盘坐标系下的Vx,Vy矢量
 * @param    Cctrl    Ctrldata_t指针
 */
static void ModeState_Scheduler(Ctrl_data_t Cctrl)
{
    Ctrl_data_t fixCtrl = Cctrl;
     debug_fixctrl=Cctrl;
    fixCtrl = debug_fixctrl;
#ifdef CORE_USING_HERO
//    if (Get_If_Go_Upstairs() && fixCtrl.motion_mode == FOLLOW_GIMBAL) // 是否是英雄正需要上台阶
//    {
    if(get_up_stairs_flag()&&get_chassis_up_stairs_finish()==0)
    {
        Motor_Speed_Read();                                // 获取底盘轮向电机速度
       fixCtrl.xyw = UP_Machine_SM(&status, motor_speed); // 根据状态机算出实际速度矢量
       fixCtrl.motion_mode = GO_UPSTAIRS;                 // 将运动模式设置为上台阶
   }
   else if(get_down_stairs_flag())
   {
       Screw_Motor_Move(Back_Screw_Motor, 520, 0);

    //    Motor_Speed_Read();                                // 获取底盘轮向电机速度
    //    fixCtrl.xyw = Down_Machine_SM(&status, motor_speed); // 根据状态机算出实际速度矢量
    //    fixCtrl.motion_mode = GO_UPSTAIRS;   
   }
   else 
   {
       status=Forward_Screw_Move_Up;
       Screw_Motor_Move(Forward_Screw_Motor, 0, 1);
       Screw_Motor_Move(Back_Screw_Motor, 0, 1);
   }
#endif

    switch (fixCtrl.motion_mode) // 控制模式由控制端更改
    {
#ifdef CORE_USING_HERO
    case GO_UPSTAIRS: // 英雄上台阶
        break;
    case CHASSIS_DANGLING_X: // 底盘悬空X轴
        chasis_dangling_x();
        break;
#endif
    case CHASS_AUTO: // 底盘自主
        fixCtrl.xyw = MotPack_Only_Chass(fixCtrl.xyw);
        break;
    case NO_FOLLOW: // 不跟随，小陀螺模式
    case SLOW_GYRO:
    case FAST_GYRO:
    case CHANGE_GYRO:
        fixCtrl.xyw = MotPack_Small_Top(&fixCtrl);
        break;
    case FOLLOW_GIMBAL: // 跟随，以云台坐标系控制底盘
//    case MOVE_BACK:
  //  case FOLLOWBACK_GIMBAL:
    {
        /* 前馈 + PID 叠加补偿，仅跟随模式生效（小陀螺不用IMU数据） */
        // fixCtrl.xyw.vel.x += (ff_x + fix_speed_x.Output);
        // fixCtrl.xyw.vel.y +=( ff_y + fix_speed_y.Output);
        fixCtrl.xyw = MotPack_Follow_Gim(&fixCtrl, 0);
        break;

    }

    default: // 底盘自主
        fixCtrl.xyw = MotPack_Only_Chass(fixCtrl.xyw);
        break;
    }
    // debug_fixctrl = fixCtrl; // 控制源控制数据
                             // fixCtrl.xyw = MotPack_Power_Limit(&fixCtrl);
    Set_Plan_Vxyw_Process(fixCtrl);
    // debug_fixctrl = fixCtrl;
    //  修改pid的设定值
    //  chassis_ctrl(&debug_fixctrl.xyw, &Force_xyw);
    #ifdef CORE_USING_2026_INFANTRY_2
    chassis_ctrl(&Speed_xyw, &Force_xyw,fixCtrl.motion_mode);
    #elif defined(CORE_USING_HERO)
    if(fixCtrl.motion_mode!=CHASSIS_DANGLING_X)
      chassis_ctrl(&Speed_xyw, &Force_xyw );
    #else
      chassis_ctrl(&Speed_xyw, &Force_xyw );
    #endif
}

uint32_t rt_tick = 0;
// 该线程总的来说是根据云台发来的数据进行处理转化为底盘轮子的设定值--其中考虑调度模式等细节问题
static void Chassis_Ctrl_Thread(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("Chassis_Ctrl_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    while (1)
    {
        Ctrl_data_t *Ctrl_data = NULL;
        rt_sem_take(&ChassCtrl_sem, RT_WAITING_FOREVER);
     //   Look_Cnt[0]++;//用于观测线程运行频率

        /* 获得当前控制源--主要为了获得其数据*/
        nowCtrlSrc = getNowCtrlSrc_s();
#ifdef CORE_USING_SENTRY // 如果是哨兵
        // 哨兵模式下的特殊处理，防止串口掉线后底盘不动
        if (getIsSentryMode() && ctrl_source[CS_Visual].if_valid == 0 && ctrl_source[CS_Visual].if_used == 1)
        {
            if (nowCtrlSrc->data)
            {
                Ctrl_data = (Ctrl_data_t *)nowCtrlSrc->data; /*得到原始设定值*/
                Ctrl_data->xyw.angvel = 5000;                // 视觉控制掉线时默认转速
                Ctrl_data->xyw.angvel = 5000;                // 视觉控制掉线时默认转速
            }
        }
#endif
        if (nowCtrlSrc->data)
        {
            Ctrl_data = (Ctrl_data_t *)nowCtrlSrc->data; /*得到原始设定值*/
            ModeState_Scheduler(*Ctrl_data);             /*得到真正底盘坐标系控制的数据*/
        }
        else
        {
            Ctrl_data_t NoCtr;
            rt_memset(&NoCtr, 0, sizeof(NoCtr));
           ModeState_Scheduler(NoCtr);
        }
        SWDG_FEED(id);
    }
}

static rt_err_t chassisCtrlInit(void)
{
    rt_err_t res = RT_ERROR;
    rt_thread_t thread = RT_NULL;

    ctrlSrcInit_s(); /*初始化数据源结构体 安排数据源优先级等*/
    // MotSmooth_Vxyw_init(); /*初始化平滑和模式及切换时的基本数据*/
    Set_Planner_init(); // 新修改的设定值规划结构体初始化

    Set_Planner_Up_motor_init();//抬升机构四个电机设定值规划结构体初始化
    // 跟随模式PID初始化
    Follow_pid_init();
    //全局速度PID初始化
    // Fix_Speed_pid_init();
    // 卡尔曼滤波融合初始化 
  //  Kalman_init();
#ifdef CORE_USING_HERO
    //抬升机构对零点
    
    // 抬升机构前移pid初始化
    Up_Machine_pid_init();
#endif
    // 初始化信号量
    res = rt_sem_init(&ChassCtrl_sem, "Cctrl_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    // 该线程总的来说是根据控制端发来的数据进行处理转化为pid底盘车轮的设定目标值
    thread = rt_thread_create("Chassis_Thread",
                              Chassis_Ctrl_Thread,
                              RT_NULL,
                              THREAD_STACK_CHASSIS_CTRL,
                              THREAD_PRIO_CHASSIS_CTRL,
                              THREAD_TICK_CHASSIS_CTRL);

    // 线程创建失败返回false
    if (thread == RT_NULL)
        return RT_ERROR;

    // 线程启动失败返回false
    if (rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    // 创建线程定时器 定时释放信号量
    rt_timer_t timer = rt_timer_create("Cctrl_timer",
                                       ChassCtrl_IRQHandler,
                                       RT_NULL,
                                       CHASSIS_CTRL_PERIOD,
                                       RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    // 启动定时器
    res = rt_timer_start(timer);
    if (res != RT_EOK)
        return res;
    return res;
}

/**
 * @brief   底盘控制初始化
 * @param   None
 * @return  rt_err_t 是否正常初始化
 */
rt_err_t Chassis_Ctrl_Init(void)
{
    rt_err_t res = RT_ERROR;

    /*根据由外部控制端计算出来的底盘各个轮子的设定值，来进行pid计算并发送最后的can报文*/
    res = Wheels_Init();

    if (res != RT_EOK)
        return res;

    /*基本控制逻辑以用来得到各个轮子的设定值*/
    res = chassisCtrlInit();

    if (res != RT_EOK)
        return res;

    //Look_Init();

    return RT_EOK;
}

/*------------------------------------向外接口函数----------------------------------------------------------*/

// xy平移速度矢量,单位mm/s; 自转角速度,单位0.1°/s
Mot_base_t Get_Chassis_Set_Speed(void)
{
    Mot_base_t Mot_base = {0, 0, 0};

    if (nowCtrlSrc && nowCtrlSrc->data)
    {
        Mot_base = (((Ctrl_data_t *)(nowCtrlSrc->data))->xyw);
    }
    return Mot_base;
}
