#include "ThreadCtrl.h"
#include "drv_motor.h"
#include "drv_wheel.h"
#include "app_monitor.h"
#include "drv_can_com.h"
#include "drv_utils.h"
#include "fun_powerCtrl.h"
#include "func_MonHandling.h"
#include "app_RefSystem.h"

#ifdef BSP_CHASSIS_USE_SPI_CAN
#include "drv_spiCanMaster.h"
#define Data_Type  SPICAN_MsgOnTransfer_t     //设置接收数据类型
#define Rx_fun_p    SpicanRx_fun_p            //设置回调函数类型
#define Rx_fun_e    SPI_CAN_CallBack_e        //设置回调函数枚举索引类型
#define Channal_e   SPI_CAN_e                 //强制转换发送接口枚举类型
#define attachRXfun(motorType,callback_f)    AttachSpiCanCallBack((Rx_fun_e)motorType,(Rx_fun_p)callback_f); 
#define TX_Trans(canDev,msg)                 SPI_CANSTrans((Channal_e)canDev,(Data_Type *) msg);
#define TX_TransNULL(canDev,msg)             SPI_CANSTransNULL((Channal_e)canDev,(Data_Type *) msg);

#elif  !defined (BSP_CHASSIS_USE_SPI_CAN)
#define Data_Type  struct rt_can_msg          //设置接收数据类型
#define Rx_fun_p    canRx_fun_p               //设置回调函数类型
#define Rx_fun_e    canRxFun_e                //设置回调函数枚举索引类型
#define Channal_e   canDev_e                  //强制转换发送接口枚举类型
#define attachRXfun(ID,callback_f)            attachCanRXfun((Rx_fun_e)ID,(Rx_fun_p)callback_f); 
#define TX_Trans(canDev,msg)                 CanTrans((Channal_e)canDev, (Data_Type*)msg);
#define TX_TransNULL(canDev,msg)             CanTransNull((Channal_e)canDev, (Data_Type*) msg);

#endif 


#if defined BSP_CHASSIS_USE_SPI_CAN
SPICAN_MsgOnTransfer_t txmsg_angle={.id=STDID_ANGLE_WHEEL_CURRENT,.ide=RT_CAN_STDID,.rtr = RT_CAN_DTR,.len = 8};  //发送舵向电机电流报文结构体
SPICAN_MsgOnTransfer_t txmsg_run={.id=STDID_RUN_WHEEL,.ide=RT_CAN_STDID,.rtr = RT_CAN_DTR,.len = 8};    //发送轮向电流报文结构体
#else 
struct rt_can_msg txmsg_angle={.id=STDID_ANGLE_WHEEL_CURRENT,.ide=RT_CAN_STDID,.rtr = RT_CAN_DTR,.len = 8};  //发送舵向电机电流报文结构体
struct rt_can_msg txmsg_run={.id=STDID_RUN_WHEEL,.ide=RT_CAN_STDID,.rtr = RT_CAN_DTR,.len = 8};    //发送轮向电流报文结构体
#endif



#ifdef USE_CAPACITY // 低电量保护
Scpr_manage_t Scpr_manage;
#endif


/*所有顺序为转向轮和驱动轮---RF-LF-LB-RB*/
static struct rt_timer Cal_Timer;   /*线程定时器*/
static struct rt_semaphore Cal_sem; /*线程信号量*/

static uint32_t zero[4] = {RF_ANG_ZERO_ANGLE,
                           LF_ANG_ZERO_ANGLE,
                           LB_ANG_ZERO_ANGLE,
                           RB_ANG_ZERO_ANGLE};

/*pid的target为电机快转子的角速度控制单位rpm*/
Wheel_t wheel[WHEELS_GROUP_NUM][WHEEL_NUM];       //轮组数据

/*车轮基本信息结构体*/
Wheels_info_t Wheels_Ctrlinfo[WHEEL_NUM];  // 轮子的各种数据如pid can信息等
MotorStuck_t stuck[WHEELS_GROUP_NUM][WHEEL_NUM]; // 电机堵转

/**
 * @brief 定时器回调,用于唤醒电机pid计算线程
 * 
 * @param parameter 
 */
static void Cal_IRQHandler(void *parameter) // 
{
    rt_sem_release(&Cal_sem);
}

static void CheckMotorIsStuck(Wheel_Group_e group, const Motor_t *const motor, Wheel_Local_e wheel_type);
static rt_uint8_t Run_wheel_PID(void);
static rt_uint8_t Angle_wheel_PID(void);
static void Angle_wheel_deadzone(void);
static rt_err_t chassis_motor_Init(uint8_t isWaitCom);
static void Wheel_Group_Motor_Init(Wheel_Group_e group);
static void MotorWaitCom(void);
static void WheelCtrlInfoDeInit(void);
static void Wheel_set_kind(void);
// static void Refresh_AngWheels_Motor(Data_Type *Msg);
// static void Refresh_RunWheels_Motor(Data_Type*Msg);
static void Refresh_Chassis_Motor(Data_Type*Msg);
static void Angle_wheel_CanTrans(Data_Type * txmsg);
static void Run_wheel_CanTrans(Data_Type* txmsg);
static void Motor_Speed_Limit(Wheel_Local_e wheel_type, float max_limit);




/*------------------------------------电机堵转----------------------------------------------------------*/

//TODO  检查电机堵转逻辑
// 检查电机堵转
static void CheckMotorIsStuck(Wheel_Group_e group, const Motor_t *const motor, Wheel_Local_e wheel_type)
{
#define FLAG1 (motor->dji.speed == 0 && abs(motor->dji.current) > 0)
#define FLAG2 (fabsf(motor->spe.i_limit) == fabsf(motor->spe.i_value))
#define FLAG3 (fabsf(motor->spe.set) > 10)
    // 每隔一定时间进行检查
    if (rt_tick_get() - stuck[group][wheel_type].lastTick > 15)
    {
        stuck[group][wheel_type].lastTick = rt_tick_get();

        // 当没有堵转则检查
        if (stuck[group][wheel_type].isStuck == 0)
        {
            if (FLAG1 && FLAG2 && FLAG3)
            {
                if (stuck[group][wheel_type].stuckCounts > 150)
                {
                    stuck[group][wheel_type].isStuck = 1;
                    stuck[group][wheel_type].stuckTick = rt_tick_get();
                    stuck[group][wheel_type].His_stuck++;
                    stuck[group][wheel_type].stuckCounts = 0;
                }
                else
                    stuck[group][wheel_type].stuckCounts++;
            }
        }
        else // 当存在堵转则检查堵转是否恢复
        {
            // 首先主动恢复堵转控制n次
            if (stuck[group][wheel_type].isStuck && (stuck[group][wheel_type].His_stuck < 10) &&
                (rt_tick_get() - stuck[group][wheel_type].stuckTick > 3500))
            {
                stuck[group][wheel_type].isStuck = 0;
            }
            if (fabsf(motor->spe.err) < 200)
            {
                if (stuck[group][wheel_type].stuckCounts < 150)
                {
                    stuck[group][wheel_type].stuckCounts++;
                }
                else
                {
                    stuck[group][wheel_type].isStuck = 0;
                    stuck[group][wheel_type].stuckCounts = 0;
                }
            }
        }
    }
    stuck[group][wheel_type].lastSet = motor->spe.set;
#undef FLAG1
#undef FLAG2
#undef FLAG3
}

/*外部得到堵转标志位*/
uint8_t getMotorIsStack(Wheel_Group_e group, Wheel_Local_e wheel_type)
{
    return stuck[group][wheel_type].isStuck;
}
/*外部清除堵转标志位*/
uint8_t clcStackFlag(Wheel_Group_e group, Wheel_Local_e wheel_type)
{
    if (stuck[group][wheel_type].His_stuck < 200)
    {
        stuck[group][wheel_type].isStuck = 0;
        return 1;
    }
    else
        return 0;
}
/*------------------------------------电机堵转 END----------------------------------------------------------*/
/*------------------------------------轮向电机前馈计算----------------------------------------------------------*/
// out = -0.000036*set^2 + 0.173347*set + 470.586251 这里set表示期望速度设定值，out代表在该速度下的摩擦力前馈直接设定到电机控制量下的值
float test_k_feed = 0.4;
float RUN_wheel_forward_cal(float set)
{
    if (set > 200.f)
    {
        return -0.000036 * set * set + set * test_k_feed + 470.60; // 0.1734
    }
    else if (set < -200.f)
    {
        return 0.000036 * set * set + set * test_k_feed - 470.60;
    }
    else
    {
        return 503.8 * set / 200; // 在-200到200之间返回近似一次斜坡的函数，防止函数不连续导致的抖动问题
    }
}

void RUN_wheel_forward_set()
{
    // for (int i = 0; i < WHEELS_GROUP_NUM; i++)
    // {
    //     if (wheel[i][WHEEL_RUN].motor.dji.reverse_flag)
    //     {
    //         wheel[i][WHEEL_RUN].motor.spe.feedforward = -RUN_wheel_forward_cal(wheel[i][WHEEL_RUN].motor.spe.set);
    //     }
    //     else
    //     {
    //         wheel[i][WHEEL_RUN].motor.spe.feedforward = RUN_wheel_forward_cal(wheel[i][WHEEL_RUN].motor.spe.set);
    //     }
    // }
}
/*------------------------------------轮向电机前馈计算----------------------------------------------------------*/

/*------------------------------------PID计算部分----------------------------------------------------------*/
extern float force_out[4][2];
static float SpeedNow[4];                   // 轮向电机当前速度
static float spefilterK = SPE_PID_FILTER_K; // 轮向电机速度滤波系数

float k_feed = 1.f;
/**
 * @brief 轮向电机PID计算
 * 
 * @return rt_uint8_t 返回1表示电机PID计算正常，返回0表示舵向电机PID计算异常，异常时在后续会将电机输出置为0
 */
static rt_uint8_t Run_wheel_PID(void)
{
    float SpeedForward[WHEELS_GROUP_NUM][WHEEL_NUM] = 0;
    if ((Wheels_Ctrlinfo[WHEEL_RUN].Crtl_mode == LOOP_CTRL) && (Wheels_Ctrlinfo[WHEEL_RUN].Wheel_Enabled)) // 电机控制信息为闭环控制，且电机使能
    {
        // 输入限制使得控制电机的数据rpm不超过额定转速 当超过限制同比例更改设定值
        Motor_Speed_Limit(WHEEL_RUN, RATED_SPEED_RUN);

        for (int group = 0; group < WHEELS_GROUP_NUM; group++) // 得到某位置电机对应的pid输出值
        {
            if ((wheel[group][WHEEL_RUN].motor.dji.FreshTick) && (rt_tick_get() - wheel[group][WHEEL_RUN].motor.dji.FreshTick < MOTOR_OFFLINE_TIME)) //&& !getMotorIsStack(group, WHEEL_RUN))
            {
                SpeedNow[group] = UTILS_LP_FAST(SpeedNow[group], Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor), spefilterK); // 控制输入单位:快转子的rpm,即电机转速*减速比
                if (fabsf(wheel[group][WHEEL_RUN].motor.spe.err) > RUN_PID_I_THREADHOLD)                                           // 判断误差是否小于阈值
                    wheel[group][WHEEL_RUN].motor.spe.err *= I_CTRL_K;
                RUN_wheel_forward_set();
                Motor_SpeedPIDCalculate(&wheel[group][WHEEL_RUN].motor, SpeedNow[group]); // pid计算结果,单位:电流
                // wheel[group][WHEEL_RUN].motor.spe.out = k_feed * force_out[group][WHEEL_RUN] + wheel[group][WHEEL_RUN].motor.spe.feedforward + wheel[group][WHEEL_RUN].motor.spe.out;
                wheel[group][WHEEL_RUN].motor.spe.out = k_feed * force_out[group][WHEEL_RUN] + wheel[group][WHEEL_RUN].motor.spe.out;
            }
            else
            {
                wheel[group][WHEEL_RUN].motor.spe.out = 0;
                M_SETBIT(Wheels_Ctrlinfo[WHEEL_RUN].Wheels_Motor_Offline, group);
            }
        }
        return 1;
    }
    else    //如果电机为非闭环控制，且电机未使能，则返回0
    {   
        return 0;
    }

}

static float AngleNow[4];   //舵向电机当前角度
static float AngFilterK = AGN_PID_FILTER_K;//舵向电机角度滤波系数
/**
 * @brief 舵向电机PID计算
 * 
 * @return rt_uint8_t 返回1表示舵向电机PID计算正常，返回0表示舵向电机PID计算异常，异常时在后续会将电机输出置为0
 */
static rt_uint8_t Angle_wheel_PID(void)
{
    static uint8_t runtimes=0;
    float speedForward = 0;
    float SpeedNow = 0;
    if ((Wheels_Ctrlinfo[WHEEL_ANGLE].Crtl_mode == LOOP_CTRL) && (Wheels_Ctrlinfo[WHEEL_ANGLE].Wheel_Enabled))//电机控制信息为闭环控制，且电机使能
    {
            // 输入限制使得控制电机的数据rpm不超过额定转速 当超过限制同比例更改设定值
            Motor_Speed_Limit(WHEEL_ANGLE, RATED_SPEED_ANGLE);
            for (int group = 0; group < WHEELS_GROUP_NUM; group++) // 得到某位置电机对应的pid输出值
            {
                /*转换零点，数据范围(-180,180]*/
                //FbangleFix[group] = utils_Circle_Shortest_Dis(zero[group], wheel[0][group].dji.angle, 0, 8192) * 360.f / 8192.f;
                //电机刷新且电机在线，且电机未堵转，进行pid计算
                if ((wheel[group][WHEEL_ANGLE].motor.dji.FreshTick)&& (wheel[group][WHEEL_ANGLE].wheel_kind==STEER_WHEEL) && (rt_tick_get() - wheel[group][WHEEL_ANGLE].motor.dji.FreshTick < MOTOR_OFFLINE_TIME) )//&& !getMotorIsStack(group, WHEEL_ANGLE))
                {
                    if (runtimes % PID_CTRL_EX_PERIOD == 0) // 角度环
                    {
                        AngleNow[group] = utils_circle_number_LowPass_Filter(AngleNow[group], Motor_Read_NowAngle(&wheel[group][WHEEL_ANGLE].motor), AngFilterK, 0, 8192);

                        if (fabsf(wheel[group][WHEEL_ANGLE].motor.ang.err) > ANGLE_PID_I_THREADHOLD) // 判断误差是否小于阈值
                            wheel[group][WHEEL_ANGLE].motor.ang.i_value *= I_CTRL_K;                 // 逐渐减小积分项

                        // 外环角度环
                        Motor_AnglePIDCalculate(&wheel[group][WHEEL_ANGLE].motor, AngleNow[group]);
                        wheel[group][WHEEL_ANGLE].motor.spe.set = wheel[group][WHEEL_ANGLE].motor.ang.out + speedForward; // 得到内环角速度环设定值
                    }
                    /* 运行PID 角速度环*/
                    if (runtimes % PID_CTRL_IN_PERIOD == 0)
                    {
                        SpeedNow = Motor_Read_NowSpeed(&wheel[group][WHEEL_ANGLE].motor);          // 控制输入单位:快转子的rpm,即电机转速*减速比
                        if (fabsf(wheel[group][WHEEL_ANGLE].motor.spe.err) > RUN_PID_I_THREADHOLD) // 判断误差是否小于阈值
                        wheel[group][WHEEL_ANGLE].motor.spe.i_value *= I_CTRL_K;               // 逐渐减小积分项

                        Motor_SpeedPIDCalculate(&wheel[group][WHEEL_ANGLE].motor, SpeedNow); // pid计算结果,单位:电流
                    }
                    
                    //TODO 设计电机前馈电流补偿（初步想法使用姿态数据）
                    // 每次均进行电机电流补偿
                    if (wheel[group][WHEEL_ANGLE].motor.spe.out > wheel[group][WHEEL_ANGLE].motor.spe.out_limit_up) // 输出限幅
                    wheel[group][WHEEL_ANGLE].motor.spe.out = wheel[group][WHEEL_ANGLE].motor.spe.out_limit_up;
                    else if (wheel[group][WHEEL_ANGLE].motor.spe.out < wheel[group][WHEEL_ANGLE].motor.spe.out_limit_down)
                    wheel[group][WHEEL_ANGLE].motor.spe.out = wheel[group][WHEEL_ANGLE].motor.spe.out_limit_down;

                    M_CLRBIT(Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline,group);
                }
                else
                {
                    wheel[group][WHEEL_ANGLE].motor.spe.out = 0;
                    M_SETBIT(Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline, group);
                }
                
            }
            runtimes++; // 计数值加一
            return 1;
    }
    else    //如果电机为非闭环控制，且电机未使能，则返回0
    {
        return 0;
    }
}
/**
 * @brief 舵向电机进入停车死区，关闭输出
 * 
 */
static void Angle_wheel_deadzone(void)
{
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if(wheel[group][WHEEL_RUN].wheel_kind==STEER_WHEEL)
        {
            //使用轮向电机的设定值来决定是否关断舵向电机输出
            if (fabsf(wheel[group][WHEEL_RUN].motor.spe.set) <= 1.f)
            {
                wheel[group][WHEEL_ANGLE].motor.ang.out = 0;
                wheel[group][WHEEL_ANGLE].motor.spe.out = 0;
            }
        }
    }
}

//电机控制部分
// 车轮pid计算线程
//  该线程总的来说是根据外部(Chassis_Ctrl_Thread)计算出来的底盘设定值来进行pid计算并发送数据给电机
static void PID_Cal_Thread(void)
{
    
    CREAT_ID(id);
    ADDTOMONITOR_ID("PID_Cal_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    while (1)
    {
        rt_sem_take(&Cal_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        //如果轮系不全为舵轮，也没有影响
        if(Wheels_Ctrlinfo[WHEEL_ANGLE].Wheel_Enabled==1)
        {
            Angle_wheel_PID();
        }
        
        Run_wheel_PID();
        
        #ifdef USE_CAPACITY_2
        Power_Ctrl(&Scpr_manage, &wheel[0]);
        #endif
        #ifdef USE_CAPACITY_3
        Power_Ctrl(wheel);
        #endif  
        if(Wheels_Ctrlinfo[WHEEL_ANGLE].Wheel_Enabled==1)
        {
            Angle_wheel_deadzone();
            Angle_wheel_CanTrans(&txmsg_angle);
        }
        Run_wheel_CanTrans(&txmsg_run);
    }

}
/*------------------------------------PID计算部分  END----------------------------------------------------------*/

/*------------------------------------电机初始化----------------------------------------------------------*/
/**
 * @brief 轮组电机参数初始化
 *        轮组电机结构体初始化赋值 和 注册CAN接收回调函数
 * @param isWaitCom 
 * @return rt_err_t 
 */
static rt_err_t chassis_motor_Init(uint8_t isWaitCom)
{
    WheelCtrlInfoDeInit();
    Wheel_set_kind();   //设置轮子类型

    //按照轮组位置来初始化四个轮组电机的参数
    for(rt_uint8_t group=0;group<WHEELS_GROUP_NUM;group++)
    {
        Wheel_Group_Motor_Init(group);
    }
    // 注册回调函数,回调函数还是按照舵向和轮向分开来接收的
    attachRXfun(rx_chasMotor, Refresh_Chassis_Motor);
    if (isWaitCom)
        MotorWaitCom(); /*等待底盘所有电机第一次通信完毕*/
    return RT_EOK;
}
/**
 * @brief   底盘电机初始化
 */
rt_err_t Wheels_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    // 初始化电机结构体以及PID
    res = chassis_motor_Init(0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    if (res != RT_EOK)
        return RT_ERROR;
//功率控制部分
#ifdef USE_CAPACITY_2
    powerCtrlInit(&Scpr_manage);
#endif

#ifdef USE_CAPACITY_3
    powerCtrlInit();
#endif

    // 初始化信号量
    res = rt_sem_init(&Cal_sem, "PID_Cal_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    //TODO 底盘电机线程优先级修改
    // 初始化底盘线程
    thread = rt_thread_create("PID_Cal_Thread",        // 线程名
                              PID_Cal_Thread,          // 线程入口
                              RT_NULL,                 // 入口参数无
                              4096, // 线程栈
                              2,  // 线程优先级
                              3); // 线程时间片大小

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
                  1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    // 启动定时器
    res = rt_timer_start(&Cal_Timer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}
/**
 * @brief 轮组电机参数初始化
 * @param {uint8_t} isWaitCom 是否等待第一次通信
 */
static void Wheel_Group_Motor_Init(Wheel_Group_e group)
{
    if(group>=WHEELS_GROUP_NUM)
        return;
    /*初始化轮向电机结构体 */
    switch (group)
    {
        case WHEEL_RF:
        {
            /*初始化电机结构体 */
            motor_init(&wheel[group][WHEEL_RUN].motor, RUN_RF_MOTOR_ID, REDUCT_RATIO_RUN, ANGLE_CTRL_ABS, 8192, 8192, 0, RF_RUN_REVERSAL);
            /*pid初始化*/
            pid_init(&wheel[group][WHEEL_RUN].motor.spe, SPE_PID_PARAMETER);
            break;
        }
        case WHEEL_LF:
        {
            /*初始化电机结构体 */
            motor_init(&wheel[group][WHEEL_RUN].motor, RUN_LF_MOTOR_ID, REDUCT_RATIO_RUN, ANGLE_CTRL_ABS, 8192, 8192, 0, LF_RUN_REVERSAL);
            /*pid初始化*/
            pid_init(&wheel[group][WHEEL_RUN].motor.spe, SPE_PID_PARAMETER);
            break;
        }
        case WHEEL_LB:
        {
            /*初始化电机结构体 */
            motor_init(&wheel[group][WHEEL_RUN].motor, RUN_LB_MOTOR_ID, REDUCT_RATIO_RUN, ANGLE_CTRL_ABS, 8192, 8192, 0, LB_RUN_REVERSAL);
            /*pid初始化*/
            pid_init(&wheel[group][WHEEL_RUN].motor.spe, SPE_PID_PARAMETER);
            break;
        }
        case WHEEL_RB:
        {
            /*初始化电机结构体 */
            motor_init(&wheel[group][WHEEL_RUN].motor, RUN_RB_MOTOR_ID, REDUCT_RATIO_RUN, ANGLE_CTRL_ABS, 8192, 8192, 0, RB_RUN_REVERSAL);
            /*pid初始化*/
            pid_init(&wheel[group][WHEEL_RUN].motor.spe, SPE_PID_PARAMETER);
            break;
        }
        default:
            break;
    }
    //如果是舵轮，则需要额外初始化舵向电机
    if(wheel[group][WHEEL_RUN].wheel_kind==STEER_WHEEL)
    {
        switch (group)
        {
            case WHEEL_RF:
            {
                /*初始化电机结构体 */
                motor_init(&wheel[group][WHEEL_ANGLE].motor, ANGLE_RF_MOTOR_ID, REDUCT_RATIO_ANGLE, ANGLE_CTRL_ABS, 8192, 8192, 0, RF_ANG_REVERSAL);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.ang, ANGLE_ANG_PID_PARAMETER);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.spe, ANGLE_SPE_PID_PARAMETER);
                break;
            }
            case WHEEL_LF:
            {
                /*初始化电机结构体 */
                motor_init(&wheel[group][WHEEL_ANGLE].motor, ANGLE_LF_MOTOR_ID, REDUCT_RATIO_ANGLE, ANGLE_CTRL_ABS, 8192, 8192, 0, LF_ANG_REVERSAL);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.ang, ANGLE_ANG_PID_PARAMETER);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.spe, ANGLE_SPE_PID_PARAMETER);
                break;
            }
            case WHEEL_LB:
            {
                /*初始化电机结构体 */
                motor_init(&wheel[group][WHEEL_ANGLE].motor, ANGLE_LB_MOTOR_ID, REDUCT_RATIO_ANGLE, ANGLE_CTRL_ABS, 8192, 8192, 0, LB_ANG_REVERSAL);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.ang, ANGLE_ANG_PID_PARAMETER);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.spe, ANGLE_SPE_PID_PARAMETER);
                break;
            }
            case WHEEL_RB:
            {
                /*初始化电机结构体 */
                motor_init(&wheel[group][WHEEL_ANGLE].motor, ANGLE_RB_MOTOR_ID, REDUCT_RATIO_ANGLE, ANGLE_CTRL_ABS, 8192, 8192, 0, RB_ANG_REVERSAL);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.ang, ANGLE_ANG_PID_PARAMETER);
                pid_init(&wheel[group][WHEEL_ANGLE].motor.spe, ANGLE_SPE_PID_PARAMETER);
                break;
            }
            default:
                break;
        }
    }
}
/**
 * @brief 检查电机是否全部在线（第一次通信完成）
 * 
 * @param tarMotorDev 
 */
static void MotorWaitCom(void)
{
    static rt_uint8_t motor_num=0;  //计算底盘所有电机个数
    rt_uint8_t CheckSum = 0, CheckTimes = 0;
    for (int group = 0; group < WHEELS_GROUP_NUM; ++group)
    {
        if(wheel[group][WHEEL_RUN].wheel_kind==MECANUM_WHEEL||wheel[group][WHEEL_RUN].wheel_kind==OMNI_WHEEL)
        {
            motor_num += 1;
        }
        else if(wheel[group][WHEEL_RUN].wheel_kind==STEER_WHEEL)
        {
            motor_num += 2;
        }
    }
    while (RT_TRUE)
    {
        CheckSum = 0;
        for (int group = 0; group < (int)WHEELS_GROUP_NUM; ++group)
        {
            if(wheel[group][WHEEL_RUN].wheel_kind==MECANUM_WHEEL||wheel[group][WHEEL_ANGLE].wheel_kind==OMNI_WHEEL)
            {
                CheckSum +=wheel[group][WHEEL_RUN].motor.dji.Data_Valid;
            }
            else if(wheel[group][WHEEL_RUN].wheel_kind==STEER_WHEEL)
            {
                CheckSum +=wheel[group][WHEEL_RUN].motor.dji.Data_Valid;
                CheckSum +=wheel[group][WHEEL_ANGLE].motor.dji.Data_Valid;
            }
        }
        if (CheckSum == motor_num)
            break;
        rt_thread_mdelay(50);
    }
}
/**
 * @brief 初始化电机控制的基本参数，设置电机为闭环控制，电机使能，非离线状态
 * 
 * @param Wheels_info 
 */
static void WheelCtrlInfoDeInit(void)
{
    //设置轮向电机控制相关数据
    Wheels_Ctrlinfo[WHEEL_RUN].Crtl_mode = LOOP_CTRL;
    Wheels_Ctrlinfo[WHEEL_RUN].Wheel_Enabled = 1;
    Wheels_Ctrlinfo[WHEEL_RUN].Wheels_Motor_Offline = 0;
    
    //设置舵向电机控制相关数据
    rt_uint8_t if_have_steer = 0;
    for(int group=0;group<WHEELS_GROUP_NUM;group++)
    {
        if(wheel[group][WHEEL_RUN].wheel_kind==STEER_WHEEL)
        {
            if_have_steer = 1;  //如果有舵向电机，则设置舵向电机的控制数据
            break;
        }
    }
    if(if_have_steer)
    {
        Wheels_Ctrlinfo[WHEEL_ANGLE].Crtl_mode = LOOP_CTRL;
        Wheels_Ctrlinfo[WHEEL_ANGLE].Wheel_Enabled = 1;
        Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline = 0;
    }
    else
    {
        Wheels_Ctrlinfo[WHEEL_ANGLE].Crtl_mode = LOOP_CTRL;
        Wheels_Ctrlinfo[WHEEL_ANGLE].Wheel_Enabled = 0; //舵向电机不使能,不进行舵向pid计算
        Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline = 0;
    }
}
/**
 * @brief 设定底盘轮组类型，传入参数为一级指针，wheel[WHEELS_GROUP_NUM]
 *        同一个轮组上挂载电机需要设定为同一种类型的轮组
 * @param wheel 
 * @param wheel_kind 
 */
static void motor_set_kind(Wheel_t* wheel, Wheel_Kind_e wheel_kind)
{
    wheel[WHEEL_RUN].wheel_kind = wheel_kind;
    wheel[WHEEL_ANGLE].wheel_kind = wheel_kind;
}
/**
 * @brief 设定轮组类型
 * 
 */
static void Wheel_set_kind(void)
{
    motor_set_kind(wheel[WHEEL_LF], LF_WHEEL_KIND);
    motor_set_kind(wheel[WHEEL_LB], LB_WHEEL_KIND);
    motor_set_kind(wheel[WHEEL_RF], RF_WHEEL_KIND);
    motor_set_kind(wheel[WHEEL_RB], RB_WHEEL_KIND);
}
/*------------------------------------电机初始化  END----------------------------------------------------------*/


/*------------------------------------电机通信部分----------------------------------------------------------*/
/**
 * @brief CAN接受函数-刷新保存接受到电机的数据
 * @param {rt_can_msg} *msg  电机的数据
 * @param {Wheel_local_e} local 车轮位置 如果有顺序则一般默认为RF-LF-LB-RB
 */
// static void Refresh_AngWheels_Motor(Data_Type *Msg)
// {
//     uint8_t group = 0;
//     switch (Msg->id)
//     {
//     case ANGLE_RF_MOTOR_ID:
//         group = WHEEL_RF;
//         break;
//     case ANGLE_LF_MOTOR_ID:
//         group = WHEEL_LF;
//         break;
//     case ANGLE_LB_MOTOR_ID:
//         group = WHEEL_LB;
//         break;
//     case ANGLE_RB_MOTOR_ID:
//         group = WHEEL_RB;
//         break;
//     default:
//         return;
//     }
//     motor_readmsg(Msg->data, &wheel[group][WHEEL_ANGLE].motor.dji);
// }

/**
 * @brief CAN接受函数-刷新保存接受到电机的数据
 * @param {rt_can_msg} *msg  电机的数据
 * @param {Wheel_local_e} local 车轮位置 如果有顺序则一般默认为RF-LF-LB-RB
 */
// static void Refresh_RunWheels_Motor(Data_Type*Msg)
// {
//     uint8_t group = 0;
//     switch (Msg->id)
//     {
//     case RUN_RF_MOTOR_ID:
//         group = WHEEL_RF;
//         break;
//     case RUN_LF_MOTOR_ID:
//         group = WHEEL_LF;
//         break;
//     case RUN_LB_MOTOR_ID:
//         group = WHEEL_LB;
//         break;
//     case RUN_RB_MOTOR_ID:
//         group = WHEEL_RB;
//         break;
//     default:
//         return;
//     }
//     motor_readmsg(Msg->data, &wheel[group][WHEEL_RUN].motor.dji);
// }
/**
 * @brief 
 * 
 * @param Msg 
 */
static void Refresh_Chassis_Motor(Data_Type*Msg)
{
    uint8_t group = WHEELS_GROUP_NUM;
    Wheel_Local_e wheel_type = WHEEL_NUM;
    switch (Msg->id)
    {
    case RUN_RF_MOTOR_ID:
        group = WHEEL_RF;
        wheel_type=WHEEL_RUN;
        break;
    case RUN_LF_MOTOR_ID:
        group = WHEEL_LF;
        wheel_type=WHEEL_RUN;
        break;
    case RUN_LB_MOTOR_ID:
        group = WHEEL_LB;
        wheel_type=WHEEL_RUN;
        break;
    case RUN_RB_MOTOR_ID:
        group = WHEEL_RB;
        wheel_type=WHEEL_RUN;
        break;
    default:
        break;
    }
    switch (Msg->id)
    {
    case ANGLE_RF_MOTOR_ID:
        wheel_type=WHEEL_ANGLE;
        group = WHEEL_RF;
        break;
    case ANGLE_LF_MOTOR_ID:
        wheel_type=WHEEL_ANGLE;
        group = WHEEL_LF;
        break;
        wheel_type=WHEEL_ANGLE;
    case ANGLE_LB_MOTOR_ID:
        wheel_type=WHEEL_ANGLE;
        group = WHEEL_LB;
        break;
    case ANGLE_RB_MOTOR_ID:
        wheel_type=WHEEL_ANGLE;
        group = WHEEL_RB;
        break;
    default:
        break;
    }
    if(group<WHEELS_GROUP_NUM&&wheel_type<WHEEL_NUM)
    {
        motor_readmsg(Msg->data, &wheel[group][wheel_type].motor.dji);
        return;
    }
    else 
    {
        return;
    }
}
/**
 * @brief 舵向电机控制数据发送
 *          !注意电机所接CAN线，
 * @param txmsg 发送数据结构体
 */
static void Angle_wheel_CanTrans(Data_Type * txmsg)
{
    //此处为了实现底盘轮组电机CAN线可以挂载到任意CAN上（但是如果存在舵轮轮组必须保证该轮组舵向电机和轮向电机同时在一个can线上）
    #if RF_CAN_DEV==USED_CAN1 && LF_CAN_DEV==USED_CAN1 && LB_CAN_DEV==USED_CAN1 && RB_CAN_DEV==USED_CAN1
    Channal_e can_device=USED_CAN1;
    #elif RF_CAN_DEV==USED_CAN2 && LF_CAN_DEV==USED_CAN2 && LB_CAN_DEV==USED_CAN2 && RB_CAN_DEV==USED_CAN2 
    Channal_e can_device=USED_CAN2;
    #else
    Channal_e can_device=USED_CAN_1_2;
    #endif
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        if(wheel[group][WHEEL_ANGLE].wheel_kind==STEER_WHEEL)
        {
            //电机输出电流限幅
            wheel[group][WHEEL_ANGLE].motor.spe.out = utils_clamp(wheel[group][WHEEL_ANGLE].motor.spe.out, wheel[group][WHEEL_ANGLE].motor.spe.out_limit_down, wheel[group][WHEEL_ANGLE].motor.spe.out_limit_up);
            // 待发送电流报文数据 这里要求电机id连续
            txmsg->data[(wheel[group][WHEEL_ANGLE].motor.dji.motorID - RBID_6020) * 2] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[group][WHEEL_ANGLE].motor)) >> 8);
            txmsg->data[(wheel[group][WHEEL_ANGLE].motor.dji.motorID - RBID_6020) * 2 + 1] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[group][WHEEL_ANGLE].motor)));
        }
        else 
        {
            // 待发送电流报文数据 这里要求电机id连续
            txmsg->data[(wheel[group][WHEEL_ANGLE].motor.dji.motorID - RBID_6020) * 2] = 0;
            txmsg->data[(wheel[group][WHEEL_ANGLE].motor.dji.motorID - RBID_6020) * 2 + 1] = 0;
        }
    }
    TX_Trans(can_device,txmsg);
}
/**
 * @brief 轮向电机控制数据发送
 * 
 * @param txmsg 发送数据结构体
 */
static void Run_wheel_CanTrans(Data_Type* txmsg)
{
    //此处为了实现底盘轮组电机CAN线可以挂载到任意CAN上（但是如果存在舵轮轮组必须保证该轮组舵向电机和轮向电机同时在一个can线上）
    #if RF_CAN_DEV==USED_CAN1 && LF_CAN_DEV==USED_CAN1 && LB_CAN_DEV==USED_CAN1 && RB_CAN_DEV==USED_CAN1
    Channal_e can_device=USED_CAN1;
    #elif RF_CAN_DEV==USED_CAN2 && LF_CAN_DEV==USED_CAN2 && LB_CAN_DEV==USED_CAN2 && RB_CAN_DEV==USED_CAN2 
    Channal_e can_device=USED_CAN2;
    #else
    Channal_e can_device=USED_CAN_1_2;
    #endif
    for (int group = 0; group < WHEELS_GROUP_NUM; group++)
    {
        //电机输出电流限幅
        wheel[group][WHEEL_RUN].motor.spe.out = utils_clamp(wheel[group][WHEEL_RUN].motor.spe.out, wheel[group][WHEEL_RUN].motor.spe.out_limit_down, wheel[group][WHEEL_RUN].motor.spe.out_limit_up);
        // 待发送电流报文数据 这里要求电机id连续
        txmsg->data[(wheel[group][WHEEL_RUN].motor.dji.motorID - RBID_3508) * 2] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[group][WHEEL_RUN].motor)) >> 8);
        txmsg->data[(wheel[group][WHEEL_RUN].motor.dji.motorID - RBID_3508) * 2 + 1] = (rt_uint8_t)(((rt_int16_t)Motor_Read_OutSpeed(&wheel[group][WHEEL_RUN].motor)));
    }
    TX_Trans(can_device, txmsg);
}
/*------------------------------------电机通信部分  END----------------------------------------------------------*/

/*------------------------------------电机状态修改和设置----------------------------------------------------------*/
static float range_angle_180(float angle)
{
    while (angle <= -180.f)
        angle += 360.f;
    while (angle > 180.f)
        angle -= 360.f;
    (void)range_angle_180;
    return angle;
}
static float range_angle_360(float angle)
{
    while (angle < 0)
        angle += 360;
    while (angle > 360)
        angle -= 360;
    (void)range_angle_360;
    return angle;
}
static float range_angle_8192(float angle)
{
    while (angle < 0)
        angle += 8192;
    while (angle > 8192)
        angle -= 8192;
    (void)range_angle_8192;
    return angle;
}
/**
 * @brief Get the Target Fix Angle object
 *          得到角度轮的零点转化过后的值 数据范围(-180,180]，表示舵向电机目标设定值相对于车体的角度
 * @param lo 
 * @return float 
 */
float getTargetFixAngle(Wheel_Group_e group)
{
    float set = utils_Circle_Shortest_Dis(zero[group], wheel[WHEEL_ANGLE][group].motor.ang.set, 0, 8192) * 360.f / 8192.f;
    float angeltm = set;
    while (angeltm > 180)
        angeltm -= 360;
    while (angeltm < -180)
        angeltm += 360;
    return angeltm;
}
/**
 * @brief Get the Now Fix Angle object
 *          得到角度轮的零点转化过后的值 数据范围(-180,180],表示舵向电机当前状态相对于车体的角度
 * @param lo 
 * @return float 范围[-180,180]
 */
float getNowFixAngle(Wheel_Group_e group)
{
    float angeltm = 0;
    angeltm=utils_Circle_Shortest_Dis(zero[group], wheel[group][WHEEL_ANGLE].motor.dji.angle, 0, 8192) * 360.f / 8192.f;
    while (angeltm > 180)
        angeltm -= 360;
    while (angeltm < -180)
        angeltm += 360;
    return angeltm;
}
/**
 * @brief   限制各个电机的输入设定速度 若超出范围则同比例缩小速度设定值，如果对应轮组没有舵向电机其默认值为0，也不会造成影响
 * @param   max_limit 最大速度，单位rpm
 */
static void Motor_Speed_Limit(Wheel_Local_e wheel_type, float max_limit)
{
    float set_max, rate;
    /*限制输入为正数*/
    max_limit = fabsf(max_limit);
    /*找到4个值的绝对值的最大值*/
    set_max = utils_max_of_4(fabsf(Motor_Read_Set_Speed((Wheel_Group_e)WHEEL_RF, wheel_type)),
                             fabsf(Motor_Read_Set_Speed((Wheel_Group_e)WHEEL_LF, wheel_type)),
                             fabsf(Motor_Read_Set_Speed((Wheel_Group_e)WHEEL_LB, wheel_type)),
                             fabsf(Motor_Read_Set_Speed((Wheel_Group_e)WHEEL_RB, wheel_type)));
    if (set_max > max_limit)
    {
        rate = max_limit / set_max; /*同比例缩小速度设定值*/
        for (int group = 0; group < WHEELS_GROUP_NUM; group++)
        {
            Motor_Write_SetSpeed_ABS(&wheel[group][wheel_type].motor,
                                     rate * Motor_Read_Set_Speed((Wheel_Group_e)group,wheel_type));
        }
    }
}

/**
 * @brief 设定轮子的角度 输入时认为0度为正前方，逆时针方向增大，单位为0.1度
 * @param {St_Wheel_e} wheel_type 驱动轮还是角度轮
 * @param {Wheel_local_e} local 安装位置
 * @param {float} angle 控制角度 单位为0.1度 范围为-1800-1800
 */
void Wheel_Angle_Set(Wheel_Group_e group, float angle)
{
    if(group >= WHEELS_GROUP_NUM)
    {
        return;
    }
    /*转换零点，数据范围(0,8192]*/
    volatile float angleTemp = 0;
    angleTemp = range_angle_360(angle / 10.f) * 8192.f / 360.f;
    angleTemp = range_angle_8192(zero[group] + angleTemp);
    Motor_Write_SetAngle_ABS(&wheel[group][WHEEL_ANGLE].motor, angleTemp);
}
/**
 * @brief 设定轮子的线速度大小单位mm/s 内部进行pid控制方向和单位转化 pid控制单位为rpm,和电机反馈速度单位一致
 * 
 * @param group 
 * @param speed 
 */
void Wheel_Speed_Set(Wheel_Group_e group, float speed)
{
    /*由轮子的线速度单位为mm/s,转化为输出轴的角速度，再转化为控制转子的角速度单位为rmp*/
    speed = speed * VCONVERT2W_k * RAD2RPM_K * REDUCT_RATIO_RUN;
    switch (group)
    {
    case WHEEL_RF:
    case WHEEL_RB:
    case WHEEL_LF:
    case WHEEL_LB:
        Motor_Write_SetSpeed_ABS(&wheel[group][WHEEL_RUN].motor, speed);
        break;
    default:
        break;
    }
}

/**
 * @brief 读取轮向电机当前速度(输出车轮转速 mm/s)
 * 
 * @param group 
 * @return float 
 */
float Wheel_Run_Read_NowSpeed(Wheel_Group_e group)
{
    /*由快转子的角速度单位为rmp,转化为实际轮子的线速度单位为mm/s */
    float speed = Motor_Read_NowSpeed(&wheel[group][WHEEL_RUN].motor) / REDUCT_RATIO_RUN * RPM2RAD_K * WCONVERT2V_k;
    switch (group)
    {
    case WHEEL_RF:
    case WHEEL_RB:
    case WHEEL_LF:
    case WHEEL_LB:
        return speed;
    default:
        return 0.f;
    }
}
/**
 * @brief 读取舵向电机的当前角度
 * 
 * @param group 
 * @return float 范围[-180,180]
 */
float Wheel_Angele_Read_NowAngle(Wheel_Group_e group)
{
    /*由快转子的角速度单位为rmp,转化为实际轮子的线速度单位为mm/s */
    float angle = getNowFixAngle(group) ;
    switch (group)
    {
    case WHEEL_RF:
    case WHEEL_RB:
    case WHEEL_LF:
    case WHEEL_LB:
        return angle;
    default:
        return 0.f;
    }
}

/**
 * @brief 读取底盘电机速度PID的速度设定值 
 * 
 * @param group 
 * @param wheel_type 
 * @return float 单位为rpm
 */
float Motor_Read_Set_Speed(Wheel_Group_e group,Wheel_Local_e wheel_type)
{
    return Motor_Read_SetSpeed(&wheel[group][wheel_type].motor);
}
/**
 * @brief 读取舵向电机角度PID的速度设定值 
 * 
 * @param group 
 * @param wheel_type 
 * @return float 对应为编码器的绝对值
 */
float Wheel_Read_Set_Angle(Wheel_Group_e group,Wheel_Local_e wheel_type)
{
    return Motor_Read_SetAngle(&wheel[group][wheel_type].motor);
}
/**
 * @brief 读取电机PID的电流输出 
 * 
 * @param group 
 * @param wheel_type 
 * @return float 
 */
float Wheel_Read_Out_Speed(Wheel_Group_e group,Wheel_Local_e wheel_type)
{
    return Motor_Read_OutSpeed(&wheel[group][wheel_type].motor);
}

/**
 * @brief 读取电机的速度 PID 结构体
 * 
 * @param group 
 * @param wheel_type 
 * @return * pid_t* 
 */
pid_t *Wheel_Read_Speed_PID(Wheel_Group_e group,Wheel_Local_e wheel_type)
{
    return &wheel[group][wheel_type].motor.spe;
}
/**
 * @brief 读取电机的角度 PID 结构体
 * 
 * @param group 
 * @param wheel_type 
 * @return * pid_t* 
 */
pid_t *Wheel_Read_Angele_PID(Wheel_Group_e group,Wheel_Local_e wheel_type)
{
    return &wheel[group][wheel_type].motor.ang;
}
/**
 * @brief 读取底盘电机当前的离线情况
 * 从低位到高位顺序为舵向电机的RF-LF-LB-RB，轮向电机的RF-LF-LB-RB 对应位置 1 代表离线
 * @return rt_uint8_t
 */
rt_uint8_t Read_Wheels_Offline_State(void)
{
    uint8_t state[WHEEL_NUM];
    state[WHEEL_RUN] = Wheels_Ctrlinfo[WHEEL_RUN].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
    state[WHEEL_ANGLE] = Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
    return (state[WHEEL_ANGLE] << WHEELS_GROUP_NUM) & state[WHEEL_RUN];
}

/**
 * @brief 使能/失能底盘电机  Wheel_Local_e可选舵向电机还是轮向电机
 * 
 * @param wheel_type 
 * @param EN 
 */
void Wheel_Enable(Wheel_Local_e wheel_type, rt_uint8_t EN)
{
    if (wheel_type < WHEEL_NUM)
    {
        Wheels_Ctrlinfo[wheel_type].Wheel_Enabled = (rt_uint8_t)EN;
    }
}
/*------------------------------------电机状态修改和设置  END----------------------------------------------------------*/




/*------------------------------------待处理----------------------------------------------------------*/
typedef struct
{
    float current;
    float speed;
    float k_dyn;
    float confirmK;
} MotorSlip_t;

MotorSlip_t slip[4];
static void getSlipK(const Motor_t tarMotorDev[4])
{
    for (int i = 0; i < 4; i++)
    {
        slip[i].current = tarMotorDev[i].dji.current;
        slip[i].speed = tarMotorDev[i].dji.speed;
        if (tarMotorDev[i].dji.speed)
        {
            slip[i].k_dyn = slip[i].current / slip[i].speed;
            if (slip[i].k_dyn > 80)
                slip[i].confirmK = 1 / slip[i].k_dyn;
            else if (slip[i].k_dyn > 40)
                slip[i].confirmK = 10 / slip[i].k_dyn;
            else
                slip[i].confirmK = 0;
            slip[i].confirmK = utils_clamp(slip[i].confirmK, 0, 1);
        }
        if (tarMotorDev[i].dji.speed == 0)
        {
            slip[i].confirmK = 1;
        }
    }
    (void)getSlipK;
}
//该函数在轮速计中使用，由于现在没有使用轮速计，这里就留空了
float getCtrlSlipComfirK(uint8_t i)
{
    if (i < 4)
        return slip[i].confirmK;
    else
        return 0;
}
// /*初始化电机pid控制的基本参数*/
// static void MotorPID_CtrlDeInit(SteerMotorPID_Ctrl_t *pid_info)
// {
//     pid_info->pid_speedPeriod = PID_CTRL_IN_PERIOD;
//     pid_info->pid_anglePeriod = PID_CTRL_EX_PERIOD;
//     pid_info->Count_Max = PID_CTRL_IN_PERIOD + PID_CTRL_EX_PERIOD;

//     pid_info->angleFilterK = AGN_PID_FILTER_K;
//     pid_info->runFilterK = SPE_PID_FILTER_K;
//     pid_info->CurrentK_angleMotor = 70;

//     pid_info->ForwardRateK = 0.1f;
// }

// /**
//  * @brief   改变车轮的控制模式  St_Wheel_e可选驱动轮还是角度轮
//  * @param   mode  闭环还是不闭环
//  */
// void Wheels_Change_Mode(St_Wheel_e wheel_type, Crtl_e mode)
// {
//     Wheels_Ctrlinfo[wheel_type].Crtl_mode = mode;

//     /*默认使用无闭环模式后，底盘设定速度降为0*/
//     if (mode == NO_CTRL)
//     {
//         for (int lo = 0; lo < WHEELS_NUM; lo++)
//             Wheel_Speed_Set((St_Wheel_e)wheel_type, (Wheel_local_e)lo, 0);
//     }
// }

// // 修改角度轮四个电机的pid参数,暂时用在调试上
// void AngleWheels_Modify_Spid(float kp, float ki, float kd, float i_limit,
//                              float out_limit_up, float out_limit_down)
// {
//     for (int lo = 0; lo < WHEELS_NUM; lo++)
//         pid_init(&wheel[STEER_WHEEL_ANGLE][lo].spe, kp, ki, kd, i_limit, out_limit_up, out_limit_down);
// }

// // 修改驱动轮四个电机的pid参数,暂时用在调试上
// void RunWheels_Modify_Spid(float kp, float ki, float kd, float i_limit,
//                            float out_limit_up, float out_limit_down)
// {
//     for (int lo = 0; lo < WHEELS_NUM; lo++)
//         pid_init(&wheel[STEER_WHEEL_RUN][lo].spe, kp, ki, kd, i_limit, out_limit_up, out_limit_down);
// }


