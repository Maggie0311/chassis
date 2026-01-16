#include "drv_wheel.h"

#ifdef USE_CAPACITY // 低电量保护
Scpr_manage_t Scpr_manage;
#endif

/*所有顺序为转向轮和驱动轮---RF-LF-LB-RB*/
static struct rt_timer Cal_Timer;   /*线程定时器*/
static struct rt_semaphore Cal_sem; /*线程信号量*/

MotorStuck_t stuck[WHEELS_GROUP_NUM][WHEEL_NUM]; // 电机堵转
Wheel_t wheel[WHEELS_GROUP_NUM][WHEEL_NUM]; // 轮组数据
/**
 * @brief 定时器回调,用于唤醒电机pid计算线程
 *
 * @param parameter
 */
static void Cal_IRQHandler(void *parameter) //
{
    rt_sem_release(&Cal_sem);
}

static void MotorWaitCom(void);


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

        /*pid等电机控制计算*/
        Motor_Control_Loop();

#ifdef USE_CAPACITY_2
        Power_Ctrl(&Scpr_manage, &wheel[0]);
#endif
#ifdef USE_CAPACITY_3
        Power_Ctrl(wheel);
#endif

        /*can报文发送*/
        Motor_Transmit_Loop();
    }
}

/*------------------------------------电机初始化----------------------------------------------------------*/
/**
 * @brief   底盘电机初始化
 */
rt_err_t Wheels_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    // 初始化电机结构体以及PID
    chassis_motor_init();
    if (res != RT_EOK)
        return RT_ERROR;

#ifdef USE_CAPACITY_3
    powerCtrlInit();
#endif

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
                  1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    // 启动定时器
    res = rt_timer_start(&Cal_Timer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}

/**
 * @brief 检查电机是否全部在线（第一次通信完成）
 *
 * @param tarMotorDev
 */
static void MotorWaitCom(void)
{
    static rt_uint8_t motor_num = 0; // 计算底盘所有电机个数
    rt_uint8_t CheckSum = 0, CheckTimes = 0;
    for (int group = 0; group < WHEELS_GROUP_NUM; ++group)
    {
        if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL || wheel[group][WHEEL_RUN].wheel_kind == OMNI_WHEEL)
        {
            motor_num += 1;
        }
        else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
        {
            motor_num += 2;
        }
    }
    while (RT_TRUE)
    {
        CheckSum = 0;
        for (int group = 0; group < (int)WHEELS_GROUP_NUM; ++group)
        {
            if (wheel[group][WHEEL_RUN].wheel_kind == MECANUM_WHEEL || wheel[group][WHEEL_ANGLE].wheel_kind == OMNI_WHEEL)
            {
                CheckSum += wheel[group][WHEEL_RUN].motor.dji.Data_Valid;
            }
            else if (wheel[group][WHEEL_RUN].wheel_kind == STEER_WHEEL)
            {
                CheckSum += wheel[group][WHEEL_RUN].motor.dji.Data_Valid;
                CheckSum += wheel[group][WHEEL_ANGLE].motor.dji.Data_Valid;
            }
        }
        if (CheckSum == motor_num)
            break;
        rt_thread_mdelay(50);
    }
}


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
 * @brief Get the Now Fix Angle object
 *          得到角度轮的零点转化过后的值 数据范围(-180,180],表示舵向电机当前状态相对于车体的角度
 * @param lo
 * @return float 范围[-180,180]
 */
float getNowFixAngle(Wheel_Group_e group)
{
    float angeltm = 0;
//    angeltm = utils_Circle_Shortest_Dis(zero[group], wheel[group][WHEEL_ANGLE].motor.dji.angle, 0, 8192) * 360.f / 8192.f;
    while (angeltm > 180)
        angeltm -= 360;
    while (angeltm < -180)
        angeltm += 360;
    return angeltm;
}

/**
 * @brief 设定轮子的角度 输入时认为0度为正前方，逆时针方向增大，单位为0.1度
 * @param {St_Wheel_e} wheel_type 驱动轮还是角度轮
 * @param {Wheel_local_e} local 安装位置
 * @param {float} angle 控制角度 单位为0.1度 范围为-1800-1800
 */
void Wheel_Angle_Set(Wheel_Group_e group, float angle)
{
    if (group >= WHEELS_GROUP_NUM)
    {
        return;
    }
    /*转换零点，数据范围(0,8192]*/
    volatile float angleTemp = 0;
    angleTemp = range_angle_360(angle / 10.f) * 8192.f / 360.f;
//    angleTemp = range_angle_8192(zero[group] + angleTemp);
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
 * @brief 读取电机的角度 PID 结构体
 *
 * @param group
 * @param wheel_type
 * @return * pid_t*
 */
pid_t *Wheel_Read_Angele_PID(Wheel_Group_e group, Wheel_Local_e wheel_type)
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
    //state[WHEEL_RUN] = Wheels_Ctrlinfo[WHEEL_RUN].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
    //state[WHEEL_ANGLE] = Wheels_Ctrlinfo[WHEEL_ANGLE].Wheels_Motor_Offline & ((1 << WHEELS_GROUP_NUM) - 1);
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
        //Wheels_Ctrlinfo[wheel_type].Wheel_Enabled = (rt_uint8_t)EN;
    }
}
/*------------------------------------电机状态修改和设置  END----------------------------------------------------------*/

