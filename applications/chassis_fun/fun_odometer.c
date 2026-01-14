#include "fun_odometer.h"
#include "fun_kalman.h"
#include "drv_IMU.h"
#include "mod_motion.h"

#include <arm_math.h>
#include "drv_motionSolve.h"
#include "drv_wheel.h"
#include "string.h"
#include "ThreadCtrl.h"
#include "drv_Setplanning3D.h"
// 实时里程计数据
Odometer_t Odom_Frusion;
encoderData_t odom_encoder;

kalman_filter_t Odom_kalmanTest[2];
Odometer_t Odom_FrusionTest;

// 计算里程计时使用的imu的数据
Odometer_IMU_t OdomIMU_Data;
highPassFiler_t gyroFilter[1];

// 卡尔曼滤波的一些数据
static kalman2_state_t Odo_Kalman[kalmanAll];
// 用于记录历史数据
static HistoryData_t HistoryData[HISTORY_NUM];
static uint8_t historyIndex;

// 初始化卡尔曼滤波的值以及初始坐标等
static void OdomInit(void)
{
    OdomIMU_Data.imu_pos[0] = IMU_POS_X;
    OdomIMU_Data.imu_pos[1] = IMU_POS_Y;
    OdomIMU_Data.imu_pos[2] = IMU_POS_Z;

    /*角速度处理*/
    OdomIMU_Data.gz_filterK = GZ_FILTER_K;
    for (int i = 0; i < sizeof(gyroFilter) / sizeof(gyroFilter[0]); i++)
    {
        gyroFilter[i].cutOffFre = 1;
        gyroFilter[i].period = THREAD_PRIO_ODO;
    }

    for (int i = 0; i < kalmanAll; i++)
    {
        kalman_DeInit(&Odo_Kalman[i]);
    }

    /*速度vx*/
    Odo_Kalman[0].Q_x = 0.5;
    Odo_Kalman[0].Q_xdot = 0.5;
    Odo_Kalman[0].R_x = 0.5;
    Odo_Kalman[0].R_xdot = 0.2;

    /*速度vy*/
    Odo_Kalman[1].Q_x = 0.5;
    Odo_Kalman[1].Q_xdot = 0.5;
    Odo_Kalman[1].R_x = 0.5;
    Odo_Kalman[1].R_xdot = 0.2;

    Odom_Frusion.Distance.x = 0;
    Odom_Frusion.Distance.y = 0;
    Odom_Frusion.Distance.z = 0;

    for (int i = 0; i < kalmanAll; i++)
    {
        kalman_filter_init(&Odom_kalmanTest[i]);
    }
}

static void highpass_filter(highPassFiler_t *handle, float raw)
{
    if (!handle->cutOffFre)
    {
        handle->rawData = raw;
        handle->outData = raw;
        return;
    }
    float dt = handle->period / 1000.f;               // 采样周期
    float RC = 1.0f / (2.f * PI * handle->cutOffFre); // RC时间常数
    float alpha = RC / (RC + dt);                     // 滤波器系数

    handle->rawData = raw;
    handle->outData = alpha * (handle->outData + handle->rawData - handle->preData); // 滤波后的数据
    handle->preData = handle->rawData; 
		// 更新上一个数据点的值
		(void)highpass_filter;
}

#if (IS_USE_IMU == 1) && defined(USING_STEER_WHEEL)
// 得到补偿后的加速度计的值 w °/s
static void GetOdomIMUData(float accW)
{
    // 1.融合角速度
    Get_Chassis_IMU_Gyro((imu_gyro_t *)&OdomIMU_Data.gyro);
    highpass_filter(&gyroFilter[0], OdomIMU_Data.gyro.gz); /*单位°/s*/
    Odom_Frusion.DataVFusion[speed_w] = (1 - OdomIMU_Data.gz_filterK) * accW +
                                        OdomIMU_Data.gz_filterK * gyroFilter[0].outData;

    VectorXYZ_Str v, r, w, a;

    // 根据位置和角速度计算离心加速度 单位均为国际单位
    r.x = OdomIMU_Data.imu_pos[0] / 1000.f;
    r.y = OdomIMU_Data.imu_pos[1] / 1000.f;
    r.z = 0;
    w.x = 0;
    w.y = 0;
    w.z = Odom_Frusion.DataVFusion[speed_w] * DEG_TO_RAD;

    /*υ = ω x r，计算底盘中心的速度矢量*/
    Vector3D_CrossProd(&w, &r, &v);

    /*a = ω x v，计算底盘中心的速度矢量*/
    Vector3D_CrossProd(&w, &v, &a);

    // 得到离心加速度
    OdomIMU_Data.ACC_Fix[0] = a.x;
    OdomIMU_Data.ACC_Fix[1] = a.y;

    if (getIsFixFinsh())
    {
        // 得到补偿后的加速度计的值
        OdomIMU_Data.acc.ax = (float)Get_Chassis_IMU_Data(IMU_ACCX) - OdomIMU_Data.ACC_Fix[0];
        OdomIMU_Data.acc.ay = (float)Get_Chassis_IMU_Data(IMU_ACCY) - OdomIMU_Data.ACC_Fix[1];
        OdomIMU_Data.acc.az = (float)Get_Chassis_IMU_Data(IMU_ACCZ) - OdomIMU_Data.ACC_Fix[2];
    }
}
#endif

float comfirK = 0;
// 计算里程
void OdomCalculate(void)
{
    Mot_base_t motionTmp;

    // 单位均为国际单位 角度为弧度制
    float cal_vx, cal_vy, cal_angle;

    static Odo_TimeType time_interval, last_time; // 单位ms
    if (!Odom_Frusion.isFirst)
    {
        Odom_Frusion.isFirst = 1;
        last_time = odo_getTime();
        return;
    }

    // 根据上一次速度计算当前的位置 以及得到当前的角度
    time_interval = (odo_getTime() - last_time); // 单位ms

    // 编码器计算
    odom_encoder.Distance.x += odom_encoder.speed[speed_x] * time_interval; // 距离单位为mm
    odom_encoder.Distance.y += odom_encoder.speed[speed_y] * time_interval; // 距离单位为mm
    odom_encoder.t = time_interval;

#if (IS_USE_IMU == 1) && defined(USING_STEER_WHEEL)
    float tm = 0;
    for (int i = 0; i < 4; i++)
        tm += getCtrlSlipComfirK(i);
    comfirK = tm / 4.f;
    if (comfirK > 0.95f)
        Odom_Frusion.isSlip = 1;
    else
        Odom_Frusion.isSlip = 0;

    if (odom_encoder.motionReal.vel.x == 0 && odom_encoder.motionReal.vel.x == 0)
    {
        comfirK = 1;
        Odom_Frusion.isSlip = 1;
    }

    // 融合数据计算
    Odom_Frusion.Distance.x += Odom_Frusion.DataVFusion[speed_x] * time_interval * (1 - comfirK); // 距离单位为mm
    Odom_Frusion.Distance.y += Odom_Frusion.DataVFusion[speed_y] * time_interval * (1 - comfirK); // 距离单位为mm
    Odom_Frusion.t = time_interval;
#endif

    // (-180-180] 逆时针为正
    Get_Chassis_IMU_Elur((imu_euler_t *)&OdomIMU_Data.euler);
    // 得到当前编码器设定的的速度和角速度
    motionTmp = getSetMotion();
    odom_encoder.motionReal.vel.x = motionTmp.vel.x / 1000.f; // 单位m/s
    odom_encoder.motionReal.vel.y = motionTmp.vel.y / 1000.f; // 单位m/s
    odom_encoder.motionReal.angvel = motionTmp.angvel / 10.f; // 单位°/s
    odom_encoder.nowAngle = OdomIMU_Data.euler.yaw;           // 得到当前角度 单位°[-180-180]

    // 由编码器计算当前odom坐标系下的速度
    cal_vx = odom_encoder.motionReal.vel.x;
    cal_vy = odom_encoder.motionReal.vel.y;
    cal_angle = odom_encoder.nowAngle * PI / 180.f;
    odom_encoder.speed[speed_x] = -cal_vy * sinf(cal_angle) + cal_vx * cosf(cal_angle);
    odom_encoder.speed[speed_y] = cal_vy * cosf(cal_angle) + cal_vx * sinf(cal_angle);

#if (IS_USE_IMU == 1) && defined(USING_STEER_WHEEL)
    // 融合数据得到当前的速度和角速度

    GetOdomIMUData(odom_encoder.motionReal.angvel); // 根据融合的角速度得到当前的IMU的值

    // // 2. 得到融合速度
    // Odom_Frusion.DataVFusion[speed_x] = kalman2_filter(&Odo_Kalman[kalman_vx], odom_encoder.motionReal.vel.x, OdomIMU_Data.acc.ax, (float)time_interval / 1000.f);
    // Odom_Frusion.DataVFusion[speed_y] = kalman2_filter(&Odo_Kalman[kalman_vy], odom_encoder.motionReal.vel.y, OdomIMU_Data.acc.ay, (float)time_interval / 1000.f);

    // 2. 得到融合速度
    Odom_Frusion.DataVFusion[speed_x] = kalman_filter_update(&Odom_kalmanTest[kalman_vx], odom_encoder.motionReal.vel.x, OdomIMU_Data.acc.ax, (float)time_interval / 1000.f);
    Odom_Frusion.DataVFusion[speed_y] = kalman_filter_update(&Odom_kalmanTest[kalman_vy], odom_encoder.motionReal.vel.y, OdomIMU_Data.acc.ay, (float)time_interval / 1000.f);

    // // 2. 得到融合速度
    // Odom_FrusionTest.DataVFusion[speed_x] = kalman_filter_update(&Odom_kalmanTest[kalman_vx], odom_encoder.motionReal.vel.x, OdomIMU_Data.acc.ax, (float)time_interval / 1000.f);
    // Odom_FrusionTest.DataVFusion[speed_y] = kalman_filter_update(&Odom_kalmanTest[kalman_vy], odom_encoder.motionReal.vel.y, OdomIMU_Data.acc.ay, (float)time_interval / 1000.f);

    Odom_Frusion.nowAngle = OdomIMU_Data.euler.yaw; // 得到当前角度 单位°[-180-180]

    // 3.由融合数据计算当前odom坐标系下的速度
    cal_vx = Odom_Frusion.DataVFusion[speed_x];
    cal_vy = Odom_Frusion.DataVFusion[speed_y];
    cal_angle = Odom_Frusion.nowAngle * PI / 180.f;
    Odom_Frusion.DataVFusion[speed_x] = -cal_vy * sinf(cal_angle) + cal_vx * cosf(cal_angle);
    Odom_Frusion.DataVFusion[speed_y] = cal_vy * cosf(cal_angle) + cal_vx * sinf(cal_angle);
#endif

    // 存储历史数据
    memcpy(&HistoryData[historyIndex].Data, &Odom_Frusion.Distance, sizeof(Odom_Frusion.Distance));
    HistoryData[historyIndex].time = odo_getTime();
    historyIndex++;
    if (historyIndex >= HISTORY_NUM)
        historyIndex = 0;
    last_time = odo_getTime();
}

// 得到解算速度和融合速度，方便测试
float GetSpeed(OdometerType_e SpeedType, uint8_t IsDataFusion)
{
    if (IsDataFusion)
    {
        switch (SpeedType)
        {
        case speed_x:
            return Odom_Frusion.DataVFusion[speed_x];
        case speed_y:
            return Odom_Frusion.DataVFusion[speed_y];
        case speed_w:
            return Odom_Frusion.DataVFusion[speed_w];

        default:
            break;
        }
    }
    else
    {
        switch (SpeedType)
        {
        case speed_x:
            return odom_encoder.speed[speed_x];
        case speed_y:
            return odom_encoder.speed[speed_y];
        case speed_w:
            return odom_encoder.speed[speed_w];

        default:
            break;
        }
    }

    return 0;
}

/**
 * @brief 获得距离值 单位mm
 * @param {OdometerdDis_e} DisType 可以选择得到一个数据或者全部数据。如果要得到全部数据则可以随意填
 * @param {float} *data 得到的一个数据
 * @return {*} 全部数据
 */
Distance_t GetDistance_Frusion(OdometerdDis_e DisType, float *data)
{
    if (data == NULL)
    {
        return Odom_Frusion.Distance;
    }
    switch (DisType)
    {
    case dis_x:
        *data = Odom_Frusion.Distance.x;
        break;
    case dis_y:
        *data = Odom_Frusion.Distance.y;
        break;
    case dis_z:
        *data = Odom_Frusion.Distance.z;
        break;
    case dis_all:
        *data = Odom_Frusion.Distance.ALL;
        break;
    default:
        break;
    }
    return Odom_Frusion.Distance;
}

/**
 * @brief 获得仅仅编码器的距离值 单位mm
 * @param {OdometerdDis_e} DisType 可以选择得到一个数据或者全部数据。如果要得到全部数据则可以随意填
 * @param {float} *data 得到的一个数据
 * @return {*} 全部数据
 */
Distance_t GetDistance_Encoder(OdometerdDis_e DisType, float *data)
{
    if (data == NULL)
        return odom_encoder.Distance;

    switch (DisType)
    {
    case dis_x:
        *data = odom_encoder.Distance.x;
        break;
    case dis_y:
        *data = odom_encoder.Distance.y;
        break;
    case dis_z:
        *data = odom_encoder.Distance.z;
        break;
    case dis_all:
        *data = odom_encoder.Distance.ALL;
        break;
    default:
        break;
    }
    return odom_encoder.Distance;
}

/**
 * @brief 外部更新坐标点
 * @param {OdometerdDis_e} DisType 要更新的坐标类型
 * @param {float} Distance 可靠的坐标点数据
 */
void SetDistance(OdometerdDis_e DisType, float Distance)
{
    switch (DisType)
    {
    case dis_x:
        Odom_Frusion.Distance.x = Distance;
        break;
    case dis_y:
        Odom_Frusion.Distance.y = Distance;
        break;
    case dis_z:
        Odom_Frusion.Distance.z = Distance;
        break;
    case dis_all:
        Odom_Frusion.Distance.ALL = Distance;
        break;
    default:
        break;
    }
}

uint8_t getIsSlip(void)
{
    return Odom_Frusion.isSlip;
}
/*------------------------------------//线程数据暂时存放----------------------------------------------------------*/
static rt_thread_t Odo_Thread = RT_NULL; // 线程
static rt_timer_t Odo_Timer = RT_NULL;   // 定时器
static rt_sem_t Odo_Sem = RT_NULL;       // 信号量

static void Odo_Timer_Timeout_Handler(void *parameter)
{
    if (Odo_Sem)
    {
        while (rt_sem_trytake(Odo_Sem) == RT_EOK)
            continue;
        rt_sem_release(Odo_Sem);
    }
}

static void Odo_Thread_Entry(void *parameter)
{
    while (1)
    {
        // 延时
        rt_sem_take(Odo_Sem, RT_WAITING_FOREVER);
        OdomCalculate();
    }
}

rt_err_t ChassisIMU_Init(void)
{
    // IMU接受初始化
    ChassisIMURxInit();
    OdomInit();
    Odo_Sem = rt_sem_create("OdoSem", 0, RT_IPC_FLAG_FIFO);
    if (!Odo_Sem)
        return RT_ERROR;
    rt_sem_trytake(Odo_Sem);

    Odo_Timer = rt_timer_create(
        "OdoTimer", Odo_Timer_Timeout_Handler,
        RT_NULL, THREAD_PEROID_ODO, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (Odo_Timer)
    {
        if (rt_timer_start(Odo_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

    Odo_Thread = rt_thread_create(
        "Odometer", Odo_Thread_Entry, RT_NULL,
        THREAD_STACK_ODO, THREAD_PRIO_ODO, THREAD_TICK_ODO);

    // 查看是否创建成功
    if (Odo_Thread != RT_NULL)
    {
        if (rt_thread_startup(Odo_Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

    return RT_EOK;
}
/*------------------------------------XXX----------------------------------------------------------*/
