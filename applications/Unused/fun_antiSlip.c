#include "fun_antiSlip.h"
#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif
#include "ThreadCtrl.h"
#include "app_ChassisCtrl.h"
#include "drv_IMU.h"

IsSlipInfo_t IsSlipInfo;
SlipCaliData_t SlipCaliData;

static struct rt_completion completion; // 完成量
static rt_timer_t timer;                // 定时器

static void Get_IMU_data(void)
{
    IsSlipInfo.Acc_speed.vx = Get_Chassis_IMU_Data(IMU_ACCX);
    IsSlipInfo.Acc_speed.vy = Get_Chassis_IMU_Data(IMU_ACCY);
    IsSlipInfo.Equation[axis_x] = SlipCaliData.motion_eqa[0];
}

/**
 * @brief :判断是否打滑
 * @return {*}返回1表示打滑
 */
uint8_t Chassis_IsSlip(Mot_base_t Movemotion)
{
    float Cal_V = 0, Real_V[2] = {0, 0};

    Get_IMU_data();
    IsSlipInfo.Set_speed = Movemotion;

    Real_V[axis_x] = IsSlipInfo.Set_speed.vel.x;
    Real_V[axis_y] = IsSlipInfo.Set_speed.vel.y;

    for (int fori; fori < 2; fori++)
    {
        Cal_V = IsSlipInfo.Equation[fori].k * IsSlipInfo.Acc_speed.vx + IsSlipInfo.Equation[fori].b;
        if ((fabsf(Cal_V - Real_V[fori])) > MAX_DIFFERENCE)
            IsSlipInfo.Slip_Counts++;
    }
    if (IsSlipInfo.Slip_Counts > SLIP_THRESHOLD)
    {
        IsSlipInfo.IsSlip = 1;
        IsSlipInfo.Slip_Counts = 0;
        return 1;
    }
    else
        return 0;
}

static void antiSlipTimerHandler(void *parameter)
{
    rt_completion_done(&completion);
}

static void antiSlip_thread(void *parameter)
{
    Mot_base_t motionTmp;
    imu_t imuData;
    while (1)
    {
        uint16_t index = 0;
        rt_completion_wait(&completion, RT_WAITING_FOREVER);

        if (!SlipCaliData.isfinish && SlipCaliData.isCali)
        {
            motionTmp = Get_Chassis_Set_Speed();
            Get_Chassis_IMU_AllData(&imuData);
            SlipCaliData.slip_setData[index] = motionTmp.angvel * 10.f; // 角速度单位0.1°/s
            SlipCaliData.slip_imuData[index] = imuData.gyro.gz;
            if (index >= LEAST_SQUARE_DATANUM)
            {
                SlipCaliData.isfinish = 1;
                index = 0;
            }
            else
                index++;
        }
        else
        {
            // 统计完成计算线性方程
            leastSquareData(&SlipCaliData.slip_setData[0], &SlipCaliData.slip_imuData[0],
                            LEAST_SQUARE_DATANUM, &SlipCaliData.motion_eqa[0]);
        }
    }
}

rt_err_t antiSlip_threadInit(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread = RT_NULL;

    /* 初始化完成量对象 */
    rt_completion_init(&completion);

    thread = rt_thread_create("antiSlip_thread",
                              antiSlip_thread,
                              RT_NULL,
                              THREAD_STACK_IMU_CALI,
                              THREAD_PRIO_IMU_CALI,
                              THREAD_TICK_IMU_CALI);

    // 线程创建失败返回false
    if (thread == RT_NULL)
        return RT_ERROR;

    // 线程启动失败返回false
    if (rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    // 创建线程定时器
    timer = rt_timer_create("antiSlip_timer",
                            antiSlipTimerHandler,
                            RT_NULL,
                            THREAD_PEROID_CALI,
                            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    // 启动定时器
    res = rt_timer_start(timer);
    if (res != RT_EOK)
        return res;
    return res;
}

/**
 * @brief 最小二乘法拟合两个数据的线性关系
 * @param {float} *datax 方程的x
 * @param {float} *datay 方程的y
 * @param {uint16_t} dataNum 拟合的数据量
 * @param {Equation_t} *Equation 得到的方程系数 y=kx+b
 */
void leastSquareData(float *datax, float *datay, uint16_t dataNum, Equation_t *Equation)
{
    float Equation_x[LEAST_SQUARE_DATANUM];
    float Equation_y[LEAST_SQUARE_DATANUM];

    int i = 0;
    float equation_b = 0, equation_k = 0, sum_x = 0, sum_y = 0, Denominator = 0, numerator = 0;
    uint16_t realNum = dataNum > LEAST_SQUARE_DATANUM ? LEAST_SQUARE_DATANUM : dataNum;

    rt_memcpy(Equation_x, datax, realNum); // 方程的x
    rt_memcpy(Equation_x, datay, realNum); // 方程的y

    // 计算数和
    for (i = 0; i < realNum; i++)
    {
        sum_x += Equation_x[i];
        sum_y += Equation_y[i];
    }

    // 计算平均值
    float x_ave = sum_x / realNum; // 方程的x的平均值
    float y_ave = sum_y / realNum; // 方程的y的平均值

    for (i = 0; i < realNum; i++) // 最小二乘法的公式
    {
        numerator += (Equation_x[i] - x_ave) * (Equation_x[i] - x_ave);   // 计算分母
        Denominator += (Equation_x[i] - x_ave) * (Equation_y[i] - y_ave); // 计算分子
    }

    if (numerator != 0)
    {
        // 得到方程系数
        equation_k = Denominator / numerator;
        equation_b = y_ave - equation_k * x_ave;

        if (Equation)
        {
            Equation->k = equation_k;
            Equation->b = equation_b;
        }
    }
}
// 对加速度计的值分区间处理
void utils_process(float min, float max, float step, float *ProcessData)
{
    float temp_left = min, temp_right = min + step;
    float average = (temp_left + temp_right) / 2;

    // 区间个数为整数
    uint8_t step_counts = (uint8_t)((max - min) / step);

    // 保证数值合理
    if ((max - min) < step)
        return;
    if ((max < min) || (step == 0))
        return;

    if (ProcessData != NULL)
    {
        // 上下基本限制
        if (*ProcessData > max)
            *ProcessData = max;
        if (*ProcessData < min)
            *ProcessData = min;

        // 找到区间的中值
        for (int i = 0; i < step_counts; i++)
        {
            if ((*ProcessData > temp_left) && (*ProcessData < temp_right))
            {
                *ProcessData = average;
                break;
            }
            temp_left = temp_left + step;
            temp_right = temp_right + step;
            average = (temp_left + temp_right) / 2;
        }
    }
}
