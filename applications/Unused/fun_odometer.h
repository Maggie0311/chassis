#ifndef __FUN_ODOMETER_H__
#define __FUN_ODOMETER_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_ChassisCtrl.h"

#ifndef PI
#define PI (3.1415926535f)
#endif
#ifndef DEG_TO_RAD // 度转弧度
#define DEG_TO_RAD (0.0174532f)
#endif
#ifndef RAD_TO_DEG // 度转弧度
#define RAD_TO_DEG (57.29578f)
#endif

#define IS_USE_IMU 1

/*角速度使用时相信陀螺仪的程度 为1完全相信陀螺仪*/
#define GZ_FILTER_K (1)

// imu放置相对于底盘中心的位置 xyz 单位毫米
#define IMU_POS_X (-48)
#define IMU_POS_Y (-200)
#define IMU_POS_Z (10)

// 存储数据量
#define HISTORY_NUM 10

// 为了适配不同框架
#define odo_getTime() rt_tick_get()
#define Odo_TimeType rt_tick_t

typedef enum
{
    speed_x,
    speed_y,
    speed_w,

} OdometerType_e;

typedef enum
{
    kalman_vx,
    kalman_vy,
    kalman_w,
    kalmanAll,

} OdomKalman_e;

typedef enum
{
    dis_x,
    dis_y,
    dis_z,
    dis_all,

} OdometerdDis_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Odometer_euler_t;

typedef struct
{
    float gx;
    float gy;
    float gz;
} Odometer_gyro_t;

typedef struct
{
    float ax;
    float ay;
    float az;

} Odometer_acc_t;

typedef struct
{

    Odometer_euler_t euler; // 单位°
    Odometer_gyro_t gyro;   // 单位°/s
    Odometer_acc_t acc;     // 单位m/s^2

    float ACC_Fix[3]; // imu的补偿离心加速度数据
    float imu_pos[3]; // imu放置相对于底盘中心的位置 xyz 单位mm
    float gz_filterK; // 为1则完全相信陀螺仪
} Odometer_IMU_t;

typedef struct
{
    float x;
    float y;
    float z;
    float ALL;

} Distance_t;

typedef struct
{
    Distance_t Distance;   // 由编码器计算得到的距离 单位mm
    Mot_base_t motionReal; // 编码器的速度值  速度单位m/s 角速度单位°/s
    float nowAngle;        // 由编码器计算得到的角度 单位°[-180-180]
    float speed[3];        // 由编码器计算得到的odom下的速度 单位m/s
    uint32_t t;
} encoderData_t;

typedef struct
{
    uint8_t isFirst;      // 积分开始时间标志
    float nowAngle;       // 由计算得到的角度 单位°[-180-180]
    float DataVFusion[3]; // 速度单位m/s 角速度单位为°/s
    Distance_t Distance;  // 距离单位mm
    uint8_t isSlip;
    uint32_t t;
} Odometer_t;

typedef struct
{
    Distance_t Data;
    Odo_TimeType time;

} HistoryData_t;

typedef struct
{
    float rawData;      // 当前原始值
    float outData;      // 滤波后的值
    float preData;      // 上一次原始值
    uint16_t period;    // 单位ms
    uint16_t cutOffFre; // 截至频率 单位hz
} highPassFiler_t;

rt_err_t ChassisIMU_Init(void);
// 得到解算速度和融合速度，方便测试
float GetSpeed(OdometerType_e SpeedType, uint8_t IsDataFusion);
void OdomCalculate(void);
/**
 * @brief 获得距离值
 * @param {OdometerdDis_e} DisType 可以选择得到一个数据或者全部数据。如果要得到全部数据则可以随意填
 * @param {float} *data 得到的一个数据
 * @return {*} 全部数据
 */
Distance_t GetDistance_Frusion(OdometerdDis_e DisType, float *data);
Distance_t GetDistance_Encoder(OdometerdDis_e DisType, float *data);
void SetDistance(OdometerdDis_e DisType, float Distance);

#endif
