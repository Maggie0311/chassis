#ifndef DRV_IMU_H
#define DRV_IMU_H
#include <rtthread.h>
#include <rtdevice.h>

#define IS_USE_IMU_CALI 1
#define IMU_CALI_COUNTS 100
#define IMU_CALI_FLAG 8888

/* 四元数 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
}AHRS_Quaternion_t;

typedef enum
{
    IMU_YAW,
    IMU_PITCH,
    IMU_ROLL,

    IMU_GYROX,
    IMU_GYROY,
    IMU_GYROZ,

    IMU_ACCX,
    IMU_ACCY,
    IMU_ACCZ,

    IMU_ALL,

} imu_data_type_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} imu_euler_t;

typedef struct
{
    float gx;
    float gy;
    float gz;
} imu_gyro_t;

typedef struct
{
    float ax;
    float ay;
    float az;

} imu_accel_t;
typedef struct
{
    imu_euler_t euler_angle;
    imu_gyro_t gyro;
    imu_accel_t acc;       // 加速度
    AHRS_Quaternion_t qua; // 四元数
} imu_t;

int ChassisIMURxInit(void);

//接受
void Refresh_Gimbal_IMU_Data(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Elur(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Gyro(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Acc(struct rt_can_msg *msg);
void Refresh_Chassis_IMU_Quaternion(struct rt_can_msg *msg); // 刷新底盘IMU的四元数

// 返回云台imu数据 (-180-180]
float Get_Gimbal_IMU_Data(imu_data_type_e type);

// 得到底盘IMU的欧拉角 (-180-180] 逆时针为正
imu_euler_t Get_Chassis_IMU_Elur(imu_euler_t *Getdata);

// 得到底盘IMU的四元数
AHRS_Quaternion_t Get_Chassis_IMU_Quaternion(void);

// 返回底盘imu数据
float Get_Chassis_IMU_Data(imu_data_type_e type);

// 返回底盘imu加速度数据
void Get_Chassis_IMU_Acc(imu_accel_t *Getdata);

// 返回底盘imu三轴陀螺仪数据
void Get_Chassis_IMU_Gyro(imu_gyro_t *Getdata);

// 得到底盘IMU的四元数
AHRS_Quaternion_t Get_Chassis_IMU_Quaternion(void);

// 得到底盘IMU的相关数据
imu_t Get_Chassis_IMU_AllData(imu_t *imuData);

// 返回IMU是否校准完成
uint8_t getIsFixFinsh(void);
#endif
