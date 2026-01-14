#include "drv_IMU.h"
#include "drv_flash.h"
#include "drv_can_com.h"
#include "math.h"

// 206 -48
imu_t HERO_Gimbal_IMU; // 云台向左/上旋转时对应轴的数据增大
imu_t Chassis_RawIMU;  // 底盘IMU的原始数据
imu_t Chas_IMU_fix;    // 坐标系转化和零点校准后的imu数据
float imu_zero[IMU_ALL];
// 记录零点的变量
float imu_zeroSumData[IMU_ALL];
float imu_zeroFixData[IMU_ALL];
uint32_t imu_zeroCounts[IMU_ALL];

rt_inline float Get_Chassis_IMU_ZeroFixData(imu_data_type_e type);

#define ACCL_X_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_ACCY)
#define ACCL_Y_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_ACCX)
#define ACCL_Z_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_ACCZ)

#define GYRO_X_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_GYROY)
#define GYRO_Y_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_GYROX)
#define GYRO_Z_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_GYROZ)

// 绕x轴旋转pitch，绕y轴旋转roll,绕z轴旋转yaw。
#define ELUR_PITCH_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_ROLL)
#define ELUR_ROLL_SOURCE -Get_Chassis_IMU_ZeroFixData(IMU_PITCH)
#define ELUR_YAW_SOURCE Get_Chassis_IMU_ZeroFixData(IMU_YAW)

rt_inline float Get_Chassis_IMU_ZeroFixData(imu_data_type_e type)
{
    return imu_zeroFixData[type];
}

// 得到imu的零点
rt_inline void getIMU_zero(uint8_t index)
{
    if (index < IMU_ALL)
    {
        if (imu_zeroCounts[index] != IMU_CALI_FLAG)
        {
            if (imu_zeroCounts[index] < IMU_CALI_COUNTS)
            {
                imu_zeroCounts[index]++;
                switch (index)
                {
                case IMU_YAW:
                case IMU_PITCH:
                case IMU_ROLL:
                    imu_zeroSumData[index] += Chassis_RawIMU.euler_angle.yaw;
                    imu_zeroSumData[index + 1] += Chassis_RawIMU.euler_angle.pitch;
                    imu_zeroSumData[index + 2] += Chassis_RawIMU.euler_angle.roll;
                    break;

                case IMU_GYROX:
                case IMU_GYROY:
                case IMU_GYROZ:
                    imu_zeroSumData[index] += Chassis_RawIMU.gyro.gx;
                    imu_zeroSumData[index + 1] += Chassis_RawIMU.gyro.gy;
                    imu_zeroSumData[index + 2] += Chassis_RawIMU.gyro.gz;
                    break;

                case IMU_ACCZ:
                case IMU_ACCX:
                case IMU_ACCY:
                    imu_zeroSumData[index] += Chassis_RawIMU.acc.ax;
                    imu_zeroSumData[index + 1] += Chassis_RawIMU.acc.ay;
                    imu_zeroSumData[index + 2] += Chassis_RawIMU.acc.az;
                    break;
                default:
                    break;
                }
            }
            else
            {
                imu_zero[index] = imu_zeroSumData[index] / imu_zeroCounts[index];
                imu_zero[index + 1] = imu_zeroSumData[index + 1] / imu_zeroCounts[index];
                imu_zero[index + 2] = imu_zeroSumData[index + 2] / imu_zeroCounts[index];
                imu_zeroCounts[index] = IMU_CALI_FLAG;
            }
        }
    }
}

// 由欧拉角更新四元数
void AHRS_Euler2Quarternion(imu_euler_t *eulr, AHRS_Quaternion_t *ahrs)
{
    float sin_half_p, sin_half_r, sin_half_y = 0.0f;
    float cos_half_p, cos_half_r, cos_half_y = 0.0f;
    sin_half_p = sinf(0.5f * eulr->pitch);
    sin_half_r = sinf(0.5f * eulr->roll);
    sin_half_y = sinf(0.5f * eulr->yaw);
    cos_half_p = cosf(0.5f * eulr->pitch);
    cos_half_r = cosf(0.5f * eulr->roll);
    cos_half_y = cosf(0.5f * eulr->yaw);

    // 得到四元数
    ahrs->q0 = (cos_half_r * cos_half_p * cos_half_y) + (sin_half_r * sin_half_p * sin_half_y);
    ahrs->q1 = (sin_half_r * cos_half_p * cos_half_y) - (cos_half_r * sin_half_p * sin_half_y);
    ahrs->q2 = (cos_half_r * sin_half_p * cos_half_y) + (sin_half_r * cos_half_p * sin_half_y);
    ahrs->q3 = (cos_half_r * cos_half_p * sin_half_y) - (sin_half_r * sin_half_p * cos_half_y);
}

// 云台欧拉角 单位为°
void Refresh_Gimbal_IMU_Data(struct rt_can_msg *msg)
{
    HERO_Gimbal_IMU.euler_angle.pitch = *((float *)&msg->data[0]);
    HERO_Gimbal_IMU.euler_angle.yaw = *((float *)&msg->data[4]);
}

// 由四元数更新得到的欧拉角      单位为°
void Refresh_Chassis_IMU_Elur(struct rt_can_msg *msg)
{
    Chassis_RawIMU.euler_angle.pitch = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 100.0f; // pitch
    Chassis_RawIMU.euler_angle.yaw = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 100.0f;   // yaw
    Chassis_RawIMU.euler_angle.roll = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 100.0f;  // roll

    // 进行零点转化
    imu_zeroFixData[IMU_YAW] = Chassis_RawIMU.euler_angle.yaw - imu_zero[IMU_YAW];
    imu_zeroFixData[IMU_PITCH] = Chassis_RawIMU.euler_angle.pitch - imu_zero[IMU_PITCH];
    imu_zeroFixData[IMU_ROLL] = Chassis_RawIMU.euler_angle.roll - imu_zero[IMU_ROLL];

    // 进行坐标系转化
    Chas_IMU_fix.euler_angle.yaw = ELUR_YAW_SOURCE;
    Chas_IMU_fix.euler_angle.pitch = ELUR_PITCH_SOURCE;
    Chas_IMU_fix.euler_angle.roll = ELUR_ROLL_SOURCE;

    // 四元数转化0点和坐标系转化
    AHRS_Euler2Quarternion(&Chas_IMU_fix.euler_angle, &Chas_IMU_fix.qua);
#if (IS_USE_IMU_CALI == 1)
    getIMU_zero(IMU_YAW);
#endif
}

// 底盘陀螺仪计补偿后的原始数据  角速度单位为°/s
void Refresh_Chassis_IMU_Gyro(struct rt_can_msg *msg)
{
    Chassis_RawIMU.gyro.gy = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 300.0f;
    Chassis_RawIMU.gyro.gz = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 300.0f;
    Chassis_RawIMU.gyro.gx = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 300.0f;

    // 进行零点转化
    imu_zeroFixData[IMU_GYROX] = Chassis_RawIMU.gyro.gx - imu_zero[IMU_GYROX];
    imu_zeroFixData[IMU_GYROY] = Chassis_RawIMU.gyro.gy - imu_zero[IMU_GYROY];
    imu_zeroFixData[IMU_GYROZ] = Chassis_RawIMU.gyro.gz - imu_zero[IMU_GYROZ];

    // 进行坐标系转化
    Chas_IMU_fix.gyro.gx = GYRO_X_SOURCE;
    Chas_IMU_fix.gyro.gy = GYRO_Y_SOURCE;
    Chas_IMU_fix.gyro.gz = GYRO_Z_SOURCE;

#if (IS_USE_IMU_CALI == 1)
    getIMU_zero(IMU_GYROX);
#endif
}

// 底盘加速度计补偿后的原始数据 加速度单位为m/s^2
void Refresh_Chassis_IMU_Acc(struct rt_can_msg *msg)
{
    Chassis_RawIMU.acc.ax = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 100.0f; // ax
    Chassis_RawIMU.acc.ay = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 100.0f; // ay
    Chassis_RawIMU.acc.az = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 100.0f; // az

    // 进行零点转化
    imu_zeroFixData[IMU_ACCX] = Chassis_RawIMU.acc.ax - imu_zero[IMU_ACCX];
    imu_zeroFixData[IMU_ACCY] = Chassis_RawIMU.acc.ay - imu_zero[IMU_ACCY];
    imu_zeroFixData[IMU_ACCZ] = Chassis_RawIMU.acc.az - imu_zero[IMU_ACCZ];

    // 进行坐标系转化
    Chas_IMU_fix.acc.ax = ACCL_X_SOURCE;
    Chas_IMU_fix.acc.ay = ACCL_Y_SOURCE;
    Chas_IMU_fix.acc.az = ACCL_Z_SOURCE;

#if (IS_USE_IMU_CALI == 1)
    getIMU_zero(IMU_ACCX);
#endif
}

// 底盘IMU的四元数 由欧拉角进行校准
void Refresh_Chassis_IMU_Quaternion(struct rt_can_msg *msg)
{
    Chassis_RawIMU.qua.q0 = ((rt_int16_t)(msg->data[0] << 8 | msg->data[1])) / 100.0f;
    Chassis_RawIMU.qua.q1 = ((rt_int16_t)(msg->data[2] << 8 | msg->data[3])) / 100.0f;
    Chassis_RawIMU.qua.q2 = ((rt_int16_t)(msg->data[4] << 8 | msg->data[5])) / 100.0f;
    Chassis_RawIMU.qua.q3 = ((rt_int16_t)(msg->data[6] << 8 | msg->data[7])) / 100.0f;
}

/*------------------------------------XXX----------------------------------------------------------*/

/***
 * @brief 返回云台imu数据
 * @param type : 数据类型
 * @retval 云台IMU数据,范围[-180~180]
 ***/
float Get_Gimbal_IMU_Data(imu_data_type_e type)
{
    switch (type)
    {
    case IMU_YAW:
        return HERO_Gimbal_IMU.euler_angle.yaw;
    case IMU_PITCH:
        return HERO_Gimbal_IMU.euler_angle.pitch;
    default:
        break;
    }
    return 0;
}
/***
 * @brief 返回云台imu全部数据
 * @param type : 数据类型
 * @retval 云台IMU数据,范围[-180~180]
 ***/
imu_euler_t Get_Gimbal_IMU_TotalData(void)
{
    return HERO_Gimbal_IMU.euler_angle;
}

/*------------------------------------XXX----------------------------------------------------------*/

/***
 * @brief 返回底盘imu数据
 * @param type : 数据类型
 * @retval 底盘IMU数据,角度单位°,范围[-180~180] 速度单位m/s^2 角速度单位为°/s
 ***/
float Get_Chassis_IMU_Data(imu_data_type_e type)
{
    switch (type)
    {
    case IMU_YAW:
        return Chas_IMU_fix.euler_angle.yaw;
    case IMU_PITCH:
        return Chas_IMU_fix.euler_angle.pitch;
    case IMU_ROLL:
        return Chas_IMU_fix.euler_angle.roll;

    case IMU_GYROX:
        return Chas_IMU_fix.gyro.gx;
    case IMU_GYROY:
        return Chas_IMU_fix.gyro.gy;
    case IMU_GYROZ:
        return Chas_IMU_fix.gyro.gz;

    case IMU_ACCX:
        return Chas_IMU_fix.acc.ax;
    case IMU_ACCY:
        return Chas_IMU_fix.acc.ay;
    case IMU_ACCZ:
        return Chas_IMU_fix.acc.az;
    default:
        break;
    }
    return 0;
}
/***
 * @brief 返回底盘imu三轴加速度数据
 * @param type : 数据类型
 * @retval 底盘IMU加速度
 ***/
void Get_Chassis_IMU_Acc(imu_accel_t *Getdata)
{
    Getdata->ax = Chas_IMU_fix.acc.ax;
    Getdata->ay = Chas_IMU_fix.acc.ay;
    Getdata->az = Chas_IMU_fix.acc.az;
}

// 返回底盘imu三轴陀螺仪数据     // 单位°/s
void Get_Chassis_IMU_Gyro(imu_gyro_t *Getdata)
{
    Getdata->gx = Chas_IMU_fix.gyro.gx;
    Getdata->gy = Chas_IMU_fix.gyro.gy;
    Getdata->gz = Chas_IMU_fix.gyro.gz;
}

// 得到底盘IMU的四元数
AHRS_Quaternion_t Get_Chassis_IMU_Quaternion(void)
{
    return Chas_IMU_fix.qua;
}

// 得到底盘IMU的相关数据 数据都是以底盘坐标系 y轴前进 x轴右向
imu_t Get_Chassis_IMU_AllData(imu_t *imuData)
{
    if (NULL != imuData)
    {
        imuData->acc = Chas_IMU_fix.acc;
        imuData->gyro = Chas_IMU_fix.gyro;
        imuData->euler_angle = Chas_IMU_fix.euler_angle;
        imuData->qua = Chas_IMU_fix.qua;
    }
    return Chas_IMU_fix;
}

// 得到底盘IMU的欧拉角 单位度(-180-180] 逆时针为正
imu_euler_t Get_Chassis_IMU_Elur(imu_euler_t *Getdata)
{
    if (Getdata)
    {
        Getdata->pitch = Chas_IMU_fix.euler_angle.pitch;
        Getdata->yaw = Chas_IMU_fix.euler_angle.yaw;
        Getdata->roll = Chas_IMU_fix.euler_angle.roll;
    }
    return Chas_IMU_fix.euler_angle;
}

// 返回IMU是否校准完成
uint8_t getIsFixFinsh(void)
{
    uint8_t couns = 0;
    for (int i = 0; i < IMU_ALL; i++)
    {
        if (imu_zeroCounts[i] == IMU_CALI_FLAG)
        {
            couns++;
        }
    }
    if (couns >= 3)
        return 1;
    else
        return 0;
}
int ChassisIMURxInit(void)
{
    attachCanRXfun(rx_chasIMU_gyro, Refresh_Chassis_IMU_Gyro);
    attachCanRXfun(rx_chasIMU_elur, Refresh_Chassis_IMU_Elur);
    attachCanRXfun(rx_chasIMU_acc, Refresh_Chassis_IMU_Acc);
    attachCanRXfun(rx_chasIMU_Quaternion, Refresh_Chassis_IMU_Quaternion);
    return 1;
}
