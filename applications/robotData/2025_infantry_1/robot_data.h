#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

/*CAN ID 相关参数*/
#define ANGLE_RF_MOTOR_ID 0x206 // 底盘电机电调 ID
#define ANGLE_LF_MOTOR_ID 0x205
#define ANGLE_LB_MOTOR_ID 0x207
#define ANGLE_RB_MOTOR_ID 0x208
#define RUN_RF_MOTOR_ID 0x201
#define RUN_LF_MOTOR_ID 0x202
#define RUN_LB_MOTOR_ID 0x203
#define RUN_RB_MOTOR_ID 0x204

#define CHASSIS_IMU_ANGLE_RX 0x001 // 从底盘IMU接收的角度数据
#define CHASSIS_IMU_W_RX 0x002     // 从底盘IMU接收的角速度数据
#define CHASSIS_IMU_ACCEL_RX 0x003 // 从底盘IMU接收的加速度数据
#define CHASSIS_IMU_Quaternion_RX 0x004

#ifdef YAW_MOTOR_6020
#define YAW_MOTOR_RXID 0x205   // 云台电机反馈报文 ID
#define PITCH_MOTOR_RXID 0x206 // 云台电机反馈报文 ID
#endif
#ifdef YAW_MOTOR_DM
#define YAW_MOTOR_RXID 0x03   // 云台电机反馈报文 ID
#define PITCH_MOTOR_RXID 0x04 // 云台电机反馈报文 ID
#endif

#define GIMBAL_CCTRL_RX 0x100    // 云台控制底盘相关
#define GIMBAL_DATA_RX 0x401     // 从云台接收的各类数据
#define GIMBAL_IMU_DATA_RX 0x402 // 从云台接收的IMU数据

#define SCPR_RX 0x096 // 从超级电容端接收
#define SCPR_TX 0x095 // 向超级电容端发送

#define GIMBAL_SHOOT_TX 0x105 // 裁判系统枪管信息
#define GIMBAL_DATA_TX 0x106  // 向云台发送各类数据
/*------------------------------------CAN ID----------------------------------------------------------*/

/*------------------------------------CAN 设备----------------------------------------------------------*/
typedef enum
{
    USED_CAN1,
    USED_CAN2,
    USED_CAN_1_2,
    USED_CAN_all,
} canDev_e;
//底盘电机逻辑和这部分略有不同
#if RF_CAN_DEVICE_NAME == 1
#define RF_CAN_DEV USED_CAN1
#elif RF_CAN_DEVICE_NAME == 2
#define RF_CAN_DEV USED_CAN2
#endif
#if LF_CAN_DEVICE_NAME == 1
#define LF_CAN_DEV USED_CAN1
#elif LF_CAN_DEVICE_NAME == 2
#define LF_CAN_DEV USED_CAN2
#endif
#if LB_CAN_DEVICE_NAME == 1
#define LB_CAN_DEV USED_CAN1
#elif LB_CAN_DEVICE_NAME == 2
#define LB_CAN_DEV USED_CAN2
#endif
#if RB_CAN_DEVICE_NAME == 1
#define RB_CAN_DEV USED_CAN1
#elif RB_CAN_DEVICE_NAME == 2
#define RB_CAN_DEV USED_CAN2
#endif


// #define GIMBAL_CAN_DEV USED_CAN2        // 云台can设备
#if GIMBAL_CAN_DEVICE_NAME == 1
#define GIMBAL_CAN_DEV "can1"
#elif GIMBAL_CAN_DEVICE_NAME == 2
#define GIMBAL_CAN_DEV "can2"
#else 
#error "GIMBAL_CAN_DEVICE_NAME is not defined right!"
#endif

// #define IMU_GIMBAL_CAN_DEV USED_CAN2    // 云台IMUcan设备
#if IMU_GIMBAL_CAN_DEVICE_NAME == 1
#define IMU_GIMBAL_CAN_DEV USED_CAN1
#elif IMU_GIMBAL_CAN_DEVICE_NAME == 2
#define IMU_GIMBAL_CAN_DEV USED_CAN2
#else 
#error "IMU_GIMBAL_CAN_DEVICE_NAME is not defined right!"
#endif

// #define SCPR_CAN_DEV USED_CAN1          // 超级电容设备
#if SCPR_CAN_DEVICE_NAME == 1
#define SCPR_CAN_DEV "can1"
#elif SCPR_CAN_DEVICE_NAME == 2
#define SCPR_CAN_DEV "can2"
#else 
#error "SCPR_CAN_DEVICE_NAME is not defined right!"
#endif

// #define IMU_CHASSIS_CAN_DEV USED_CAN1   // 底盘IMUcan设备
#if IMU_CHASSIS_CAN_DEVICE_NAME == 1
#define IMU_CHASSIS_CAN_DEV USED_CAN1
#elif IMU_CHASSIS_CAN_DEVICE_NAME == 2
#define IMU_CHASSIS_CAN_DEV USED_CAN2
#else 
#error "IMU_CHASSIS_CAN_DEVICE_NAME is not defined right!"
#endif

/*功率控制相关参数*/


/*PID相关参数部分*/