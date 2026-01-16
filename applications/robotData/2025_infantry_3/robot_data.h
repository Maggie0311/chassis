#ifndef __ROBOT_DATA_H__
#define __ROBOT_DATA_H__

/*CAN ID 相关参数*/

/*功率控制相关参数*/

/*轮子相关数据*/
#define VEHICLE_LONG (342.f)       // 前后轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_WIDTH (342.f)      // 左右轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_DIAMETER (483.66f) // 对角轮间距（用于计算全向轮解算），单位mm
#define WHEEL_RADIUS (63.5f)       // 轮子半径 单位毫米

// 底盘方位角参数,底盘正前方为0°，逆时针为正方向,单位/°，在舵向电机正解算时需要
#define RF_Wheel_Azimuth (-45.f)
#define LF_Wheel_Azimuth (45.f)
#define LB_Wheel_Azimuth (135.f)
#define RB_Wheel_Azimuth (-135.f)

/*跟随角pid相关参数*/
#define FOLLOW_PID_1 15, 0, 10, 0, 4000, -4000 // kp, ki, kd, i_lim, out_lim_up, out_lim_down   // 一直在跟随模式下的跟随角pid参数
#define FOLLOW_PID_2 40, 0, 10, 0, 4000, -4000 // 由非跟随进入跟随时的跟随角pid参数，目的是为了实现非跟随模式下响应能够较快

/*设定值规划及前馈相关参数*/
/*底盘速度设定值的平滑，SMOOTH_ADD/SMOOTH_PERIOD m/s^2（或100°/s^2） (类型float)*/
#define WEIGHT 20.0f                   // 质量 kg
#define rotational_inertia 0.5f        // 转动惯量 kg·m²
#define PLAN_PERIOD (float)0.005       // s,频率为200Hz
#define VEL_MAX_X (float)5000          //    mm/s
#define VEL_MAX_Y (float)5000          //    mm/s
#define VEL_MAX_W (float)10000         //   0.1°/s
#define ACC_MAX_X (float)4000          // mm/s²
#define ACC_MAX_Y (float)4000          // mm/s²
#define ACC_MAX_W (float)30000         // 0.1°/s²
#define DOUBLE_ACC_MAX_X (float)30000  // mm / s³
#define DOUBLE_ACC_MAX_Y (float)30000  // mm / s³
#define DOUBLE_ACC_MAX_W (float)200000 // 0.1°/ s³

#endif