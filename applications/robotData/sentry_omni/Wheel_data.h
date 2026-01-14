#ifndef __HCHASSIS_DATA_H__
#define __HCHASSIS_DATA_H__

// 全向轮哨兵
/*------------------------------------电机以及车轮安装后固有参数----------------------------------------------------------*/
/*电机安转方向*/
// 当车身向规定前方向运动时，电机返回转速为负值则需将其置1 电机转子朝内
// #define RUN_REVERSAL 0
#define RF_RUN_REVERSAL 1
#define LF_RUN_REVERSAL 0
#define LB_RUN_REVERSAL 0
#define RB_RUN_REVERSAL 1
// 逆时针转动舵电机，舵电机编码器值从大到小则需要取反--即将其置1 电机转子朝内
// #define ANG_ANGLE_REVERSAL 1
#define RF_ANG_REVERSAL 1
#define LF_ANG_REVERSAL 1
#define LB_ANG_REVERSAL 1
#define RB_ANG_REVERSAL 1

// 轮子相关数据
#define VEHICLE_LONG (342.f)       // 前后轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_WIDTH (342.f)      // 左右轮间距（用于麦轮和舵轮解算），单位mm
#define VEHICLE_DIAMETER (483.66f) // 对角轮间距（用于计算全向轮解算），单位mm
#define WHEEL_RADIUS (63.5f)       // 轮子半径 单位毫米

// 底盘方位角参数,底盘正前方为0°，逆时针为正方向,单位/°，在舵向电机正解算时需要
#define RF_Wheel_Azimuth (-45.f)
#define LF_Wheel_Azimuth (45.f)
#define LB_Wheel_Azimuth (135.f)
#define RB_Wheel_Azimuth (-135.f)

// 根据电机转子方向调整零位
// note 新底盘需要重新标定的数据
#define RF_ANG_ZERO_ANGLE 2095 // 舵轮舵的四个电机零位 1
#define LF_ANG_ZERO_ANGLE 2070 // 舵轮舵的四个电机零位 2
#define LB_ANG_ZERO_ANGLE 4133 // 舵轮舵的四个电机零位 3
#define RB_ANG_ZERO_ANGLE 3493 // 舵轮舵的四个电机零位 4

/*------------------------------------云台----------------------------------------------------------*/
#define YAW_ZERO_ANGLE 59200  // 云台电机零位（DM）
#define IF_USE_FOLLOW_BACK 1  // 是否使用后向跟随
#define PITCH_ZERO_ANGLE 4687 // 云台电机零位

#define YAW_ANGLE_REVERSAL 0 // 俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，
                             // 逆时针转动云台yaw轴电机，yaw轴电机编码器值从大到小则需要取反--即将其置1

#define PITCH_ANGLE_REVERSAL 0 // 枪口往上抬，pitch轴电机编码器值从大到小则需要取反--即将其置1
/*------------------------------------云台----------------------------------------------------------*/
/*------------------------------------底盘----------------------------------------------------------*/

#define FOLLOW_PID_1 15, 0.1, 10, 0, 2000, -2000 // kp, ki, kd, i_lim, out_lim_up, out_lim_down   // 一直在跟随模式下的跟随角pid参数
#define FOLLOW_PID_2 40, 0, 10, 0, 2000, -2000 // 由非跟随进入跟随时的跟随角pid参数，目的是为了实现非跟随模式下响应能够较快
#define FOLLOW_PID_3 40, 0, 10, 0, 2000, -2000
/*---------------------------平滑相关参数---------------------------------------------*/
/*底盘速度设定值的平滑，SMOOTH_ADD/SMOOTH_PERIOD m/s^2（或100°/s^2） (类型float)*/
#define ACC_MAX_X (float)5         //    m/s
#define ACC_MAX_Y (float)5         //    m/s
#define ACC_MAX_W (float)20.0      //   10°/s
#define Accel_factor_X (float)2.f  // 加速平滑系数
#define Accel_factor_Y (float)2.f  // 加速平滑系数
#define Accel_factor_W (float)1.5f // 加速平滑系数
#define DECEL_FACTOR_X (float)0.7f // 减速平滑系数
#define DECEL_FACTOR_Y (float)0.7f // 减速平滑系数
#define DECEL_FACTOR_W (float)0.7f // 减速平滑系数

// #define SMOOTH_DATA_MODE 0       // 0: 默认模式，1：随姿态改变的模式
// #define SMOOTH_IF_USE_KEY_CTRL 0 // 是否使用ctrl键(根据操作手按键改变加速度的模式)
// #if SMOOTH_DATA_MODE == 1 || SMOOTH_IF_USE_KEY_CTRL == 1
// #define SMOOTH_MAX_angel (12.0f)
// #define SMOOTH_ADD_MAX 5.5f
// #define SMOOTH_ADD_MIN 2.8f
// #define SMOOTH_MODE SM_NORMAL // X/Y线速度平滑模式 SM_NORMAL/SM_MIRROR
// #define SMOOTH_ADD_UP 7.f     // 1ms增加的速度(mm/s)->(1m/s^2)
// #define SMOOTH_ADD_DOWN 7.f   // 1ms减少的速度(mm/s)->(1m/s^2)
// #elif SMOOTH_DATA_MODE == 0
// #define SMOOTH_MODE SM_NORMAL // X/Y线速度平滑模式 SM_NORMAL/SM_MIRROR
// #define SMOOTH_ADD_UP 2.8f    // 1ms增加的速度(mm/s)->(1m/s^2)
// #define SMOOTH_ADD_DOWN 4.5f  // 1ms减少的速度(mm/s)->(1m/s^2)
// #endif
// #define SMOOTH_START 2000.f // 启动区速度大小(mm/s)

// #define SMOOTH_ADD_UP_AC_1 12.f   // 1ms增加的角速度(0.1°/s)->(100°/s^2)
// #define SMOOTH_ADD_DOWN_AC_1 12.f // 1ms减少的角速度(0.1°/s)->(100°/s^2)
// #define SMOOTH_START_AC 300.f     // 启动区速度大小(mm/s)
// #define SMOOTH_ADD_UP_AC_2 12.f   // 跟随时的参数
// #define SMOOTH_ADD_DOWN_AC_2 12.f
// #define SMOOTH_MODE_AC SM_MIRROR // W角速度平滑模式 SM_NORMAL/SM_MIRROR
/*---------------------------------------------------------------------------*/

#define LOW_BATTERY_TL 10.0f     // 判断超级电容低电量的下界限，单位百分比，0-100
#define LOW_BATTERY_TH 20.0f     // 判断超级电容低电量的上界限，单位百分比，0-100
#define RESERVE_BATTERY_TL 30.0f // 操作手需要预留电量时留下来的电量下界
#define RESERVE_BATTERY_TH 60.0f // 操作手需要预留电量时留下来的电量上界

#if defined(WHEEL_MOTOR_3508)               // 驱动电机使用3508
#define REDUCT_RATIO_RUN (3591.0f / 187.0f) // 电机减速比
#define RATED_SPEED_RUN 9000                // 电机额定转速 当超过时直接限制速度
#define CURRENT_LIMIT_RUN 4 * 12288.0f      // 最大电流限制，对应实际电流15A，单位 /16384*20 A

#elif defined(WHEEL_MOTOR_2006)    // 驱动电机使用2006
#define REDUCT_RATIO_RUN (36.0f)   // 电机减速比
#define RATED_SPEED_RUN 14976      // 电机额定转速
#define CURRENT_LIMIT_RUN 15000.0f // 最大电流限制，对应实际电流15A，单位 /10000*10 A
#endif

#if defined(STEER_MOTOR_6020)
#define REDUCT_RATIO_ANGLE (1) // 电机减速比
// TODO 修改额定转速
#define RATED_SPEED_ANGLE 16384          // 电机额定转速 RATED_SPEED_ANGLE
#define CURRENT_LIMIT_ANGLE 4 * 12288.0f // 最大电流限制，对应实际电流15A，单位 /10000*10 A
#endif

#define VCONVERT2W_k ((float)(1.f / WHEEL_RADIUS)) // 车轮速度转电机输出轴角速度 速度的单位为mm/s 角速度单位为rad/s
#define WCONVERT2V_k ((float)WHEEL_RADIUS)         // 电机输出轴角速度转车轮速度 速度的单位为mm/s 角速度单位为rad/s

#define K_YAW_DeltaT (20.0f)
/*------------------------------------功率----------------------------------------------------------*/
#define KEY_C_FUN 1            // 超级电容C键功能选择：0:无功能;1:超电开关功能;2:电量保留功能
#define SCAP_MAX_CURRENT (8.f) // 电容最大放电响应电流
/*电机功率参数*/
#define Power_C_sum (3.5f) // 固定静态功率（换板子也会影响该数据）

#define Wheel_Power_K1 (490.f)     //(4.5e+02) //(1.15e+02) //(1.0e+02)
#define Wheel_Power_K2 (4.309e-07) //(1.0e-07) //(6.0e-07)  //(4.1e-07)
#define Wheel_Power_C (0.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Wheel_Power_C_sum (6.0f)   // 固定静态功率（换板子也会影响该数据）
#define Steer_Power_K1 (10.f)      //
#define Steer_Power_K2 (0.0000015) //
#define Steer_Power_C (0.f)        // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
// #define Steer_Power_C_sum (10.0f)  // 固定静态功率（换板子也会影响该数据）

#endif //__HCHASSIS_DATA_H__
