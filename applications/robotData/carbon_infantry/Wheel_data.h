#ifndef __HCHASSIS_DATA_H__
#define __HCHASSIS_DATA_H__

// 碳步
/*------------------------------------电机以及车轮安装后固有参数----------------------------------------------------------*/
/*电机安转方向*/
typedef enum
{
    W_OUTSIDE, // 输出轴朝外
    W_INSIDE   // 输出轴朝内

} Wheel_install_e;

#define INSTALL_DIR W_INSIDE // 底盘所有电机安装方向 针对于设定值的更改
#define MOTOR_IS_REVERSE 0   // 当车身向规定前方向运动时，电机返回转速为负值则需将其置1

#define VEHICLE_LONG 325  // 前后轮间距，单位mm
#define VEHICLE_WIDTH 370 // 左右轮间距，单位mm米
#define WHEEL_RADIUS 70   // 麦轮半径 单位毫米

/*------------------------------------云台----------------------------------------------------------*/
#define YAW_ZERO_ANGLE 8145   // 云台电机零位
#define IF_USE_FOLLOW_BACK 1  // 是否使用后向跟随
#define PITCH_ZERO_ANGLE 3423 // 云台电机零位

#define YAW_ANGLE_REVERSAL 0 // 俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，
                             // 逆时针转动云台yaw轴电机，yaw轴电机编码器值从大到小则需要取反--即将其置1

#define PITCH_ANGLE_REVERSAL 0 // 枪口往上抬，pitch轴电机编码器值从大到小则需要取反--即将其置1
/*------------------------------------云台----------------------------------------------------------*/
/*---------------------------平滑相关参数---------------------------------------------*/
/*底盘速度设定值的平滑，SMOOTH_ADD/SMOOTH_PERIOD m/s^2（或100°/s^2） (类型float)*/
#define SMOOTH_DATA_MODE 1       // 0: 默认模式，1：随姿态改变的模式
#define SMOOTH_IF_USE_KEY_CTRL 0 // 是否使用ctrl键(根据操作手按键改变加速度的模式)
#if SMOOTH_DATA_MODE == 1 || SMOOTH_IF_USE_KEY_CTRL == 1
#define SMOOTH_MAX_angel (12.0f)
#define SMOOTH_ADD_MAX 7.0f
#define SMOOTH_ADD_MIN 2.8f
#define SMOOTH_MODE SM_NORMAL // X/Y线速度平滑模式 SM_NORMAL/SM_MIRROR
#define SMOOTH_ADD_UP 5.0f    // 1ms增加的速度(mm/s)->(1m/s^2)
#define SMOOTH_ADD_DOWN 5.0f  // 1ms减少的速度(mm/s)->(1m/s^2)
#elif SMOOTH_DATA_MODE == 0
#define SMOOTH_MODE SM_MIRROR // X/Y线速度平滑模式 SM_NORMAL/SM_MIRROR
#define SMOOTH_ADD_UP 2.8f    // 1ms增加的速度(mm/s)->(1m/s^2)
#define SMOOTH_ADD_DOWN 5.0f  // 1ms减少的速度(mm/s)->(1m/s^2)
#endif
#define SMOOTH_START 300.f // 启动区速度大小(mm/s)

#define SMOOTH_ADD_UP_AC_1 7.2f   // 1ms增加的角速度(0.1°/s)->(100°/s^2)
#define SMOOTH_ADD_DOWN_AC_1 12.f // 1ms减少的角速度(0.1°/s)->(100°/s^2)
#define SMOOTH_START_AC 300.f     // 启动区速度大小(mm/s)
#define SMOOTH_ADD_UP_AC_2 7.2f   // 跟随时的参数
#define SMOOTH_ADD_DOWN_AC_2 12.f
#define SMOOTH_MODE_AC SM_MIRROR // W角速度平滑模式 SM_NORMAL/SM_MIRROR
/*---------------------------------------------------------------------------*/
#define LOW_BATTERY_TL 10.0f     // 判断超级电容低电量的下界限，单位百分比，0-100
#define LOW_BATTERY_TH 20.0f     // 判断超级电容低电量的上界限，单位百分比，0-100
#define RESERVE_BATTERY_TL 30.0f // 操作手需要预留电量时留下来的电量下界
#define RESERVE_BATTERY_TH 60.0f // 操作手需要预留电量时留下来的电量上界

#define WHEEL_MOTOR_3508
#if defined(WHEEL_MOTOR_3508)                // 驱动电机使用3508
#define REDUCT_RATIO_3508 (268.0f / 17.0f) // 电机减速比
#define RATED_SPEED_3508 16384                // 电机额定转速 当超过时直接限制速度
#define CURRENT_LIMIT_3508 4 * 12288.0f      // 最大电流限制，对应实际电流15A，单位 /16384*20 A
#endif

#define STEER_MOTOR_6020
#if defined(STEER_MOTOR_6020) 
#define REDUCT_RATIO_6020 (1)   // 电机减速比
#define RATED_SPEED_6020 16384      // 电机额定转速
#define CURRENT_LIMIT_6020 4 * 12288.0f // 最大电流限制，对应实际电流15A，单位 /10000*10 A
#endif 

#if defined(WHEEL_MOTOR_2006)
#define REDUCT_RATIO_2006 (36.0f)   // 电机减速比
#define RATED_SPEED_2006 10000      // 电机额定转速
#define CURRENT_LIMIT_2006 4*15000.0f // 最大电流限制，对应实际电流15A，单位 /10000*10 A
#endif 

#define VCONVERT2W_k ((float)(1.f / WHEEL_RADIUS)) // 车轮速度转电机输出轴角速度 速度的单位为mm/s 角速度单位为rad/s
#define WCONVERT2V_k ((float)WHEEL_RADIUS)         // 电机输出轴角速度转车轮速度 速度的单位为mm/s 角速度单位为rad/s
// 小陀螺yaw轴角度补偿系数
// 【偏移角随此数值增大而向顺时针方向旋转】
#define K_YAW_DeltaT (12.0f)
/*------------------------------------功率----------------------------------------------------------*/
#define KEY_C_FUN 1            // 超级电容C键功能选择：0:无功能;1:超电开关功能;2:电量保留功能
#define SCAP_MAX_CURRENT (8.f) // 电容最大放电响应电流
/*电机功率参数*/
#define Wheel_Power_K1 (6.0e+02) //(2.33e+02)
#define Wheel_Power_K2 (1.1e-06) //(6.453e-07)
#define Wheel_Power_C (0.f)      // 电机静态功率;其效果可以合并计入下面的固定静态功率中，所以暂时不用
#define Wheel_Power_C_sum (7.4f) // 固定静态功率（换板子也会影响该数据）

#endif //__HCHASSIS_DATA_H__
