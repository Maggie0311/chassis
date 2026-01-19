#include "Wheel_data.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "drv_motionSolve.h"
#include "robot_control.h"
#define WHEEL_MOTOR_NUM 4
#define GIM_MOTOR_NUM 2

// 底盘电机实例
DJI_3508_MotorInstance wheel_motor[WHEEL_MOTOR_NUM];
DJI_6020_MotorInstance gim_motor[GIM_MOTOR_NUM];

uint8_t isSetValid[2] = {1, 1};       /*debug*,1为开，0为关/
      
      /*底盘四个车轮(或者说轮毂)的期望线速度 单位mm/s 输出顺序RF-LF-LB-RB*/
float motorSetSpeed[WHEEL_MOTOR_NUM]; // 经过解算后的数据(速度)
float motorSetForce[WHEEL_MOTOR_NUM]; // 经过解算后的数据(加速度)

// 底盘四个电机配置参数模板
DJI_3508_Motor_Init_Config_s wheel_motor_init_config[WHEEL_MOTOR_NUM] = {
    // 右前轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_RF",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
        },
        .Motor_operate_setting = {
            .Motor_Init = DJI_3508_Motor_Init,
            .Motor_Enable = DJI_3508_Motor_Enable,
            .Motor_Disable = DJI_3508_Motor_Disable,
            .Motor_ErrorHandle = DJI_3508_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_3508_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_3508_Motor_Transmit,
        },
        // TODO:调Pid这堆
        .controller_param_init_config = {
            .Speed_pid = {.Kp = 15, .Ki = 0.1, .Kd = 0.03, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 7000},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,

            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x201, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 左前轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_LF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
        },
        .Motor_operate_setting = {
            .Motor_Init = DJI_3508_Motor_Init,
            .Motor_Enable = DJI_3508_Motor_Enable,
            .Motor_Disable = DJI_3508_Motor_Disable,
            .Motor_ErrorHandle = DJI_3508_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_3508_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_3508_Motor_Transmit,
        },
        // TODO:调Pid这堆
        .controller_param_init_config = {
            .Speed_pid = {.Kp = 15, .Ki = 0.1, .Kd = 0.03, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 7000},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x202, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },

    },
    // 左后轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_LB",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
        },
        .Motor_operate_setting = {
            .Motor_Init = DJI_3508_Motor_Init,
            .Motor_Enable = DJI_3508_Motor_Enable,
            .Motor_Disable = DJI_3508_Motor_Disable,
            .Motor_ErrorHandle = DJI_3508_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_3508_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_3508_Motor_Transmit,

        },
        // TODO:调Pid这堆
        .controller_param_init_config = {
            .Speed_pid = {.Kp = 15, .Ki = 0.1, .Kd = 0.03, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 7000},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x203, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },

    },
    // 右后轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_LR",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
        },
        .Motor_operate_setting = {
            .Motor_Init = DJI_3508_Motor_Init,
            .Motor_Enable = DJI_3508_Motor_Enable,
            .Motor_Disable = DJI_3508_Motor_Disable,
            .Motor_ErrorHandle = DJI_3508_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_3508_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_3508_Motor_Transmit,

        },
        // TODO:调Pid这堆
        .controller_param_init_config = {
            .Speed_pid = {.Kp = 15, .Ki = 0.1, .Kd = 0.03, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 7000},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x204, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    }};

int chassis_motor_init()
{
    int ret = 0;
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        ret = Motor_Attach((MotorInstance *)&wheel_motor[i], (Motor_Init_Config_s *)&wheel_motor_init_config[i]);
        Power_Motor_Attach((MotorInstance *)&wheel_motor[i]);
        if (ret != 0)
        {
            return ret; // 如果注册失败将直接返回错误，停止后续电机注册
        }
    }
    return ret;
}


DJI_6020_Motor_Init_Config_s    gim_motor_init_config[GIM_MOTOR_NUM]={
    //YAW轴电机
    {
        .motor_obj_init_config = {
            .name = "Yaw_M",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = GM6020,
            .state = MOTOR_STATE_NEED_INIT,
        },
       .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = NULL,
        },
        .controller_param_init_config=NULL,

        .can_init_config = {
            .can_device_name = "can2",
            .rx_id = 0x205, // TODO:设置ID
            .tx_id = NULL,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
         .DJI6020_Motor_Measure_Init_Config = {
        .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
        .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
        .RATIO = 1.0,
        .ENCODER_LEN = 8192,
        .ENCODER_ZERO_OFFSET = 0.0,
        .ctrl_mode = CTRL_MODE_VOLTAGE,
        }
    },
    //pitch轴电机

     {
        .motor_obj_init_config = {
            .name = "Pitch_M",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = GM6020,
            .state = MOTOR_STATE_NEED_INIT,
        },
       .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = NULL,
        },
        .controller_param_init_config=NULL,

        .can_init_config = {
            .can_device_name = "can2",
            .rx_id = 0x206, // TODO:设置ID
            .tx_id = NULL,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
         .DJI6020_Motor_Measure_Init_Config = {
        .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
        .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
        .RATIO = 1.0,
        .ENCODER_LEN = 8192,
        .ENCODER_ZERO_OFFSET = 0.0,
        .ctrl_mode = CTRL_MODE_VOLTAGE,
        }
    }

};

int gim_motor_init()
{
    int ret = 0;
    for (int i = 0; i < GIM_MOTOR_NUM; i++)
    {
        ret = Motor_Attach((MotorInstance *)&gim_motor[i], (Motor_Init_Config_s *)&gim_motor_init_config[i]);
        if (ret != 0)
        {
            return ret; // 如果注册失败将直接返回错误，停止后续电机注册
        }
        Gim_Motor_Attach((MotorInstance *)&gim_motor[i]);

    }
    return ret;
}

static Mot_base_t Force_Allo_to_wheels(float motorSetForce[4], Mot_base_t *Force_xyw) // 进行加速度解算到各个轮子上
{
    Mot_base_t tarForce;                                                        // 电机输出轴上的力（N）
    tarForce.vel.x = Force_xyw->vel.x * WEIGHT / 1000.f;                        // 整个底盘x方向力的平滑后设定值(N)
    tarForce.vel.y = Force_xyw->vel.y * WEIGHT / 1000.f;                        // 整个底盘Y方向力的平滑后设定值(N)
    tarForce.angvel = Force_xyw->angvel / 10.f * DEGTORAD * rotational_inertia; // 整个底盘扭矩的平滑后设定值(N·m)

    // Omni_Force_Resolve(*Fxyw, group, motorSetForce[group]);
}

// 摩擦力前馈计算
float test_k_feed = 0.4;
static float RUN_wheel_forward_cal(float set)
{
    if (set > 200.f)
    {
        return -0.000036 * set * set + set * test_k_feed + 470.60; // 0.1734
    }
    else if (set < -200.f)
    {
        return 0.000036 * set * set + set * test_k_feed - 470.60;
    }
    else
    {
        return 503.8 * set / 200; // 在-200到200之间返回近似一次斜坡的函数，防止函数不连续导致的抖动问题
    }
}

/**
 * @brief 将雅可比矩阵解算出的电机的力转换为电机的扭矩，进而转换为电流
 *
 * @param motorSetForce
 * @param force_out
 */
static void Get_force_out(float motorSetForce[WHEEL_MOTOR_NUM])
{
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        if (wheel_motor[i].Motor_obj.reverse_flag == 1)
        {

            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = -motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f - RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
        }
        else
        {
            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
        }
    }
}

static void Wheel_Speed_Set(float motorSetSpeed[WHEEL_MOTOR_NUM])
{
    // for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    // {
    //     /*由轮子的线速度单位为mm/s,转化为输出轴的角速度，再转化为控制转子的角速度单位为rmp*/
    //     motorSetSpeed[i] = motorSetSpeed[i] * VCONVERT2W_k * RAD2RPM_K * wheel_motor->dji3508_motor_measure.RATIO;
    //     Motor_Set_Target((MotorInstance*)&wheel_motor[i], motorSetSpeed[i]);
    // }
}

void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw)
{
    if (1 == isSetValid[0])
    {
        // Omni_Resolve(Speed_xyw, group, motorSetSpeed[WHEEL_MOTOR_NUM]);//底盘速度解算到各个电机上

        Force_Allo_to_wheels(motorSetForce, Force_xyw); // 进行加速度解算到各个电机上

        Get_force_out(motorSetForce); // 获取前馈值
    }
    if (1 == isSetValid[1])
    {
        Wheel_Speed_Set(motorSetSpeed); // 设置电机速度环设定值
    }
}