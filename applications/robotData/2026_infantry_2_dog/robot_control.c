/**2026 狗洞步兵 （未出生）*/
#include "robot_data.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "drv_motionSolve.h"
#include "robot_control.h"
#include "drv_GimMotor.h"
#include "fun_powerCtrl.h"

#define WHEEL_MOTOR_NUM 4
#define GIM_MOTOR_NUM 2
#define TRACK_MOTOR_NUM 2

#define WHEEL_RF 0
#define WHEEL_LF 1
#define WHEEL_LH 2
#define WHEEL_RH 3
#define TRACK_R 4
#define TRACK_L 5


// 底盘电机实例
DJI_3508_MotorInstance wheel_motor[WHEEL_MOTOR_NUM];
DJI_6020_MotorInstance gim_motor[GIM_MOTOR_NUM];
DJI_3508_MotorInstance track_motor[TRACK_MOTOR_NUM];//新增履带电机

uint8_t isSetValid_debug[2] = {1, 1}; /*debug*,1为开，0为关/

/*底盘四个车轮(或者说轮毂)的期望线速度 单位mm/s 输出顺序RF-LF-LB-RB*/
float motorSetSpeed[WHEEL_MOTOR_NUM]; // 经过解算后的数据(速度)
float motorSetForce[WHEEL_MOTOR_NUM]; // 经过解算后的数据(加速度)

// 跟随角pid实例
PIDInstance follow_pid_fast, follow_pid_slow;

// 底盘四个电机配置参数模板
DJI_3508_Motor_Init_Config_s wheel_motor_init_config[WHEEL_MOTOR_NUM] = {
    // 右前轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_RF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = WHEEL_RF,
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

        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp+30, .Ki = RUN_PID_Ki+0.5, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit , .CoefA = 502.0, .CoefB = 2.0, .IntegralLimit = 6000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 12000},
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
            .RATIO = 13.733f,               // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 左前轮
    {
        .motor_obj_init_config = {
            .name = "Wheel_LF",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = WHEEL_LF,
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
        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp+30, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd+0.5, .Improve = PID_Integral_Limit , .CoefA = 502.0, .CoefB = 2.0, .IntegralLimit = 4500, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 15000},
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
            .RATIO = 13.733f,               // 搭配C620电调，默认减速比为19
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
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = WHEEL_LH,
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
        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp+10, .Ki = RUN_PID_Ki+0.5, .Kd = RUN_PID_Kd+0.5, .Improve = PID_Integral_Limit , .CoefA = 502.0, .CoefB = 2.0, .IntegralLimit = 6000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 13000},
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
            .RATIO = 13.733f,               // 搭配C620电调，默认减速比为19
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
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = WHEEL_RH,
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
        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit , .CoefA = 502.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
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
            .RATIO = 13.733f,               // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    }};



//新增履带电机配置模板
DJI_3508_Motor_Init_Config_s track_motor_init_config[TRACK_MOTOR_NUM] = {
    // 履带电机
    {
        .motor_obj_init_config = {
            .name = "Track_RF",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = TRACK_R,
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

        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit , .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,

            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can2",
            .rx_id = 0x202, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.2f,               // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    {
        .motor_obj_init_config = {
            .name = "Track_LF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_WHEEL,
            .index = TRACK_L,
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

        .controller_param_init_config = {
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 7000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,

            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can2",
            .rx_id = 0x204, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.2f,               // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
};

//todo 目前还没有调用履带电机初始化，等出车
int track_motor_init()
{
    int ret = 0;
    for (int i = 0; i < TRACK_MOTOR_NUM; i++)
    {
        ret = Motor_Attach((MotorInstance *)&track_motor[i], (Motor_Init_Config_s *)&track_motor_init_config[i]);
        if (ret != 0)
        {
            return ret; // 如果注册失败将直接返回错误，停止后续电机注册
        }
    }
    return ret;
}
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
    track_motor_init();
    return ret;
}
DJI_6020_Motor_Init_Config_s gim_motor_init_config[GIM_MOTOR_NUM] = {
    // YAW轴电机
    {
        .motor_obj_init_config = {
            .name = "Yaw_M",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
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
        .controller_param_init_config = NULL,

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
            .ENCODER_ZERO_OFFSET = 574,
            .ctrl_mode = CTRL_MODE_VOLTAGE,
        }},
    // pitch轴电机

    {.motor_obj_init_config = {
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
     .controller_param_init_config = NULL,

     .can_init_config = {
         .can_device_name = "can2",
         .rx_id = 0x207, // TODO:设置ID
         .tx_id = NULL,
         .can_module_callback = DJI_6020_Motor_RefreshData,
         .parent = NULL,
     },
     .DJI6020_Motor_Measure_Init_Config = {
         .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
         .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
         .RATIO = 1.0,
         .ENCODER_LEN = 8192,
         .ENCODER_ZERO_OFFSET = 230,
         .ctrl_mode = CTRL_MODE_VOLTAGE,
     }}

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
         ret = Gim_Motor_Attach((MotorInstance *)&gim_motor[i]);
         if (ret != 0)
         {
             return ret; // 如果注册失败将直接返回错误，停止后续电机注册
         }
    }
    return ret;
}

void Follow_pid_init(void)
{
    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

    pid_config.Kp = 40;
    pid_config.Ki = 0;
    pid_config.Kd = 15;
    pid_config.MaxOut = 16000;
    pid_config.DeadBand = 0.1;
    pid_config.IntegralLimit = 600;

    PIDInit(&follow_pid_fast, &pid_config);

    pid_config.Kp = 10;
    pid_config.Ki = 0;
    pid_config.Kd = 15;
    pid_config.MaxOut = 16000;
    pid_config.DeadBand = 0.1;
    pid_config.IntegralLimit = 400;
    PIDInit(&follow_pid_slow, &pid_config);
}

static Mot_base_t Force_Allo_to_wheels(float motorSetForce[4], Mot_base_t *Force_xyw) // 进行加速度解算到各个轮子上
{
    Mot_base_t tarForce;                                                        // 电机输出轴上的力（N）
    tarForce.vel.x = Force_xyw->vel.x * WEIGHT / 1000.f;                        // 整个底盘x方向力的平滑后设定值(N)
    tarForce.vel.y = Force_xyw->vel.y * WEIGHT / 1000.f;                        // 整个底盘Y方向力的平滑后设定值(N)
    tarForce.angvel = Force_xyw->angvel / 10.f * DEGTORAD * rotational_inertia; // 整个底盘扭矩的平滑后设定值(N·m)

    // Omni_Force_Resolve(tarForce, motorSetForce);
}

// 摩擦力前馈计算 这里的数值是采集数据后在matlab中进行回归拟合得到的
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
   { // {
       //     if(i==0||i==3)
       //     {
       //     wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real  + 2000;//RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
       //     }
       //     if(i==1||i==2)
       //     {
       //         wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real  +2200;//1.7* RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
       //     }
       if(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target>200.0f)
           wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real  + 2200.0f;//RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
       if (wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target<-200.0f)
           wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real  -2200.0f;//RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
           else
               wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real + 11.0f * wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target; // RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
    }
}


static void Wheel_Speed_Set(float motorSetSpeed[WHEEL_MOTOR_NUM])
{
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        Motor_Set_Target((MotorInstance *)&wheel_motor[i], motorSetSpeed[i]);
    }
}
float vel_x=0;
void Inverse_to_Track(Mot_base_t Speed_xyw)
{
    
        vel_x = Speed_xyw.vel.x*RADTORPM/150.f;
        // Motor_Set_Target((MotorInstance *)&track_motor[0], vel_x);
        // Motor_Set_Target((MotorInstance *)&track_motor[1], vel_x);


}

void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw)
{
    if (1 == isSetValid_debug[0])
    {
        Omni_Inverse_Kinematics_new(*Speed_xyw, 45.f, motorSetSpeed); // 底盘速度解算到各个电机上
        Omni_Force_Resolve(*Force_xyw, 45.f, motorSetForce);//力逆解算
        //  Force_Allo_to_wheels(motorSetForce, Force_xyw);        // 进行加速度解算到各个电机上
        Inverse_to_Track(*Speed_xyw);
        Get_force_out(motorSetForce); // 获取前馈值
    }
    if (1 == isSetValid_debug[1])
    {
        Wheel_Speed_Set(motorSetSpeed); // 设置电机速度环设定值
    }
}

/*正解算对应函数*/
void Forward_Kinematics(void)
{
    float temp[WHEEL_MOTOR_NUM] = {0};
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        temp[i] = wheel_motor[i].Motor_obj.speed;
    }
    Omni_Forward_Kinematics(temp);
}
