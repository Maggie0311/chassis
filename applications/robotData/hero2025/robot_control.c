/** hero2025 —— 25年老英雄（四轮麦轮）复活
 *  普通构型：4×3508 麦轮 + 本机 6020 云台（yaw/pitch）
 *  参考：2026_infantry_1 干净模板，运动学解算改为麦轮（Mecanum_Inverse_Kinematics）
 *  标 TODO 项需按实车标定（CAN ID / 减速比 / PID / 轮向 reverse_flag 等）
 */

#include "robot_data.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "drv_motionSolve.h"
#include "robot_control.h"
#include "drv_GimMotor.h"
#include "fun_powerCtrl.h"

#define WHEEL_MOTOR_NUM 4
#define GIM_MOTOR_NUM 1

#define WHEEL_RF 0
#define WHEEL_LF 1
#define WHEEL_LH 2
#define WHEEL_RH 3

// 底盘电机实例
DJI_3508_MotorInstance wheel_motor[WHEEL_MOTOR_NUM];
DJI_6020_MotorInstance gim_motor[GIM_MOTOR_NUM];

uint8_t isSetValid_debug[2] = {1, 1}; /*debug: 1为开，0为关*/

/*底盘四个轮期望转速(RPM) 顺序 RF-LF-LB-RB*/
float motorSetSpeed[WHEEL_MOTOR_NUM]; // 经过解算后的数据(速度)
float motorSetForce[WHEEL_MOTOR_NUM]; // 力逆解算输出(扭矩)

// 跟随角pid实例
PIDInstance follow_pid_fast, follow_pid_slow;
PIDInstance fix_speed_x, fix_speed_y;

// 底盘四个麦轮电机配置（TODO: rx_id / reverse_flag / RATIO / PID 按实车标定）
DJI_3508_Motor_Init_Config_s wheel_motor_init_config[WHEEL_MOTOR_NUM] = {
    // 右前轮 RF
    {
        .motor_obj_init_config = {
            .name = "Wheel_RF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE, // TODO: 按实车
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x203, // TODO: 设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.2f,               // TODO: 搭配C620电调，默认减速比约19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 左前轮 LF
    {
        .motor_obj_init_config = {
            .name = "Wheel_LF",
            .reverse_flag = MOTOR_DIRECTION_NORMAL, // TODO: 按实车
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x204, // TODO: 设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207,
            .current_to_torque = 0.3,
            .RATIO = 19.2f, // TODO
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 左后轮 LB
    {
        .motor_obj_init_config = {
            .name = "Wheel_LB",
            .reverse_flag = MOTOR_DIRECTION_NORMAL, // TODO: 按实车
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x201, // TODO: 设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207,
            .current_to_torque = 0.3,
            .RATIO = 19.2f, // TODO
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 右后轮 RB
    {
        .motor_obj_init_config = {
            .name = "Wheel_RB",
            .reverse_flag = MOTOR_DIRECTION_REVERSE, // TODO: 按实车
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 16000, .DeadBand = 0.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x202, // TODO: 设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207,
            .current_to_torque = 0.3,
            .RATIO = 19.2f, // TODO
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
            return ret; // 注册失败直接返回，停止后续电机注册
        }
    }
    return ret;
}

// 云台电机配置（本机 6020 yaw/pitch）TODO: rx_id / 零点 / ctrl_mode 按实车
DJI_6020_Motor_Init_Config_s gim_motor_init_config[GIM_MOTOR_NUM] = {
    // YAW轴电机
    {
        .motor_obj_init_config = {
            .name = "Yaw_M",
            .reverse_flag = MOTOR_DIRECTION_NORMAL, // TODO: 按实车
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
            .rx_id = 0x209, // TODO: 设置ID
            .tx_id = NULL,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI6020_Motor_Measure_Init_Config = {
            .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
            .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
            .RATIO = 1.0,
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 2840, // TODO: 云台yaw零位
            .ctrl_mode = CTRL_MODE_VOLTAGE,
        }
    // PITCH轴电机
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
            return ret;
        }
        ret = Gim_Motor_Attach((MotorInstance *)&gim_motor[i]);
        if (ret != 0)
        {
            return ret;
        }
    }
    return ret;
}

void Follow_pid_init(void)
{
    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

    pid_config.Kp = 30; // TODO: 待整定
    pid_config.Ki = 0;
    pid_config.Kd = 6;
    pid_config.MaxOut = 3000;
    pid_config.DeadBand = 0.1;
    pid_config.IntegralLimit = 600;

    PIDInit(&follow_pid_fast, &pid_config);

    pid_config.Kp = 10; // TODO: 待整定
    pid_config.Ki = 0;
    pid_config.Kd = 15;
    pid_config.MaxOut = 16000;
    pid_config.DeadBand = 0.1;
    pid_config.IntegralLimit = 400;
    PIDInit(&follow_pid_slow, &pid_config);
}

void Fix_Speed_pid_init(void)
{
    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

    pid_config.Kp = 0.4; // TODO: 待整定
    pid_config.Ki = 0.005;
    pid_config.Kd = 0.05;
    pid_config.MaxOut = 2000;
    pid_config.DeadBand = 10;
    pid_config.IntegralLimit = 400;

    PIDInit(&fix_speed_y, &pid_config);

    pid_config.Kp = 0.4;
    pid_config.Ki = 0.005;
    pid_config.Kd = 0.05;
    pid_config.MaxOut = 2000;
    pid_config.DeadBand = 0.05;
    pid_config.IntegralLimit = 400;
    PIDInit(&fix_speed_x, &pid_config);
}

/**
 * @brief 将力逆解算出的各轮扭矩转换为电流前馈
 *        纯转换，无摩擦前馈；后续可按实车补充摩擦/重力前馈
 */
static void Get_force_out(float motorSetForce[WHEEL_MOTOR_NUM])
{
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward =
            motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque
                              / wheel_motor[i].dji3508_motor_measure.current_to_real;
    }
}

static void Wheel_Speed_Set(float motorSetSpeed[WHEEL_MOTOR_NUM])
{
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        Motor_Set_Target((MotorInstance *)&wheel_motor[i], motorSetSpeed[i]);
    }
}

void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw)
{
    if (1 == isSetValid_debug[0])
    {
        Mecanum_Inverse_Kinematics(*Speed_xyw, motorSetSpeed); // 麦轮逆解算
       // Omni_Force_Resolve(*Force_xyw, 45.f, motorSetForce);   // 力逆解算（麦轮与45°全向轮力雅可比等价）
      //  Get_force_out(motorSetForce);                          // 转换为电流前馈
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
    Mecanum_Forward_Kinematics(temp, &real_xyw);
}
