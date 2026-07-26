#include "Wheel_data.h"
#include "drv_motor_new.h"
#include "DJI_3508.h"
#include "DJI_6020.h"
#include "LK_MF_MG.h"
#include "drv_motionSolve.h"
#include "robot_control.h"
#include "drv_GimMotor.h"
#include "fun_powerCtrl.h"
#include "drv_SPICAN.h"

#include "drv_IMU_Can.h"
#include "DM_Motor.h"
#include "Wheel_data.h"
#include <arm_math.h>

#define WHEEL_MOTOR_NUM 4
#define UP_MOTOR_NUM 4
#define STEER_MOTOR_NUM 4
#define GIM_MOTOR_NUM 1

#define WHEEL_RF 0
#define WHEEL_LF 1
#define WHEEL_LH 2
#define WHEEL_RH 3

// 底盘电机实例
DJI_3508_MotorInstance wheel_motor[WHEEL_MOTOR_NUM];
DJI_3508_MotorInstance up_motor[UP_MOTOR_NUM];

DJI_6020_MotorInstance steer_motor[STEER_MOTOR_NUM];
DM_MotorInstance gim_motor[GIM_MOTOR_NUM];
extern SecondOrder_Set_Follow_t set_planner_up_motor[UP_MOTOR_NUM];

uint8_t isSetValid_debug[2] = {1, 1};       /*debug*,1为开，0为关*/
float up_motor_zero_offset[UP_MOTOR_NUM]; // 轮毂电机零点
      
      /*底盘四个车轮(或者说轮毂)的期望线速度 单位mm/s 输出顺序RF-LF-LB-RB*/
float motorSetAngle_steer[WHEEL_MOTOR_NUM]; // 经过解算后的数据(速度)
float motorSetSpeed_run[WHEEL_MOTOR_NUM];   // 经过解算后的数据()

float motorSetForce[WHEEL_MOTOR_NUM]; // 经过解算后的数据(加速度)
float debug_feedforward_set_run[WHEEL_MOTOR_NUM];

float motor_speed[UP_MOTOR_NUM];          // 轮向电机速度值
float up_motor_zero_offset[UP_MOTOR_NUM]; // 丝杆电机零点
float up_motor_angle_get[UP_MOTOR_NUM];   // 丝杆电机角度值

uint8_t rear_screw_stall = 0;             // 后丝杆堵转标志，由堵转回调置1，状态机消费后清零

// 跟随角pid实例
PIDInstance follow_pid_fast, follow_pid_slow;
PIDInstance  fix_speed_x,fix_speed_y;
PIDInstance up_machine_pid;
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,

            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x201, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_1,
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x202, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_1,
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
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x203, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_1,
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
            .name = "Wheel_RB",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
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
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Speed_pid = {.Kp = RUN_PID_Kp, .Ki = RUN_PID_Ki, .Kd = RUN_PID_Kd, .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate, .CoefA = 52.0, .CoefB = 2.0, .IntegralLimit = 3000, .MaxOut = 12000, .DeadBand = 1.5},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x204, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_1,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 19.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    }};
uint8_t ZERO_SUCCESS = 0;
void Zero_RF_set(PIDInstance *pid)
{
    static uint8_t init_flag =1;
    if (init_flag)
    {
        up_motor_zero_offset[0] = up_motor[0].Motor_obj.total_angle;
        Motor_Set_Target((MotorInstance *)&up_motor[0], up_motor_zero_offset[0]+5);
        init_flag = 0;
        up_motor[0].Motor_obj.Motor_ctrl.Angle_pid.MaxOut=500;
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        ZERO_SUCCESS++;
    }
}
void Zero_LF_set(PIDInstance *pid)
{
    static uint8_t init_flag =1;
    if (init_flag)
    {
        up_motor_zero_offset[1] = up_motor[1].Motor_obj.total_angle;
        Motor_Set_Target((MotorInstance *)&up_motor[1], up_motor_zero_offset[1]+5);
        init_flag = 0;
        up_motor[1].Motor_obj.Motor_ctrl.Angle_pid.MaxOut=500;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        ZERO_SUCCESS++;
    }
}
void Zero_LB_set(PIDInstance *pid)
{
    static uint8_t init_flag =1;
    if (init_flag)
    {
        up_motor_zero_offset[2] = up_motor[2].Motor_obj.total_angle;
        Motor_Set_Target((MotorInstance *)&up_motor[2], up_motor_zero_offset[2]+5);
        init_flag = 0;
        up_motor[2].Motor_obj.Motor_ctrl.Angle_pid.MaxOut=500;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        ZERO_SUCCESS++;
    }
    rear_screw_stall = 1;
}
void Zero_RB_set(PIDInstance *pid)
{
    static uint8_t init_flag =1;
    if (init_flag)
    {
        up_motor_zero_offset[3] = up_motor[3].Motor_obj.total_angle;
        Motor_Set_Target((MotorInstance *)&up_motor[3], up_motor_zero_offset[3]+5);
        init_flag = 0;
        up_motor[3].Motor_obj.Motor_ctrl.Angle_pid.MaxOut=500;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        ZERO_SUCCESS++;
    }
    rear_screw_stall = 1;
}
uint8_t Zero_Check(void)
{
    return ZERO_SUCCESS;
}


void Zero_Init()
{
   Motor_Set_Target((MotorInstance *)&up_motor[0], -500);
   Motor_Set_Target((MotorInstance *)&up_motor[1], -500);
   Motor_Set_Target((MotorInstance *)&up_motor[2], -500);
   Motor_Set_Target((MotorInstance *)&up_motor[3], -500);
}
// 抬升机构四个电机初始化
//todo 前两轮活动范围外转子450度，后0到五百二十度
DJI_3508_Motor_Init_Config_s up_motor_init_config[UP_MOTOR_NUM] = {
    // 右前抬升
    {
        .motor_obj_init_config = {
            .name = "UP_RF",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_NONE,
            .index = 0,
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
            .Angle_pid = {.Kp = DOWN_PID_Kp_Angle, .Ki = DOWN_PID_Ki_Angle, .Kd = DOWN_PID_Kd_Angle, .Improve = PID_Integral_Limit, .MaxOut = 300},
            .Speed_pid = {.Kp = DOWN_PID_Kp_Speed, .Ki = DOWN_PID_Ki_Speed, .Kd = DOWN_PID_Kd_Speed, .Improve = PID_Integral_Limit  |PID_ErrorHandle, .IntegralLimit = 5000, .MaxOut = 3000, .DeadBand = 0.5,.ErrorHandleCallback = Zero_RF_set,},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16384},
            .Closeloop_Type = SPEED_LOOP|MULTI_ANGLE_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x201, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_2,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 27.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    },
    // 左前抬升
    {
        .motor_obj_init_config = {
            .name = "UP_LF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_NONE,
            .index = 0,
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
            .Angle_pid = {.Kp = DOWN_PID_Kp_Angle, .Ki = DOWN_PID_Ki_Angle, .Kd = DOWN_PID_Kd_Angle, .Improve = PID_Integral_Limit , .MaxOut = 300},
            .Speed_pid = {.Kp = DOWN_PID_Kp_Speed, .Ki = DOWN_PID_Ki_Speed, .Kd = DOWN_PID_Kd_Speed, .Improve = PID_Integral_Limit |PID_ErrorHandle, .IntegralLimit = 5000, .MaxOut =3000, .DeadBand = 0.5,.ErrorHandleCallback=Zero_LF_set},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16384},
            .Closeloop_Type = SPEED_LOOP|MULTI_ANGLE_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x202, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_2,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 27.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },

    },
    // 左后抬升机构
    {
        .motor_obj_init_config = {
            .name = "UP_LB",
            .reverse_flag = MOTOR_DIRECTION_NORMAL,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_NONE,
            .index = 0,
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
            .Angle_pid = {.Kp = DOWN_PID_Kp_Angle, .Ki = DOWN_PID_Ki_Angle, .Kd = DOWN_PID_Kd_Angle, .Improve = PID_Integral_Limit , .MaxOut = 300},
            .Speed_pid = {.Kp = DOWN_PID_Kp_Speed, .Ki = DOWN_PID_Ki_Speed, .Kd = DOWN_PID_Kd_Speed, .Improve = PID_Integral_Limit |PID_ErrorHandle, .IntegralLimit = 3000, .MaxOut = 3000, .DeadBand = 0.5,.ErrorHandleCallback=Zero_LB_set},
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP|MULTI_ANGLE_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x203, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_2,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 27.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },

    },
    // 右后抬升
    {
        .motor_obj_init_config = {
            .name = "UP_RB",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = M3508,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_NONE,
            .index = 0,
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
            .Angle_pid = {.Kp = DOWN_PID_Kp_Angle, .Ki = DOWN_PID_Ki_Angle, .Kd = DOWN_PID_Kd_Angle, .Improve = PID_Integral_Limit , .MaxOut = 300},
            .Speed_pid = {.Kp = DOWN_PID_Kp_Speed, .Ki = DOWN_PID_Ki_Speed, .Kd = DOWN_PID_Kd_Speed, .Improve = PID_Integral_Limit |PID_ErrorHandle,  .IntegralLimit = 5000, .MaxOut = 3000, .DeadBand = 0.5,.ErrorHandleCallback=Zero_RB_set },
            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 16000},
            .Closeloop_Type = SPEED_LOOP|MULTI_ANGLE_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = CURRENT_FEEDFORWARD,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x204, // TODO:设置ID
            .tx_id = 0x200,
            .can_module_callback = DJI_3508_Motor_RefreshData,
            .parent = NULL,
            .spi_slave = SpicanSlave_2,
        },
        .DJI3508_Motor_Measure_Init_Config = {
            .current_to_real = 0.0012207, // 3508电机的电流转换系数20A/16384
            .current_to_torque = 0.3,     // 3508电机的电流转扭矩系数
            .RATIO = 27.0,                // 搭配C620电调，默认减速比为19
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 0.0,
        },
    }};
// 底盘四个舵向电机配置参数模板
DJI_6020_Motor_Init_Config_s steer_motor_init_config[STEER_MOTOR_NUM] = {
    // 右前轮
    {
        .motor_obj_init_config = {
            .name = "steer_RF",
            .reverse_flag = MOTOR_DIRECTION_REVERSE,
            .motor_type = GM6020,
            .state = MOTOR_STATE_NEED_INIT,
            .power_ctrl_type = POWER_CTRL_STEER},
        .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_6020_Motor_Transmit,
        },
        .controller_param_init_config = {
            // todo pid参数
            .Angle_pid = {.Kp = 6.2, .Kd = 3, .Ki = 0, .Improve = PID_Integral_Limit, .IntegralLimit = 35,
                          .MaxOut = 350, // 最大输出单位为RPM,
                          .DeadBand = 0.1,

            },
            .Speed_pid = {
                .Kp = 30,
                .Kd = 1,
                .Ki = 0.1,
                .MaxOut = 8000,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2000,
            },

            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 12000

            },
            .Closeloop_Type = SINGLE_ANGLE_LOOP | SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x205, // TODO:设置ID
            .tx_id = 0x1FE,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI6020_Motor_Measure_Init_Config = {
            .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
            .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
            .RATIO = 1.0,
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 2257.0,
            .ctrl_mode = CTRL_MODE_CURRENT,
        }},

    // 左前
    {
        .motor_obj_init_config = {.name = "steer_LF", .reverse_flag = MOTOR_DIRECTION_REVERSE, .motor_type = GM6020, .state = MOTOR_STATE_NEED_INIT, .power_ctrl_type = POWER_CTRL_STEER},
        .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_6020_Motor_Transmit,
        },
        .controller_param_init_config = {
            // todo pid参数
            .Angle_pid = {
                .Kp = 6.2, .Kd = 3, .Ki = 0, .Improve = PID_Integral_Limit, .IntegralLimit = 40,
                .MaxOut = 350 ,// 最大输出单位为RPM
                .DeadBand = 0.1
            },
            .Speed_pid = {
                .Kp = 32,
                .Kd = 1,
                .Ki = 0.2,
                .MaxOut = 12000,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1000,
            },

            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 12000

            },
            .Closeloop_Type = SINGLE_ANGLE_LOOP | SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x206, // TODO:设置ID
            .tx_id = 0x1FE,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI6020_Motor_Measure_Init_Config = {
            .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
            .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
            .RATIO = 1.0,
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 2239.0,
            .ctrl_mode = CTRL_MODE_CURRENT,
        },
    },
    // 左后
    {
        .motor_obj_init_config = {.name = "steer_LB", .reverse_flag = MOTOR_DIRECTION_REVERSE, .motor_type = GM6020, .state = MOTOR_STATE_NEED_INIT, .power_ctrl_type = POWER_CTRL_STEER},
        .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_6020_Motor_Transmit,
        },
        .controller_param_init_config = {
            // todo pid参数
            .Angle_pid = {
                .Kp = 6.2, .Kd = 6, .Ki = 0, .Improve = PID_Integral_Limit, .IntegralLimit = 40,
                .MaxOut = 350 ,// 最大输出单位为RPM
                .DeadBand = 0.1,
            },
            .Speed_pid = {
                .Kp = 35,
                .Kd = 1,
                .Ki = 0.3,
                .MaxOut = 9000,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2000,
            },

            .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 12000

            },
            .Closeloop_Type = SINGLE_ANGLE_LOOP | SPEED_LOOP,
            .Calc_Pre = {
                .speed_calc_Prescaler = 1,
                .angle_calc_Prescaler = 2,
            },
            .Feedforward_Type = FEEDFORWARD_NONE,
        },
        .can_init_config = {
            .can_device_name = "can1",
            .rx_id = 0x207, // TODO:设置ID
            .tx_id = 0x1FE,
            .can_module_callback = DJI_6020_Motor_RefreshData,
            .parent = NULL,
        },
        .DJI6020_Motor_Measure_Init_Config = {
            .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
            .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
            .RATIO = 1.0,
            .ENCODER_LEN = 8192,
            .ENCODER_ZERO_OFFSET = 5685.0,
            .ctrl_mode = CTRL_MODE_CURRENT,
        },
    },
    // 右后
    {.motor_obj_init_config = {.name = "steer_RB", .reverse_flag = MOTOR_DIRECTION_REVERSE, .motor_type = GM6020, .state = MOTOR_STATE_NEED_INIT, .power_ctrl_type = POWER_CTRL_STEER}, .Motor_operate_setting = {
            .Motor_Init = DJI_6020_Motor_Init,
            .Motor_Enable = DJI_6020_Motor_Enable,
            .Motor_Disable = DJI_6020_Motor_Disable,
            .Motor_ErrorHandle = DJI_6020_Motor_ErrorHandle,
            .Motor_RefreshData = DJI_6020_Motor_RefreshData,
            .Motor_Control = NULL,
            .Motor_Transmit = DJI_6020_Motor_Transmit,
                                                                                                                                                                                        },
     .controller_param_init_config = {
         // todo pid参数
         .Angle_pid = {
             .Kp = 6.2, .Kd = 6, .Ki = 0, .Improve = PID_Integral_Limit, .IntegralLimit = 40,
             .MaxOut = 350 ,// 最大输出单位为RPM,
             .DeadBand = 0.1
         },
         .Speed_pid = {
					 
             .Kp = 38,
             .Kd = 1,
             .Ki = 0.3,
             .MaxOut = 12000,
             .Improve = PID_Integral_Limit,
             .IntegralLimit = 2500,
         },

         .Current_pid = {.Kp = 0, .Kd = 0, .Ki = 0, .MaxOut = 12000

         },
         .Closeloop_Type = SINGLE_ANGLE_LOOP | SPEED_LOOP,
         .Calc_Pre = {
             .speed_calc_Prescaler = 1,
             .angle_calc_Prescaler = 2,
         },
         .Feedforward_Type = FEEDFORWARD_NONE,
     },
     .can_init_config = {
         .can_device_name = "can1",
         .rx_id = 0x208, // TODO:设置ID
         .tx_id = 0x1FE,
         .can_module_callback = DJI_6020_Motor_RefreshData,
         .parent = NULL,
     },
     .DJI6020_Motor_Measure_Init_Config = {
         .current_to_real = 0.0001831, // 6020电机的电流转换系数3A/16384
         .current_to_torque = 0.741,   // 6020电机的电流转扭矩系数
         .RATIO = 1.0,
         .ENCODER_LEN = 8192,
         .ENCODER_ZERO_OFFSET = 4494.0,
         .ctrl_mode = CTRL_MODE_CURRENT,
     }}};
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

    for (int i = 0; i < STEER_MOTOR_NUM; i++)
    {
        ret = Motor_Attach((MotorInstance *)&steer_motor[i], (Motor_Init_Config_s *)&steer_motor_init_config[i]);
        Power_Motor_Attach((MotorInstance *)&steer_motor[i]);
        if (ret != 0)
        {
            return ret; // 如果注册失败将直接返回错误，停止后续电机注册
        }
    }
    // todo 抬升机构
    for(int i=0;i<UP_MOTOR_NUM;i++)
    {
        ret = Motor_Attach((MotorInstance *)&up_motor[i], (Motor_Init_Config_s *)&up_motor_init_config[i]);
        if (ret != 0)
        {
            return ret; // 如果注册失败将直接返回错误，停止后续电机注册
        }
    }
     return ret;
}


DM_Motor_Init_Config_s gim_dm_motor_init_config[GIM_MOTOR_NUM] =
    {
        {
            .motor_obj_init_config = {
                .name = "Yaw_M",
                .reverse_flag = MOTOR_DIRECTION_NORMAL,
                .motor_type = DM4310,
                .state = MOTOR_STATE_NOTCTRL,
            },
            .Motor_operate_setting = {
                .Motor_Init = DM_Motor_Init,
                .Motor_Enable = DM_Motor_Enable,
                .Motor_Disable = DM_Motor_Disable,
                .Motor_ErrorHandle = DM_Motor_ErrorHandle,
                .Motor_RefreshData = DM_Motor_RefreshData,
                .Motor_Control = NULL,
                .Motor_Transmit = NULL,
            },
        .controller_param_init_config = NULL,

            .can_init_config = {
                .can_device_name = "can2",
                .rx_id = 0x02,
                .tx_id = NULL,
                .can_module_callback = DM_Motor_RefreshData,
                .parent = NULL,
                .spi_slave=NULL
            },
            .DM_Motor_Measure_Init_Config = {
                .RATIO = 1.0,
                .ENCODER_LEN = 65535,
                .ENCODER_ZERO_OFFSET = 60025,
                .DM_P_MAX = 3.1415926f,
                .DM_P_MIN = -3.1415926f,
                .DM_V_MAX = 45.f,
                .DM_V_MIN = -45.f,
                .DM_T_MAX = 18.f,
                .DM_T_MIN = -18.f,
            },
        }

};


int gim_motor_init()
{
    int ret = 0;
    for (int i = 0; i < GIM_MOTOR_NUM; i++)
    {
        ret = Motor_Attach((MotorInstance *)&gim_motor[i], (Motor_Init_Config_s *)&gim_dm_motor_init_config[i]);
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


/**
 * @brief 丝杆电机移动，包含对零点操作，zero_flag为1时进行零点操作
 *
 * @param motor
 * @param move
 * @param zero_flag
 */
void Screw_Motor_Move(Screw_Motor_e motor, float target,uint8_t up_or_down)
{
   if(motor == Forward_Screw_Motor)
    {
        SecondOrder_Set_Follow_Cal(&set_planner_up_motor[0], target, 0.0f);
        SecondOrder_Set_Follow_Cal(&set_planner_up_motor[1], target, 0.0f); 
    }
    else
   { 
       SecondOrder_Set_Follow_Cal(&set_planner_up_motor[2], target, 0.0f); 
       SecondOrder_Set_Follow_Cal(&set_planner_up_motor[3], target, 0.0f); 
    }
    if (motor == Forward_Screw_Motor&&up_or_down==UP)
     {
        up_motor[0].Motor_obj.Motor_ctrl.Angle_pid.Kp=UP_PID_Kp_Angle;
        up_motor[0].Motor_obj.Motor_ctrl.Angle_pid.Kd=UP_PID_Kd_Angle;
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.Kp=UP_PID_Kp_Speed;
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.Ki=UP_PID_Ki_Speed;

        up_motor[1].Motor_obj.Motor_ctrl.Angle_pid.Kp=UP_PID_Kp_Angle-10;
        up_motor[1].Motor_obj.Motor_ctrl.Angle_pid.Kd=UP_PID_Kd_Angle-1;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.Kp=UP_PID_Kp_Speed-5;  
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.Ki=UP_PID_Ki_Speed;

        
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
        Motor_Set_Target((MotorInstance *)&up_motor[0], set_planner_up_motor[0].p+ up_motor_zero_offset[0]); // TODO
        Motor_Set_Target((MotorInstance *)&up_motor[1], set_planner_up_motor[1].p+ up_motor_zero_offset[1]); // TODO
    }

    else if(motor == Forward_Screw_Motor&&up_or_down==DOWN)
     { 
        up_motor[0].Motor_obj.Motor_ctrl.Angle_pid.Kp=DOWN_PID_Kp_Angle;
        up_motor[0].Motor_obj.Motor_ctrl.Angle_pid.Kd=DOWN_PID_Kd_Angle;
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.Kp=DOWN_PID_Kp_Speed;
        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.Ki=DOWN_PID_Ki_Speed;

        up_motor[1].Motor_obj.Motor_ctrl.Angle_pid.Kp=DOWN_PID_Kp_Angle;        
        up_motor[1].Motor_obj.Motor_ctrl.Angle_pid.Kd=DOWN_PID_Kd_Angle;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.Kp=DOWN_PID_Kp_Speed;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.Ki=DOWN_PID_Ki_Speed;

        up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=2000;
        up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=2000;
        Motor_Set_Target((MotorInstance *)&up_motor[0], set_planner_up_motor[0].p+ up_motor_zero_offset[0]); // TODO
        Motor_Set_Target((MotorInstance *)&up_motor[1], set_planner_up_motor[1].p+ up_motor_zero_offset[1]); // TODO
    }
    else if (motor == Back_Screw_Motor&&up_or_down==UP)
    {
        up_motor[2].Motor_obj.Motor_ctrl.Angle_pid.Kp=UP_PID_Kp_Angle-8;
        up_motor[2].Motor_obj.Motor_ctrl.Angle_pid.Kd=UP_PID_Kd_Angle;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.Kp=UP_PID_Kp_Speed-5;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.Ki=UP_PID_Ki_Speed;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16384;
        up_motor[3].Motor_obj.Motor_ctrl.Angle_pid.Kp=UP_PID_Kp_Angle-8;
        up_motor[3].Motor_obj.Motor_ctrl.Angle_pid.Kd=UP_PID_Kd_Angle;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.Kp=UP_PID_Kp_Speed-5;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.Ki=UP_PID_Ki_Speed;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16384;
        Motor_Set_Target((MotorInstance *)&up_motor[2], set_planner_up_motor[2].p+ up_motor_zero_offset[2]); // TODO
        Motor_Set_Target((MotorInstance *)&up_motor[3], set_planner_up_motor[3].p+ up_motor_zero_offset[3]); // TODO
    }
    else if(motor == Back_Screw_Motor&&up_or_down==DOWN)
    {
        up_motor[2].Motor_obj.Motor_ctrl.Angle_pid.Kp=DOWN_PID_Kp_Angle+20;
        up_motor[2].Motor_obj.Motor_ctrl.Angle_pid.Kd=DOWN_PID_Kd_Angle;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.Kp=DOWN_PID_Kp_Speed+5;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.Ki=DOWN_PID_Ki_Speed;
        up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=3000;

        up_motor[3].Motor_obj.Motor_ctrl.Angle_pid.Kp=DOWN_PID_Kp_Angle+20;        
        up_motor[3].Motor_obj.Motor_ctrl.Angle_pid.Kd=DOWN_PID_Kd_Angle;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.Kp=DOWN_PID_Kp_Speed+5;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.Ki=DOWN_PID_Ki_Speed;
        up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=3000;
        Motor_Set_Target((MotorInstance *)&up_motor[2], set_planner_up_motor[2].p+ up_motor_zero_offset[2]); // TODO
        Motor_Set_Target((MotorInstance *)&up_motor[3], set_planner_up_motor[3].p+ up_motor_zero_offset[3]); // TODO
    }
}

int If_Ready_To_Down(float target)
{
 
   up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=1000;
   up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=1000;
//     Motor_Set_Target((MotorInstance *)&up_motor[2], -400);
//    Motor_Set_Target((MotorInstance *)&up_motor[3], -400);
   if(fabsf(up_motor[2].Motor_obj.total_angle-target-up_motor_zero_offset[2]) < 300.0f&&fabsf(up_motor[2].Motor_obj.total_angle-target-up_motor_zero_offset[2]) <300.f)
   {
     up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=10000;
     up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=10000;
     return 1;
   }
   else 
   return 0;
}

int If_Ready_To_Down_F(float target)
{
 
   up_motor[0].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=800;
   up_motor[1].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=800;
//     Motor_Set_Target((MotorInstance *)&up_motor[2], -400);
//    Motor_Set_Target((MotorInstance *)&up_motor[3], -400);
   if(fabsf(up_motor[0].Motor_obj.total_angle-target-up_motor_zero_offset[0]) < 280.0f&&fabsf(up_motor[1].Motor_obj.total_angle-target-up_motor_zero_offset[1]) <280.f)
   {
     up_motor[2].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
     up_motor[3].Motor_obj.Motor_ctrl.Speed_pid.MaxOut=16000;
     return 1;
   }
   else 
   return 0;
}


/**
 * @brief 判断电机是否到达目标角度
 *
 * @param motor 电机实例
 * @param move 电机移动方向
 * @return int 1为到达目标角度，0为未到达目标角度
 */
int If_Target_Reached(Screw_Motor_e motor,float target)
{
    if (motor == Forward_Screw_Motor)
    {
        if (fabsf(up_motor[0].Motor_obj.total_angle-target-up_motor_zero_offset[0]) < 50.0f && fabsf(target+up_motor_zero_offset[1]-up_motor[1].Motor_obj.total_angle) < 50.0f)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else if (motor == Back_Screw_Motor)
{
   if (fabsf(up_motor[2].Motor_obj.total_angle-target-up_motor_zero_offset[2]) < 50.0f && fabsf(target+up_motor_zero_offset[3]-up_motor[3].Motor_obj.total_angle) < 50.0f)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}
// 读取轮向电机速度值并积分得到位移
//float position=0;
static rt_tick_t pos_last_tick = 0;
static float pos_intigral[2] = {0};
void Motor_Speed_Read()
{
    for (int i = 0; i < UP_MOTOR_NUM; i++)
    {
        motor_speed[i] = 3.1416f*wheel_motor[i].Motor_obj.speed*WHEEL_RADIUS*2.f/60.f;
    }

    rt_tick_t now_tick = rt_tick_get();
    if (pos_last_tick == 0)
    {
        pos_last_tick = now_tick;
        return;
    }
    float dt = (now_tick - pos_last_tick) / 1000.0f;
    pos_last_tick = now_tick;

    pos_intigral[0] += motor_speed[2] * dt;
    pos_intigral[1] += motor_speed[3] * dt;
    //position = (pos_intigral[0] + pos_intigral[1]) / 2.0f;
}

// 读取抬升机构电机角度值
void Up_Motor_Angle_Read()
{
    for (int i = 0; i < UP_MOTOR_NUM; i++)
    {
        up_motor_angle_get[i] = up_motor[i].Motor_obj.total_angle;
    }
}

// 抬升机构对零点

void Follow_pid_init(void)
{
    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

  
    pid_config.Kp = 40;
    pid_config.Ki = 0.05;
    pid_config.Kd = 10;
    pid_config.MaxOut = 8000;
    pid_config.DeadBand = 5;
    pid_config.IntegralLimit = 400;

    PIDInit(&follow_pid_fast, &pid_config);

    pid_config.Kp = 20;
    pid_config.Ki = 0;
    pid_config.Kd = 10;
    pid_config.MaxOut = 11000;
    pid_config.DeadBand = 0.5;
    pid_config.IntegralLimit = 400;
    PIDInit(&follow_pid_slow, &pid_config);
}

void Fix_Speed_pid_init(void)
{
    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

    // 有新息前馈分担快速响应，PID只负责消除稳态误差
    // Ki 降低 + 积分限幅收紧，避免打滑结束后积分回退造成"先正后反"
    pid_config.Kp = 0.4;
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

// 抬升机构pid初始化
void Up_Machine_pid_init(void)
{

    PID_Init_Config_s pid_config;
    rt_memset(&pid_config, 0, sizeof(pid_config));
    pid_config.Improve = PID_Integral_Limit | PID_ChangingIntegrationRate;

    pid_config.Kp = 28; // TODO
    pid_config.Ki = 0.01;
    pid_config.Kd = 5;
    pid_config.MaxOut = 800;
    pid_config.DeadBand = 0;
    pid_config.IntegralLimit = 0;
    PIDInit(&up_machine_pid, &pid_config);
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

float debug_feedforward_set_run[WHEEL_MOTOR_NUM] = {0};
float debug_kb = 0.01f;
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
/**
 * @brief 将雅可比矩阵解算出的电机的扭矩转换为电流
 *
 * @param motorSetForce
 * @param force_out
 */
static void Get_force_out(float motorSetForce[WHEEL_MOTOR_NUM])
{
    // for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    // {
    //     wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 暂时不加摩擦
    //     // debug_feedforward_set_run[i] = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real; // 转换为电机的扭矩

    //     // + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target);暂时不加摩擦
    // }

     float k_back = 1.0f + debug_kb * fabsf(imu_can_data.pitch_angle);
    k_back = CLAMP(k_back, 1.0f, 1.5f);
    // for (int i = 0; i < 2; i++)
    // {
    //     wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f +RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
    // }
    // for (int i = 2; i < WHEEL_MOTOR_NUM; i++)
    // {
    //     wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = motorSetForce[i] * k_back;
    // }
    for (int i = 0; i < 2; i++)
    {
        if(wheel_motor[i].Motor_obj.Motor_ctrl.Current_pid.Output>0)
        {
            debug_feedforward_set_run[i] = motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f * (2.0f - k_back) ;//+ RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = fabsf(motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f * (2.0f - k_back));// + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
        }
        else 
            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = -fabsf(motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f * (2.0f - k_back));// + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
    }
    for (int i = 2; i < WHEEL_MOTOR_NUM; i++)
    {
         if(wheel_motor[i].Motor_obj.Motor_ctrl.Current_pid.Output>0)
        {
            debug_feedforward_set_run[i]=motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f *  k_back ;//+ RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = fabsf(motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f *  k_back);// + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
        }
        else 
            wheel_motor[i].Motor_obj.Motor_ctrl.feedforward.current_feedforward = -fabsf(motorSetForce[i] / wheel_motor[i].dji3508_motor_measure.current_to_torque / wheel_motor[i].dji3508_motor_measure.current_to_real * WHEEL_RADIUS / 1000.f *  k_back);// + RUN_wheel_forward_cal(wheel_motor[i].Motor_obj.Motor_ctrl.Speed_pid.Target); // 转换为电机的扭矩
    }
}

static void Wheel_Speed_Set(float motorSetSpeed_run[WHEEL_MOTOR_NUM], float motorSetAngle_steer[WHEEL_MOTOR_NUM])
{
    for (int i = 0; i < WHEEL_MOTOR_NUM; i++)
    {
        float cos_factor = arm_cos_f32(steer_motor[i].Motor_obj.Motor_ctrl.Angle_pid.Err * 0.0174533);
        if (cos_factor < 0.0f)
            cos_factor = 0.0f; // 舵向误差>90°时不反转轮速，轮速方向由运动学解算器决定
        motorSetSpeed_run[i] *= cos_factor;
        Motor_Set_Target((MotorInstance *)&wheel_motor[i], motorSetSpeed_run[i]);
        Motor_Set_Target((MotorInstance *)&steer_motor[i], motorSetAngle_steer[i]);
    }
}
float debug_gravity_ff =0;
float gravity_feedforward(Mot_base_t Force_xyw)
{
    debug_gravity_ff = WEIGHT * imu_can_data.z_accel * 10.0f * arm_sin_f32(imu_can_data.pitch_angle * DEGTORAD) * 1000.0f; // 单位mN
    return debug_gravity_ff;
}
//**
void chassis_ctrl(Mot_base_t *Speed_xyw, Mot_base_t *Force_xyw,uint8_t motion_mode)
{
    if (1 == isSetValid_debug[0])
    {
        Steer_Inverse_Kinematics_test(*Speed_xyw, motorSetSpeed_run, motorSetAngle_steer); // 舵轮逆解算
        //Steer_Force_Resolve(*Force_xyw, motorSetForce);                                    // 进行力解算到各个电机上，输出为扭矩

        //Get_force_out(motorSetForce); // 获取前馈值

        if(If_Target_Reached(Back_Screw_Motor,0)&&If_Target_Reached(Forward_Screw_Motor,0))
        {
        float gravity_ff = gravity_feedforward(*Force_xyw);//加上重力前馈
        Force_xyw->vel.x += gravity_ff;
        Steer_Force_Resolve(*Force_xyw, motorSetForce);//力逆解算
        Get_force_out(motorSetForce); // 获取前馈值
        }
    }
    if (1 == isSetValid_debug[1])
    {
        Wheel_Speed_Set(motorSetSpeed_run, motorSetAngle_steer); // 设置电机速度环设定值
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
    // Omni_Forward_Kinematics(temp);
}

void chasis_dangling_x(void)
{
     Motor_Set_Target((MotorInstance *)&steer_motor[0],-45 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[1],45 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[2],-45 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[3],45 ); 
}

void chasis_zero_x(void)
{
     Motor_Set_Target((MotorInstance *)&steer_motor[0],0 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[1],0 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[2],0 ); 
     Motor_Set_Target((MotorInstance *)&steer_motor[3],0 ); 
}