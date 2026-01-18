# 尝试整理一下云台通信
## 软件构架
### 旧软件架构
- `drv_gimCtrlRX`：云台通信回调，还有一些对外的数据读取接口函数  
- `drv_GimMotor`: 云台电机的解算等  
- `fun_protocolGim` :云台IMU（？）的数据处理  
- `mod_GimbalComTx` :云台需要的数据的获取与发送  
### 新软件架构
- `drv_Gimbal_Com` ：底层收发
- ：数据处理（获取）
- ：数据处理（解算）
## 云台cam通信整理
### Tx
Tx中需要分别发送两部分信息，枪口信息与杂项信息，其中枪口信息需要向裁判系统获取（可以考虑注册回调函数直接更新），其实杂项信息也是（）
```c
typedef __packed struct
{
    // 枪口状态数据
    uint16_t cooling_heat     // 枪口热量
    uint16_t cooling_rate     // 枪口每秒冷却值
    uint16_t cooling_limit    // 枪口热量上限
    uint16_t speed_limit      // 枪口速度上限
    uint16_t cooling_buff     // 枪口冷却具体值（直接值，值为 x 表示热量冷却增加 x/s）（26改）
} Gim_Shooter_t;        // 发送给云台的枪口数据

typedef __packed struct
{
    // 杂项数据
    unsigned isStartGame : 1;   // 是否开始比赛
    unsigned hurtID : 3;        // 装甲板ID，未受攻击时为FF
    unsigned tempColor : 1;     // 红蓝方
    unsigned ourOutpost : 1;    // 己方前哨战是否存活
    unsigned enermyOutpost : 1; // 敌方前哨战是否存活
#ifdef CORE_USING_SENTRY
    unsigned SentryIsFire : 1;
#endif
    unsigned speed : 16;        // 弹速
    unsigned SentryHP : 16;     // 哨兵的血量
//!:烧饼超字节了，往上挪了挪
    uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
    uint16_t target_robot_ID; // 要作用的目标机器人 ID           当发送位置信息时，该项为 0
} Gim_Reftrans_t;        // 发送给云台的裁判系统数据

```
### Rx
- **将报文写入控制源数据`Ctrl_data_info_t gimbal_ctrl`，解算好像也扔进去了一点（好像是角度换算要用）**
```c
/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg)
{
    SWDG_FEED(gimRxMonId);
    gimbal_ctrl.vel_x = (float)MSG_READ_2BYTES(0, 1);
    gimbal_ctrl.vel_y = (float)MSG_READ_2BYTES(2, 3);
    gimbal_ctrl.mode = (Motion_mode_e)(msg->data[6]);

    // 云台跟随角外部修改
    if (gimbal_ctrl.mode == NO_FOLLOW)
    {
        gimbal_ctrl.fol_ang = 0;
        gimbal_ctrl.angvel = 0;
    }
    if (gimbal_ctrl.mode == FOLLOW_GIMBAL)
    {
        // gimbal_ctrl.fol_ang = (float)MSG_READ_2BYTES(4, 5) / 8192.0f * 360.0f; // 云台端通信跟随角单位为8192(对应360°)
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(4, 5); // 云台发送角速度单位0.1度/s，逻辑为在跟随模式下，云台直接发送YAW轴角速度值给底盘
        gimbal_ctrl.fol_ang = 0;
    }
    else if (gimbal_ctrl.mode == SLOW_GYRO || gimbal_ctrl.mode == FAST_GYRO ||gimbal_ctrl.mode == CHANGE_GYRO)
    {
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(4, 5);
        gimbal_ctrl.fol_ang = 0;
    }
    MaxPower = msg->data[7];

    Source_WriteData_s(CS_GIMBAL, &gimbal_ctrl);

    gimbal_ctrl.getYawAngel = (getYawAngelCallBack)Get_GM_ANGLE_360f;
    gimbal_ctrl.arg = GM_YAW;
    gimbal_ctrl.folIsReverse = !YAW_ANGLE_REVERSAL;                  /*当底盘在云台的顺时针方向角度为正所以跟随时反向*/
    Package_Write(dataNum[Gim_Ctrl], gimbal_ctrl); // 更新数据服务器的数据
}

/***
 * @brief    处理云台发送的各类数据报文
 * @param    msg     can2报文
 ***/
void Refresh_Gimbaldata(struct rt_can_msg *msg)
{
    SWDG_FEED(gimRxMonId);
    gimbal_msg_freshtick = rt_tick_get();

    if (gimRxDebug)
        rt_memcpy(&gimbal_msg, msg->data, sizeof(gimbal_msg) < 8 ? sizeof(gimbal_msg) : 8);
    Package_Write(dataNum[Gim_data], gimbal_msg); // 更新数据服务器的数据
    if (gimbal_msg.chassis_reset)
        Robot_Chassis_Reset(RT_TRUE);
}
// 接收云台算出的底盘姿态角度数据
void Refresh_Chassis_Gimbalcaled_IMU_Data(struct rt_can_msg *msg)
{
    chassis_imu.pitch = *((float *)&msg->data[0]);
    chassis_imu.roll = *((float *)&msg->data[4]);
    imu_data_freshtick = rt_tick_get();
}

/***
 * @brief CAN回调-更新云台电机数据，并转化为底盘坐标系下可用的数据
 * @note  在俯视图底盘坐标系(x轴正方向为右手方向，y轴正方向为正前方)下，
          云台和底盘正方向重合yaw轴电机角度数据转为0，yaw轴电机逆时针旋转,其速度转化为正
          修正后角度数据范围(-8192/2,8192/2](3508/6020)|(-65536/2,65536/2](DM) 速度范围不变
 * @note  pitch轴编码器数值：云台枪管往上抬 >0，云台枪管水平时 =0.
 * @param message 云台yaw or pitch电机报文
 ***/
void Refresh_GimbalMotor(struct rt_can_msg *message)
{
    switch (message->id)
    {
    case YAW_MOTOR_RXID:
#ifdef YAW_MOTOR_6020
        Assign_Motor_Data(&yaw_data, message);
        Fix_GimbalMotor(&yaw_data, YAW_ZERO_ANGLE, YAW_ANGLE_REVERSAL);
#endif
#ifdef YAW_MOTOR_DM
        Assign_Motor_Data_DM(&yaw_data, message);
        Fix_GimbalMotor(&yaw_data, YAW_ZERO_ANGLE, YAW_ANGLE_REVERSAL);
#endif
        break;

    case PITCH_MOTOR_RXID:
        Assign_Motor_Data(&pitch_data, message);
        Fix_GimbalMotor(&pitch_data, PITCH_ZERO_ANGLE, PITCH_ANGLE_REVERSAL);
        break;

    default:
        break;
    }
}

typedef __packed struct
{
    // 工作状态数据
    unsigned ui_reset : 1;                // 重置 UI 标志位
    unsigned chassis_reset : 1;           // 底盘复位指令
    unsigned current_menu : 4;            // 当前所处菜单
    unsigned motion_mode : 3;             // 底盘模式

    unsigned if_aimbot_best_dis : 1;      // 当前是否为云台认为的最佳射击距离
    unsigned strike_mode : 1;             // 发射机构模式(高/低射频)
    unsigned magazine_status : 1;         // 弹舱盖设定状态
    unsigned heatlimit_status : 1;        // 热量状态(是否开启热量限制)
    unsigned aimbot_mode : 3;             // 瞄准模式

    unsigned self_color : 1;              // 当前己方颜色
    unsigned GimIfSuperCapOutput : 1;  // 底盘功率限制(是否开启超级电容电量自动节省)(C键)
    unsigned rub_started : 1;             // 摩擦轮已开启
    unsigned now_client_control : 1;      // 当前使用客户端模式
    unsigned capacity_close_flag : 1;     // 当前是否关闭超级电容充电(未使用)
    unsigned force_refsystem_offline : 1; // 当前是否强制裁判系统离线
    unsigned set_chassis_mode : 2;        // 当前设置的本地底盘模式

    unsigned set_ammobooster_mode : 2;    // 当前设置的本地发射机构模式
    unsigned visual_target_found : 2;     // 视觉是否找到目标（用于在UI上提示操作手是否找到目标）
    unsigned Energy_is_already : 1;       // 能量机关拟合完成标志位
    unsigned Wire_less_charge : 1;        // 无线充电标志位（1打开无线充电，0关闭无线充电）
    unsigned reserve : 4;                 // 保留位
    
    unsigned Gimbal_if_reset : 1;         // 云台是否发生了复位事件
    unsigned Key_ctrl : 1;                // ctrl 键是否按下-按下时采用高的加速度值
    // 离线模块数据
    unsigned visual_com_online : 1;      // 视觉通信是否正常
    unsigned visual_working_correct : 1; // 视觉工作是否正常
    unsigned yaw_motor_online : 1;       // Yaw 轴电机通信正常
    unsigned pitch_motor_online : 1;     // Pitch 轴电机通信正常

    unsigned right_rub_motor_online : 1; // 右侧摩擦轮电机通信正常
    unsigned left_rub_motor_online : 1;  // 左侧摩擦轮电机通信正常
    unsigned launch_motor_online : 1;    // 播弹盘电机通信正常
    unsigned strike_stuck : 1;           // 发射机构卡弹
#ifdef CORE_USING_SENTRY
    unsigned sentryMode : 1; // 哨兵模式
#endif
} gimbal_msg_t; // 杂项数据包
```
## 裁判系统数据获取整理
```c

```
## 云台电机相关

```c
static Motordata_t pitch_data; // pitch轴当前数据（以水平为0度
Motordata_t yaw_data;          // yaw轴数据

typedef struct
{
    rt_uint32_t angle;      // 转子角度，大小范围[0~8191]6020[0~65535]DM
    rt_int16_t speed;       // 转子转速，单位rpm
    rt_int16_t current;     // 实际转矩电流
    rt_uint8_t temperature; // 电机的实际温度

    rt_int32_t fix_angle;      // 修正过后的角度，大小范围(-4096~4096]6020(-32768~32768]DM
    rt_int32_t filterFixangle; // 滤波修正过后的角度，大小范围(-4096~4096]6020(-32768~32768]DM
    rt_tick_t FreshTick;       // 数据更新的时刻
    float speed_convers;       // 电机反馈报文转换为RPM的速度
} Motordata_t;
```
## 解算整理
### 旧


