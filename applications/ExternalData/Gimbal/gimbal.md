# 尝试整理一下云台通信
## 软件构架
- `drv_gimCtrlRX`：云台通信回调，还有一些对外的数据读取接口函数  
- `drv_GimMotor`: 云台电机的解算等  
- `fun_protocolGim` :云台IMU（？）的数据处理  
- `mod_GimbalComTx` :云台需要的数据的获取与发送  
## 通信整理
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
    unsigned speed : 16;        // 弹速
    unsigned SentryHP : 16;     // 哨兵的血量
#ifdef CORE_USING_SENTRY
    unsigned SentryIsFire : 1;
#endif
    uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息 无按键按下则为 0
    uint16_t target_robot_ID; // 要作用的目标机器人 ID           当发送位置信息时，该项为 0
} Gim_Mixed_t;        // 发送给云台的杂项数据
```