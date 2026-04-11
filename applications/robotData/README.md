# robotData/ 目录

机器人参数配置层，按机型隔离配置。

## 子目录

- `sentry_omni/` - 哨兵全向轮配置
- `hero_2026/` - 2026 英雄配置
- `2026_infantry_1/` - 2026 步兵 1 号配置

## 配置内容

每个机型目录包含：
- `robot_data.h` - 硬件参数（轮距、轮径、速度/加速度限幅）
- `robot_control.c` - 电机初始化、功率参数、PID 参数
- `robot_message.c` - CAN ID 映射与电机报文注册

## 切换机型

修改 `SConscript` 引用不同的 robotData 子目录即可。
