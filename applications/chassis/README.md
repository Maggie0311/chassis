# chassis/ 目录

底盘核心控制层，负责运动控制、运动学解算和电机驱动。

## 文件清单

- `app_ChassisCtrl.c` - 主控制线程（2ms 周期），调度全部控制流水线
- `controlMsg.c` - 多数据源仲裁，管理控制指令优先级
- `mod_motion.c` - 运动模式解析（跟随云台、小陀螺、自主运动等）
- `drv_motionSolve.c` - 全向轮运动学正/逆解算
- `drv_wheel.c` - 电机执行线程（PID + 功率控制 + CAN 发送）
- `protect.c` - 安全保护（离线检测、急停）

## 控制流程

详见 [系统架构文档 - 底盘控制主线程流水线](../chassis_fun/suprCap3/系统架构文档.md#三底盘控制主线程流水线)
