# Gimbal/ 目录

云台与底盘的 CAN 通信模块。

## 文件清单

- `drv_GimMotor.c` - 云台电机数据接收（yaw/pitch 角度、速度）
- `drv_Gimbal_Com.c` - CAN 帧打包和解包
- `fun_protocolGim.c` - 云台协议解析，输出控制数据到控制层

## 通信内容

- 接收：云台 yaw 轴角度、角速度
- 发送：底盘运动状态反馈
