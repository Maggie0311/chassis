# ExternalData/ 目录

外部通信层，负责数据收发，不参与控制逻辑。

## 子目录

- [Gimbal/](Gimbal/README.md) - 云台 ↔ 底盘 CAN 通信
- [Refsystem/](Refsystem/README.md) - 裁判系统 UART 通信
- [superCap3/](superCap3/README.md) - 超级电容 CAN 通信
- [Visual/](Visual/README.md) - 视觉/自主运动指令通信（哨兵专用）

## 设计原则

通信层只负责协议解析和数据打包，解析后的数据写入控制层的数据源，由控制层进行仲裁和使用。
