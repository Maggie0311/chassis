# 代码探索指引

代码按功能模块组织，每个目录负责不同的功能层：

## 核心目录

- [applications/](../applications/README.md) - 应用层代码
  - Main/ - 系统启动和线程管理
  - chassis/ - 底盘控制核心
  - ExternalData/ - 外部通信（云台、裁判系统、超电、视觉）
  - chassis_fun/ - 功能算法（功率控制、向量运算）
  - robotData/ - 机器人参数配置
  - UI/ - 裁判系统 UI 绘制

- [drivers/](../drivers/README.md) - 驱动层代码

- [board/](../board/README.md) - 板级配置

## 系统架构

详细的系统架构说明见：
- [哨兵底盘系统架构](../applications/chassis_fun/suprCap3/系统架构文档.md)
- [功率控制理论文档](../applications/chassis_fun/suprCap3/完整形态文档.md)
