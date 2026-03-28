# Main/ 目录

系统启动入口和全局线程管理。

## 文件清单

- `main.c` - 系统初始化序列，串行拉起所有子系统
- `ThreadCtrl.h` - 全局线程优先级和栈大小常量定义
- `fun_rttMonitor.c` - RT-Thread 系统状态监视器
- `func_MonHandling.c` - 看门狗异常处理（超时复位）

## 启动流程

1. 硬件初始化（时钟、GPIO、外设）
2. RT-Thread 内核启动
3. 依次初始化各子系统模块
4. 创建控制线程和通信线程
5. 启动调度器
