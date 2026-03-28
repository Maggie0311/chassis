# 新代码开发指引

## 添加新模块

1. **创建源文件**
   - 在对应目录下创建 `.c` 和 `.h` 文件
   - 应用层代码放在 `applications/` 对应子目录
   - 驱动代码放在 `drivers/`

2. **加入编译系统**
   - 修改对应目录的 `SConscript` 文件
   - 添加源文件路径到编译列表

3. **初始化调用**
   - 在 `applications/Main/main.c` 中添加初始化调用
   - 或创建独立线程

## 开发规范

- 使用 RT-Thread API 进行线程、信号量、消息队列操作
- 遵循现有代码的命名规范
- 添加必要的注释说明模块功能

## 常用操作

- [添加新的控制模式](dev/add_control_mode.md)
- [添加新的通信协议](dev/add_protocol.md)
- [修改功率控制参数](dev/power_control_tuning.md)
