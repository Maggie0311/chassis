---
name: jlink-debug
description: JLink 调试 STM32F407ZG 底盘项目，支持编译、下载、调试、擦除等操作
---

# JLink 调试底盘项目

快速编译、下载和调试 STM32F407ZG 底盘控制系统。

## 快速使用

```bash
# 编译项目
/jlink-debug build

# 下载程序
/jlink-debug download

# 一键编译并下载
/jlink-debug build_and_download

# 启动调试
/jlink-debug debug
```

## 可用命令

### 编译相关
- `build` - 编译项目
- `rebuild` - 清理并重新编译
- `build_and_download` - 编译并下载

### 下载相关
- `download` - 下载程序
- `verify` - 验证下载

### 调试相关
- `debug` - 启动 JLink 调试会话

### 芯片操作
- `erase` - 擦除芯片
- `reset` - 复位单片机
- `status` - 检查连接状态
