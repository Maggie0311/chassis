# 命令执行指引

## 编译命令

**SCons 编译：**
```bash
cd d:\HERO\hero2026\rt-thread_-hero\bsp\src\2026_chassis_test
scons
```

**清理编译产物：**
```bash
scons -c
```

**重新生成 Keil 工程：**
```bash
scons --target=mdk5
```

## 调试命令

**查看线程状态：**
```
msh> list_thread
```

**查看内存使用：**
```
msh> free
```

## 注意事项

- 使用 Git Bash 或 Windows Terminal 执行命令
- 确保路径中没有中文字符
- 编译前确保已安装 RT-Thread ENV 工具
