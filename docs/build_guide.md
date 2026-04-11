# 编译构建指引

## 快速编译

**Keil MDK 编译：**
1. 打开 `project.uvprojx`
2. 点击 Build 按钮编译
3. 点击 Download 按钮下载到开发板

**命令行编译：**
```bash
scons
```

## 详细文档

- [编译环境配置](build/environment_setup.md) - 工具链安装和配置
- [多机型编译](build/multi_robot_build.md) - 切换不同机型（哨兵/步兵/英雄）
- [常见编译错误](build/build_errors.md) - 编译问题排查
