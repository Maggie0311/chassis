# board/ 目录

板级配置文件，包含硬件初始化和链接脚本。

## 文件清单

- `board.c` - 板级初始化代码
- `board.h` - 板级配置头文件
- `CubeMX_Config/` - STM32CubeMX 配置文件
- `linker_scripts/` - 链接脚本

## 功能说明

负责 MCU 时钟配置、GPIO 初始化、外设时钟使能等底层硬件初始化。
