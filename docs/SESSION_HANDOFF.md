# X5 迁移工作交接文档（跨对话续接用）

> **用途**：新对话从这里恢复上下文。配合 `docs/rdk_x5_board_log.md`（板子病历本，操作 #1~#6 已记录）使用。
> 最后更新：1kHz 模板 PASS 收官（操作 #8，唤醒延迟 23us）；下一站 SocketCAN 点电机。视觉通道已修复（DeepSeek 系视觉模型）。

## 一、背景与目标

用户是 RoboMaster 战队电控（熟悉 C / RT-Thread / STM32），正在把基于 RT-Thread 的底盘控制（本仓库，见 `applications/`）迁移到 **RDK X5**。路线已定：**算法平移 + 底座替换**（pthread/SCHED_FIFO + SocketCAN + termios，不写内核驱动，不做 AMP）。Linux 侧已铺好 PREEMPT-RT 地基。

## 二、板子现状

| 项 | 值 |
|---|---|
| 板卡 | RDK X5（Sunrise5，8×A55），Ubuntu 22.04.5，root/sunrise |
| 内核 | **6.1.83-rt28 PREEMPT_RT 已启用**；回滚：还原 `/boot/*.bak` → `cd /boot && mkimage -C none -A arm -T script -d boot.cmd boot.scr` → reboot |
| 网络 | Windows 移动热点 `RLLR/12345678`，板 wlan0=192.168.137.93（DHCP），MobaXterm SSH；串口 COM6(CH340) 留作日志/救砖 |
| 已知怪癖 | 开机一条无害 spinlock_rt BUG 告警；无 RTC 时钟归零（联网 NTP 自校）；SFTP 默认落 `/root/`，正式目录 `/data/hello_app/` |
| 环境 | 板上 gcc 可用；gpiod/Hobot.GPIO 已装；tesseract 无关 |
| 服务 | `hello_gpio` 已 enable：reboot 终验通过，开机自闪 LED（回滚见病历本操作 #7） |

## 三、GPIO 关键结论（已实证，勿重查）

- 三套编号：物理脚号（接线）/ BCM 名（表格）/ 内核全局号（代码）
- **BCM GPIO5 = 物理脚 29 = gpiochip4（34120000.gpio, base 379）+ line 20 = sysfs 全局号 399**
- 权威映射表：板上 `/usr/local/lib/python3.10/dist-packages/Hobot/GPIO/gpio_pin_data.py` 的 `RDK_X5_PIN`（行格式：[dt节点, 全局号, 物理脚, BCM, 名, 复用功能...]）
- GND 脚 = 6/9/14/20/25/30/34/39；5 脚是 GPIO3 不是地
- 40-pin SoC 侧网络名见原理图 `x5_app/RDK_X5_IO_CONN_PUBLIC_V1.1.pdf`（LSIO_UART/SPI/PWM 系列）

## 四、任务进度

1. ✅ hello_file（操作 #4）
2. ✅ gpio_toggle（操作 #5/#6，GPIO399 1Hz 翻转，LED 闪）
3. ✅ systemd 自启（操作 #7，reboot 后 LED 自闪终验通过；三步上手收官）
4. ✅ 1kHz 周期模板（操作 #8）：v2 复测 **PASS**——唤醒延迟 23us、周期 jitter 29us、180s 零漂移；sleep 对照组少跑 1425 拍（累积漂移实证）。五件套骨架定版：SCHED_FIFO + TIMER_ABSTIME + mlockall + 绑核 + cpu_dma_latency

## 五、工作区产物

| 文件 | 说明 |
|---|---|
| `docs/rdk_x5_board_log.md` | 板子病历本（所有操作+回滚方法） |
| `docs/rdk_x5_stage1_tutorial.md` | **第一阶段跟做手册**（三步上手 + 1kHz 骨架，含全部实测数据与踩坑实录，新人入门用） |
| `docs/rdk_x5_stage1_impl_log.md` | **第一阶段实现过程实录**（逐功能件的实现叙事：命令→回传→决策→代码设计，v1→v2 演进全程） |
| `docs/linux_study_guide.md` | 10 章学习指南（第4章=1kHz模板，第6章=SocketCAN，重点） |
| `x5_app/` | hello_file.c / gpio_toggle.c(GPIO_N=399 定稿) / hello_gpio.service / README.md / mcp_pdf_client.py / 两份硬件PDF |
| `x5_app/mcp_pdf_client.py` | 直驱 pdf-reader MCP 的客户端（DSH 无 MCP 客户端，用 `python x5_app/mcp_pdf_client.py <pdf> get_pdf_info read_pdf_text`） |
| `x5_app/period_1khz.c` | 1kHz 周期模板 **v2 定版**（第4章五件套：SCHED_FIFO+绑核+mlockall+TIMER_ABSTIME+cpu_dma_latency，abs/sleep 对照，自带统计/直方图/判定；板上实测 PASS：唤醒延迟 23us） |

## 六、下一步路线

1. ✅ 三步上手收官 → 2. ✅ 1kHz 周期模板（PASS：唤醒延迟 23us，操作 #8）→ 3. 🔄 SocketCAN 点电机（第6章，can0 / M3508 0x200 帧）→ 4. cyclictest 实时性验证 → 5. 算法平移（`applications/chassis_fun` 等去 rt_ 化）

## 七、协作备忘

- 图像通道**已修复**（2026-09-10，视觉模型换 DeepSeek 系后实测可用；此前 bigmodel anthropic 兼容层丢图：glm-4.6v 不进图、glm-4.5v 把纯红方块看成"白色"）。终端内容仍首选复制粘贴文字，更精确
- 用户偏好：中文、循序渐进、要讲原理、板子操作必须记录进病历本
- 本机工具：Tesseract-OCR（`C:\Program Files\Tesseract-OCR`，中文需 chi_sim）、miniconda python（有 PIL/pypdf）、pip 可联网
