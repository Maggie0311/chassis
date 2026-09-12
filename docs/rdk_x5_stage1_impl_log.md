# RDK X5 第一阶段实现过程实录

> **定位**：记录本阶段四个功能件是**怎么一步步做出来的**——每步敲了什么命令、屏幕回了什么、据此做了什么决定、代码为什么写成这样，含全部失败与弯路。
> 与 [跟做手册](rdk_x5_stage1_tutorial.md) 的分工：手册是提炼后的复现路径，本文是原始工程过程。
> 所有终端输出摘自当次真实回传；操作编号对应 [板子病历本](rdk_x5_board_log.md) #4~#8。
> 时间：2026-09（PC 侧时间；板子无 RTC，开机时钟归零）

## 总览

| # | 功能件 | 产物 | 关键决策/弯路 | 最终状态 |
|---|---|---|---|---|
| 1 | 第一个自编进程 | `hello_file.c` | 选 `/data` 分区做程序家 | ✅ 操作 #4 |
| 2 | GPIO 翻转 | `gpio_toggle.c` | **编号考古五步证据链**（BCM5≠5） | ✅ 操作 #5/#6 |
| 3 | 开机自启 | `hello_gpio.service` | service 文件曾未上板→排障 | ✅ 操作 #7 |
| 4 | 1kHz 实时骨架 | `period_1khz.c` (v1→v2) | 首轮 123µs FAIL→压深 idle→23µs | ✅ 操作 #8 |

---

## 一、hello_file：第一个自编进程（操作 #4）

### 1.1 需求与设计

需求：在指定目录创建文件写入 `helloworld`，`cat` 验证——等价 MCU 的"点亮工程"。
设计决策：
- 目录选 `/data/hello_app`：`/data` 是用户数据分区，重刷系统不一定会动它，且无权限别扭（`/root` 只当中转站）
- 程序里自带 `mkdir`（`EEXIST` 容错）：目录可能先于程序被手工建好，已存在不算错——"双保险"
- 每步系统调用都 `perror`：第一天就养成"出错要能看见"的习惯

### 1.2 代码（全文，43 行）

```c
/* hello_file.c —— 步骤1：在指定目录创建文件并写入 helloworld
 * 流程：mkdir 建目录 → fopen 建/开文件 → fprintf 写入 → fclose 保存关闭
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>   /* mkdir 用 */
#include <sys/types.h>
#include <errno.h>      /* perror 用 */

#define APP_DIR  "/data/hello_app"     /* 指定的工作目录 */
#define FILE_PATH APP_DIR "/hello.txt" /* 要创建的文件 */

int main(void)
{
    /* 1. 创建目录（已存在不算错误，EEXIST 直接继续） */
    if (mkdir(APP_DIR, 0755) != 0 && errno != EEXIST) {
        perror("mkdir " APP_DIR);
        return 1;
    }
    /* 2. 打开文件："w" = 不存在则创建，存在则清空重写 */
    FILE *fp = fopen(FILE_PATH, "w");
    if (fp == NULL) {
        perror("fopen " FILE_PATH);
        return 1;
    }
    /* 3. 写入内容 */
    fprintf(fp, "helloworld\n");
    /* 4. 关闭文件（缓冲区落盘发生在这一步，忘了 close = 数据可能没写进去） */
    if (fclose(fp) != 0) {
        perror("fclose");
        return 1;
    }
    printf("[OK] 已写入 %s，用 cat %s 查看\n", FILE_PATH, FILE_PATH);
    return 0;
}
```

### 1.3 板上实现过程

```
SFTP 上传 hello_file.c → /root/          # MobaXterm 左侧面板拖拽
mkdir -p /data/hello_app && cd /data/hello_app
cp /root/hello_file.c .
gcc hello_file.c -o hello_file           # 板上 gcc 开箱可用，未装任何包
./hello_file
→ [OK] 已写入 /data/hello_app/hello.txt
cat /data/hello_app/hello.txt
→ helloworld                             # 验收通过
```



## 二、GPIO 翻转：从编号考古到点灯（操作 #5/#6）

### 2.1 问题定义

需求：C 代码控制一路 GPIO 高低翻转。看似简单，真问题是：**`/sys/class/gpio` 里该写几号？**
板卡资料只给了"BCM GPIO5 = 物理 29 脚"，但 sysfs 要的是内核全局号——两套编号。

### 2.2 编号考古实录（五步证据链，操作 #5）

**第 1 步：想当然，被拒绝**

```
# echo 5 > /sys/class/gpio/export
bash: echo: write error: Invalid argument
```

→ 结论：表里的 GPIO5 是 BCM **名字**，不是内核全局号。三套体系（物理脚/BCM 名/全局号）互不相干，从这步开始坐实。

**第 2 步：摸编号空间**

```
for c in /sys/class/gpio/gpiochip*; do echo "$c base=$(cat $c/base)"; done
→ 6 块 gpiochip，base = 347 / 379 / 411 / 434 / 466 / 498
gpioinfo   → 40-pin 上所有线 unnamed
```

→ 类比 STM32：这不是扁平编号，是 6 组"端口"（GPIOA~F 的味道），每片 base + line 偏移 = 全局号。但 40-pin 全 unnamed，没法直接对上号。

**第 3 步：读硬件原理图（PDF 直读）**

用 `x5_app/mcp_pdf_client.py` 直驱 pdf-reader MCP 读 `RDK_X5_IO_CONN_PUBLIC_V1.1.pdf`：
40-pin 在 SoC 侧的真实网络名是 LSIO_UART/SPI/PWM 系列（如 29 脚 = `LSIO_SPI2_SCLK_PWM0_3V3`）。
→ 知道了复用能力，但还没拿到全局号。

**第 4 步：翻设备树**

发现 Pi 风格 `__overrides__`（uart/spi/i2c/pwm 可 overlay）+ 大量 `pconf-*` 引脚配置节点。
→ 确认引脚复用由 overlay 控制，但没有"BCM→全局号"映射表。

**第 5 步：决定性证据——官方库 + strace**

板上 Hobot.GPIO 库（pip 包）的映射表 `/usr/local/lib/python3.10/dist-packages/Hobot/GPIO/gpio_pin_data.py` 的 `RDK_X5_PIN`：

```
行格式: ["soc/...gpio@xxx", 全局号, 物理脚, BCM号, 'GPIOx', '复用功能', ...]
grep 查 GPIO5 → 全局号 399，物理脚 29，gpiochip 34120000.gpio（base 379）+ line 20
```

再用 strace 抓官方库实际写 `/sys/class/gpio/gpio399/*`——**实锤**。

**中途冤案**：手动 `echo 399 > export` + value=1 后万用表量到 0V，一度怀疑结论；官方库同机制却能点灯 → 判定为表笔接触问题。教训：**测量结论建立在接触可靠上**。

### 2.3 最终定论

| 项 | 值 |
|---|---|
| BCM 名 | GPIO5 |
| 物理脚 | **29**（接线用） |
| 内核定位 | gpiochip4（34120000.gpio，base 379）+ line 20 |
| **sysfs 全局号** | **399**（代码用） |
| 原理图网络 | LSIO_SPI2_SCLK_PWM0_3V3 |

### 2.4 gpio_toggle.c 设计要点（全文见 `x5_app/gpio_toggle.c`）

- `gpio_write(attr, val)` 小工具：sysfs 属性文件都是"开-写-关"，抽成函数防重复
- **export 容错**：引脚可能已被官方库/上次测试导出（再 export 报 busy），失败后用 `access(value_path, W_OK)` 探测——可写就继续用，不可写才报错退出。这段是被 strace 观察逼出来的
- 主循环 `fopen→fprintf→fclose` 每秒一次翻 `value`，`sleep(1)` 控节奏（此处只需 1Hz，不追求精度）

### 2.5 板上实现

```
SFTP → /root/ → 归位 → gcc gpio_toggle.c -o gpio_toggle → ./gpio_toggle
→ 终端每秒打印 GPIO399 = 0/1，物理 29 脚 LED（串 330Ω）1Hz 稳定闪烁   # 验收通过
```

目录规约定型：源码/产物统一 `/data/hello_app/`，`/root/` 只当中转，裸文件随手清。

---

## 三、systemd 开机自启（操作 #7）

### 3.1 service 文件设计（逐行为什么）

```ini
[Unit]
Description=RDK X5 first app: GPIO toggle (hello world of embedded Linux)
After=multi-user.target          # 排在多用户目标之后（系统基本就绪再起）

[Service]
Type=simple                      # ExecStart 进程不死即算活着，最普通型
ExecStart=/data/hello_app/gpio_toggle   # 必须绝对路径；二进制已在 /data 分区
Restart=always                   # 崩了自动拉起——MCU 看门狗的粗糙等价物
RestartSec=2                     # 拉起间隔，防疯狂重启

[Install]
WantedBy=multi-user.target       # enable 时在 multi-user.target.wants/ 建软链
```

设计依据：程序无前台交互、无就绪通知（不用 notify/forking）；放在 `/data` 上的二进制路径稳定。

### 3.2 安装过程实录（真实回传）

```
# cp /root/hello_gpio.service /etc/systemd/system/
# systemctl daemon-reload
# systemctl enable --now hello_gpio.service
Created symlink /etc/systemd/system/multi-user.target.wants/hello_gpio.service
  → /etc/systemd/system/hello_gpio.service.        ← enable 的本质就是这个软链

# systemctl status hello_gpio --no-pager
● hello_gpio.service - RDK X5 first app: GPIO toggle (hello world of embedded Linux)
     Loaded: loaded (/etc/systemd/system/hello_gpio.service; enabled; ...)
     Active: active (running) since Thu 2026-09-10 18:11:30 CST; 61ms ago
   Main PID: 3528 (gpio_toggle)
        Tasks: 1 (limit: 8034)
        Memory: 192.0K
        CPU: 4ms
      CGroup: /system.slice/hello_gpio.service
             └─3528 /data/hello_app/gpio_toggle
```

字段解读：`Loaded: enabled` = 自启标记已建；`Active: running` + Main PID = `--now` 已把它拉起；Memory/CPU 顺带展示这进程多轻量。

### 3.3 故障与排障实录（本阶段最有价值的弯路）

**现象**：手动运行 LED 正常闪；reboot 后不闪。此前问卷一度误记"自启成功"，用户澄清后进入排障。

**排障三连**（设计好的定位链）：

```bash
ls -l /etc/systemd/system/hello_gpio.service   # ① 文件在不在？
systemctl status hello_gpio --no-pager         # ② 状态 enabled/running？
journalctl -u hello_gpio -n 50 --no-pager      # ③ 服务日志说什么？
```

**真相**：终端截图（视觉通道恢复后读取）显示 `ls /data/hello_app/` 里只有 5 个旧文件，SFTP 面板 `/root/` 也没有——**service 文件从未被传上板**。不是 systemd 问题、不是权限问题，是流程漏了一步。

**修复**：SFTP 补传 → 3.2 的安装流程 → `reboot` 终验：登录横幅显示 `Last login: Sat Jan 1 08:00:28 2000`（无 RTC 时钟归零，已知怪癖），用户未手动启动任何程序，**LED 自动高低翻转** → 通过。

**沉淀的方法论**：
- `status` 的 running 只证明"现在活着"，enabled 只证明"开机想拉"，**唯一完整证据是 reboot 不登录看现象**（MCU 冷启动测试思维）
- 粘贴命令块前必须先完成前置上传动作（本阶段两次翻车都在这）
- 服务的 stdout 进 journald 不上串口，不登录时硬件现象是唯一无人值守证据

### 3.4 回滚方法（记录在案）

```bash
systemctl disable --now hello_gpio
rm /etc/systemd/system/hello_gpio.service && systemctl daemon-reload
```

---

## 四、1kHz 实时骨架 period_1khz.c（操作 #8，v1→v2）

### 4.1 设计目标与选型

需求：把"RT-Thread 高优先级线程 + 硬定时器的 1kHz 确定性"搬到 Linux。骨架必须可测量、可自判。

选型（对照学习指南第 4 章）：

| 手段 | 解决什么 |
|---|---|
| pthread + `SCHED_FIFO` prio 80 | 抢占确定性（范围 1-99，控制环建议 70-90） |
| `clock_nanosleep(TIMER_ABSTIME)` | 周期零漂移（vs `usleep` 的"醒来再睡"累积漂移） |
| `mlockall(MCL_CURRENT\|MCL_FUTURE)` | 防换页缺页抖动 |
| `pthread_setaffinity_np` 绑核 CPU3 | 防核间迁移抖动 |
| （v2 补）`/dev/cpu_dma_latency`=10µs | 压深度 idle 退出延迟 |

### 4.2 程序架构设计（全文见 `x5_app/period_1khz.c`，约 300 行）

- **双线程分工**：rt 线程只做"睡-醒-统计-（未来的控制计算）"，往共享结构体写数字；主线程 SCHED_OTHER，负责每 10s 打印进度和收尾报告。**实时循环里不做 printf 等慢 syscall**——这是抖动之源，输出交给低优先级线程慢慢打。真控制环沿用此纪律
- **两个统计口径分开**：① 周期（相邻两醒的间隔）→ 直方图与 jitter；② 唤醒延迟（实醒时刻 − 绝对 deadline）→ abs 模式的过关指标。sleep 模式无锚点，只统计周期
- **首拍不计入**（校准拍）；`long` 对齐读写天然原子，教学代码不上锁，注释说明竞态无害范围
- **50µs 分桶直方图**（12 桶覆盖 900~1500µs，超界并末桶）：直接看分布形状
- **自动判定 + 内核自检**：`uname` 查 `PREEMPT_RT`；abs 模式按"唤醒延迟 <100µs 且拍数差 <±1000"打 PASS/FAIL
- **双模式对照**：`./period_1khz [abs|sleep] [秒数]`——同一份代码里 `usleep(1000)` 作对照组，漂移看得见

核心循环（骨架的心脏）：

```c
while (g_run) {
    if (cfg->use_abstime) {
        next.tv_nsec += PERIOD_NS;                       /* 锚点先推进 */
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);  /* 睡到绝对时刻 */
    } else {
        usleep(PERIOD_US);                               /* 对照组：醒来再睡 */
    }
    if (!g_run) break;
    clock_gettime(CLOCK_MONOTONIC, &now);                /* 实醒时刻 → 统计 */
    /* ==== 控制计算放这里：读传感器 → PID → CAN 发送 ==== */
}
```

### 4.3 v1 上板实测（真实输出摘录）

```
内核: 6.1.83-rt28 #2 SMP PREEMPT_RT ...        ← RT 内核自检通过
模式=abs  周期=1000us  时长=180s  绑核CPU=3  SCHED_FIFO prio=80
...
运行 180.0s   拍数 180024 / 预期 180025 （差 1）      ← 18 万拍零漂移
实际周期 us: min=886 avg=999.5 max=1112 （jitter=226）
唤醒延迟 max = 123 us （过关线 <100）
判定: FAIL
直方图: 950~1000us 桶 111478 次；1000~1050us 桶 66172 次；长尾到 1112us
```

先看到的**好消息**：拍数差 1（还是启动偏移）、avg 999.5µs——绝对锚点零漂移实证；
以及一个有趣签名：min=886µs < 1000µs——某拍迟到 114µs 后，下一拍仍睡到自己的绝对锚点，间隔被"短补"。usleep 法永远不可能出现短于标称的周期。

**坏消息**：唤醒延迟 123µs 超线。

### 4.4 根因分析与 v2 演进

排查思路（排除法）：
1. 有 sudo、SCHED_FIFO 设置无警告、绑核成功 → 调度配置无问题
2. RT 内核在跑 → 不是内核问题
3. **嫌疑集中：CPU 深度 idle**。机理：周期任务每拍只干几 µs 活就睡 ~1ms，CPU3 空闲掉进深度省电态（cpuidle 深档），从深 idle 退出的硬件代价几十~上百 µs——这与"没人抢核却晚醒 123µs"的画像吻合
4. 次要噪声源：hello_gpio 服务每秒文件 IO + journald、cpufreq 动态调频升频延迟

**v2 修复**（pm_qos 惯用法）：

```c
/* 持有 /dev/cpu_dma_latency 并写入目标延迟，内核即不进入退出延迟超过它的 idle 态；
 * fd 必须一直开着（关掉约束即解除），真控制环同理 */
int latency_fd = open("/dev/cpu_dma_latency", O_WRONLY);
if (latency_fd >= 0) {
    int32_t target_lat_us = 10;
    if (write(latency_fd, &target_lat_us, sizeof(target_lat_us)) == sizeof(target_lat_us))
        printf("cpu_dma_latency = %d us（已压制深度 idle）\n", target_lat_us);
} else perror("[警告] 打不开 /dev/cpu_dma_latency");
/* ... main 结束前 close(latency_fd) */
```

配套环境优化（测量期）：`systemctl stop hello_gpio`（去 1Hz 服务噪声）、全核 governor 定 `performance`（去升频延迟）。

### 4.5 v2 复测实录（真实输出摘录）

```
cpu_dma_latency = 10 us（已压制深度 idle）        ← v2 生效标志
...
运行 180.0s   拍数 180016 / 预期 180017 （差 1）
实际周期 us: min=988 avg=999.4 max=1017 （jitter=29）
唤醒延迟 max = 23 us （过关线 <100）
判定: PASS —— 这套骨架可以承载 1kHz 控制环
直方图: 950~1000us 桶 108593 次；1000~1050us 桶 71422 次；两桶之外全零
```

**123µs → 23µs**：压 idle 一项拿掉 100µs，"深 idle 退出延迟是主嫌"实锤。直方图收拢到两桶且无长尾。

### 4.6 sleep 对照实验（同日真实输出摘录）

```
模式=sleep  周期=1000us  时长=180s ...
运行 180.0s   拍数 178591 / 预期 180016 （差 1425）
实际周期 us: min=1006 avg=1007.5 max=1039 （jitter=33）
每拍平均多耗 7.5 us （= avg周期-1000），累积漂移 1425 拍
直方图: 全部落入 1000~1050us 单桶
```

与 abs 的三项对照（工程意义）：

| | abs | sleep | 说明 |
|---|---|---|---|
| min 周期 | **988**（可短补） | **1006**（永不早醒） | "至少睡"语义 vs 绝对锚点 |
| 180s 拍数差 | 1 | **−1425** | 0.79% 单向累积漂移 |
| 直方图 | 以 1000 为中心两桶收拢 | 整体右移单桶 | 漂移方向的直观形状 |

0.79% 的慢放进真底盘：1kHz 积分的速度/里程，180 秒偏 1.4 秒的量——这就是周期必须锚在绝对时间上的原因。

### 4.7 结论

**五件套定版**：SCHED_FIFO + TIMER_ABSTIME + mlockall + 绑核 + cpu_dma_latency。
指标：唤醒延迟 max 23µs / 周期 jitter 29µs / 180s 零漂移。控制计算直接填进循环体标记处，计时骨架原封不动。

### 4.8 追问：这骨架是"sleep 实现的"吗？能不能上硬件定时器？（Q&A）

**Q：正常应该用定时器驱动定时任务，你这个是不是 sleep？**（DIODE，2026-09-10）

A：API 名字带 sleep，驱动它的**就是硬件定时器**。链条：A55 通用定时器（CLOCK_MONOTONIC 的底层硬件计数器）→ 内核 hrtimer 按绝对截止点编程一次硬件定时器中断 → 中断触发唤醒我们的 SCHED_FIFO 线程抢占 CPU。与 MCU「TIM 中断 + 回调」物理上同类，唯一差别是"回调"为可调度线程——能睡眠、能阻塞收 CAN、有优先级，比裸 ISR 上下文更强。实测 23µs 唤醒延迟即「硬件中断→线程入口」的端到端延迟，纯软件轮询给不出这个数；真正用 sleep 拼周期的是对照组 usleep 版（180s 漂 1425 拍）。

**Q：那有机会直接用硬件定时器吗？**

A：用户态做不到也不需要。ARM 定时器中断寄存器在 EL1，用户态不能武装/接收裸定时器中断；硬来只能写内核驱动（路线明确不写），且内核 hrtimer 唤醒用户态走的正是 clock_nanosleep 这条路。`timerfd_create`/`timer_create` 等 API 后端同样是 hrtimer，对独占式 RT 线程无增益。

**眼见为实**：`sudo strace -c ./period_1khz abs 10` —— 每拍唯一 syscall 是 `clock_nanosleep`（= 定时器编程一次）；`clock_gettime` 不在列表（走 vDSO 直读硬件计数器，零 syscall）。

**Q：用延时的话，两轮延时之间的时间会被叠加进去；被打断定时就乱了，应该用软件定时器**（DIODE 追问，同日）

A：预言的失效模式属于**相对延时**（usleep 醒来再睡），正是我们的对照组——实测每拍多 7.5µs、180s 少 1425 拍，全部命中。正式骨架用**绝对锚点**：`next.tv_nsec += PERIOD_NS` 在固定网格 t₀, t₀+1ms, t₀+2ms… 上前进，与实际醒来时刻无关，中间耗时只影响"这拍睡多短"，不进周期公式——实测 180,016/180,017 拍零叠加。被打断同理：本拍迟到，下拍 deadline 仍在网格上，自动**短补**（min=988µs<1000µs 即短补签名）——打断的代价是一拍延迟，不是永久漂移。RT-Thread 软件定时器（tick 扫超时链表）在 Linux 的对应物就是内核 hrtimer（ns 级），用户态三个门面后端相同：`clock_nanosleep(ABSTIME)`（独占 RT 线程，本骨架）、`timerfd_create`（epoll 事件循环）、`timer_create`（信号驱动）。压榨验证：`stress-ng --cpu 8` 下跑 abs，FIFO 应不受影响。

---

## 五、产物清单

**PC 侧（本仓库）**：`x5_app/`：hello_file.c、gpio_toggle.c（GPIO_N=399 定稿）、hello_gpio.service、period_1khz.c（**v2 定版**）、README.md、两份硬件 PDF、mcp_pdf_client.py。
**板上**：`/data/hello_app/` 全部产物；`/etc/systemd/system/hello_gpio.service`（enabled，开机自闪）。


*实录完。原始逐条记录见 [rdk_x5_board_log.md](rdk_x5_board_log.md)；提炼版跟做路径见 [rdk_x5_stage1_tutorial.md](rdk_x5_stage1_tutorial.md)。*
