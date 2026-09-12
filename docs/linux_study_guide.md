# Linux 学习指南 —— 从 RT-Thread 到 RDK X5

> 全书 10 章：第 1~5 章打地基（心智模型 / 生存技能 / 构建 / 实时调度 / 串口），第 6~10 章上战场（SocketCAN / GPIO / IPC / 调试观测 / 四周学习计划）。
> 所有示例代码均可在 RDK X5（8 核 A55 / Ubuntu 22.04 / PREEMPT-RT 6.1.83-rt28）上直接编译运行；每章末尾附动手实验与过关标准。

---


> 读者：RoboMaster 战队嵌入式开发者，熟悉 C / RT-Thread / STM32，正在把底盘控制代码迁移到 RDK X5（8 核 A55，Ubuntu 22.04，已装 PREEMPT-RT）。外设：CAN（M3508）、DBUS 遥控串口（100000 8E1）、多路 UART、GPIO。会基础命令但没系统学过 Linux——这份指南就是为补这块地图而写的。

## 目录

- 第 1 章 心智模型：Linux vs RT-Thread
- 第 2 章 基本生存技能：目录、权限、apt、systemd、journalctl
- 第 3 章 构建与工程组织：gcc / Makefile / CMake / git
- 第 4 章 线程与实时调度：pthread 与 1kHz 周期任务模板
- 第 5 章 串口实战 termios：/dev/ttyS* 与 DBUS 遥控器

## 第 1 章 心智模型：Linux vs RT-Thread

**这章解决什么问题**：你习惯的"一个固件镜像、main 里直接摸寄存器"的世界观，搬到 Linux 后哪里不变、哪里塌了。读完应能回答：我的代码跑在什么态？硬件归谁管？为什么串口成了一个"文件"？为什么不需要自己写驱动？

### 1.1 用户态 / 内核态 / syscall

| | RT-Thread | Linux |
|---|---|---|
| 运行形态 | 单一固件镜像，裸机特权级 | 内核 + 多个用户态进程 |
| 硬件访问 | 直接读写寄存器 | 必须 syscall 请内核代办：`open/read/write/ioctl` |
| 隔离性 | 一个野指针全机死机 | 进程间 MMU 隔离，控制程序崩了系统还在 |
| 并存程序 | 固件里那几个线程 | SSH、日志服务、你的控制程序互不干扰 |

### 1.2 一切皆文件

| 对象 | RT-Thread | Linux |
|---|---|---|
| 串口 | `rt_device_find("uart3")` | `open("/dev/ttyS3")` |
| CAN | BSP 自定义接口 | SocketCAN：`socket()`（下半部分讲） |
| GPIO | `rt_pin_write()` | libgpiod（`/dev/gpiochip*`） |
| 运行时信息 | `list_thread` 等命令 | `/proc`、`/sys`，cat 即可读 |

### 1.3 为什么不用自己写驱动

X5 的厂商 Ubuntu 镜像已带设备树和串口/CAN/GPIO 驱动，内核把它们暴露成 `/dev/*`。**默认姿势：用户态 open + read/write + ioctl 直接用**；只有驱动缺失或要改引脚复用时才碰设备树/内核模块（本指南不涉及）。你的角色从"写驱动的人"变成"用驱动的人"。

### 1.4 API 对照表（迁移速查）

| RT-Thread | Linux 用户态 |
|---|---|
| `rt_thread_create` / `rt_thread_startup` | `pthread_create`（第 4 章） |
| `rt_thread_mdelay` / `rt_hw_us_delay` | `clock_nanosleep`（第 4 章） |
| `rt_sem_take` / `rt_sem_release` | `sem_wait` / `sem_post` |
| `rt_mutex_take` / `rt_mutex_release` | `pthread_mutex_lock` / `unlock` |
| `rt_device_set_rx_indicate` 回调 | 读线程 + 阻塞 `read`（第 5 章） |
| `rt_tick_get` / SysTick | `clock_gettime(CLOCK_MONOTONIC)` |
| `rt_hw_interrupt_disable` | 用户态不可用 → 互斥锁/原子变量/无锁环形缓冲 |
| Keil/IAR 工程 | gcc + Makefile/CMake（第 3 章） |

### 动手实验
1. `ls /dev/ttyS* && dmesg | grep -i tty` 盘点串口；再开两个 SSH：一个跑 `top`、一个跑 `yes > /dev/null`，观察多进程并存与 CPU 占用。

**过关标准**：能说出 DBUS 对应的 `/dev/ttyS*` 名字，并说出"打开串口"在两个系统里分别对应哪两个函数调用。

## 第 2 章 基本生存技能

**这章解决什么问题**：在 Ubuntu 板子上不慌——找得到东西、动得了权限、装得上包；最关键的是把控制程序做成"开机自启、崩溃自动重启、日志可查"的 systemd 服务，这是从"固件"思维到"服务"思维的第一步。

### 2.1 目录结构（只背高频）

| 目录 | 内容 | 嵌入式类比 |
|---|---|---|
| `/home/sunrise` | 你的工程代码 | 工程目录 |
| `/etc` | 系统配置（纯文本） | rtconfig.h 的聚集体 |
| `/dev` | 设备文件：ttyS*、gpiochip*、can0 | 设备对象表 |
| `/proc`、`/sys` | 内核运行时信息 | 调试命令的输出落成文件 |
| `/var/log` | 日志（journald 管） | rt_kprintf 存档 |

### 2.2 权限：chmod / chown / dialout

- `ls -l` 的 `rwxr-xr--` 三组属主/组/其他人；`chmod 754 f` 改权限，`chown user:grp f` 改归属，提权用 `sudo`。
- **高频坑**：串口属于 `dialout` 组，普通用户 open 直接 Permission denied。一次性解决：`sudo usermod -aG dialout $USER`，重新登录生效。

### 2.3 apt 装包

```bash
sudo apt update && sudo apt install build-essential cmake git
apt search 关键词      # 找包；sudo apt remove 包名 卸载
```

### 2.4 systemd：控制程序开机自启（完整 unit）

写到 `/etc/systemd/system/chassis.service`：

```ini
[Unit]
Description=2026 chassis control
After=network.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=/home/sunrise/chassis
ExecStart=/home/sunrise/chassis/build/chassis
Restart=always
RestartSec=1
LimitMEMLOCK=infinity
LimitRTPRIO=99
CPUAffinity=3

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now chassis.service  # 开机自启 + 立即启动
systemctl status chassis                     # 看状态；restart / stop 同理
```

### 2.5 journalctl 看日志

| 命令 | 作用 |
|---|---|
| `journalctl -u chassis -f` | 实时跟踪（≈ tail -f） |
| `journalctl -u chassis -n 200` | 最近 200 行 |
| `journalctl -b -p err` | 本次开机以来的报错 |

注意：printf 进 journald 有毫秒级延迟，1kHz 循环里别高频打印——调试数据先入内存、低频落盘（下半部分讲）。

### 动手实验
1. 写个每秒 printf 的 hello 程序，照 2.4 做成 hello.service。
   **过关标准**：`reboot` 后不登录也能 `journalctl -u hello` 看到输出；`stop` 能停。
2. 复现并修复串口权限：普通用户 `cat /dev/ttyS3` 报 Permission denied → 加入 dialout 组。
   **过关标准**：`groups` 显示 dialout，重新登录后能读。

## 第 3 章 构建与工程组织

**这章解决什么问题**：没有 Keil 的"点一下 Build"，代码怎么在 X5 上变成可执行文件；工程怎么组织、何时才真的需要交叉编译；用 git 把比赛代码管起来。

### 3.1 板上直接编译

```bash
gcc -Wall -O2 -g main.c control.c dbus_uart.c -o chassis
gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread   # 用 pthread 要加
```

X5 的 8 核 A55 编译几千行的底盘工程绰绰有余，`make -j8` 分钟级完成。**先板上编译，别急着搭交叉环境**。`-g` 必带，配合 gdb 排崩溃。

### 3.2 Makefile 最小模板

```makefile
CROSS   :=
CC      := $(CROSS)gcc
CFLAGS  := -Wall -O2 -g
LDLIBS  := -lpthread -lm
SRC     := $(wildcard *.c)
BIN     := chassis

$(BIN): $(SRC)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@
```

注意：命令行行首必须是 Tab，不是空格。

### 3.3 CMake 最小模板

```cmake
cmake_minimum_required(VERSION 3.16)
project(chassis C)
add_executable(chassis main.c control.c dbus_uart.c uart.c)
target_link_libraries(chassis PRIVATE pthread m)
```
```bash
cmake -B build && cmake --build build -j8   # 产物在 build/chassis
```

### 3.4 何时需要交叉编译

| 场景 | 做法 |
|---|---|
| 日常开发（本项目规模） | 板上编译，最省事 |
| 代码量大 / x86 主力机 / CI | 装 `gcc-aarch64-linux-gnu`，`make CROSS=aarch64-linux-gnu-`，注意与板端 glibc 版本对齐 |
| 崩溃定位 | 板上直接 gdb，或 `gdbserver :1234 ./chassis` 远程 |

### 3.5 git 基本流

```bash
git init && git add -A && git commit -m "chassis on linux"
git checkout -b rt-tuning            # 开分支改参数
git commit -am "tune 1kHz gains"
git checkout main && git merge rt-tuning
git remote add origin <url> && git push -u origin main
```

`.gitignore` 至少写：`build/`、`*.o`、`chassis`。

### 动手实验
1. 用 CMake 模板构建第 4 章的 period_1khz。
   **过关标准**：`cmake --build build` 成功且产物可运行。
2. `git init` 本目录，走一遍"分支 → 改一行 → 合并"。
   **过关标准**：`git log --oneline --all --graph` 可见合并轨迹。

## 第 4 章 线程与实时调度

**这章解决什么问题**：RT-Thread 里"高优先级线程 + 硬定时器保证 1kHz"的确定性，在 Linux 上怎么拿回来：pthread 建线程、SCHED_FIFO 定优先级、mlockall 锁内存、绑核防迁移，最后给一份可直接抄的 1kHz 周期任务模板。

### 4.1 pthread 最小集

```c
#include <pthread.h>
void *worker(void *arg) { /* 干活 */ return NULL; }
pthread_t tid;
pthread_create(&tid, NULL, worker, NULL);  // 默认 SCHED_OTHER
pthread_join(tid, NULL);                   // 等它结束
```

### 4.2 调度策略与优先级

| 策略 | 优先级 | 用在哪 |
|---|---|---|
| SCHED_OTHER（默认） | nice 值，非实时 | 日志、网络、慢速解析 |
| SCHED_FIFO | 1–99，高优先级无条件抢占低者 | 1kHz 控制 loop（选它） |
| SCHED_RR | 1–99，同级轮转 | 多个同级实时线程 |

警告：SCHED_FIFO 线程不让出 CPU 就永远霸占，会把 SSH、journald 全饿死——实时循环里必须有 `clock_nanosleep` 或阻塞 IO。

### 4.3 为什么 sleep/usleep 做不了周期任务

`usleep(1000)` 的语义是"醒来之后**再**睡 1ms"，于是每周期 = 计算 + 调度延迟 + 1ms，误差逐周期累加，1kHz 下一小时能漂出几秒。正确姿势：以 CLOCK_MONOTONIC 为轴，`clock_nanosleep(TIMER_ABSTIME)` 睡到**绝对**时刻——本周期算得慢就晚睡少睡，Deadline 锚点不动，长期零漂移；真错过点了立即返回追赶。

### 4.4 1kHz 周期任务模板（完整可编译）

```c
/* period_1khz.c —— 编译：gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread
 * 运行：sudo ./period_1khz，回车结束（权限见 4.5） */
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_NS 1000000ULL               /* 1 kHz */
static volatile int g_run = 1;

static void *rt_loop(void *arg)
{
    (void)arg;
    struct timespec next, now;
    long max_jit_us = 0;
    /* 1) 实时优先级：SCHED_FIFO，prio=80（范围 1-99，控制 loop 建议 70-90） */
    struct sched_param sp = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
        perror("SCHED_FIFO 失败（需 root / cap_sys_nice / LimitRTPRIO）");
    /* 2) 绑核：固定在 CPU3（0-7，挑空闲核，按实际改） */
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(3, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);

    clock_gettime(CLOCK_MONOTONIC, &next);
    while (g_run) {
        next.tv_nsec += PERIOD_NS;          /* 下一个绝对截止点 */
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        clock_gettime(CLOCK_MONOTONIC, &now);            /* 实际醒来时刻 */
        long jit = (now.tv_sec - next.tv_sec) * 1000000L
                 + (now.tv_nsec - next.tv_nsec) / 1000;  /* 晚醒 µs 数 */
        if (jit > max_jit_us) max_jit_us = jit;

        /* ==== 这里放控制计算：读传感器 → PID → CAN 发送 ==== */
    }
    printf("max jitter = %ld us\n", max_jit_us);
    return NULL;
}

int main(void)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);     /* 3) 锁内存：禁止换页，防缺页抖动 */
    pthread_t tid;
    pthread_create(&tid, NULL, rt_loop, NULL);
    getchar();
    g_run = 0;
    pthread_join(tid, NULL);
    return 0;
}
```

### 4.5 运行权限

三选一：`sudo ./period_1khz`；`sudo setcap cap_sys_nice+ep ./period_1khz`（免 sudo，重编译后要重打，git 不保存 xattr）；或用 2.4 的 systemd 服务（LimitRTPRIO=99）。确认 PREEMPT_RT 生效：`uname -v` 输出里含 `PREEMPT_RT`。

### 动手实验
1. 编译运行 3 分钟，记录 max jitter；再把模板改成 `usleep(1000)`（去掉绝对时间逻辑），同时统计循环次数。
   **过关标准**：ABSTIME 版 max jitter < 100 µs 且 3 分钟循环次数 ≈ 180000；usleep 版次数明显少（差 > 1000），肉眼可见累积漂移。

## 第 5 章 串口实战 termios

**这章解决什么问题**：`rt_device_find` + `set_rx_indicate` 那套在 Linux 上的对应物——open 哪个设备、termios 各字段怎么配 raw 模式、阻塞读线程怎么写，最后落地 DBUS 遥控器的非标准 100000 波特率。

### 5.1 通用配置流程（标准波特率）

```c
/* uart.c：通用串口初始化，返回 fd */
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int uart_open(const char *dev, speed_t baud)   /* baud 传 B115200 这类常量 */
{
    int fd = open(dev, O_RDWR | O_NOCTTY);     /* 默认阻塞模式 */
    struct termios t;
    tcgetattr(fd, &t);
    cfmakeraw(&t);                             /* raw：内核不做任何加工 */
    cfsetispeed(&t, baud);
    cfsetospeed(&t, baud);
    t.c_cflag |= CLOCAL | CREAD;               /* 无 modem 线 + 使能接收 */
    t.c_cc[VMIN]  = 1;                         /* 凑够 1 字节 read 即返回 */
    t.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &t);
    tcflush(fd, TCIOFLUSH);
    return fd;
}
```

### 5.2 termios 字段速查

| 字段/标志 | 含义 | 典型设置 |
|---|---|---|
| `c_cflag` 的 CS5–CS8 | 数据位 | CS8 |
| PARENB / PARODD | 校验使能 / 奇校验 | 无校验全清；DBUS 置 PARENB |
| CSTOPB | 停止位（置=2，清=1） | 通常清；DBUS 按实际 |
| CRTSCTS | 硬件流控 | 单线场景必须清 |
| `c_cc[VMIN]/[VTIME]` | read 阻塞的最小字节 / 超时 | 按协议定 |

### 5.3 阻塞读线程（替代 rx_indicate 回调）

```c
/* 骨架：read 无数据时挂起，不占 CPU，取代中断回调 */
static volatile int g_run = 1;
static void *rx_thread(void *arg)
{
    int fd = *(int *)arg; uint8_t buf[256]; ssize_t n;
    while (g_run) {
        n = read(fd, buf, sizeof buf);
        if (n > 0)      { /* 写环形缓冲，或直接在此解析 */ }
        else if (n < 0) break;  /* 出错退出，实用中应重开串口 */
    }
    return NULL;
}
```

与 RT-Thread 的本质差别：回调跑在中断上下文不敢久留 → 读线程是普通线程，可随意解析、加锁、打印，收与析天然解耦。

### 5.4 DBUS：100000 8E1（偶校验；停止位按接收机实际，DT7 实测 2 位）

POSIX 波特率常量里没有 100000，须用 termios2 的 BOTHER 自定义；且 `<asm/termbits.h>` 与 `<termios.h>` 的结构体定义冲突，DBUS 配置必须放独立 .c（不能与 5.1 同文件混编）：

```c
/* dbus_uart.c —— gcc -c dbus_uart.c，链接进工程 */
#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>                     /* termios2/BOTHER，勿混 <termios.h> */
#include <sys/ioctl.h>

int dbus_open(const char *dev)                /* 100000 波特率，8 数据位，偶校验 */
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    struct termios2 t;
    ioctl(fd, TCGETS2, &t);
    t.c_cflag &= ~CSIZE;  t.c_cflag |= CS8;    /* 8 数据位 */
    t.c_cflag |= PARENB;  t.c_cflag &= ~PARODD;/* 偶校验 */
    t.c_cflag |= CSTOPB;                       /* 2 停止位；实测 1 位就删掉本行 */
    t.c_cflag &= ~CRTSCTS;                     /* 无流控 */
    t.c_cflag |= CLOCAL | CREAD;
    t.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT
                 | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    t.c_oflag = 0;
    t.c_lflag = 0;                             /* raw 模式 */
    t.c_cc[VMIN] = 1;  t.c_cc[VTIME] = 0;
    t.c_cflag &= ~CBAUD;  t.c_cflag |= BOTHER; /* 自定义波特率 */
    t.c_ispeed = 100000;  t.c_ospeed = 100000;
    ioctl(fd, TCSETS2, &t);
    return fd;
}
```

帧格式：18 字节 / 14 ms；`ch0 = buf[0] | ((buf[1] & 0x07) << 8)`，摇杆约 364–1684、中位 1024。解析建议：滚动缓冲里对齐帧头后按 18 字节切帧，用"各通道值都落在合法区间"校验。老内核不支持 BOTHER 时的备胎：`ioctl(TIOCSSERIAL)` + `ASYNC_SPD_CUST` 自定义除数。

### 5.5 与 RT-Thread 串口对照

| RT-Thread | Linux |
|---|---|
| `rt_device_find` + `rt_device_open` | `open("/dev/ttyS3", O_RDWR \| O_NOCTTY)` |
| `rt_device_control` 设波特率/格式 | `tcsetattr` / `TCSETS2` |
| `set_rx_indicate` 回调 + 信号量 | 读线程阻塞 `read` + 环形缓冲 |
| 设备名由板级配置决定 | 设备名由设备树决定，`dmesg \| grep tty` 查 |

### 动手实验
1. 环回：`stty -F /dev/ttyS4 115200 raw -echo` 后短接 TX/RX，`echo hi > /dev/ttyS4` 的同时另一终端 `cat /dev/ttyS4`。
   **过关标准**：接收端看到原始字节。
2. 接 DBUS 接收机，dbus_open + 读线程打印 ch0，打杆测试。
   **过关标准**：中位 ≈1024，两端 ≈364/1684，数值稳定不跳（跳变多半是停止位或波特率配错）。

---

# 下半部分（第 6~10 章）

> 平台：RDK X5（8 核 A55 / Ubuntu 22.04 / PREEMPT-RT）。上半部分（1~5 章）已覆盖心智模型、基本命令、构建、pthread 实时调度与 termios 串口。

## 本部分目录

| 章 | 主题 | 一句话 |
|---|---|---|
| 6 | SocketCAN 实战 | CAN 收发变成 socket read/write，驱动 M3508 |
| 7 | GPIO 与 RGB 灯 | /sys/class/gpio 与 gpiod 两条路 |
| 8 | 进程间通信 IPC | mqueue / 共享内存，控制进程与视觉进程解耦 |
| 9 | 调试与观测 | top/gdb/strace/journalctl + 周期抖动自测 |
| 10 | 四周学习计划 | 从生存到 1kHz 集成小项目 |

---

## 第 6 章 SocketCAN 实战

本章解决：**Linux 下 CAN 怎么收发**。RT-Thread 里你用 `rt_device_find("can1")` + `rt_device_set_rx_indicate()` 回调；Linux 的 SocketCAN 把总线抽象成 socket，收电机反馈帧就是阻塞 `read()`，发电流帧就是 `write()`——"回调 + 信号量"内核已经替你做完。

### 6.1 命令行三板斧

```bash
ip link set can0 up type can bitrate 1000000  # 起总线，1Mbps（M3508/GM6020 标准）
ifconfig -a                                   # 或 ip -details link show can0，确认 UP 与位率
ip -s -d link show can0                       # 看 RX/TX 错误计数，判断总线健康
candump can0                                  # 抓所有帧：ID#数据
candump can0,201:7FF                         # 只看 0x201（ID 掩码过滤）
cansend can0 200#0064000000000000            # 手发：电机1 电流 raw=100（约 0.12A）
ip link set can0 down                         # 停总线
```

### 6.2 帧格式速查（C620 电调）

| 帧 ID | 方向 | 含义 |
|---|---|---|
| 0x200 | 发送 | 1~4 号电机电流，4×int16 大端 |
| 0x1FF | 发送 | 5~8 号电机电流（GM6020 电压帧用 0x1FF/0x2FF） |
| 0x201~0x204 | 接收 | 反馈：机械角(0~8191) / 转速 rpm / 电流 mA / 温度 |

电流映射：raw ∈ [-16384, 16384] ↔ [-20A, +20A]，即 `raw = mA * 16384 / 20000`。

### 6.3 完整 C 示例

```c
/* can_motor.c —— SocketCAN 驱动 M3508：读线程收反馈，主线程发 0x200 电流帧
 * 编译: gcc can_motor.c -o can_motor -lpthread
 * 运行: sudo ./can_motor can0
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

static void *rx_thread(void *arg)            /* 相当于 RT-Thread 的 rx_indicate 回调 */
{
    int s = *(int *)arg;
    struct can_frame f;
    while (g_run) {
        if (read(s, &f, sizeof(f)) != (int)sizeof(f)) continue;
        if (f.can_id >= 0x201 && f.can_id <= 0x204 && f.can_dlc >= 6) {
            int angle = (f.data[0] << 8) | f.data[1];        /* 0~8191 对应 0~360 度 */
            int rpm   = (int16_t)((f.data[2] << 8) | f.data[3]);
            int ma    = (int16_t)((f.data[4] << 8) | f.data[5]);
            printf("[0x%03x] ang=%4d rpm=%5d mA=%5d t=%d\n",
                   f.can_id, angle, rpm, ma, f.data[6]);
        }
    }
    return NULL;
}

static int send_currents(int s, const int16_t c[4])  /* 0x200：4 个 int16 大端电流 */
{
    struct can_frame f = { .can_id = 0x200, .can_dlc = 8 };
    for (int i = 0; i < 4; i++) {
        f.data[2 * i]     = (uint8_t)(c[i] >> 8);
        f.data[2 * i + 1] = (uint8_t)c[i];
    }
    return write(s, &f, sizeof(f));
}

int main(int argc, char **argv)
{
    const char *ifname = argc > 1 ? argv[1] : "can0";
    signal(SIGINT, on_sig);

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct ifreq ifr = { 0 };
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);                     /* 接口名转编号 */

    struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    pthread_t th;
    pthread_create(&th, NULL, rx_thread, &s);

    int16_t c[4] = { 0 };
    while (g_run) {                                   /* 三角波扫描 1 号电机，约 ±2A，1kHz */
        for (int v = 0;     v <= 1600 && g_run; v += 8) { c[0] = v; send_currents(s, c); usleep(1000); }
        for (int v = 1600;  v >= -1600 && g_run; v -= 8) { c[0] = v; send_currents(s, c); usleep(1000); }
        for (int v = -1600; v < 0 && g_run; v += 8)    { c[0] = v; send_currents(s, c); usleep(1000); }
    }
    pthread_join(th, NULL);
    close(s);
    return 0;
}
```

### 6.4 对照 RT-Thread

| RT-Thread | SocketCAN | 说明 |
|---|---|---|
| `rt_device_find("can1")` | socket + bind（按接口名） | 打开设备 |
| `rt_device_set_rx_indicate` 回调 + 信号量 | 独立线程阻塞 `read()` | 内核已把帧放进 socket 队列 |
| `rt_device_write` 发帧 | `write()` 整个 `can_frame` | 一帧一次 |
| 过滤器 `can_filter` | `setsockopt(CAN_RAW_FILTER)` | 只收关心的 ID，减 CPU |
| 中断里不能久留 | 用户态随便 printf | 收发都在进程上下文 |

### 动手实验

1. `candump can0` 挂后台，`cansend` 手发 0x200 帧，观察电机反馈 0x201 内容变化。
2. 编译运行 `can_motor.c`，电机按三角波转动，同时另一个终端 `ip -s -d link show can0` 看错误计数是否为 0。

**过关标准**：反馈帧里 rpm 方向与电流符号一致；错误计数不涨；能把 1kHz 发送改成 500Hz 并在 candump 中验证。

---

## 第 7 章 GPIO 与 RGB 灯

本章解决：**没有 rt_pin_write 的日子怎么点灯**。Linux 把 GPIO 藏在驱动后面，用户态通过 sysfs 文件或 gpiod 字符设备访问——本质是"写文件即设电平"，适合 RGB 状态灯这种低速场景。

### 7.1 sysfs 方式（最直观）

```bash
echo 332 > /sys/class/gpio/export        # 导出引脚（编号 = chip 基址 + 行号，查原理图）
echo out > /sys/class/gpio/gpio332/direction
echo 1   > /sys/class/gpio/gpio332/value   # 输出高
cat /sys/class/gpio/gpio332/value          # direction=in 时读电平
echo 332 > /sys/class/gpio/unexport
```

### 7.2 gpiod 命令行（新接口，推荐查脚用）

```bash
gpiodetect                          # 列出 gpiochip
gpioinfo gpiochip0                  # 每行电平/方向/消费者，找空闲脚
gpioset gpiochip0 12=1              # chip0 行 12 输出高
gpioget gpiochip0 12
```

sysfs 已标记废弃但仍在用；gpiod 对应 `libgpiod`，C 接口是 `gpiod_chip_open/line_set_value`，本文用 sysfs 讲原理。

### 7.3 C 示例：RGB 轮换

```c
/* rgb_gpio.c —— sysfs GPIO 控制 RGB
 * 编译: gcc rgb_gpio.c -o rgb_gpio
 * 运行: sudo ./rgb_gpio <R脚> <G脚> <B脚>
 */
#include <stdio.h>
#include <stdlib.h>

static int gpio_write(int num, const char *file, const char *val)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/%s", num, file);
    FILE *fp = fopen(path, "w");
    if (!fp) { perror(path); return -1; }
    fputs(val, fp);
    fclose(fp);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 4) { fprintf(stderr, "用法: %s <R> <G> <B>\n", argv[0]); return 1; }
    int pin[3] = { atoi(argv[1]), atoi(argv[2]), atoi(argv[3]) };
    char buf[16];

    for (int i = 0; i < 3; i++) {                 /* 导出并设为输出 */
        snprintf(buf, sizeof(buf), "%d", pin[i]);
        FILE *fp = fopen("/sys/class/gpio/export", "w");
        if (!fp || fputs(buf, fp) < 0) { perror("export"); return 1; }
        fclose(fp);
        gpio_write(pin[i], "direction", "out");
    }

    for (int step = 0; ; step = (step + 1) % 3) { /* R->G->B 轮流亮，1s 一换 */
        gpio_write(pin[0], "value", step == 0 ? "1" : "0");
        gpio_write(pin[1], "value", step == 1 ? "1" : "0");
        gpio_write(pin[2], "value", step == 2 ? "1" : "0");
        sleep(1);
    }
}
```

注意：共阳/共阴决定 1 是灭还是亮；呼吸灯需要 PWM（sysfs 无 PWM，用 `gpiod` 线配置或内核 pwm 子系统，本文不展开）。

### 动手实验

1. 纯 shell 三条命令点亮一个 LED，再用 `gpiodetect/gpioinfo` 找到同一引脚的 chip/行号。
2. 跑 `rgb_gpio.c`，把三个脚编号换成板上 RGB 实际引脚。

**过关标准**：shell 与 C 两种方式都能亮/灭；在代码注释里记下 chip 基址与行号换算关系。

---

## 第 8 章 进程间通信 IPC

本章解决：**控制进程和视觉/云台/超电进程怎么说话**。视觉常用 Python 且要独立重启，控制环不能被它拖死——拆成多进程后就地取材：小命令用 mqueue，高频大块数据用共享内存。

### 8.1 选型

| 机制 | 消息边界 | 开销 | 适用 |
|---|---|---|---|
| pipe | 无（字节流） | 每次拷贝 | 单向日志、简单指令 |
| POSIX mqueue | 有，带优先级 | 拷贝 | 速度指令等小包，天然限流 |
| shm + 信号量 | 自定结构 | 零拷贝 | 位姿、图像等高频大数据 |
| Unix socket | 流/报文 | 拷贝 | 将来跨机器再考虑 |

pipe 最小示例（父子进程）：

```c
int fd[2]; pipe(fd);
if (fork() == 0) { close(fd[1]); char b[64]; int n = read(fd[0], b, 63); b[n] = 0; printf("%s", b); _exit(0); }
close(fd[0]); write(fd[1], "ping\n", 5); wait(NULL);
```

### 8.2 POSIX 消息队列完整示例

```c
/* ipc_mq.c —— 父进程发 cmd，子进程收（fork 出来的两个"进程"）
 * 编译: gcc ipc_mq.c -o ipc_mq -lrt
 */
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <mqueue.h>
#include <unistd.h>
#include <sys/wait.h>

#define QNAME "/chassis_cmd"
typedef struct { float vx; float wz; } cmd_t;

int main(void)
{
    struct mq_attr attr = { .mq_maxmsg = 16, .mq_msgsize = sizeof(cmd_t) };
    mqd_t q = mq_open(QNAME, O_CREAT | O_RDWR, 0600, &attr);
    if (q == (mqd_t)-1) { perror("mq_open"); return 1; }

    pid_t pid = fork();
    if (pid == 0) {                                    /* 接收方：1kHz 控制进程 */
        cmd_t c; unsigned prio;
        for (int i = 0; i < 5; i++) {
            if (mq_receive(q, (char *)&c, sizeof(c), &prio) < 0) { perror("mq_receive"); _exit(1); }
            printf("[recv] vx=%.2f wz=%.2f prio=%u\n", c.vx, c.wz, prio);
        }
        _exit(0);
    }
    cmd_t c = { .vx = 1.0f, .wz = 0.5f };              /* 发送方：视觉/遥控解析进程 */
    for (int i = 0; i < 5; i++) {
        c.vx += 0.5f;
        if (mq_send(q, (const char *)&c, sizeof(c), 1) < 0) perror("mq_send");
        usleep(100000);
    }
    wait(NULL);
    mq_close(q);
    mq_unlink(QNAME);
    return 0;
}
```

队列残留可 `ls /dev/mqueue/` 查看；`mq_maxmsg` 满后 `mq_send` 阻塞（`O_NONBLOCK` 则返回 `EAGAIN`）——发送方丢帧策略在此定。

### 8.3 共享内存 + 信号量简例

```c
/* ipc_shm.c —— 生产者写位姿，消费者读
 * 编译: gcc ipc_shm.c -o ipc_shm -lrt
 */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/wait.h>

typedef struct { long seq; double pose[3]; } shm_t;     /* 双方约定的内存布局 */

int main(void)
{
    int fd = shm_open("/chassis_shm", O_CREAT | O_RDWR, 0600);
    ftruncate(fd, sizeof(shm_t));
    shm_t *p = mmap(NULL, sizeof(shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    sem_t *sem = sem_open("/chassis_sem", O_CREAT, 0600, 0);

    if (fork() == 0) {                                  /* 消费者：视觉读底盘位姿 */
        for (int i = 0; i < 5; i++) {
            sem_wait(sem);
            printf("[consumer] seq=%ld x=%.2f\n", p->seq, p->pose[0]);
        }
        _exit(0);
    }
    for (int i = 1; i <= 5; i++) {                      /* 生产者：控制环 1kHz 写 */
        p->seq = i;
        p->pose[0] = i * 1.5;
        sem_post(sem);
        usleep(100000);
    }
    wait(NULL);
    munmap(p, sizeof(shm_t));
    shm_unlink("/chassis_shm");
    sem_close(sem);
    sem_unlink("/chassis_sem");
    return 0;
}
```

残留对象在 `/dev/shm/`；改 `shm_t` 布局必须两端同步重编译（自管 ABI）。

### 8.4 对照 RT-Thread

| RT-Thread | POSIX | 说明 |
|---|---|---|
| `rt_mq_create/send/recv` | `mq_open/mq_send/mq_receive` | 都有边界与深度，`timeout` 换 `mq_timedreceive` |
| `rt_sem_take/release` | `sem_wait/sem_post` | 命名信号量跨进程共享 |
| 邮箱/队列放内核堆 | 对象在 `/dev/mqueue`、`/dev/shm` | 用户态持久，重启要清理 |

### 动手实验

1. 跑 `ipc_mq.c`；把 `mq_maxmsg` 改成 2、发送间隔改 0，观察阻塞/`EAGAIN` 行为。
2. 跑 `ipc_shm.c`；故意只删 `sem_post`，看消费者永久阻塞，再用 `ipcs`/`/dev/shm` 清理残留。

**过关标准**：能口头说清"命令小包用 mq、高频位姿用 shm"的理由；会清理两种残留对象。

---

## 第 9 章 调试与观测

本章解决：**程序跑起来之后黑盒怎么办**。RT-Thread 有线程列表和 `rt_kprintf`；Linux 给你线程级 top、gdb 断点、syscall 追踪、系统日志，以及一个量化实时性的自测工具。

### 9.1 看线程：top / htop

```bash
top -H -p $(pidof can_motor)   # -H 每行一个线程，等价线程列表（PR 列看 RT 优先级）
htop                           # F5 树形；按 P 排 CPU；能看到 SCHED_FIFO 优先级
```

### 9.2 gdb 基本用法

| 命令 | 作用 |
|---|---|
| `gcc -g xxx.c` | 带调试符号编译 |
| `gdb ./can_motor` 后 `b can_motor.c:60` / `b send_currents` | 下断点 |
| `r` / `c` / `n` / `s` | 运行 / 继续 / 下一行 / 进入函数 |
| `bt` / `t a a bt` | 崩溃后看调用栈 / 所有线程的栈 |
| `p rpm` / `p *c@4` | 打印变量 / 数组 |
| `gdb -p $(pidof x)` | attach 运行中的进程（会冻住全部线程，1kHz 环会丢拍） |

### 9.3 strace 与 journalctl

```bash
strace -c ./jitter 1000 1000                     # syscall 计数+耗时汇总
strace -p $(pidof can_motor) -e trace=read,write,clock_nanosleep -T
journalctl -b -k | grep -i -E "can|thermal"      # 本次启动的内核日志
journalctl -f -u my.service                        # 跟踪某服务
journalctl --since "10 min ago"
```

### 9.4 周期抖动自测（cyclictest 原理极简版）

```c
/* jitter.c —— 测每拍实际周期：min/avg/max + 直方图
 * 编译: gcc -O2 jitter.c -o jitter
 * 运行: sudo chrt -f 80 ./jitter 1000 5000    # 1kHz、5000 拍、SCHED_FIFO 80
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv)
{
    long period_us = argc > 1 ? atol(argv[1]) : 1000;
    long loops     = argc > 2 ? atol(argv[2]) : 5000;
    long step_us   = 50;                        /* 直方图桶宽 */
    enum { NBUCKET = 12, CENTER = 2 };          /* 覆盖 period-100us ~ period+500us */
    long hist[NBUCKET] = { 0 };
    long min_us = 1L << 40, max_us = 0, sum_us = 0;
    struct timespec next, now, prev;

    clock_gettime(CLOCK_MONOTONIC, &next);
    prev = next;

    for (long i = 0; i < loops; i++) {
        next.tv_nsec += period_us * 1000;       /* 绝对时间睡眠：误差不累积 */
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        clock_gettime(CLOCK_MONOTONIC, &now);

        long p_us = (now.tv_sec - prev.tv_sec) * 1000000L
                  + (now.tv_nsec - prev.tv_nsec) / 1000;   /* 本拍实际周期 */
        prev = now;
        if (i == 0) continue;                   /* 首拍校准，不计入 */
        if (p_us < min_us) min_us = p_us;
        if (p_us > max_us) max_us = p_us;
        sum_us += p_us;
        long idx = (p_us - (period_us - CENTER * step_us)) / step_us;
        if (idx < 0) idx = 0;
        if (idx >= NBUCKET) idx = NBUCKET - 1;  /* 超范围并入末桶 */
        hist[idx]++;
    }

    printf("period=%ldus loops=%ld\n", period_us, loops);
    printf("min=%ld avg=%.1f max=%ld (jitter=%ldus)\n",
           min_us, (double)sum_us / (loops - 1), max_us, max_us - min_us);
    for (int i = 0; i < NBUCKET; i++) {
        long lo = period_us - CENTER * step_us + i * step_us;
        long bar = hist[i] * 40 / (loops - 1);
        printf("%6ld..%6ld us | %5ld ", lo, lo + step_us, hist[i]);
        for (long k = 0; k < bar; k++) putchar('#');
        putchar('\n');
    }
    return 0;
}
```

原理与 cyclictest 相同：绝对时间 `clock_nanosleep` + 前后 `clock_gettime` 差值即抖动。差别只是它一次只测一个周期、不做多线程统计。

### 动手实验

1. 三种条件各跑一次并记录成表：`./jitter`、`sudo chrt -f 80 ./jitter`、后者加后台 `stress-ng --cpu 8`。
2. `strace -c ./jitter` 数一拍内的 syscall；`gdb` attach `can_motor`，在 `send_currents` 断点后 `bt`。

**过关标准**：SCHED_FIFO + 空载下 `max-min < 100us`（RT 内核）；能解释普通调度在 CPU 压力下长尾来自哪里；循环体内除 sleep/gettime 外无其他 syscall。

---

## 第 10 章 四周学习计划

本章解决：**怎么把 1~9 章排进一个月**。节奏是工作日三个晚上各 1 小时读章做小实验，周末 3 小时做里程碑集成；所有代码入库，抖动数据记进 `docs/latency_log.md`。

| 周 | 学习章节 | 周中实验 | 周末里程碑 |
|---|---|---|---|
| 1 | 复习 1~3 章：登录/文件/权限/构建 | ssh+tmux 上板；apt 装 gcc/gdb/can-utils；写 CMake 编 hello 并 scp 上板 | 板上一条命令从源码到运行，产出二进制 |
| 2 | 4~5 章：调度 + termios | SCHED_FIFO 线程空转 1kHz；termios 按帧解析 DBUS 原始字节 | 终端实时打印摇杆/拨杆通道值；对比两种调度策略 |
| 3 | 6~7 章：CAN + GPIO | candump/cansend 手动收发；跑 can_motor.c 三角波；GPIO 点 RGB | 电机开环点动 + RGB 指示状态，反馈帧稳定 |
| 4 | 8~10 章：IPC + 调试 | ipc_mq / ipc_shm 双进程传数据；jitter 三条件测试 | 集成小项目：1kHz 控制环 + DBUS 遥控 + CAN 电机，抖动记录在案 |

集成小项目验收：断开 DBUS 自动置零电流；`htop` 可见控制线程 RT 优先级；Ctrl+C 干净退出（电机停转、总线 down）。

---

*完。上半部分见 linux_guide_part1.md（第 1~5 章）。*
