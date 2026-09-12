# RDK X5 迁移第一阶段跟做手册：从零到 1kHz 实时骨架

> **本文性质**：本板（2026 赛季英雄队 X5）从"刷好系统"到"1kHz 实时控制骨架达标"的完整实战记录。
> 所有命令、数据、坑都是真实踩出来的——跟着做即可复现。
> 与 [实现过程实录](rdk_x5_stage1_impl_log.md) 的分工：实录记录"怎么一步步做出来"（含命令回传与弯路），本文是提炼后的复现路径。
> 配套材料：源码在 `x5_app/`；原理详解见 [学习指南](linux_study_guide.md)（第 2/3/4/7 章）；完整操作历史见 [板子病历本](rdk_x5_board_log.md)（操作 #4~#8）。
> 最后更新：2026-09-10（PC 侧时间）

## 0. 这一阶段做完你会得到什么

| 里程碑 | 验收标志 | 本板实测 |
|---|---|---|
| ① 第一个自编进程 | 板上 gcc 编译运行 hello_file | ✅ 操作 #4 |
| ② GPIO 控硬件 | C 代码翻转 LED 1Hz | ✅ 操作 #5/#6 |
| ③ 开机自启 | reboot 不登录 LED 自闪 | ✅ 操作 #7 |
| ④ 1kHz 实时骨架 | 唤醒延迟 <100µs 且零漂移 | ✅ **23µs**，操作 #8 |

一句话总结第四步的成果：**SCHED_FIFO + TIMER_ABSTIME + mlockall + 绑核 + cpu_dma_latency 五件套**，
唤醒延迟 max 23µs、周期 jitter 29µs、180 秒 18 万拍零漂移——这就是未来底盘控制环的计时骨架。

## 1. 前置条件

| 项 | 要求 | 说明 |
|---|---|---|
| 板卡 | RDK X5（Sunrise5，8×A55），Ubuntu 22.04 | 串口 COM6(CH340) 留作救砖/看日志 |
| 内核 | **PREEMPT_RT 实时内核** | 验证：`uname -v` 含 `PREEMPT_RT`；切换方法见病历本操作 #1（改 boot.cmd + mkimage，可回滚） |
| 网络 | 板子与 PC 同网，SSH 可达 | 我们用"PC 开移动热点、板子 wlan0 连上"方案；IP 由 DHCP 分配 |
| 工具 | MobaXterm（SSH + 左侧 SFTP 面板） | 或任意 SSH/SFTP 客户端 |
| 编译 | 板上自带 gcc | 无需交叉编译、无需装包 |

**目录约定**（全程遵守）：
- `/data/hello_app/` —— 源码与产物的正式家
- `/root/` —— SFTP 中转站（传完就 `cp` 走、删掉）
- PC 侧工作区 `x5_app/` 存所有源码的"母本"

**工作流铁律**：AI 助手/文档给的命令块里凡有"先上传文件"的前置动作，**必须先做完再粘贴命令**——本阶段两次翻车（service 文件、period_1khz.c）都是跳过上传直接粘贴，报一串 `No such file or directory`。

## 2. 步骤一：第一个自编进程（10 分钟）

**目标**：打通"传码 → 板上编译 → 运行 → 验证"闭环（等价于 MCU 的"点亮工程"）。

1. SFTP 把 `x5_app/hello_file.c` 拖到板子 `/root/`
2. SSH 执行：

```bash
mkdir -p /data/hello_app && cd /data/hello_app
cp /root/hello_file.c .
gcc hello_file.c -o hello_file
./hello_file        # 预期输出 [OK] 已写入 /data/hello_app/hello.txt
cat hello.txt       # 预期 helloworld
```

**原理一句话**：Linux 上"跑程序"就是普通的可执行文件 + shell，gcc 直接出 aarch64 本机码，没有 Keil/烧录环节。

## 3. 步骤二：GPIO 控硬件（含编号考古）

**目标**：C 代码让一个引脚输出方波，LED 1Hz 闪烁。

### 3.1 三套编号体系（最大的坑，先背下来）

| 体系 | 用在哪 | 本板例子 |
|---|---|---|
| 物理脚号 | **接线**看这个 | 29 脚 |
| BCM 名 | 官方表格里叫这个 | GPIO5 |
| 内核全局号 | **代码**写这个（sysfs） | **399** |

三套互不相干！`echo 5 > export` 报 Invalid argument 就是因为把 BCM 名当全局号用了。

**本板定论（已三重实证：官方库源码表 + strace + 实测点灯）**：

> BCM GPIO5 = 物理脚 29 = gpiochip4（34120000.gpio，base 379）+ line 20 = **sysfs 全局号 399**
> GND 脚：6/9/14/20/25/30/34/39（**5 脚不是地，是 GPIO3**）

### 3.2 换其他引脚怎么查（方法论，不能抄别人的号）

```bash
# 权威映射表（BCM → 全局号），在板上执行：
grep -A2 -B2 "GPIOx" /usr/local/lib/python3.10/dist-packages/Hobot/GPIO/gpio_pin_data.py
# 行格式: ["soc/...gpio", 全局号, 物理脚, BCM号, 'GPIOx', '复用功能', ...]

# 编号空间全景（类似 STM32 的 GPIOA~F 多组端口）：
for c in /sys/class/gpio/gpiochip*; do echo "$c base=$(cat $c/base)"; done
```

### 3.3 先 shell 手动点一遍（调试基本法：先验证介质，再写代码）

```bash
echo 399 > /sys/class/gpio/export            # 已导出报 busy，无妨
echo out > /sys/class/gpio/gpio399/direction
echo 1 > /sys/class/gpio/gpio399/value       # 万用表量 29 脚 vs GND：3.3V
echo 0 > /sys/class/gpio/gpio399/value       # 0V
```

### 3.4 上 C 程序

SFTP 传 `x5_app/gpio_toggle.c` → `gcc gpio_toggle.c -o gpio_toggle` → `./gpio_toggle`。
硬件：物理 29 脚 → 330Ω → LED → GND。

**验收**：LED 稳定 1Hz 闪，终端每秒打印电平。
**原理一句话**：Linux 把 GPIO 抽象成 `/sys/class/gpio` 下的文件，写文件 = 控引脚——这就是"一切皆文件"。

**万用表教训**：手动 export 后量到 0V 不代表引脚坏——先怀疑表笔/杜邦线接触（我们冤枉过一次）。

## 4. 步骤三：开机自启（像 MCU 上电即跑）

**目标**：reboot 后不登录，LED 自己闪。

### 4.1 service 文件（全文就这么多，`x5_app/hello_gpio.service`）

```ini
[Unit]
Description=RDK X5 first app: GPIO toggle (hello world of embedded Linux)
After=multi-user.target

[Service]
Type=simple
ExecStart=/data/hello_app/gpio_toggle
Restart=always          # 进程崩了 2 秒后自动拉起（MCU 看门狗的味道）
RestartSec=2

[Install]
WantedBy=multi-user.target
```

### 4.2 安装启用（**先 SFTP 传文件**，再粘贴）

```bash
cp /root/hello_gpio.service /etc/systemd/system/
systemctl daemon-reload                          # 让 systemd 重新扫描 unit 文件
systemctl enable --now hello_gpio.service        # enable=开机自启(建软链) now=顺带立刻启动
systemctl status hello_gpio --no-pager           # 看 Active: active (running)
rm /root/hello_gpio.service                      # 清中转
reboot                                            # 终极验证！
```

reboot 后**不登录**，等 30~60 秒看 LED。

**三条命令的原理**：`/etc/systemd/system/` 是 systemd 认的管理员 unit 目录；`daemon-reload` 让它重扫磁盘（不 reload 它不知道有新文件）；`enable` 本质是在 `multi-user.target.wants/` 建软链接，开机到 multi-user 阶段顺带拉起。服务里的 printf 进 journald（`journalctl -u hello_gpio -f` 实时看），不上串口控制台——所以不登录时 **LED 是唯一证据**。

### 4.3 踩坑实录：自启" mysteriously 失败"

**现象**：手动 `./gpio_toggle` 好的，reboot 后不闪。
**排障三连**（按顺序执行，哪步暴露问题查哪步）：

```bash
ls -l /etc/systemd/system/hello_gpio.service   # 文件在吗？
systemctl status hello_gpio --no-pager         # 状态？enabled？running？
journalctl -u hello_gpio -n 50 --no-pager      # 服务日志说什么？
```

**我们这次的真相**：`ls /data/hello_app/` 里压根没有 service 文件——**它从未被 SFTP 传上板**，自启自然无从谈起。不是 systemd 的问题，不是权限问题。
**教训**：① 命令块粘贴前先完成前置上传；② `systemctl status` 显示 running 只代表"现在活着"，`enabled` 只代表"开机想拉"，**唯一完整证据是 reboot 不登录看现象**（= MCU 的冷启动测试思维）。

## 5. 步骤四：1kHz 实时骨架（本阶段核心）

**目标**：在 Linux 上拿回 RT-Thread "高优先级线程 + 硬定时器" 的 1kHz 确定性。

### 5.1 为什么 sleep(1ms) 不行、TIMER_ABSTIME 行

- `usleep(1000)` 语义是"醒来**再**睡 1ms"→ 每周期 = 计算 + 调度延迟 + 1ms，**误差逐拍累积**，而且"至少睡"语义决定它永远不会早醒（实测 min 周期 1006µs，从没低于 1000）
- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next)` 睡到**绝对时刻**：本拍算得慢就晚睡少睡，锚点序列不动 → 零漂移；迟到拍之后会出现"短补"（实测 min 988µs），这就是绝对锚点的签名

### 5.2 五件套（缺一件都会掉指标）

| 手段 | 作用 | 对应 RT-Thread 概念 |
|---|---|---|
| `SCHED_FIFO` prio=80 | 就绪即抢占，不被普通任务压 | 高优先级线程 |
| `clock_nanosleep(TIMER_ABSTIME)` | 绝对时间睡眠，零漂移 | 硬定时器超时 |
| `mlockall()` | 锁内存，防换页缺页抖动 | MCU 没有 swap 可言 |
| `pthread_setaffinity_np` 绑核 | 防核间迁移抖动 | 单核绑核 |
| **`/dev/cpu_dma_latency`=10µs** | 压住深度 idle 退出延迟 | MCU 不睡觉 |

**最后一件是我们实测踩出来的**：周期任务每拍只干几 µs 活就睡 1ms，核趁机掉进深度省电态，从深 idle 爬出来要几十~上百 µs——首轮实测 123µs FAIL 的主因。程序里 `open` 该文件并 `write` 一个 int32 目标延迟，**fd 保持开着**约束才生效。

### 5.3 跑法（源码：`x5_app/period_1khz.c`，先 SFTP 上传！）

```bash
systemctl stop hello_gpio        # 测量期去掉 1Hz 服务噪声，测完记得 start 回来
echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor   # 定频
cp /root/period_1khz.c /data/hello_app/ && cd /data/hello_app
gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread
sudo ./period_1khz              # abs 模式 180s（正式测量），启动应有 "cpu_dma_latency = 10 us"
sudo ./period_1khz sleep 180    # usleep 对照组：亲眼看累积漂移
```

程序自动报：每 10s 进度、拍数/预期、周期 min/avg/max、唤醒延迟 max、50µs 分桶直方图、PASS/FAIL 判定；还会自检内核是否 PREEMPT_RT。

### 5.4 实测数据（供对照，你的板子应同量级）

| | abs（定版） | sleep（对照） |
|---|---|---|
| 唤醒延迟 max | **23 µs**（首轮未压 idle 时 123µs） | — |
| 180s 拍数 | 180016 / 180017（差 1） | 178591（**少 1425 拍**） |
| 平均周期 | 999.4 µs | 1007.5 µs（每拍多耗 7.5µs） |
| 周期 min/max | 988 / 1017 | 1006 / 1039 |
| 直方图 | 全落 950~1050 两桶 | 单桶右移到 1000~1050 |

读图要点：abs 直方图以 1000µs 为中心对称收拢；sleep 直方图整体右移且**永远不越回 1000 以下**——单向漂移的直观形状。
0.79% 的慢对控制环意味着：1kHz 积分速度/里程，180 秒偏出 1.4 秒的量——所以周期必须锚在绝对时间上。

### 5.5 如果你的唤醒延迟超标，排查顺序

1. 有没有 `sudo`（SCHED_FIFO 设置失败会打警告）？
2. `uname -v` 是不是 PREEMPT_RT？
3. 启动有没有打出 `cpu_dma_latency = 10 us`？（深 idle 是最大嫌疑）
4. 换个空闲核（`htop` 看占用，改源码 `CPU_N`）
5. 测量期停掉无关服务、governor 定频 performance
6. 还不行 → 内核启动参数隔离（`isolcpus=3 nohz_full=3 rcu_nocbs=3`，改动大，最后再用）

## 6. 常见坑速查（本阶段真实遇到过）

| 现象 | 原因/解决 |
|---|---|
| 粘贴命令后一串 `No such file or directory` | **前置的 SFTP 上传没做**——先传文件、肉眼确认面板里有，再粘贴 |
| `echo 5 > export` 报 Invalid argument | 把 BCM 名当全局号；查法见 3.2 |
| export 报 `Device or resource busy` | 引脚被设备树占用，换一个 |
| 手动量电平 0V 但程序能点亮 | 万用表表笔接触问题，先怀疑测量再怀疑板子 |
| 板子时钟显示 2000 年 | 无 RTC 电池，联网 NTP 自校；期间 apt 报证书错先 `date -s` 校时 |
| 串口粘贴出 `[[200~` 乱码 | bracketed-paste 残留，无害，Ctrl+U 清行重粘 |
| service 装了没自启 | 按 4.3 排障三连查；文件到底上没上板？ |
| 唤醒延迟 100µs+ | 见 5.5 排查链，深 idle 是头号嫌疑 |
| SFTP 传完找不到文件 | SFTP 默认落家目录 `/root/`，不是当前 SSH 所在目录 |

## 7. 下一站预告：SocketCAN 点电机（指南第 6 章）

硬件盘点清单：① CAN 接口方案（X5 的 40-pin **没有** CAN 复用脚，考古结论只有 UART/SPI/PWM/I2C——大概率走 USB-CAN 适配器）；② C620 电调 + M3508 + 24V 动力电；③ 120Ω 终端电阻（短线调试可暂免）。
上车第一课三板斧：`ip link set can0 up type can bitrate 1000000` → `candump can0` → `cansend can0 200#...`，然后把 0x200 发帧 / 0x201 收反馈填进本阶段的 1kHz 循环体。

---

*本文由 2026-09 迁移实战整理；数据原始记录见 [rdk_x5_board_log.md](rdk_x5_board_log.md) 操作 #4~#8。*
