# 三步上手：进程 → GPIO → 自启

> 目标：跑通"自己编译 → 板上运行 → 控制硬件 → 开机自启"全流程，等效于 MCU 开发的最小闭环。
> 对应学习指南章节：步骤1→第3章，步骤2→第7章，步骤3→第2章（`docs/linux_study_guide.md`）。
> 完成每步后，把结果登记到 `docs/rdk_x5_board_log.md` 第四节状态表。

## 步骤 1：跑起来第一个自己编译的进程

1. **传文件**：MobaXterm 左侧 SFTP 面板，把 `hello_file.c` 拖进板子 `/root/`（或任意临时目录）。

2. **SSH 到板子上编译运行**：

   ```bash
   mkdir -p /data/hello_app && cd /data/hello_app
   cp /root/hello_file.c .                # 挪到正式目录（程序里也会自己 mkdir，双保险）
   gcc hello_file.c -o hello_file         # 没有 gcc 就 apt install -y gcc
   ./hello_file
   ```

3. **验证**（预期输出）：

   ```bash
   cat /data/hello_app/hello.txt
   helloworld
   ```

   **过关标准**：cat 看到 helloworld；`ls -l hello_file` 是你刚编译出的可执行文件。

## 步骤 2：GPIO 高低电平翻转

> **本板结论（已考古实证）**：BCM GPIO5 = 物理脚 29 = gpiochip4（base 379）+ line 20 = **sysfs 全局号 399**。
> 证据链：`Hobot.GPIO` 官方库 `gpio_pin_data.py` 的 `RDK_X5_PIN` 表 + strace 抓到官方库实际写 `/sys/class/gpio/gpio399/*`。
> 编号体系见学习指南/聊天记录：物理脚号（接线用）/ BCM 名字（表格用）/ 内核全局号（代码用）三套互不相干。

### 2.1 换其他引脚时怎么查（方法论，编号必须自己查，不能抄）

- 官方 40-pin 文档（BCM 名 ↔ 物理脚）：
  https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/40pin_define?v=3.5.0&p=RDK+X5
- 板上权威映射表（BCM → 全局号）：
  ```bash
  grep -A2 -B2 "GPIOx" /usr/local/lib/python3.10/dist-packages/Hobot/GPIO/gpio_pin_data.py
  # 行格式: ["soc/...gpio", 全局号, 物理脚, BCM号, 'GPIOx', '复用功能', ...]
  ```
- 编号空间：`for c in /sys/class/gpio/gpiochip*; do echo "$c base=$(cat $c/base)"; done`

### 2.2 shell 手动点一遍（先验证引脚可用，再写代码——调试基本法）

```bash
echo 399 > /sys/class/gpio/export            # 已导出会报 busy，无妨，继续
echo out > /sys/class/gpio/gpio399/direction
echo 1 > /sys/class/gpio/gpio399/value       # 万用表（黑笔在 6 脚 GND）量 29 脚：应 3.3V
echo 0 > /sys/class/gpio/gpio399/value       # 应回 0V
cat /sys/class/gpio/gpio399/value            # 读回当前电平
echo 399 > /sys/class/gpio/unexport          # 用完释放
```

### 2.3 跑 C 程序

1. 硬件：**物理 29 脚** → 330Ω 电阻 → LED → GND（6/9/14/20/25/30/34/39 任一脚；或直接万用表量）。
2. `gpio_toggle.c` 的 `GPIO_N` 已写好 **399**（含"已导出容错"逻辑，无需再改）。
3. 传到板上，编译运行：

   ```bash
   cd /data/hello_app
   gcc gpio_toggle.c -o gpio_toggle
   ./gpio_toggle        # LED 应 1 秒一闪；Ctrl+C 停
   ```

**过关标准**：LED 以 1Hz 稳定闪烁；终端每秒打印 GPIO 状态。

## 步骤 3：开机自启（像 MCU 一样）

```bash
cd /data/hello_app
cp hello_gpio.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now hello_gpio.service
systemctl status hello_gpio              # Active: active (running) 即成功
journalctl -u hello_gpio -f              # 看 GPIO 翻转日志，Ctrl+C 退出
```

**终极验证**：`reboot`，板子起来后**不登录任何东西**，LED 自己开始闪 —— 这就是 Linux 版的"上电即跑"。

**过关标准**：重启后 LED 自动闪烁；`systemctl status` 显示 running。

## 常见坑速查

| 现象 | 原因/解决 |
|------|-----------|
| export 报 `Device or resource busy` | 引脚被设备树占用，换一个 |
| gcc: command not found | `apt install -y gcc`（连了热点才有网） |
| service 起不来 | `journalctl -u hello_gpio -n 50` 看原因；检查 ExecStart 路径是否绝对路径 |
| 改了 .c 没生效 | 重新 gcc；service 方式还要 `systemctl restart hello_gpio` |
| 板子时钟显示 2000 年 | 无 RTC 电池，联网后 NTP 自动校；apt 报证书错先 `date -s` 校时 |

## 全部完成后

1. 更新 `docs/rdk_x5_board_log.md`：状态表打勾 + 按"操作记录模板"补三条操作记录（#4/#5/#6）。
2. 下一站：把这套骨架换成 1kHz 控制环（学习指南第 4 章的模板），接上 SocketCAN（第 6 章）点亮第一个电机。

## 下一站：1kHz 周期实时模板（三步上手收官后）

材料：`x5_app/period_1khz.c`（学习指南第 4 章：SCHED_FIFO + 绑核 + mlockall + clock_nanosleep TIMER_ABSTIME）。

```bash
cd /data/hello_app
gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread
sudo ./period_1khz             # abs 模式（绝对时间睡眠）跑 180 秒
sudo ./period_1khz sleep 180   # usleep 对照组：亲眼看累积漂移
```

过关标准：abs 模式唤醒延迟 max < 100 µs、180 秒拍数 ≈ 180000；sleep 模式拍数明显少（差 > 1000）。
程序自带统计/直方图/判定，跑完把输出复制粘贴回对话（不要截图），登记进病历本操作 #8。

**实测记录（2026-09-10）**：v1 首轮唤醒延迟 123µs FAIL → v2 加 cpu_dma_latency 压深 idle 后 **23µs PASS**（jitter 29µs、180s 零漂移）；sleep 对照组 180s 少跑 1425 拍（每拍多耗 7.5µs，单向累积）。
