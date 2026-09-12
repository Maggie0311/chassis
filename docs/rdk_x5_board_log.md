# RDK X5 板子操作记录

> **用途**：记录对这块 RDK X5 做过的所有操作、创建过的文件/目录，作为"板子病历本"。
> **维护规则**：每对板子做一次新的操作（改配置/装软件/建目录/刷内核……），就在「操作时间线」追加一节，并在「文件清单」登记。
> 最后更新：操作 #8 完成——1kHz 模板 v2 复测 PASS（唤醒延迟 23us / jitter 29us / 零漂移）；下一站 SocketCAN 点电机（第 6 章）（日期以板子 `date` 为准，板子无 RTC，联网前显示 2000-01-01）

---

## 一、板子基本信息

| 项目 | 内容 |
|------|------|
| 板卡 | RDK X5（地平线 Sunrise 5，8× Cortex-A55） |
| 系统 | Ubuntu 22.04.5 LTS（官方镜像） |
| 内核 | **6.1.83-rt28（PREEMPT_RT 实时内核，已启用）** ← 原为标准内核 6.1.83 |
| 串口 | USB 转 CH340，COM6，控制台 ttyS0 |
| 登录 | root / sunrise |
| 网络 | wlan0 连 Windows 移动热点 `RLLR`，DHCP 分到 **192.168.137.93**（网关 192.168.137.1 = 电脑） |
| 日常访问 | MobaXterm SSH（root@192.168.137.93）；串口留作看日志/救砖 |

## 二、操作时间线

### 操作 #1：启用 PREEMPT_RT 实时内核 ✅

**目的**：标准内核实时性不足，切换到官方预编译实时内核（对应 RDK 文档《应用实时内核》章节）。

执行过的命令（按顺序）：

```bash
ls -lh /boot/Image-rt                                    # 确认实时内核镜像存在（23M）
cp /boot/boot.cmd  /boot/boot.cmd.bak                    # 备份引导脚本源文件
cp /boot/boot.scr  /boot/boot.scr.bak                    # 备份引导脚本二进制
sed -i 's/imagefile="Image"/imagefile="Image-rt"/' /boot/boot.cmd
grep -n imagefile /boot/boot.cmd                         # 验证已改为 Image-rt
which mkimage                                            # → /usr/bin/mkimage（已预装）
cd /boot
mkimage -C none -A arm -T script -d boot.cmd boot.scr    # 重新生成（Data Size 2783 Bytes）
sync && reboot
```

**结果**：重启后登录横幅显示 `GNU/Linux 6.1.83-rt28 aarch64`，切换成功。

**回滚方法**（切回标准内核）：

```bash
cp /boot/boot.cmd.bak /boot/boot.cmd
cd /boot && mkimage -C none -A arm -T script -d boot.cmd boot.scr
reboot
```

**已知问题**：开机日志有一条
`BUG: sleeping function called from invalid context at kernel/locking/spinlock_rt.c:46`（pid 160, kworker/7:1）。
属 RT 内核对厂商驱动的调试告警，非致命，系统正常启动。目前仅开机出现一次，**持续观察**；若运行负载时频繁出现需排查。

### 操作 #2：WiFi 连接（Windows 热点共享） ✅

**目的**：摆脱串口调试，走 SSH。校园网有 portal 认证，故用"电脑连校园网认证 → 开移动热点共享给板子"方案。

- Windows 侧：移动热点 SSID `RLLR`，密码 `12345678`，频带"任何可用频率"，共享来源为已认证的校园网连接；并设置了 MobaXterm 主密码。
- 板上执行：

```bash
sudo nmcli device wifi connect "RLLR" password "12345678"
ip addr show wlan0        # → inet 192.168.137.93/24
```

**结果**：`wlan0 successfully activated`，IP 192.168.137.93（DHCP）。NetworkManager 已保存该连接，**重启自动重连**。

**备用（未执行）**：若嫌 DHCP 换 IP，可固定：

```bash
nmcli connection modify "RLLR" ipv4.method manual \
  ipv4.addresses 192.168.137.200/24 ipv4.gateway 192.168.137.1 ipv4.dns "223.5.5.5"
nmcli connection down "RLLR"; nmcli connection up "RLLR"
```

### 操作 #3：MobaXterm 环境配置（PC 侧） ✅

| 配置项 | 状态 |
|--------|------|
| 串口会话 COM6（CH340） | 保留：U-Boot/救砖/内核日志监视器 |
| SSH 会话 root@192.168.137.93:22 | ✅ 已建，用户名已填，密码已存（主密码加密） |
| 光标 | 颜色已改（可见性 OK） |
| 粘贴警告（paste confirmation） | **保留全部勾选**（多行粘贴过目，防误执行） |
| 已知坑：串口粘贴出 `[[200~` 乱码 | bracketed-paste 残留，无害；粘完看行首，乱码就 Ctrl+U 清行重粘；可用 `bind 'set enable-bracketed-paste off'` 关闭 |

### 操作 #4：编译并运行第一个自研程序 hello_file ✅（三步上手 · 步骤1）

**目的**：跑通"传码 → 板上编译 → 运行 → 验证"流程。

**执行过的操作**：
- SFTP 上传 `x5_app/hello_file.c` → 板上 `/root/`
- `mkdir -p /data/hello_app && cd /data/hello_app`
- `cp /root/hello_file.c .`
- `gcc hello_file.c -o hello_file`（板上 gcc 开箱可用，未装新包）
- `./hello_file` → 输出 `[OK] 已写入 /data/hello_app/hello.txt`

**结果**：`cat /data/hello_app/hello.txt` → `helloworld`，**步骤 1 过关**。
（插曲：粘贴时把 README 预期输出行 `helloworld` 也粘进去被执行，报 command not found，无害。）

**回滚/清理**：`rm -rf /data/hello_app` 即可完全清除（外加 `rm /root/hello_file.c`）。

### 操作 #5：GPIO 编号考古（三步上手 · 步骤2 前置调查） ✅

**目的**：确定 BCM GPIO5（物理 29 脚）在内核里的真实 sysfs 编号。

**调查过程**：
1. `echo 5 > /sys/class/gpio/export` 报 Invalid argument → 表里的 GPIO5 是 BCM 名，不是内核全局号
2. 板上 6 块 gpiochip（base 347/379/411/434/466/498，类似 STM32 的 GPIOA~F 六组端口），`gpioinfo` 显示 40-pin 线全部 unnamed
3. MCP 直读硬件原理图 PDF（`x5_app/RDK_X5_IO_CONN_PUBLIC_V1.1.pdf`）拿到 40-pin 在 SoC 侧真实网络名（LSIO_UART1/2/6/7、LSIO_SPI1/2、LSIO_PWM0~7、LSIO_SDA/SCL 系列）
4. 设备树发现 Pi 风格 `__overrides__`（uart1/2/3/6/7、i2c0/1/5、spi1/2、pwm0~3、dw_i2s1）+ 大量 `pconf-*` 引脚配置节点
5. **决定性证据**：官方库 `Hobot.GPIO`（pip 包，`/usr/local/lib/python3.10/dist-packages/Hobot/GPIO/gpio_pin_data.py`）的 `RDK_X5_PIN` 表（board_id 0x301/0x302）+ strace 抓到它实际写 `/sys/class/gpio/gpio399/*`

**最终结论**：
| 项 | 值 |
|---|---|
| BCM 名 | GPIO5 |
| 物理脚 | **29**（接线用这个；GND = 6/9/14/20/25/30/34/39，**5 脚不是地，是 GPIO3**） |
| 内核定位 | gpiochip4（34120000.gpio，base 379）+ line 20 |
| sysfs 全局号 | **399** |
| 原理图网络 | LSIO_SPI2_SCLK_PWM0_3V3 域（复用功能 SPI2_SCLK / PWM0） |

**教训记录**：
- 手动 `echo 399 > export` + value=1 曾测得 0V，但官方库同机制点亮 → 判定为测量时表笔/杜邦线接触问题；**万用表结论也要建立在接触可靠上**
- 官方库无 pinmux 魔法，走的就是 sysfs；引脚默认复用状态未深究（当前作为 GPIO 可用即可）

**软件变更**：`x5_app/gpio_toggle.c` → `GPIO_N 399` + export 容错（已导出则继续）；README 步骤 2 更新为本板结论。

### 操作 #6：gpio_toggle 编译运行，LED 稳定翻转 ✅（三步上手 · 步骤2）

**过程**：SFTP 落 /root/（SFTP 默认进家目录）→ 归位 /data/hello_app/ → `gcc gpio_toggle.c -o gpio_toggle` → `./gpio_toggle`。
**结果**：物理 29 脚 LED 1Hz 稳定翻转，**步骤 2 过关**。
**目录规约**：源码/产物统一 `/data/hello_app/`；`/root/` 只当中转；裸文件已清理。

### 操作 #7：systemd 开机自启 —— 排障修复后 reboot 终验通过 ✅（三步上手 · 步骤3）

> **勘误**：本节曾按"reboot 后 LED 自闪验证通过"记录为已完成（问卷误选）。用户随后澄清：手动运行正常、**自启从未生效**。以下为修正后的真实状态。

**证据**（2026-09-10 截图，视觉通道修复后读取）：
- 终端：`./gpio_toggle` 手动运行，`GPIO399 = 0/1` 正常翻转（步骤 2 功能无恙）
- `ls /data/hello_app/` → 仅 gpio_toggle / gpio_toggle.c / hello_file / hello_file.c / hello.txt，**无 hello_gpio.service**
- SFTP 面板 `/root/` 中转区仅 gpio_toggle.c / hello_file.c —— **service 文件未上板**

**根因**：`hello_gpio.service`（Windows 侧 `x5_app/hello_gpio.service`）从未通过 SFTP 上传，板上无服务文件可安装。

**修复流程**（已下发，待用户执行回传）：
1. MobaXterm SFTP 把 `hello_gpio.service` 拖到板子 `/root/`
2. `cp /root/hello_gpio.service /etc/systemd/system/ && systemctl daemon-reload`
3. `systemctl enable --now hello_gpio && systemctl status hello_gpio --no-pager`
4. `reboot` 后不登录看 LED 自闪 → 完成后本节改 ✅ 补终验记录

**回滚方法**（安装成功后若要彻底取消自启）：

```bash
systemctl disable --now hello_gpio
rm /etc/systemd/system/hello_gpio.service && systemctl daemon-reload
```

**执行回传**（2026-09-10 18:11 CST，用户粘贴）：
- `ls /etc/systemd/system/hello_gpio.service` → No such file（根因再确认）
- SFTP 上传 + `cp` + `daemon-reload` + `enable --now` → `Created symlink multi-user.target.wants/hello_gpio.service → /etc/systemd/system/hello_gpio.service`
- `systemctl status`：Loaded (enabled)；**Active: active (running)**；Main PID 3528 = `/data/hello_app/gpio_toggle`；Memory 192K
- 中转清理：`/root/hello_gpio.service` 已删；两个 `.c` 报不存在（此前已清理，无害）
- **reboot 终验**（2026-09-10 晚，PC 侧时间）：重启后用户未手动启动任何程序，LED 即自动高低翻转 —— **终验通过，操作 #7 完成 ✅**。三步上手（进程 → GPIO → 自启）全部收官。
  （登录横幅时间 Jan 1 2000 为无 RTC 怪癖，联网后 NTP 自校；排障教训：service 文件当初没 SFTP 上板导致自启"从未生效"，问卷误记一次已勘误。）

### 操作 #8：1kHz 周期模板上板实测，v2 优化后 PASS ✅

**目的**：验证 SCHED_FIFO + TIMER_ABSTIME 骨架在本板的实时性（指南第 4 章过关标准：唤醒延迟 <100µs 且 180s 拍数 ≈180000）。

**已执行**：SFTP 上传 period_1khz.c（v1）→ `gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread` → `sudo ./period_1khz`（abs 模式 180s，用户粘贴完整输出）。

**首轮数据（abs / 180s / 绑核 CPU3 / SCHED_FIFO 80）**：
- 内核自检：`6.1.83-rt28 #2 SMP PREEMPT_RT` ✅
- 拍数 **180024 / 预期 180025（差 1）**——绝对锚点零漂移实证；avg 周期 999.5us
- 直方图主体 950~1050us 占 98.6%；min=886us（迟到拍"短补"效应 = 绝对锚点签名）
- **唤醒延迟 max = 123us > 100us 过关线 → 判定 FAIL，待优化复测**

**分析**：主嫌 = CPU 深度 idle 退出延迟（每拍干几 us 活就睡 1ms，核掉深 idle，退出代价几十~上百 us，与调度无关）；次嫌 = hello_gpio 每秒文件 IO + journald、cpufreq 动态调频。

**优化措施（已下发，待复测）**：v2 源码持有 /dev/cpu_dma_latency=10us 压深 idle；板上停 hello_gpio + 全核 performance 定频；复测 abs + 补测 sleep 对照组。

**复测数据（v2，abs / 180s，用户粘贴）**：
- 启动横幅确认 `cpu_dma_latency = 10 us（已压制深度 idle）` 生效
- 唤醒延迟 max **123us → 23us**，判定 **PASS** ✅——"深 idle 退出延迟是主嫌"实锤：压 idle 一项就拿掉了 100us
- 拍数 180016/180017（差 1），avg 999.4us，周期 min=988 / max=1017（jitter 29us），直方图全部落在 950~1050us 两桶
- **sleep 对照组（180s）**：拍数 178591，**少跑 1425 拍**（= 慢 1.425s，≈0.79%）；avg 1007.5us，每拍多耗 7.5us；min=1006——usleep"至少睡"语义决定它永远不会早醒，误差单向累积，与 abs 的 min=988（迟到短补）形成教科书级对照

**结论**：**SCHED_FIFO + TIMER_ABSTIME + mlockall + 绑核 + cpu_dma_latency 五件套**在本板达标（唤醒延迟 23us、jitter 29us、零漂移），1kHz 控制环骨架定版，后续直接把控制计算填进循环体。

**回滚/清理**：`rm /data/hello_app/period_1khz*`；governor 重启自动恢复默认。

## 三、板上文件 / 目录变更清单



| 路径 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/boot/boot.cmd` | 修改 | 操作 #1 | `imagefile="Image"` → `"Image-rt"` |
| `/boot/boot.scr` | 重新生成 | 操作 #1 | mkimage 打包产物 |
| `/boot/boot.cmd.bak` | 新建 | 操作 #1 | **保留**，回滚用 |
| `/boot/boot.scr.bak` | 新建 | 操作 #1 | **保留**，回滚用 |
| `/root/hello_file.c` | 新建(SFTP) | 操作 #4 | 源码中转，可删 |
| `/data/hello_app/` | 新建 | 操作 #4 | 自研程序目录 |
| `/data/hello_app/hello_file.c` | 新建 | 操作 #4 | 步骤1 源码 |
| `/data/hello_app/hello_file` | 新建(编译) | 操作 #4 | 步骤1 可执行文件 |
| `/data/hello_app/hello.txt` | 新建(程序生成) | 操作 #4 | 程序写出的文件 |
| `/data/hello_app/gpio_toggle.c` | 新建(SFTP) | 操作 #6 | 步骤2 源码（GPIO_N=399 定稿） |
| `/data/hello_app/gpio_toggle` | 新建(编译) | 操作 #6 | 步骤2 可执行文件 |
| `/etc/systemd/system/hello_gpio.service` | 已安装 | 操作 #7 | 生效副本：enabled，reboot 终验通过（开机自闪） |
| `/data/hello_app/period_1khz.c` | 新建(SFTP) | 操作 #8 | 1kHz 模板源码（**v2 定版**：含 cpu_dma_latency 压深 idle） |
| `/data/hello_app/period_1khz` | 新建(编译) | 操作 #8 | 1kHz 模板可执行文件 |

## 四、任务状态

### 三步上手（进程 → GPIO → 自启）—— ✅ 全部完成

| 步骤 | 内容 | 状态 |
|------|------|------|
| 1 | 编译运行 hello_file：指定目录创建文件写入 helloworld，cat 验证 | ✅ 操作 #4 |
| 2 | C 代码控制一路 GPIO 高低电平翻转（GPIO399 / 物理 29 脚，LED 1Hz） | ✅ 操作 #5、#6 |
| 3 | systemd 开机自启，reboot 不登录 LED 自闪 | ✅ 操作 #7（曾因 service 未上板排障，修复后终验通过） |

### 1kHz 周期实时模板（学习指南第 4 章）—— ✅ 完成（操作 #8）

- 首轮 abs：零漂移 ✅ 但唤醒延迟 123us 超线 → v2 加 /dev/cpu_dma_latency 压深 idle
- 复测 abs：**PASS**——唤醒延迟 max 23us、周期 jitter 29us、180s 零漂移
- sleep 对照组：180s 少跑 1425 拍（usleep 累积漂移实证）
- 五件套定版：SCHED_FIFO + TIMER_ABSTIME + mlockall + 绑核 + cpu_dma_latency
- 下一站：SocketCAN 点电机（指南第 6 章，can0 / M3508 0x200 帧）

## 五、后续记录模板（每次操作复制一节追加）

```markdown
### 操作 #N：标题（一句话） ✅/❌
**目的**：
**执行命令**：
**结果**：
**回滚方法**：
```

## 六、Windows 侧工作区产物（相关）

| 文件 | 说明 |
|------|------|
| `docs/linux_study_guide.md` | Linux 学习指南（10 章，871 行） |
| `docs/rdk_x5_board_log.md` | 本文档 |
| `x5_app/` | 三步上手任务代码与服务文件 |
