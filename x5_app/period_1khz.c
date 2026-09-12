/* period_1khz.c —— 1kHz 周期任务模板（学习指南第 4 章，迁移骨架的核心）
 * v2：新增 /dev/cpu_dma_latency 压深度 idle —— 首轮实测唤醒延迟 123us 的主因就是
 *     核每拍睡 ~1ms 掉进深 idle，退出代价几十~上百 us。
 *
 * 解决什么问题：RT-Thread 里"高优先级线程 + 硬定时器"的 1kHz 确定性，在 Linux 上用
 *   SCHED_FIFO 定优先级 + 绑核 + mlockall + clock_nanosleep(TIMER_ABSTIME) 拿回来。
 *
 * 两种模式（同一份代码做对照实验，指南 4.3 的原理亲眼可见）：
 *   abs  （默认）绝对时间睡眠：睡到"下一拍的绝对时刻"。本拍算得慢就晚睡少睡，
 *               锚点不动、误差不累积，长期零漂移 —— 以后控制环就用它
 *   sleep       usleep(1000)：醒来之后"再"睡 1ms，每拍多出计算+调度延迟，越跑越慢
 *
 * 编译(板上): gcc -Wall -O2 period_1khz.c -o period_1khz -lpthread
 * 运行(板上): sudo ./period_1khz              # abs 模式跑 180 秒（正式测量）
 *             sudo ./period_1khz abs 60       # abs 模式只跑 60 秒
 *             sudo ./period_1khz sleep 180    # usleep 对照组 180 秒
 *             （SCHED_FIFO 需要 root；Ctrl+C 可提前结束，统计照常打印）
 *
 * 过关标准（指南第 4 章动手实验）：
 *   abs   : 唤醒延迟 max < 100 us，且 180 秒拍数 ≈ 180000（零漂移）
 *   sleep : 拍数比 abs 明显少（差 > 1000）—— 累积漂移的活证据
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>

#define PERIOD_US 1000                 /* 1 kHz；换 500Hz 就改 2000 */
#define PERIOD_NS (PERIOD_US * 1000)
#define RT_PRIO   80                   /* SCHED_FIFO 优先级（1-99，控制环建议 70-90） */
#define CPU_N     3                    /* 绑核：0-7 挑空闲核（htop 看占用），防迁移抖动 */

/* 周期直方图（同指南 9.4：桶宽 50us，覆盖 900~1500us，超界并入末桶） */
enum { NBUCKET = 12, CENTER = 2, STEP_US = 50 };

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* rt 线程写、主线程读的统计区。
 * long 的对齐读写在 AArch64 上天然原子；中途打印可能读到"半新半旧"的数，
 * 只影响显示不影响正确性（教学代码从简，不上锁）。 */
static struct {
    volatile long loops;                                  /* 已执行拍数 */
    volatile long min_per_us, max_per_us, sum_per_us;     /* 实际周期统计 */
    volatile long max_lat_us;                             /* abs 模式：唤醒延迟 = 实际醒来 - 应醒时刻 */
    long hist[NBUCKET];                                   /* 周期直方图（结束后才读，无竞争） */
} g_st = { .min_per_us = 1L << 40 };

struct rt_cfg { int use_abstime; };

static void *rt_loop(void *arg)
{
    struct rt_cfg *cfg = arg;
    struct timespec next, now, prev;

    /* 1) 实时优先级：让内核像 RT-Thread 的高优先级线程一样"就绪即抢占"地调度本线程
     *    警告：SCHED_FIFO 线程不让出 CPU 就会霸占核（饿死 SSH/journald），
     *    所以实时循环里必须有 clock_nanosleep / 阻塞 IO 这种让出点 */
    struct sched_param sp = { .sched_priority = RT_PRIO };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "[警告] SCHED_FIFO 失败: %s（请用 sudo 跑，或 setcap cap_sys_nice+ep ./period_1khz）\n",
                strerror(errno));

    /* 2) 绑核：任务在核间迁移本身就能带来几 us~几十 us 抖动 */
    cpu_set_t set;
    CPU_ZERO(&set); CPU_SET(CPU_N, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
        perror("[警告] 绑核失败");

    /* next = 周期锚点轴（abs 模式的绝对 deadline 序列）；prev = 上一拍实际时刻 */
    clock_gettime(CLOCK_MONOTONIC, &next);
    prev = next;

    while (g_run) {
        if (cfg->use_abstime) {
            /* 3) 绝对时间睡眠：先把锚点推进到下一拍，再睡到那个"绝对时刻"。
             *    本拍干得久 → 睡得短；真错过点了 → 立即返回追赶。
             *    锚点序列独立于执行时长，所以误差永不累积 */
            next.tv_nsec += PERIOD_NS;
            while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        } else {
            /* 对照组：醒来之后再睡 1ms → 每拍 = 1ms + 本拍开销 + 唤醒延迟，逐拍累积 */
            usleep(PERIOD_US);
        }
        if (!g_run) break;                       /* 被 Ctrl+C / 计时到点唤醒，别再计入统计 */

        clock_gettime(CLOCK_MONOTONIC, &now);    /* 实际醒来时刻 */

        /* 统计①：本拍实际周期（相邻两拍的间隔） */
        long p_us = (now.tv_sec - prev.tv_sec) * 1000000L
                  + (now.tv_nsec - prev.tv_nsec) / 1000;
        prev = now;

        g_st.loops++;
        if (g_st.loops > 1) {                    /* 首拍是校准，不计 */
            if (p_us < g_st.min_per_us) g_st.min_per_us = p_us;
            if (p_us > g_st.max_per_us) g_st.max_per_us = p_us;
            g_st.sum_per_us += p_us;
            long idx = (p_us - (PERIOD_US - CENTER * STEP_US)) / STEP_US;
            if (idx < 0) idx = 0;
            if (idx >= NBUCKET) idx = NBUCKET - 1;
            g_st.hist[idx]++;
        }
        /* 统计②：唤醒延迟（abs 模式专属）——醒来比 deadline 晚了多少 us */
        if (cfg->use_abstime) {
            long lat = (now.tv_sec - next.tv_sec) * 1000000L
                     + (now.tv_nsec - next.tv_nsec) / 1000;
            if (lat > g_st.max_lat_us) g_st.max_lat_us = lat;
        }

        /* ==================== 控制计算放这里 ====================
         * 读传感器 → 解算 → PID → CAN 发送（下一步 SocketCAN 接进来）
         * 规矩：循环内不做 printf/文件 IO 等慢 syscall——那是抖动之源；
         * 要输出就更新共享变量，让低优先级线程慢慢打（本程序就是这么干的）
         * ========================================================== */
    }
    return NULL;
}

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "abs";
    long duration_sec = argc > 2 ? atol(argv[2]) : 180;
    int use_abstime = (strcmp(mode, "abs") == 0);

    if (!use_abstime && strcmp(mode, "sleep") != 0) {
        fprintf(stderr, "用法: %s [abs|sleep] [秒数，默认180]\n", argv[0]);
        return 1;
    }

    /* 环境自检：确认跑在 PREEMPT_RT 内核上（病历本操作 #1 切的），否则抖动会偏大 */
    struct utsname u;
    uname(&u);
    printf("内核: %s %s\n", u.release, u.version);
    if (strstr(u.version, "PREEMPT_RT") == NULL && strstr(u.version, "preempt_rt") == NULL)
        printf("[提示] 版本串里没检测到 PREEMPT_RT，若是标准内核 max 延迟会明显变大\n");

    signal(SIGINT, on_sigint);
    mlockall(MCL_CURRENT | MCL_FUTURE);          /* 4) 锁内存：禁止换页，防缺页中断抖动 */

    /* 5) 压住 CPU 深度 idle（v2 新增）：周期任务每拍只干几 us 活就睡 ~1ms，核趁机
     *    掉进深度省电态，而从深 idle 退出的硬件代价是几十~上百 us——首轮实测 123us
     *    唤醒延迟的主嫌。持有 /dev/cpu_dma_latency 并写入目标延迟后，内核就不会进入
     *    退出延迟超过它的 idle 态。fd 必须一直开着（关掉约束即解除），真控制环同理。 */
    int latency_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (latency_fd >= 0) {
        int32_t target_lat_us = 10;
        if (write(latency_fd, &target_lat_us, sizeof(target_lat_us)) == sizeof(target_lat_us))
            printf("cpu_dma_latency = %d us（已压制深度 idle）\n", target_lat_us);
        else
            perror("[警告] cpu_dma_latency 写入失败");
    } else {
        perror("[警告] 打不开 /dev/cpu_dma_latency");
    }

    printf("模式=%s  周期=%dus  时长=%lds  绑核CPU=%d  SCHED_FIFO prio=%d\n",
           mode, PERIOD_US, duration_sec, CPU_N, RT_PRIO);
    printf("Ctrl+C 可提前结束\n\n");

    pthread_t tid;
    struct rt_cfg cfg = { .use_abstime = use_abstime };
    if (pthread_create(&tid, NULL, rt_loop, &cfg) != 0) {
        perror("pthread_create");
        return 1;
    }

    /* 主线程：普通调度，只负责计时和打印（实时线程里不 printf！） */
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    long next_report = 10;
    for (;;) {
        sleep(1);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
        if (!g_run || elapsed >= duration_sec) break;
        if (elapsed >= next_report) {
            next_report += 10;
            if (use_abstime)
                printf("[%4.0fs] 拍数=%ld  周期max=%ldus  唤醒延迟max=%ldus\n",
                       elapsed, g_st.loops, g_st.max_per_us, g_st.max_lat_us);
            else
                printf("[%4.0fs] 拍数=%ld  周期max=%ldus\n",
                       elapsed, g_st.loops, g_st.max_per_us);
            fflush(stdout);
        }
    }
    g_run = 0;
    pthread_join(tid, NULL);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;

    /* ---------- 汇总报告 ---------- */
    long loops  = g_st.loops;
    long min_us = (g_st.min_per_us == (1L << 40)) ? 0 : g_st.min_per_us;
    double avg  = (loops > 1) ? (double)g_st.sum_per_us / (loops - 1) : 0.0;
    long expect = (long)(elapsed * 1000.0);      /* 1kHz 理论应跑拍数 */
    long drift  = expect - loops;                /* 正=少跑了（漂移），abs 模式应≈0 */

    printf("\n========== 结果 ==========\n");
    printf("运行 %.1fs   拍数 %ld / 预期 %ld （差 %ld）\n", elapsed, loops, expect, drift);
    printf("实际周期 us: min=%ld avg=%.1f max=%ld （jitter=%ld）\n",
           min_us, avg, g_st.max_per_us, g_st.max_per_us - min_us);

    if (use_abstime) {
        printf("唤醒延迟 max = %ld us （过关线 <100）\n", g_st.max_lat_us);
        int pass = (g_st.max_lat_us < 100) && (drift > -1000 && drift < 1000);
        printf("判定: %s\n", pass ? "PASS —— 这套骨架可以承载 1kHz 控制环"
                                  : "FAIL —— 排查：是否 sudo？换空闲核？见指南 9.4");
    } else {
        printf("每拍平均多耗 %.1f us （= avg周期-1000），累积漂移 %ld 拍\n",
               avg - PERIOD_US, drift);
        printf("这就是 usleep 做不了周期任务的原因（指南 4.3）\n");
    }

    printf("周期直方图（%dus/桶，起点 %dus，超界并入末桶）:\n", STEP_US, PERIOD_US - CENTER * STEP_US);
    long samples = (loops > 1) ? (loops - 1) : 1;
    for (int i = 0; i < NBUCKET; i++) {
        long lo  = PERIOD_US - CENTER * STEP_US + i * STEP_US;
        long bar = g_st.hist[i] * 40 / samples;
        printf("%6ld..%6ld us | %6ld ", lo, lo + STEP_US, g_st.hist[i]);
        for (long k = 0; k < bar; k++) putchar('#');
        putchar('\n');
    }
    if (latency_fd >= 0) close(latency_fd);      /* 释放 pm_qos 约束 */
    return 0;
}
