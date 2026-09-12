/* gpio_toggle.c —— 步骤2：控制一路 GPIO 每 1 秒翻转一次电平
 *
 * 原理：Linux 内核把 GPIO 暴露成 /sys/class/gpio 下的文件，写文件 = 控制引脚
 *   export    写引脚编号 → 内核为你创建 /sys/class/gpio/gpioN/
 *   direction 写 "out"   → 设为输出
 *   value     写 "1"/"0" → 输出高/低电平
 *
 * !! 第一步先做两件事 !!
 *   a. 把下面 GPIO_N 改成你要用的引脚编号（查法见 x5_app/README.md 步骤2）
 *   b. 先在 shell 里手动 export 一次验证该引脚可用，再跑本程序
 *
 * 编译(板上):  gcc gpio_toggle.c -o gpio_toggle
 * 运行(板上):  ./gpio_toggle        （Ctrl+C 停止）
 * 观察:        引脚接 LED(串330Ω电阻)+GND，或用万用表/示波器看电平
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>     /* sleep 用 */
#include <errno.h>

#define GPIO_N   399    /* BCM GPIO5 / 物理脚29 = gpiochip4(base 379)+line 20 → 全局 399
                           （Hobot.GPIO 官方库 RDK_X5 表 + strace 实证，2026 考古结论） */
#define GPIO_SYS "/sys/class/gpio"

/* 小工具：往 sysfs 的某个属性文件里写一个字符串 */
static int gpio_write(const char *attr, const char *val)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", GPIO_SYS, attr);
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        perror(path);
        return -1;
    }
    fprintf(fp, "%s", val);
    fclose(fp);
    return 0;
}

int main(void)
{
    char value_path[128];
    char num[8];

    /* 1. export：向内核申请使用该引脚（已被导过则容错继续） */
    snprintf(num, sizeof(num), "%d", GPIO_N);
    if (gpio_write("export", num) != 0) {
        snprintf(value_path, sizeof(value_path), "%s/gpio%d/value", GPIO_SYS, GPIO_N);
        if (access(value_path, W_OK) != 0) {
            fprintf(stderr, "export 失败且引脚不可用：不存在或被 device tree 占用，换一个引脚试试\n");
            return 1;
        }
        printf("引脚已被导出（官方库/之前测试残留），直接使用\n");
    }

    /* 2. 设为输出方向 */
    snprintf(value_path, sizeof(value_path), "%s/gpio%d/direction", GPIO_SYS, GPIO_N);
    FILE *fp = fopen(value_path, "w");
    if (fp == NULL) {
        perror(value_path);
        return 1;
    }
    fprintf(fp, "out");
    fclose(fp);

    /* 3. 每 1 秒翻转一次 */
    snprintf(value_path, sizeof(value_path), "%s/gpio%d/value", GPIO_SYS, GPIO_N);
    printf("开始翻转 GPIO%d，Ctrl+C 停止\n", GPIO_N);
    int level = 0;
    while (1) {
        fp = fopen(value_path, "w");
        if (fp == NULL) {
            perror(value_path);
            return 1;
        }
        fprintf(fp, "%d", level);
        fclose(fp);
        printf("GPIO%d = %d\n", GPIO_N, level);
        level = !level;
        sleep(1);
    }

    /* 到不了这里；如果程序要优雅退出，应写 "in" 复位并 unexport */
}
