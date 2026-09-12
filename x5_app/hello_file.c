/* hello_file.c —— 步骤1：在指定目录创建文件并写入 helloworld
 *
 * 流程：mkdir 建目录 → fopen 建/开文件 → fprintf 写入 → fclose 保存关闭
 * 编译(板上):  gcc hello_file.c -o hello_file
 * 运行(板上):  ./hello_file
 * 验证(板上):  cat /data/hello_app/hello.txt
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
