#include "cmd.h"
#include <board.h>

// static void CanInfo(int argc, char **argv)
// {
//     if (argc == 2)
//     {
//         if (!rt_strcmp("help",argv[1]))
//         {
//             rt_kprintf("<tag>:ulog tag/All(mind and case sensitivity)\n");
//             rt_kprintf("<level>:Debug/Info/Warning/Error/All\n");
//             rt_kprintf("<cmd>:help/close\n");
//         }
//         else if (!rt_strcmp("close",argv[1]))
//         {
//             pick_lvl = 0;
//             pick_tag[0] = '\0';
//         }
//         else
//             rt_kprintf("hyw...\n");//无效命令
//     }
//     else if (argc == 3)
//     {
//         if (!rt_strcmp("Debug",argv[2]))//存对应level
//             pick_lvl = 7;
//         else if (!rt_strcmp("Info",argv[2]))
//             pick_lvl = 6;
//         else if (!rt_strcmp("Warning",argv[2]))
//             pick_lvl = 4;
//         else if (!rt_strcmp("Error",argv[2]))
//             pick_lvl = 3;
//         else if (!rt_strcmp("All",argv[2]))
//             pick_lvl = 10;
//         else
//         {
//             rt_kprintf("hyw...\n");//无效命令
//             return;
//         }   
//         memcpy(pick_tag,argv[1],sizeof(argv[1]));//存tag
//         // pick_lvl = (rt_uint32_t)strtoul(argv[2], NULL, 0);//直接输入数字
//     }
//     else
//         rt_kprintf("hyw...\n");//无效命令
// }
// MSH_CMD_EXPORT(LogPick, LogPick sample(or send LogPick help): LogPick <tag> <leve> / LogPick <cmd>);//导出命令到控制台
