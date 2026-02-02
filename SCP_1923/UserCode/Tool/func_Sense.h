#ifndef __FUNC_SENSE_H__
#define __FUNC_SENSE_H__

#include "main.h"

typedef struct
{
    char Press;           // 按下为1，否则为0
    int ConfirmCount;     // 去抖变量
    int ConfirmMax;       // 去抖阈值
    int ReleaseCount_Set; // 触发释抑 触发后为非零值，向0自减，减为0后才检测下一次触发
    int ReleaseCount_Now;
    uint16_t GPIO_Pin_In;
} Key_S;

extern void Key_Init(Key_S *Key, int ConfirmMax, int ReleaseCount, uint16_t GPIO_Pin);

// 记录按键状态
extern void Key_Rec(Key_S *Key, char Press_In);

// 可以通过在Clear传入1 将本函数设置为查询到1就清空 可配合触发释抑使用
extern int8_t Key_GetPress(Key_S *Key, char Clear);

#endif
