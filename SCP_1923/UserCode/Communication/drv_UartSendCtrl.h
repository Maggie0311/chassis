#ifndef __DRV_UARTSENDCTRL_H__
#define __DRV_UARTSENDCTRL_H__

#include "main.h"

//数组最大长度，实际可发数据个数为 BUFF_NUM_MAX - 1 。
#define BUFF_NUM_MAX 20

typedef union 
{
    float f[BUFF_NUM_MAX];
    uint8_t u8[BUFF_NUM_MAX * 4];
}Send_Buff_u;


typedef struct
{
    Send_Buff_u Send_Buff;
    int Write_P_f;
    int Write_P_u8;
}VOFA_Send_t;

extern int VOFA_Add_SendBuff(float Data);

extern void VOFA_JustFloat_Send(uint8_t div,uint32_t current_time);

#endif 
