#ifndef __FUNC_RGB_DISPLAY_H__
#define __FUNC_RGB_DISPLAY_H__

#include "rtthread.h"

typedef enum
{
    RGB_CONST,  // 固定常亮
    RGB_BREATH, // 呼吸灯
    RGB_FLASH,  // 固定频率闪烁
} RGBDisplay_Mode_E;

// 设置常亮颜色
extern void RGBDisplay_CONST(float R, float G, float B);
// 设置呼吸灯
extern void RGBDisplay_BREATH(float R, float G, float B, int Period_ms, float DampRate);
// 设置闪烁
extern void RGBDisplay_FLASH(float R, float G, float B, int Period_ms, int Pulse_ms);

// 初始化RGB灯效控制
extern int RGB_Display_Init(void);

#endif
