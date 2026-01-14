#ifndef __DRV_REMOTE_H__
#define __DRV_REMOTE_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

// 中间值为1024  最大值为1684 最小值为364 上下660 一般默认通道2和3为控制底盘 0和1为控制云台
typedef struct __Remote
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;
} Remote_t;

typedef struct __Mouse
{
    int16_t x_speed; // 向左负
    int16_t y_speed; // 向下正
    int16_t z_speed;
    int8_t press_l;
    int8_t press_r;
} Mouse_t;

typedef struct __Key_Data
{
    uint8_t W;
    uint8_t S;
    uint8_t A;
    uint8_t D;
    uint8_t Q;
    uint8_t E;
    uint8_t shift;
    uint8_t ctrl;
    uint8_t R;
    uint8_t F;
    uint8_t G;
    uint8_t Z;
    uint8_t X;
    uint8_t C;
    uint8_t V;
    uint8_t B;
} Key_Data_t;

typedef struct __KeyBoard
{
    uint16_t v;
    Key_Data_t Key_Data;
} KeyBoard_t;

typedef struct __RC_Ctrl
{
    Remote_t Remote_Data;
    Mouse_t Mouse_Data;
    KeyBoard_t KeyBoard_Data;
} RC_Ctrl_t;

/*
右边摇杆 横向通道为0 纵向为1 右上为增加
左边摇杆 横向通道为2 纵向为3
*/
typedef enum
{
    remote_ch0,
    remote_ch1,
    remote_ch2,
    remote_ch3,
    remote_s1,
    remote_s2,
} Remote_data_e;

typedef enum
{
    left_x_ch = remote_ch2,
    left_y_ch = remote_ch3,
    w_ch = remote_ch0,
} Remote_motion_e;

typedef enum
{
    rc_up = 1,
    rc_mid = 3,
    rc_down = 2,
} rc_data_e;

typedef struct
{
    float x;
    float y;
    float w;

} Remote_ctrl_t;

#define SC_DATA_GAIM (6.f)
#define SC_DATA_FILTER_K (0.85f) // 为0则不滤波

int16_t getRemoteData(Remote_data_e type);
extern RC_Ctrl_t RC_Ctrl_Data;
extern rt_err_t remote_uart_init(void);
uint8_t getRemoteIsValid(void);
#endif
