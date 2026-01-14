#ifndef _VOFA_H_
#define _VOFA_H_
#include "pid.h"
#include "board.h"
#include "drv_motor.h"

#define VOFA_TXBUF_SIZE_MAX (512)

#define PER_DATA_NUM (20)
#define VOFA_HEADER_SOF (0xA5)

#ifndef PI
#define PI (3.1415926f)
#endif

typedef enum
{
    kp_data,
    ki_data,
    kd_data,
    vx_data,
    vy_data,
    w_data,

} vofa_id_e;

typedef union vofa
{
    uint8_t data[4];
    float fdata;
} vofa_float_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float vx;
    float vy;
    float w;
    rt_device_t (*VofagetUART_dev)(void);

} vofa_idData_t;

float getVofaData(vofa_id_e id);
rt_err_t VofaThreadInit(void);
void vofaInit(void);
#endif
