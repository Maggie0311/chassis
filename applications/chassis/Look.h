#ifndef __LOOK_H__
#define __LOOK_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

rt_err_t Look_Init(void);
extern uint32_t Look_Cnt[2];

#endif