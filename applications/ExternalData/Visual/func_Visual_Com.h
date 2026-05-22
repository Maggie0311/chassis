#ifndef __FUN_VISUAL_COM_H__
#define __FUN_VISUAL_COM_H__
#include <board.h>

#include "mod_motion.h"
#include "app_RefSystem.h"

#include "visual_data.h"

typedef struct
{
    Ctrl_data_info_t ctrlData; // 转化后视觉的控制数据
} VisualCtrl_t;

extern sentry_cmd_info_t visual_rx_sentry_cmd_info;
extern com_opponent_position_t visual_tx_opponent_position;
// 与视觉通信线程
rt_err_t VisualInit(void);

#endif
