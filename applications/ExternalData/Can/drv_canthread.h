#ifndef __DRV_CANTHREAD_H__
#define __DRV_CANTHREAD_H__
#include <board.h>
#include "drv_can_com.h"

// can设备名称
#define CAN1_DEV_NAME "can1"
#define CAN2_DEV_NAME "can2"

/**
 * @brief  can1初始化，can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can1_Init(void);

/**
 * @brief  can2初始化，can2数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can2_Init(void);


#endif
