#ifndef __FUNC_OBSERBER_H__
#define __FUNC_OBSERBER_H__

#include "stm32f3xx_hal.h"


/**
* @brief 对电容电压进行观测的观测器
* 
* @param VCap       此时的测量电压
* @param NowTick    此时的时间刻
 */
void SCAP_Observer(float VCap, uint32_t NowTick);

/**
* @brief 读取此时的观测电压值
* 
* @return float 
 */
float Read_U_obs(void);

#endif /* __FUNC_OBSERBER_H__ */
