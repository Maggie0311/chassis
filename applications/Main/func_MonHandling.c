#include "func_MonHandling.h"
#include "drv_wheel.h"
#include <rthw.h>
#include "math.h"
uint32_t TIM10_tick;
uint32_t TIM10_counts;
uint8_t rtt_isError;

// 复位底盘单片机
rt_err_t Robot_Chassis_Reset(rt_bool_t status)
{
    if (status != RT_TRUE)
        return RT_EOK;

    //在复位时需要将底盘所有电机失能，这里不管底盘每个轮组类型，全部失能即可
    

    rt_thread_mdelay(200);
    rt_hw_cpu_reset();
    return RT_EOK;
}
TIM_HandleTypeDef htim10;

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM10_Init(void)
{

    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 179;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 9999;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */
    HAL_TIM_Base_Start_IT(&htim10);
    /* USER CODE END TIM10_Init 2 */
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    int err1 = 0, err2 = 0;
    __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE | TIM_IT_TRIGGER);
    err1 = rt_tick_get() - TIM10_tick ;
    err2 = TIM10_tick - rt_tick_get() ;

    if (err1 > 13 || err2>13)
    {
        if (TIM10_counts++ > 10)
            rtt_isError = 1;
    }
    else
        TIM10_counts = 0;
    TIM10_tick = rt_tick_get();
}

void rtt_ErrorHandle(void)
{
    if (rtt_isError)
    {
        Robot_Chassis_Reset(RT_TRUE);
    }
}
