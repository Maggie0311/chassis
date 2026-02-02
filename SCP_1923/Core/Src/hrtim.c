
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    hrtim.c
 * @brief   This file provides code for the configuration
 *          of the HRTIM instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "hrtim.h"

/* USER CODE BEGIN 0 */
#include "func_Sense.h"
#include "func_CurrentCtrl.h"
extern Current_Ctrl_s Current_Ctrl;
// 持续多少时间认为触发，为该�?�乘轮询的周�?
#define CONFIRM_MAX 100
// 触发释抑
#define RELEASE_COUNT 0

struct
{
  Key_S Key;            // 用于�?测状�?
  uint8_t Trigger_Flag; // 用于标志fault触发
  uint8_t EN_Flag;      // 用于表示fault使能
} Fault[4];

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_FaultCfgTypeDef pFaultCfg = {0};
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_FLT1|HRTIM_IT_FLT4;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV4) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_HIGH;
  pFaultCfg.Filter = HRTIM_FAULTFILTER_8;
  pFaultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1, HRTIM_FAULTMODECTL_ENABLED);
  pFaultCfg.Filter = HRTIM_FAULTFILTER_NONE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_ENABLED);
  pFaultCfg.Filter = HRTIM_FAULTFILTER_10;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_4, HRTIM_FAULTMODECTL_ENABLED);
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_E;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERE_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 9600*5;
  pTimeBaseCfg.RepetitionCounter = 0;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 9600;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 19200;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 28800;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 38400;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xFFDF;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT1|HRTIM_TIMFAULTENABLE_FAULT4;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER|HRTIM_TIMRESETTRIGGER_MASTER_CMP1
                              |HRTIM_TIMRESETTRIGGER_MASTER_CMP2|HRTIM_TIMRESETTRIGGER_MASTER_CMP3
                              |HRTIM_TIMRESETTRIGGER_MASTER_CMP4;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_RST;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 65503;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 200;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_NEGATIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 200;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_NEGATIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER|HRTIM_OUTPUTSET_TIMCMP2
                              |HRTIM_OUTPUTSET_MASTERPER|HRTIM_OUTPUTSET_MASTERCMP1
                              |HRTIM_OUTPUTSET_MASTERCMP2|HRTIM_OUTPUTSET_MASTERCMP3
                              |HRTIM_OUTPUTSET_MASTERCMP4;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_ACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_ACTIVE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 9000;
  pTimeBaseCfg.RepetitionCounter = 0x01;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV4;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */
  // 打开之后要注意先把所有的占空比全部设置为 0
  hhrtim1.Instance->sTimerxRegs[1].CMP1xR = 20;
  hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 60000;

  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = 20;
  hhrtim1.Instance->sTimerxRegs[0].CMP2xR = 60000;

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspInit 0 */

  /* USER CODE END HRTIM1_MspInit 0 */
    /* HRTIM1 clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PB10     ------> HRTIM1_FLT3
    PB11     ------> HRTIM1_FLT4
    PA12     ------> HRTIM1_FLT1
    */
    GPIO_InitStruct.Pin = I_FAULT_Pin|VCAP_FAULT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VIN_FAULT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(VIN_FAULT_GPIO_Port, &GPIO_InitStruct);

    /* HRTIM1 interrupt Init */
    HAL_NVIC_SetPriority(HRTIM1_TIME_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_TIME_IRQn);
    HAL_NVIC_SetPriority(HRTIM1_FLT_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_FLT_IRQn);
  /* USER CODE BEGIN HRTIM1_MspInit 1 */

  /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

  /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    PA10     ------> HRTIM1_CHB1
    PA11     ------> HRTIM1_CHB2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

  /* USER CODE END HRTIM1_MspPostInit 1 */
  }

}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

  /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();

    /**HRTIM1 GPIO Configuration
    PB10     ------> HRTIM1_FLT3
    PB11     ------> HRTIM1_FLT4
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    PA10     ------> HRTIM1_CHB1
    PA11     ------> HRTIM1_CHB2
    PA12     ------> HRTIM1_FLT1
    */
    HAL_GPIO_DeInit(GPIOB, I_FAULT_Pin|VCAP_FAULT_Pin);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |VIN_FAULT_Pin);

    /* HRTIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(HRTIM1_TIME_IRQn);
    HAL_NVIC_DisableIRQ(HRTIM1_FLT_IRQn);
  /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

  /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void GPIO_Switch_Normal(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 配置 GPIO 引脚为推挽输出模�??
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10; // 设置你需要配置的引脚
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // 推挽输出模式
  GPIO_InitStruct.Pull = GPIO_NOPULL;             // 不使用上下拉电阻
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;        // 输出速率，�?�常使用高�??

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14; // 设置你需要配置的引脚

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HRTIM_Disable_Output_Basic(void)
{
  // 关闭之前要注意先把所有的占空比全部设置为 0
  hhrtim1.Instance->sTimerxRegs[1].CMP1xR = 10;
  hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 60000;

  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = 10;
  hhrtim1.Instance->sTimerxRegs[0].CMP2xR = 60000;

  // 通过操作 HRTIM_ODISR 寄存器来实现关闭 A1/B1/C1/D1 的输�?
  hhrtim1.Instance->sCommonRegs.ODISR |= 0xFF;
}

/**
 * @brief 程序中用于关�? HRTIM 输出功能的函�?
 * @author fwlh
 * @param  param            调用时传入的参数
 */
void HRTIM_Disable_Output(void *param)
{
  HRTIM_Disable_Output_Basic();
  pid_clear(&Current_Ctrl.CurrentPID);
}

/**
 * @brief 程序中用于重新打�? HRTIM 输出功能的函�?(进入 Fault 状�?�的时�?�也可以这样打开)
 * @author fwlh
 * @param  param            My Param doc
 */
void HRTIM_Enable_Output(void *param)
{
  // 打开之后要注意先把所有的占空比全部设置为 0
  hhrtim1.Instance->sTimerxRegs[1].CMP1xR = 10;
  hhrtim1.Instance->sTimerxRegs[1].CMP2xR = 60000;

  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = 10;
  hhrtim1.Instance->sTimerxRegs[0].CMP2xR = 60000;

  pid_clear(&Current_Ctrl.CurrentPID);
  // 通过操作 HRTIM_OENR 寄存器来实现 A1/B1/C1/D1 的输�?
  hhrtim1.Instance->sCommonRegs.OENR |= 0xFF;
}

/**
 * @brief  Callback function invoked when a fault 1 interrupt occurred
 * @param  hhrtim pointer to HAL HRTIM handle  * @retval None
 * @retval None
 */
void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef *hhrtim)
{
  // 记录本次触发
  Fault[0].Trigger_Flag = 1;
  if (!Fault[0].EN_Flag)
  {
    // 未发生fault事件时，由于进入fault中断不会置close标志位，故需要手动关�?次再打开
    PWM_Output_Ctrl(FAULTPROTECT, 0);
    PWM_Output_Ctrl(FAULTPROTECT, 1);
  }
}

/**
 * @brief  Callback function invoked when a fault 2 interrupt occurred
 * @param  hhrtim pointer to HAL HRTIM handle
 * @retval None
 */
void HAL_HRTIM_Fault2Callback(HRTIM_HandleTypeDef *hhrtim)
{
  // 记录本次触发
  Fault[1].Trigger_Flag = 1;
  if (!Fault[1].EN_Flag)
  {
    // 未发生fault事件时，由于进入fault中断不会置close标志位，故需要手动关�?次再打开
    PWM_Output_Ctrl(FAULTPROTECT, 0);
    PWM_Output_Ctrl(FAULTPROTECT, 1);
  }
}

/**
 * @brief  Callback function invoked when a fault 3 interrupt occurred
 * @param  hhrtim pointer to HAL HRTIM handle
 * @retval None
 */
void HAL_HRTIM_Fault3Callback(HRTIM_HandleTypeDef *hhrtim)
{
  // 记录本次触发
  Fault[2].Trigger_Flag = 1;
  if (!Fault[2].EN_Flag)
  {
    // 未发生fault事件时，由于进入fault中断不会置close标志位，故需要手动关�?次再打开
    PWM_Output_Ctrl(FAULTPROTECT, 0);
    PWM_Output_Ctrl(FAULTPROTECT, 1);
  }
}

/**
 * @brief  Callback function invoked when a fault 4 interrupt occurred
 * @param  hhrtim pointer to HAL HRTIM handle
 * @retval None
 */
void HAL_HRTIM_Fault4Callback(HRTIM_HandleTypeDef *hhrtim)
{
  // 记录本次触发
  Fault[3].Trigger_Flag = 1;
  if (!Fault[3].EN_Flag)
  {
    // 未发生fault事件时，由于进入fault中断不会置close标志位，故需要手动关�?次再打开
    PWM_Output_Ctrl(FAULTPROTECT, 0);
    PWM_Output_Ctrl(FAULTPROTECT, 1);
  }
}

/**
 * @brief fault事件�?测轮询函数（1k�?
 * @author dty
 */
void FaultTrigger_Routine(void)
{
  int fori;
  for (fori = 0; fori < 4; fori++)
  {
    if (Fault[fori].EN_Flag)
    { // 已经认为发生了fault事件
      // 判断是否要�??出fault事件,即检测Trigger_Flag�? 0 的时�?
      Key_Rec(&(Fault[fori].Key), !Fault[fori].Trigger_Flag);
      Fault[fori].Trigger_Flag = 0;

      if (Key_GetPress(&(Fault[fori].Key), 1))
      { // 不再持续发生fault中断
        // 认为没有再发生fault事件
        Fault[fori].EN_Flag = 0;
        // 打开输出
        PWM_Output_Ctrl(FAULTPROTECT, 1);
      }
    }
    else
    { // 未认为发生了fault事件
      // 判断是否要进入fault事件,即检测Trigger_Flag�? 1 的时�?
      Key_Rec(&(Fault[fori].Key), Fault[fori].Trigger_Flag);
      Fault[fori].Trigger_Flag = 0;

      if (Key_GetPress(&(Fault[fori].Key), 1))
      { // 持续发生fault中断
        // 认为发生了fault事件
        Fault[fori].EN_Flag = 1;
        // 关闭输出
        PWM_Output_Ctrl(FAULTPROTECT, 0);
      }
    }
  }
}

/**
 * @brief 初始化fault�?�?
 * @author dty
 */
void FaultTrigger_Init(void)
{
  int fori;
  for (fori = 0; fori < 4; fori++)
  {
    Key_Init(&(Fault[fori].Key), CONFIRM_MAX, RELEASE_COUNT, 0);
    Fault[fori].EN_Flag = 0;
    Fault[fori].Trigger_Flag = 0;
  }
}

/* USER CODE END 1 */
