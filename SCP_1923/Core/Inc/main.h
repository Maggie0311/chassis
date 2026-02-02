/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_dma.h"

#include "stm32f3xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_Time.h"
#include "pid.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define LEDC15_Pin GPIO_PIN_15
#define LEDC15_GPIO_Port GPIOC
#define ADC1_I_H_Pin GPIO_PIN_0
#define ADC1_I_H_GPIO_Port GPIOA
#define ADC1_V_IN_Pin GPIO_PIN_1
#define ADC1_V_IN_GPIO_Port GPIOA
#define ADC1_V_CAP_Pin GPIO_PIN_2
#define ADC1_V_CAP_GPIO_Port GPIOA
#define ADC1_TEM_Pin GPIO_PIN_3
#define ADC1_TEM_GPIO_Port GPIOA
#define ADC2_I_IN_Pin GPIO_PIN_4
#define ADC2_I_IN_GPIO_Port GPIOA
#define I_FAULT_Pin GPIO_PIN_10
#define I_FAULT_GPIO_Port GPIOB
#define VCAP_FAULT_Pin GPIO_PIN_11
#define VCAP_FAULT_GPIO_Port GPIOB
#define LEDB12_Pin GPIO_PIN_12
#define LEDB12_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOB
#define LEDB15_Pin GPIO_PIN_15
#define LEDB15_GPIO_Port GPIOB
#define VIN_FAULT_Pin GPIO_PIN_12
#define VIN_FAULT_GPIO_Port GPIOA
#define LEDB3_Pin GPIO_PIN_3
#define LEDB3_GPIO_Port GPIOB
#define LEDB4_Pin GPIO_PIN_4
#define LEDB4_GPIO_Port GPIOB
#define LEDB5_Pin GPIO_PIN_5
#define LEDB5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// 对应 PID 控制频率
#define F 6000.f
#define F_Power_Ctrl 1000.f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
