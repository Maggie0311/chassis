#ifndef __DRV_HW_SPI_H__
#define __DRV_HW_SPI_H__

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern void drv_HW_SPI_Init(void);

#endif
