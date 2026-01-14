#include "drv_HW_SPI.h"
#include "board.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();  // Cube生成代码中的DMA2时钟使能函数,和SPI1相关的DMA通道位于DMA2
  __HAL_RCC_DMA1_CLK_ENABLE();  // Cube生成代码中的DMA1时钟使能函数,和SPI2相关的DMA通道位于DMA1
  /* DMA interrupt init */
  
  //设置SPI1的DMA通道
  // /* DMA2_Stream0_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  // /* DMA2_Stream3_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  //设置SPI2的DMA通道
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);


}


/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}



// 基于上述Cube生成代码的SPI外设初始化函数
void drv_HW_SPI_Init(void)
{
    // 开启DMA时钟 DMA实际初始化位于stm32f1xx_hal_msp.c的MSPInit中，调用于MX_SPI1_Init
    MX_DMA_Init();

    // Cube生成的stm32f1xx_hal_msp.c文件中有HAL_SPI_MspInit函数，其中会配置引脚、DMA相关信息。
    // 在MX_SPI1_Init内的HAL_SPI_Init函数中，会调用Cube生成代码的HAL_SPI_MspInit函数
    // 因此，修改Cube工程中的引脚配置并生成代码后可以正常生效
    //MX_SPI1_Init();
    MX_SPI2_Init();

}

// // 从Cube代码中复制中断服务函数

// //SPI1的DMA中断服务函数
// /**
//   * @brief This function handles DMA2 stream0 global interrupt.
//   */
// void DMA2_Stream0_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

//   /* USER CODE END DMA2_Stream0_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_spi1_rx);
//   /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

//   /* USER CODE END DMA2_Stream0_IRQn 1 */
// }

// /**
//   * @brief This function handles DMA2 stream3 global interrupt.
//   */
// void DMA2_Stream3_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

//   /* USER CODE END DMA2_Stream3_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_spi1_tx);
//   /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

//   /* USER CODE END DMA2_Stream3_IRQn 1 */
// }

//SPI2的DMA中断服务函数
/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}


/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_SPI_IRQHandler(&hspi2);

    /* leave interrupt */
    rt_interrupt_leave();
}