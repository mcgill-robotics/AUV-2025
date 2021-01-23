#include "main.h"

// Remember which clocks have been enabled.
static int ADC12_CLK_ENABLED = 0;
static int ADC34_CLK_ENABLED = 0;


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  if (hadc->Instance == ADC1)
  {
    // Enable peripheral clock.
    if (!ADC12_CLK_ENABLED) {
      __ADC12_CLK_ENABLE();
    }
    ADC12_CLK_ENABLED++;

    // ADC1 GPIO Configuration.
    // PB11 ------> ADC1_IN14
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize peripheral DMA.
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_DeInit(&hdma_adc1);
    HAL_DMA_Init(&hdma_adc1);

    // Associate the initialized DMA handle to the ADC handle.
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    // NVIC configuration for DMA interrupt (transfer completion or error).
    // Priority: high-priority.
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  }
  else if (hadc->Instance == ADC2)
  {
    // Enable peripheral clock.
    if (!ADC12_CLK_ENABLED) {
      __ADC12_CLK_ENABLE();
    }
    ADC12_CLK_ENABLED++;

    // ADC2 GPIO Configuration.
    // PB2 ------> ADC2_IN12
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize peripheral DMA.
    hdma_adc2.Instance = DMA2_Channel1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_DeInit(&hdma_adc2);
    HAL_DMA_Init(&hdma_adc2);

    // Associate the initialized DMA handle to the ADC handle.
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

    // NVIC configuration for DMA interrupt (transfer completion or error).
    // Priority: high-priority.
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  }
  else if (hadc->Instance == ADC3)
  {
    // Enable peripheral clock.
    if (!ADC34_CLK_ENABLED) {
      __ADC34_CLK_ENABLE();
    }
    ADC34_CLK_ENABLED++;

    // ADC3 GPIO Configuration.
    // PB13 ------> ADC3_IN5
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize peripheral DMA.
    hdma_adc3.Instance = DMA2_Channel5;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_DeInit(&hdma_adc3);
    HAL_DMA_Init(&hdma_adc3);

    // Associate the initialized DMA handle to the ADC handle.
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc3);

    // NVIC configuration for DMA interrupt (transfer completion or error).
    // Priority: high-priority.
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  }
  else if (hadc->Instance == ADC4)
  {
    // Enable peripheral clock.
    if (!ADC34_CLK_ENABLED) {
      __ADC34_CLK_ENABLE();
    }
    ADC34_CLK_ENABLED++;

    // ADC4 GPIO Configuration.
    // PB15 ------> ADC4_IN5
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize peripheral DMA.
    hdma_adc4.Instance = DMA2_Channel2;
    hdma_adc4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc4.Init.Mode = DMA_CIRCULAR;
    hdma_adc4.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_DeInit(&hdma_adc4);
    HAL_DMA_Init(&hdma_adc4);

    // Associate the initialized DMA handle to the ADC handle.
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc4);

    // NVIC configuration for DMA interrupt (transfer completion or error).
    // Priority: high-priority.
    HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
  }
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if (hadc->Instance == ADC1)
  {
    // Disable peripheral clock.
    ADC12_CLK_ENABLED--;
    if (!ADC12_CLK_ENABLED){
      __ADC12_CLK_DISABLE();
    }

    // ADC1 GPIO Configuration.
    // PB11 ------> ADC1_IN14
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    // Deinitialize peripheral DMA.
    HAL_DMA_DeInit(hadc->DMA_Handle);

    // Disable the NVIC configuration for DMA interrupt.
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  }
  else if (hadc->Instance == ADC2)
  {
    // Disable peripheral clock.
    ADC12_CLK_ENABLED--;
    if (!ADC12_CLK_ENABLED){
      __ADC12_CLK_DISABLE();
    }

    // ADC2 GPIO Configuration.
    // PB2 ------> ADC2_IN12
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

    // Deinitialize peripheral DMA.
    HAL_DMA_DeInit(hadc->DMA_Handle);

    // Disable the NVIC configuration for DMA interrupt.
    HAL_NVIC_DisableIRQ(DMA2_Channel1_IRQn);
  }
  else if (hadc->Instance == ADC3)
  {
    // Disable peripheral clock.
    ADC34_CLK_ENABLED--;
    if (!ADC34_CLK_ENABLED){
      __ADC34_CLK_DISABLE();
    }

    // ADC3 GPIO Configuration.
    // PB13 ------> ADC3_IN5
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);

    // Deinitialize peripheral DMA.
    HAL_DMA_DeInit(hadc->DMA_Handle);

    // Disable the NVIC configuration for DMA interrupt.
    HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);
  }
  else if (hadc->Instance == ADC4)
  {
    // Disable peripheral clock.
    ADC34_CLK_ENABLED--;
    if (!ADC34_CLK_ENABLED){
      __ADC34_CLK_DISABLE();
    }

    // ADC4 GPIO Configuration.
    // PB12 ------> ADC4_IN3
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

    // Deinitialize peripheral DMA.
    HAL_DMA_DeInit(hadc->DMA_Handle);

    // Disable the NVIC configuration for DMA interrupt.
    HAL_NVIC_DisableIRQ(DMA2_Channel5_IRQn);
  }
}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (huart->Instance == USART1)
  {
    // Enable peripheral clock.
    __USART1_CLK_ENABLE();

    // USART1 GPIO Configuration.
    // PA9 ------> USART1_TX
    // PA10 -----> USART1_RX
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (huart->Instance == USART2)
  {
    // Enable peripheral clock.
    __USART2_CLK_ENABLE();

    // USART2 GPIO Configuration.
    // PA2 ------> USART2_TX
    // PA3 ------> USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (huart->Instance == USART3)
  {
    // Enable peripheral clock.
    __USART3_CLK_ENABLE();

    // USART3 GPIO Configuration.
    // PB9 ------> USART3_TX
    // PB8 ------> USART3_RX
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART1)
  {
    // Disable peripheral clock.
    __USART1_CLK_DISABLE();

    // USART1 GPIO Configuration.
    // PA9 ------> USART1_TX
    // PA10 -----> USART1_RX
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
  }
  else if (huart->Instance == USART2)
  {
    // Disable peripheral clock.
    __USART2_CLK_DISABLE();

    // USART2 GPIO Configuration.
    // PA2 ------> USART2_TX
    // PA3 ------> USART2_RX
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
  }
  else if (huart->Instance == USART3)
  {
    // Disable peripheral clock.
    __USART3_CLK_DISABLE();

    // USART3 GPIO Configuration.
    // PB9 ------> USART3_TX
    // PB8 ------> USART3_RX
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
  }
}
