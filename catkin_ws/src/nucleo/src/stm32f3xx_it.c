#include "main.h"
#include "stm32f3xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;


void NMI_Handler(void)
{
}


void HardFault_Handler(void)
{
  // Infinite loop when Hard Fault exception occurs.
  while (1)
  {
  }
}


void MemManage_Handler(void)
{
  // Infinite loop when Memory Manage exception occurs.
  while (1)
  {
  }
}


void BusFault_Handler(void)
{
  // Infinite loop when Bus Fault exception occurs.
  while (1)
  {
  }
}


void UsageFault_Handler(void)
{
  // Infinite loop when Usage Fault exception occurs.
  while (1)
  {
  }
}


void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}


void PendSV_Handler(void)
{
}


void SysTick_Handler(void)
{
  HAL_IncTick();
}


void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}


void DMA2_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc2);
}


void DMA2_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc4);
}


void DMA2_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc3);
}
