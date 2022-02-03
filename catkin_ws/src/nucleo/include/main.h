#ifndef __MAIN_H
#define __MAIN_H

#include "adc.h"
#include "uart.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_nucleo.h"

void DMA_Init(void);
void GPIO_Init(void);
void Error_Handler(char*);
void SystemClock_Config(void);

#endif
