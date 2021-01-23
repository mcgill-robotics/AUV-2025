#include "main.h"


int main(void)
{
  HAL_Init();

  // Configure the system clock.
  SystemClock_Config();

  // Initialize LED on board.
  BSP_LED_Init(LED2);

  // Initialize UART.
  UART_Init();
  write_buffer("\n", 1);  // Force clear communication channel.
  write_buffer("[BOOTUP]\n", 9);  // Send bootup command.
  log_debug("Initialized UART");

#ifdef TWELVE_BIT_MODE
  log_debug("Running in 12 bit mode");
#else
  log_debug("Running in 8 bit mode");
#endif

  // Write core clock rate.
  char clock_buff[32];
  sprintf(clock_buff, "Running at %u Hz", SystemCoreClock);
  log_debug(clock_buff);

  // Initialize GPIO.
  log_debug("Initializing GPIO...");
  GPIO_Init();

  // Initialize DMA.
  log_debug("Initializing DMA...");
  DMA_Init();

  // Intialize ADCs.
  log_debug("Initializing ADCs...");
  ADC_Config(&hadc1, ADC1);
  ADC_Config(&hadc2, ADC2);
  ADC_Config(&hadc3, ADC3);
  ADC_Config(&hadc4, ADC4);

  // Configure ADC channels.
  log_debug("Configuring ADC channels...");
  Add_ADC_Channel(&hadc1, ADC_CHANNEL_14, 1);
  Add_ADC_Channel(&hadc2, ADC_CHANNEL_12, 1);
  Add_ADC_Channel(&hadc3, ADC_CHANNEL_5, 1);
  Add_ADC_Channel(&hadc4, ADC_CHANNEL_5, 1);

  // Calibrate ADCs.
  log_debug("Calibrating ADCs...");
  Calibrate_ADC(&hadc1);
  Calibrate_ADC(&hadc2);
  Calibrate_ADC(&hadc3);
  Calibrate_ADC(&hadc4);

  // Start ADC conversions by DMA.
  log_debug("Starting ADCs...");
  Start_ADC(&hadc1, (uint32_t*) data_1);
  // Start_ADC(&hadc2, (uint32_t*) data_2);
  // Start_ADC(&hadc3, (uint32_t*) data_3);
  // Start_ADC(&hadc4, (uint32_t*) data_4);

  while (1)
  {
    // Do nothing.
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
    | RCC_PERIPHCLK_ADC12
    | RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCOSOURCE_SYSCLK, RCC_MCO_DIV1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}


void DMA_Init(void)
{
  // Enable DMA controller clocks.
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();
}


void GPIO_Init(void)
{
  // Enable GPIO port clocks.
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
}


void Error_Handler(char* reason)
{
  while(1)
  {
    BSP_LED_Toggle(LED2);
    log_fatal(reason);
    HAL_Delay(1000);
  }
}
