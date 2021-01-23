#include "uart.h"

/**
 * Initializes USART.
 */
void UART_Init(void)
{
  uart.Instance = SERIAL;
  uart.Init.BaudRate = 230400;
  uart.Init.WordLength = UART_WORDLENGTH_8B;
  uart.Init.StopBits = UART_STOPBITS_1;
  uart.Init.Parity = UART_PARITY_NONE;
  uart.Init.Mode = UART_MODE_TX_RX;
  uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart.Init.OverSampling = UART_OVERSAMPLING_16;
  uart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&uart);
}


/**
 * Writes buffer to UART.
 */
void write_buffer(uint8_t* data, uint16_t size) {
  HAL_UART_Transmit(&uart, data, size, 1000);
}

/**
 * Writes integer to UART.
 */
void write_int(int data) {
  char buff[12];
  sprintf(buff, "%d\n", data);
  HAL_UART_Transmit(&uart, buff, sizeof(buff), 1000);
}


/**
 * Writes debug log to UART.
 */
void log_debug(char* data) {
#ifdef DEBUG
  char buff[128];
  sprintf(buff, "[DEBUG]\n%s\n", data);
  HAL_UART_Transmit(&uart, buff, strlen(buff), 1000);
#endif
}


/**
 * Writes fatal log to UART.
 */
void log_fatal(char* data) {
  char buff[128];
  sprintf(buff, "[FATAL]\n%s\n", data);
  HAL_UART_Transmit(&uart, buff, strlen(buff), 1000);
}
