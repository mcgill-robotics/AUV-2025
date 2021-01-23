#ifndef __UART_H
#define __UART_H

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32f3xx_hal.h"

UART_HandleTypeDef uart;

void UART_Init(void);

void write_int(int);
void write_string(char*);
void write_buffer(uint8_t*, uint16_t);

void log_debug(char*);
void log_fatal(char*);

#endif
