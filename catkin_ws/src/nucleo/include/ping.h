#ifndef __PING_H
#define __PING_H

#include "adc.h"
#include "main.h"
#include <stdlib.h>
#include "arm_math.h"
#include "stm32f3xx_hal.h"
#include "arm_common_tables.h"

// 2 seconds between each ping.
#define PING_PERIOD (uint32_t) 2000

// Max voltage as seen per the ADC.
#ifdef TWELVE_BIT_MODE
#define PEAK_TO_PEAK (uint16_t) 4095
#else
#define PEAK_TO_PEAK (uint16_t) 255
#endif

// DC offset as seen per the ADC.
#define DC_OFFSET PEAK_TO_PEAK / 2

uint8_t has_ping(uint32_t*, uint32_t, uint32_t);
uint32_t get_power_at_target_frequency(uint32_t* buff, uint32_t size);
uint32_t get_total_power(uint32_t* buff, uint32_t size);
uint32_t get_frequency(uint32_t* buff, uint32_t size, float32_t fs);

#endif
