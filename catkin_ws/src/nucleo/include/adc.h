#ifndef __ADC_H
#define __ADC_H

#include "main.h"
#include "ping.h"
#include <string.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_nucleo.h"

// ADC handler declarations.
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

// ADC DMA handler declarations.
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

// Define which ADC is connected to the hydrophone from which quadrant.
#define QUADRANT_I ADC1
#define QUADRANT_II ADC2
#define QUADRANT_III ADC3
#define QUADRANT_IV ADC4

// Set the buffersize as a number of samples to collect per ADC.
// The duration in seconds is equal to the sampling frequncy * BUFFERSIZE.
// The buffersize must be large enough to hold at least the 4 ms the ping
// lasts.
#define BUFFERSIZE (uint32_t) 2048

// Data storage.
__IO uint16_t data_1[BUFFERSIZE];
__IO uint16_t data_2[BUFFERSIZE];
__IO uint16_t data_3[BUFFERSIZE];
__IO uint16_t data_4[BUFFERSIZE];

// Whether a ping has been found.
static __IO uint8_t found_ping = 0;

// Whether the buffer is inverted due to finding a ping in the second half.
static __IO uint8_t buffer_inverted = 0;

// Minimum starting energy threshold in percent.
static __IO uint8_t energy_threshold = 70;

// Maintain number of active ADCs.
static __IO uint8_t active_adcs = 0;

void ADC_Config(ADC_HandleTypeDef*, ADC_TypeDef*);
void Add_ADC_Channel(ADC_HandleTypeDef*, uint32_t, uint32_t);
void Calibrate_ADC(ADC_HandleTypeDef*);
uint8_t Get_ADC_Instance(ADC_HandleTypeDef*);
char* Get_Quadrant_Header(ADC_HandleTypeDef*);
void Start_ADC(ADC_HandleTypeDef*, uint32_t*);
void Stop_ADC(ADC_HandleTypeDef*);
char* Get_Human_Readable_State(ADC_HandleTypeDef*);
char* Get_Human_Readable_Error(ADC_HandleTypeDef*);

#endif
