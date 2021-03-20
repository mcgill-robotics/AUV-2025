#include "adc.h"

// Compute half the buffersize to speed up ping search.
static const uint32_t HALF_BUFFERSIZE = BUFFERSIZE / 2;

// Compute double the buffersize to speed up sending the buffer.
static const uint32_t DOUBLE_BUFFERSIZE = 2 * BUFFERSIZE;

// Determine measurement size depending on resolution.
#ifdef TWELVE_BIT_MODE
static const uint8_t MEASUREMENT_SIZE = 2;
#else
static const uint8_t MEASUREMENT_SIZE = 1;
#endif

// Threshold in percent.
static const uint16_t THRESHOLD = 1;

// Threshold values for ping detection
static const uint8_t PING_LOWER_THRESHOLD = 5;
static const uint8_t PING_UPPER_THRESHOLD = 200;

uint8_t in_ping = 0;

void ADC_Config(ADC_HandleTypeDef* hadc, ADC_TypeDef* adc)
{
  hadc->Instance = adc;

  hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC;
#ifdef TWELVE_BIT_MODE
  hadc->Init.Resolution = ADC_RESOLUTION12b;
#else
  hadc->Init.Resolution = ADC_RESOLUTION8b;
#endif
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ScanConvMode = ENABLE;
  hadc->Init.EOCSelection = EOC_SINGLE_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.ContinuousConvMode = ENABLE;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.NbrOfDiscConversion = 0;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.DMAContinuousRequests = ENABLE;
  hadc->Init.Overrun = OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    Error_Handler("Could not initialize ADC");
  }
}


void Add_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank)
{
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = channel;
  sConfig.Rank = rank;

  // Sampling time in ADC clock cycles.
  // This value is added to a constant ADC clock cycle count dependent on the
  // ADC resolution:
  //  12 bit: 12.5 ADC clock cycles.
  //  8 bit: 8.5 ADC clock cycles.
  #ifdef TWELVE_BIT_MODE
    // 12.5 + 61.5 = 74 ADC clock cycles --> 72 MHz / 74 = 972 972.97297 Hz.
    // Experimentally, at 72MHz, this is on average approximately 971 959 Hz.
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  #else
    // 8.5 + 61.5 = 70 ADC clock cycles --> 72 MHz / 70 = 1 028 571.4286 Hz.
    // Experimentally, at 72MHz, this is on average approximately 1 027 527 Hz.
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  #endif

  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not configure ADC%u channel", instance);
    Error_Handler(error);
  }
}


void Calibrate_ADC(ADC_HandleTypeDef* hadc)
{
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not calibrate ADC%u", instance);
    Error_Handler(error);
  }
}


void Start_ADC(ADC_HandleTypeDef* hadc, uint32_t* buff)
{
  if (in_ping) {
    // Try to start.
    if (HAL_ADC_Start_DMA(hadc, buff, BUFFERSIZE) != HAL_OK)
    {
      char* error;
      uint8_t instance = Get_ADC_Instance(hadc);
      sprintf(error, "Could not start ADC%u", instance);
      Error_Handler(error);
    }
  }
  else
  {
    if (HAL_ADC_Start_DMA(hadc, buff, 30) != HAL_OK)
    {
      char* error;
      uint8_t instance = Get_ADC_Instance(hadc);
      sprintf(error, "Could not start ADC%u", instance);
      Error_Handler(error);
    }
  }

  // Increment counter.
  active_adcs++;
}


void Stop_ADC(ADC_HandleTypeDef* hadc)
{
  // Try to stop.
  if (HAL_ADC_Stop_DMA(hadc) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not stop ADC%u", instance);
    Error_Handler(error);
  }

  // Decrement counter.
  active_adcs--;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Treated as booleans
  uint8_t minAchieved = 0;
  uint8_t maxAchieved = 0; 

  // For debugging purposes
  uint8_t maxRecorded = 0;
  uint8_t minRecorded = 255; // Upper bound of 8bit int

  for (uint8_t i = 0; i < 30; i++) {
    if (data_1[i] > PING_UPPER_THRESHOLD) {
      maxAchieved = 1;
    }
    if (data_1[i] < PING_LOWER_THRESHOLD) {
      minAchieved = 1;
    }

    if (data_1[i] > maxRecorded) {
      maxRecorded = data_1[i];
    }
    if (data_1[i] < minRecorded) {
      minRecorded = data_1[i];
    }
  }

  if (minAchieved && maxAchieved) {
    Stop_ADC(&hadc1);
    in_ping = 1;
    Start_ADC(&hadc1, (uint32_t*) data_1);
    Start_ADC(&hadc2, (uint32_t*) data_2);
    Start_ADC(&hadc3, (uint32_t*) data_3);
    Start_ADC(&hadc4, (uint32_t*) data_4);
  }

  char* minMaxDebugMsg;
  sprintf(minMaxDebugMsg, "In HAL_ADC_ConvCpltCallback, minimum recorded value: %d\t maximum recodred value: %d", minRecorded, maxRecorded);
  log_debug(minMaxDebugMsg);
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // ADC conversion half-complete.
  uint32_t sum = 0;
  char energy_buff[30];
  if (in_ping) {
    in_ping = 0;

    // Checking which ADC is passed to the method
    if (Get_ADC_Instance(hadc) == 1)
    {
      sprintf(energy_buff, "ADC1 trigger on half");
      log_debug(energy_buff);
    }
    else if (Get_ADC_Instance(hadc) == 2)
    {
      sprintf(energy_buff, "ADC2 trigger on half");
      log_debug(energy_buff);
    }
    else if (Get_ADC_Instance(hadc) == 3)
    {
      sprintf(energy_buff, "ADC3 trigger on half");
      log_debug(energy_buff);
    }
    else if (Get_ADC_Instance(hadc) == 4)
    {
      sprintf(energy_buff, "ADC4 trigger on half");
      log_debug(energy_buff);
    }

      // Stopping the ADCs from recording data
      Stop_ADC(&hadc1);
      Stop_ADC(&hadc2);
      Stop_ADC(&hadc3);
      Stop_ADC(&hadc4);

      // This takes the data and takes it to the Jetson
      write_buffer(Get_Quadrant_Header(&hadc1), 9);
      for (int i = 0; i < BUFFERSIZE; i += 2)
      {
        write_buffer((uint8_t*) data_1 + i, MEASUREMENT_SIZE);
      }
      write_buffer("\n", 1);

      write_buffer(Get_Quadrant_Header(&hadc2), 9);
      for (int i = 0; i < BUFFERSIZE; i += 2)
      {
        write_buffer((uint8_t*) data_2 + i, MEASUREMENT_SIZE);
      }
      write_buffer("\n", 1);

      write_buffer(Get_Quadrant_Header(&hadc3), 9);
      for (int i = 0; i < BUFFERSIZE; i += 2)
      {
        write_buffer((uint8_t*) data_3 + i, MEASUREMENT_SIZE);
      }
      write_buffer("\n", 1);

      write_buffer(Get_Quadrant_Header(&hadc4), 9);
      for (int i = 0; i < BUFFERSIZE; i += 2)
      {
        write_buffer((uint8_t*) data_4 + i, MEASUREMENT_SIZE);
      }
      write_buffer("\n", 1);

      Start_ADC(&hadc1, (uint32_t*) data_1);
    
  }
}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  char reason[128];

  // Get human readable state.
  char* human_readable_state = Get_Human_Readable_State(hadc);

  // Get human readable error.
  char* human_readable_error = Get_Human_Readable_Error(hadc);

  // Handle error.
  sprintf(reason, "ADC ERROR CALLBACK: (state: %s, error: %s)",
          human_readable_state, human_readable_error);
  Error_Handler(reason);
}


char* Get_Quadrant_Header(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == QUADRANT_I) {
    return "[DATA 1]\n";
  }
  else if (hadc->Instance == QUADRANT_II) {
    return "[DATA 2]\n";
  }
  else if (hadc->Instance == QUADRANT_III) {
    return "[DATA 3]\n";
  }
  else if (hadc->Instance == QUADRANT_IV) {
    return "[DATA 4]\n";
  }
  else {
    // Oops...
    Error_Handler("Could not determine quadrant");
  }
}


uint8_t Get_ADC_Instance(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    return 1;
  }
  else if (hadc->Instance == ADC2) {
    return 2;
  }
  else if (hadc->Instance == ADC3) {
    return 3;
  }
  else if (hadc->Instance == ADC4) {
    return 4;
  }
  else {
    // Oops...
    Error_Handler("Could not determine ADC instance");
  }
}


char* Get_Human_Readable_State(ADC_HandleTypeDef* hadc)
{
  char* human_readable_state;
  HAL_ADC_StateTypeDef state = HAL_ADC_GetState(hadc);
  switch (state) {
    case HAL_ADC_STATE_RESET:
      human_readable_state = "RESET";
      break;
    case HAL_ADC_STATE_READY:
      human_readable_state = "READY";
      break;
    case HAL_ADC_STATE_BUSY:
      human_readable_state = "BUSY";
      break;
    case HAL_ADC_STATE_BUSY_REG:
      human_readable_state = "BUSY_REG";
      break;
    case HAL_ADC_STATE_BUSY_INJ:
      human_readable_state = "BUSY_INJ";
      break;
    case HAL_ADC_STATE_BUSY_INJ_REG:
      human_readable_state = "BUSY_INJ_REG";
      break;
    case HAL_ADC_STATE_TIMEOUT:
      human_readable_state = "TIMEOUT";
      break;
    case HAL_ADC_STATE_ERROR:
      human_readable_state = "ERROR";
      break;
    case HAL_ADC_STATE_EOC:
      human_readable_state = "EOC";
      break;
    case HAL_ADC_STATE_EOC_REG:
      human_readable_state = "EOC_REG";
      break;
    case HAL_ADC_STATE_EOC_INJ:
      human_readable_state = "EOC_INJ";
      break;
    case HAL_ADC_STATE_EOC_INJ_REG:
      human_readable_state = "EOC_INJ_REG";
      break;
    case HAL_ADC_STATE_AWD:
      human_readable_state = "AWD";
      break;
    case HAL_ADC_STATE_AWD2:
      human_readable_state = "AWD2";
      break;
    case HAL_ADC_STATE_AWD3:
      human_readable_state = "AWD3";
      break;
    default:
      sprintf(human_readable_state, "%u", state);
      break;
  }
  return human_readable_state;
}


char* Get_Human_Readable_Error(ADC_HandleTypeDef* hadc)
{
  char* human_readable_error;
  uint32_t error = HAL_ADC_GetError(hadc);
  switch (error) {
    case HAL_ADC_ERROR_NONE:
      human_readable_error = "NONE";
      break;
    case HAL_ADC_ERROR_INTERNAL:
      human_readable_error = "INTERNAL";
      break;
    case HAL_ADC_ERROR_OVR:
      human_readable_error = "OVR";
      break;
    case HAL_ADC_ERROR_DMA:
      human_readable_error = "DMA";
      break;
    case HAL_ADC_ERROR_JQOVF:
      human_readable_error = "JQOVF";
      break;
    default:
      sprintf(human_readable_error, "%u", error);
      break;
  }
  return human_readable_error;
}
