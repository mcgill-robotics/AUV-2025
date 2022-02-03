#ifndef MS5803_I2C_h
#define MS5803_I2C_h

#include <Arduino.h>
#include <Wire.h>

enum measurement { PRESSURE = 0x00, TEMPERATURE = 0x10 };

// Commands
#define CMD_RESET 0x1E     // reset command
#define CMD_ADC_READ 0x00  // ADC read command
#define CMD_ADC_CONV 0x40  // ADC conversion command
#define CMD_PROM 0xA0      // Coefficient location

#define ADC_4096 0x08

class MS5803 {
 public:
  MS5803(uint8_t address);
  void reset(void);
  uint8_t begin(void);  // Collect coefficients from sensor
  float getTemperature();
  float getPressure();
  void getMeasurements();
  int32_t getRawTemperature();
  int32_t getRawPressure();
  uint8_t getStatus();

 private:
  uint8_t _address;
  uint16_t coeff[7];
  int32_t _temperature_raw;
  int32_t _pressure_raw;
  float _temperature;
  float _pressure;
  uint8_t _status;
  uint32_t getADCconversion(measurement _measurement);

  void sendCommand(uint8_t command);
  void sensorWait(uint8_t time);
};

#endif
