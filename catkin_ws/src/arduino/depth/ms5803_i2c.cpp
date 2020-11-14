#include "ms5803_i2c.h"

MS5803::MS5803(uint8_t address) {
  Wire.begin();
  _address = address;  // set interface used for communication
}

void MS5803::reset(void) {
  sendCommand(CMD_RESET);
  sensorWait(3);
}

uint8_t MS5803::begin(void) {
  for (uint8_t i = 0; i < 7; i++) {
    sendCommand(CMD_PROM + (i * 2));
    Wire.requestFrom(_address, (uint8_t)2);
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    coeff[i] = (highByte << 8) | lowByte;
  }
  return 0;
}

uint8_t MS5803::getStatus() { return _status; }

float MS5803::getTemperature() { return _temperature / 100; }

float MS5803::getPressure() { return _pressure / 10; }

void MS5803::getMeasurements() {
  _temperature_raw = getADCconversion(TEMPERATURE);
  _pressure_raw = getADCconversion(PRESSURE);

  int32_t dT;
  dT = _temperature_raw - ((int32_t)coeff[5] << 8);
  _temperature = (((int64_t)dT * coeff[6]) >> 23) + 2000;
  int64_t T2, OFF2, SENS2, OFF, SENS;  // working variables

  if (_temperature < 2000) {
    T2 = 3 * (((int64_t)dT * dT) >> 33);
    OFF2 = 3 * ((_temperature - 2000) * (_temperature - 2000)) / 8;
    SENS2 = 7 * ((_temperature - 2000) * (_temperature - 2000)) / 8;
    if (_temperature < -1500) {
      SENS2 = SENS2 + 3 * ((_temperature + 1500) * (_temperature + 1500));
    }
  } else {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }

  OFF = ((int64_t)coeff[2] << 18) + (((coeff[4] * (int64_t)dT)) >> 5);
  SENS = ((int64_t)coeff[1] << 17) + (((coeff[3] * (int64_t)dT)) >> 7);

  _temperature -= T2;
  OFF -= OFF2;
  SENS -= SENS2;
  _pressure = (((SENS * _pressure_raw) >> 21) - OFF) >> 15;
}

uint32_t MS5803::getADCconversion(measurement _measurement) {
  uint8_t highByte, midByte, lowByte;

  sendCommand(CMD_ADC_CONV + _measurement + ADC_4096);
  sensorWait(10);

  sendCommand(CMD_ADC_READ);
  Wire.requestFrom(_address, (uint8_t)3);

  while (Wire.available()) {
    highByte = Wire.read();
    midByte = Wire.read();
    lowByte = Wire.read();
  }

  return ((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;
}

void MS5803::sendCommand(uint8_t command) {
  Wire.beginTransmission(_address);
  Wire.write(command);
  _status = Wire.endTransmission();
}

void MS5803::sensorWait(uint8_t time) { delay(time); };
