#include <Wire.h>
#include "MS5803.h"
#include <Arduino.h>


#define secondOrder false
#define resetCommand  0x1E
#define pressureADCCommand 0x48
#define tempADCCommand 0x58
#define readADCCommand 0x00

int64_t coefficients[6];
int32_t temperature, pressure;
int64_t dT;
byte sensor_address;

MS5803::MS5803(byte address) {
  Wire.begin();
  sensor_address = address;
  //sensorInit();
}
//Send i2c command
void MS5803::sendCommand(byte command) {
  Wire.beginTransmission(sensor_address);
  Wire.write(command);
  Wire.endTransmission();
}

//Read compensation coefficients from PROM
void MS5803::readCoefficients() {
  for (int i=0; i<6; i++) {
    sendCommand(0xA2 + i*2); //Address of each coefficient
    Wire.requestFrom((uint8_t)sensor_address, (uint8_t)2); 
    byte byte1 = Wire.read();
    byte byte2 = Wire.read();
    uint16_t coeff = ((uint16_t)byte1<<8) + byte2;
    coefficients[i] = (int64_t)coeff;
    Serial.print("C");
    Serial.print(i+1);
    Serial.print(" = ");
    Serial.println(coeff);
  }
}

//Initialization steps for sensor
void MS5803::sensorInit() {
  sendCommand(resetCommand);
  delay(50);
  readCoefficients();
}

//Read raw temperature or pressure
uint32_t MS5803::adcRead(byte adcCommand) {
    sendCommand(adcCommand);
    delay(10);
    sendCommand(readADCCommand);
    Wire.requestFrom((uint8_t)sensor_address, (uint8_t)3);
    byte byte1 = Wire.read();
    byte byte2 = Wire.read();
    byte byte3 = Wire.read();
    uint32_t val = ((uint32_t)byte1<<16) + ((uint32_t)byte2<<8) + byte3;
    return val;
}


//Calculate actual temperature without second order compensation
void MS5803::calculateTemperature() {
  uint32_t rawTemp = adcRead(tempADCCommand);
  dT = (int64_t)rawTemp - (coefficients[4]<<8);
  temperature = 2000 + ((dT*coefficients[5])>>23);  
}

//Calculate pressure and temperature with second order compensation
void MS5803::calculatePressure() {
  calculateTemperature();
  uint32_t rawPressure = adcRead(pressureADCCommand);
  int64_t offset = (coefficients[1]<<18) + (coefficients[3]*dT>>5);
  int64_t sensitivity = (coefficients[0]<<17) + (coefficients[2]*dT>>7);
  //Second order compensation
  if(secondOrder) {
    int64_t t2, off2, sens2;
      if(temperature<2000) {
        t2 = 3 * dT>>33;
        off2 = 3 * (temperature-2000)^2 >> 1;
        sens2 = 5 * (temperature-2000)^2 >> 3;
        if(temperature<-1500) {
          off2 = off2 + 7*(temperature+1500)^2;
          sens2 = sens2 + 4*(temperature+1500)^2;
        }
      } else {
        t2 = 7 * (dT^2) >> 37;
        off2 = (temperature-2000)^2 >> 4;
        sens2 = 0;
      }
      temperature -= t2;
      offset -= off2;
      sensitivity -= sens2;
    }
  pressure = ((int64_t)rawPressure * (sensitivity>>21) - offset) >> 15;
}

//Public method to get pressure
int32_t MS5803::getPressure() {
  calculatePressure();
  return (pressure);
}
