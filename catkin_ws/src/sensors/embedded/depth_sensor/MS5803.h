#ifndef MS5803_H
#define MS5803_H

#include <Wire.h>
#include <Arduino.h>
class MS5803 {
    public:
        MS5803(byte address);
        int32_t getPressure();
        void sensorInit();
        void readCoefficients();

    private:
        void sendCommand(byte command);
        uint32_t adcRead(byte adcCommand);
        void calculateTemperature();
        void calculatePressure();
};
#endif
