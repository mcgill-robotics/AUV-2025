#ifndef MS5803_H
#define MS5803_H

#include <Wire.h>

class MS5803 {
    public:
        MS5803();
        int32_t getPressure();
        void sensorInit();

    private:
        void sendCommand(byte command);
        void readCoefficients();
        void sensorInit();
        uint32_t adcRead();
        void calculateTemperature();
        void calculatePressure();
}





#endif