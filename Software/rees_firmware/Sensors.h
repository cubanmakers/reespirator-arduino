#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>

#define SENSORS_MAX_ERRORS 5

enum SensorState {
    SensorStateOK = 0,
    SensorStateFailed = 1
}

typedef struct SensorValues_t {
    SensorState state;
    float pressure1;
    float pressure2;
};

class Sensors
{
    public:
    Sensors (Adafruit_BME280 pres1, Adafruit_BME280 pres2);
    void readPressure();
    SensorValues_t getPressure();

    private:
    Adafruit_BME280 _pres1Sensor;
    Adafruit_BME280 _pres2Sensor;
    float _pressure1;
    float _pressure2;
    SensorState _state;
    byte _errorCounter;

}

#endif