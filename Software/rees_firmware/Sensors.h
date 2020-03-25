#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "src/Adafruit_BME280/Adafruit_BME280.h"


#define SENSORS_MAX_ERRORS 5

enum SensorState {
    SensorStateOK = 0,
    SensorStateFailed = 1
};

typedef struct {
    SensorState state;
    float pressure1;
    float pressure2;
} SensorValues_t;

class Sensors
{
    public:
    Sensors ();
    Sensors (Adafruit_BME280 pres1, Adafruit_BME280 pres2);
    unsigned int begin(void);
    void readPressure();
    SensorValues_t getPressureInPascals(); 

    private:
    void _init(Adafruit_BME280 pres1, Adafruit_BME280 pres2);
    Adafruit_BME280 _pres1Sensor;
    Adafruit_BME280 _pres2Sensor;
    float _pressure1;
    float _pressure2;
    SensorState _state;
    byte _errorCounter;

};

#endif