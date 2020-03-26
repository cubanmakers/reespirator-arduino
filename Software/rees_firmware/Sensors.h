#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "src/Adafruit_BME280/Adafruit_BME280.h"
#include "defaults.h"


#define SENSORS_MAX_ERRORS 5

enum SensorState {
    SensorStateOK = 0,
    SensorStateFailed = 1
};

typedef struct {
    SensorState state;
    float pressure1;
    float pressure2;
} SensorPressureValues_t;

typedef struct {
    SensorState state;
    float volume;
} SensorVolumeValue_t;

class Sensors
{
    public:
    Sensors();
    unsigned int begin(void);
    void readPressure();
    SensorPressureValues_t getPressure(); 
    SensorVolumeValue_t getVolume();

    private:
    void _init();
    Adafruit_BME280 _pres1Sensor;
    Adafruit_BME280 _pres2Sensor;
    float _pressure1;
    float _pressure2;
    SensorState _state;
    byte _errorCounter;

};

#endif