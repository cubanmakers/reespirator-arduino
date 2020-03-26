#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include "src/Adafruit_BME280/Adafruit_BME280.h"
#include "defaults.h"
#include "calc.h"

#define SENSORS_MAX_ERRORS 5

enum SensorState
{
    SensorStateOK = 0,
    SensorStateFailed = 1
};

typedef struct
{
    SensorState state;
    float pressure1;
    float pressure2;
} SensorValues_t;

class Sensors
{
public:
    Sensors();
    Sensors(Adafruit_BME280 pres1, Adafruit_BME280 pres2);
    unsigned int begin(void);
    void readPressure();
    SensorValues_t getAbsolutePressureInPascals();
    SensorValues_t getRelativePressureInPascals();
    SensorValues_t getAbsolutePressureInCmH20();
    SensorValues_t getRelativePressureInCmH20();
    float calculateFlow();
    float calculateFilteredFlow();
    void getOffsetBetweenPressureSensors(int samples = 100);

private:
    void _init(Adafruit_BME280 pres1, Adafruit_BME280 pres2);
    Adafruit_BME280 _pres1Sensor;
    Adafruit_BME280 _pres2Sensor;
    float _pressure1;
    float _pressure2;
    float _sensorsOffset = 0.0;
    SensorState _state;
    float _lpfFlowArray[100];
    byte _errorCounter;
};

#endif