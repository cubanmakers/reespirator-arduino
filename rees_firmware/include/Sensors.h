#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>
#include <Adafruit_BME280.h>
#include "defaults.h"

#include <Honeywell_ABP.h>
#include "pinout.h"

#ifdef ENABLED_SENSOR_VOLUME_SFM3300
#include <sfm3000wedo.h>
#endif

#define SENSORS_MAX_ERRORS 5

#if ENABLED_SENSOR_VOLUME_SFM3300
#define SFM3300_OFFSET 32768
#define SFM3300_SCALE   120
#endif

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
    short volume; // ml
} SensorVolumeValue_t;

class Sensors
{
    public:
    Sensors();
    unsigned int begin(void);
    void readPressure();
    SensorPressureValues_t getRelativePressureInPascals();
    SensorPressureValues_t getAbsolutePressureInPascals();
    SensorPressureValues_t getAbsolutePressureInCmH20();
    SensorPressureValues_t getRelativePressureInCmH20();
    SensorVolumeValue_t getVolume();
    void saveVolume(void);
    void getOffsetBetweenPressureSensors(int samples = 100);
#if ENABLED_SENSOR_VOLUME
    void readVolume(void);
    void resetVolumeIntegrator(void);
    float getFlow(void);
#endif
    private:
    void _init(void);
    Adafruit_BME280 _pres1Sensor;
    Adafruit_BME280 _pres2Sensor;
#if ENABLED_SENSOR_VOLUME_SFM3300
    SFM3000wedo* _sfm3000;
#endif
    float _pressure1;
    float _pressure2;
    float _pressureSensorsOffset = 0.0;
    SensorState _state;
    byte _errorCounter;
#if ENABLED_SENSOR_VOLUME
    float _volume_ml;
    float _flow;
    volatile float _lastVolume;
    unsigned long _lastReadFlow;
#endif


};

#endif