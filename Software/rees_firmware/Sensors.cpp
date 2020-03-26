/**
 * Sensors reading module
 */
#include "src/Adafruit_BME280/Adafruit_BME280.h"
#include "Sensors.h"
#include "pinout.h"

unsigned int Sensors::begin(void) {
    // Arrancar sensores de presion 1 y 2
    if(!_pres1Sensor.begin()) {
        return 1;
    }
        
    if(!_pres2Sensor.begin()) {
        return 2;
    }
    

    /* Default settings from datasheet. */  //TODO: valorar SAMPLING_NONE, lecturas mas rapidas?
    // Ver ejemplos: https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
    _pres1Sensor.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BME280::SAMPLING_X1,     /* Temp. oversampling */
                      Adafruit_BME280::SAMPLING_X4,    /* Pressure oversampling */
                      Adafruit_BME280::SAMPLING_NONE,   /* humidity sampling */
                      Adafruit_BME280::FILTER_X4,      /* Filtering. */
                      Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */
    _pres2Sensor.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BME280::SAMPLING_X1,     /* Temp. oversampling */
                      Adafruit_BME280::SAMPLING_X4,    /* Pressure oversampling */
                      Adafruit_BME280::SAMPLING_NONE,   /* humidity sampling */
                      Adafruit_BME280::FILTER_X4,      /* Filtering. */
                      Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */

    return 0;
}

Sensors::Sensors() {
    _init();
}


void Sensors::_init () {

    Adafruit_BME280 bmp1(
    BMP_CS1,
    BMP_MOSI,
    BMP_MISO,
    BMP_SCK
    );

    Adafruit_BME280 bmp2(
    BMP_CS2,
    BMP_MOSI,
    BMP_MISO,
    BMP_SCK
    );

    _pres1Sensor = bmp1;
    _pres2Sensor = bmp2;
    _errorCounter = 0;
    _state = SensorStateFailed;
}

void Sensors::readPressure() {
    float pres1, pres2;
    // Acquire sensors data
    pres1 = _pres1Sensor.readPressure() / 100.0F; // hPa
    pres2 = _pres2Sensor.readPressure() / 100.0F; // hPa

    if (pres1 == 0.0 || pres2 == 0.0) {

        if (_errorCounter > SENSORS_MAX_ERRORS) {
            _state = SensorStateFailed;
            //TODO do something?
        } else {
            _errorCounter++;
        }

    } else {
        _state = SensorStateOK;
        _errorCounter = 0;
        _pressure1 = pres1;
        _pressure2 = pres2;
    }
}

SensorPressureValues_t Sensors::getPressure() {
    SensorPressureValues_t values;
    values.state = _state;
    values.pressure1 = _pressure1;
    values.pressure2 = _pressure2;
    return values;
}

SensorVolumeValue_t Sensors::getVolume() {
    SensorVolumeValue_t values;

    float flow = (_pressure1 - _pressure2) * DEFAULT_PRESSURE_V_FLOW_K1;

    values.state = _state;
    values.volume = flow;
    return values;
}
