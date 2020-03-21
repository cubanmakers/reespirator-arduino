/**
 * Sensors reading module
 */
#include "src/Adafruit_BMP280/Adafruit_BMP280.h"
#include "Sensors.h"


void Sensors::Sensors (Adafruit_BMP280 pres1, Adafruit_BMP280 pres2) {

    // Arrancar sensores de presion 1 y 2
    if (!bmp1.begin() || !bmp2.begin()) {
        display.clear();
        if(!bmp1.begin()) {
        display.writeLine(0, "bmp1 not found");
        Serial.println("Could not find sensor BMP280 number 1, check wiring!");
        } else {
        display.writeLine(0, "bmp2 not found");
        Serial.println("Could not find sensor BMP280 number 2, check wiring!");
        }
        display.writeLine(1, "Check wires!");
        while (1);
    }

    /* Default settings from datasheet. */  //TODO: valorar SAMPLING_NONE, lecturas mas rapidas?
    bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


        _pres1Sensor = pres1;
        _pres2Sensor = pres2;
    _errorCounter = 0;
    _state = SensorStateFailed;
}

    void readPressure() {
        sensors_event_t _pressure1Event, _pressure2Event;
        float pres1, pres2;
        // Acquire sensors data
        _pres1Sensor->getEvent(&_pressure1Event);
        _pres2Sensor->getEvent(&_pressure2Event);
        pres1 = _pressure1Event.pressure;
        pres2 = _pressure2Event.pressure;

        if (_pressure1 == 0 || _pressure2 == 0) {
            
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

    SensorValues_t getPressure() {
        SensorValues_t values;
        values.state = _state;
        values.pressure1 = _pressure1;
        values.pressure2 = _pressure2;
        return values;
    }
