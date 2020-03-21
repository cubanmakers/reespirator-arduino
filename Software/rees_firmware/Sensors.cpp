/**
 * Sensors reading module
 */
#include "src/Adafruit_BME280/Adafruit_BME280.h"
#include "Sensors.h"


void Sensors::Sensors (Adafruit_BME280 pres1, Adafruit_BME280 pres2) {

    // Arrancar sensores de presion 1 y 2
    if (!pres1.begin() || !pres2.begin()) {
        display.clear();
        if(!pres1.begin()) {
        display.writeLine(0, "bme1 not found");
        Serial.println("Could not find sensor BME280 number 1, check wiring!");
        } else {
        display.writeLine(0, "bme2 not found");
        Serial.println("Could not find sensor BME280 number 2, check wiring!");
        }
        display.writeLine(1, "Check wires!");
        while (1);
    }

    /* Default settings from datasheet. */  //TODO: valorar SAMPLING_NONE, lecturas mas rapidas?
    // Ver ejemplos: https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
    pres1.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BME280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BME280::FILTER_X16,      /* Filtering. */
                      Adafruit_BME280::STANDBY_MS_500); /* Standby time. */
    pres2.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BME280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BME280::FILTER_X16,      /* Filtering. */
                      Adafruit_BME280::STANDBY_MS_500); /* Standby time. */


    _pres1Sensor = pres1;
    _pres2Sensor = pres2;
    _errorCounter = 0;
    _state = SensorStateFailed;
}

    void readPressure() {
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

    SensorValues_t getPressure() {
        SensorValues_t values;
        values.state = _state;
        values.pressure1 = _pressure1;
        values.pressure2 = _pressure2;
        return values;
    }
