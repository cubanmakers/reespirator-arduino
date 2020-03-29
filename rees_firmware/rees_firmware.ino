/**
 * @file rees_firmware.ino
 * @author Reesistencia Team
 * @brief 
 * @version 0.1
 * @date 2020-03-29
 * 
 * @copyright GPLv3
 * 
 */

/**
 * Dependencies
 */

#include "defaults.h"
#include "pinout.h"
#include "calc.h"
#include "Sensors.h"
#include "MechVentilation.h"

#include "src/AutoPID/AutoPID.h"
#include "src/FlexyStepper/FlexyStepper.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"

/**
 * Variables
 */

#if DEBUG_STATE_MACHINE
volatile String debugMsg[15];
volatile byte debugMsgCounter = 0;
#endif

FlexyStepper * stepper = new FlexyStepper();
Sensors * sensors;
AutoPID * pid;
MechVentilation * ventilation;

VentilationOptions_t options;


/**
 * Setup
 */

void setup() {
    // Puertos serie
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial.println(F("Setup"));

    // Zumbador
    pinMode(PIN_BUZZ, OUTPUT);
    digitalWrite(PIN_BUZZ, HIGH); // test zumbador
    delay(100);
    digitalWrite(PIN_BUZZ, LOW);

    // FC efecto hall
    pinMode(PIN_ENDSTOP, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

    // Sensores de presión
    sensors = new Sensors();
    int check = sensors -> begin();
    if (check) {
        if (check == 1) {
            Serial.println(F("Could not find sensor BME280 number 1, check wiring!"));
        } else if (check == 2) {
            Serial.println(F("Could not find sensor BME280 number 2, check wiring!"));
        }
        while (1);
    }

    // PID
    pid = new AutoPID(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);
    // if pressure is more than PID_BANGBANG below or above setpoint,
    // output will be set to min or max respectively
    pid -> setBangBang(PID_BANGBANG);
    // set PID update interval
    pid -> setTimeStep(PID_TS);

    // Parte motor
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, HIGH);

    // TODO: Añadir aquí la configuarcion inicial desde puerto serie

    options.height = DEFAULT_HEIGHT;
    options.sex = DEFAULT_SEX;
    options.respiratoryRate = DEFAULT_RPM;
    options.peakInspiratoryPressure = DEFAULT_PEAK_INSPIRATORY_PRESSURE;
    options.peakEspiratoryPressure = DEFAULT_PEAK_ESPIRATORY_PRESSURE;
    options.triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;
    options.hasTrigger = false;

    ventilation = new MechVentilation(
        stepper,
        sensors,
        pid,
        options
    );

    Serial.println("Tiempo del ciclo (seg):" + String(ventilation -> getExsuflationTime() + ventilation -> getInsuflationTime()));
    Serial.println("Tiempo inspiratorio (mseg):" + String(ventilation -> getInsuflationTime()));
    Serial.println("Tiempo espiratorio (mseg):" + String(ventilation -> getExsuflationTime()));

    // TODO: Esperar aqui a iniciar el arranque desde el serial

    // Habilita el motor
    digitalWrite(PIN_EN, LOW);

    // configura la ventilación
    ventilation -> start();

    delay(1000);

    #ifndef PRUEBAS
    sensors -> readPressure();
    Timer1.initialize(TIME_BASE * 1000); // 5 ms
    Timer1.attachInterrupt(timer1Isr);
    //TODO: Option: if (Sensores ok) { arranca timer3 }
    Timer3.initialize(50); //50us
    Timer3.attachInterrupt(timer3Isr);
    #else
    Timer1.initialize(500); // 5 ms
    Timer1.attachInterrupt(timer1Isr);
    // TODO: Option: if (Sensores ok) { arranca timer3 } Timer3.initialize(50); 50us
    // Timer3.attachInterrupt(timer3Isr);
    #endif
}

/**
 * Loop
 */

void loop() {
    unsigned long time;
    time = millis();
    unsigned long static lastReadSensor = 0;

    if (time > lastReadSensor + 15) {
        #ifndef PRUEBAS
        //sensors -> readPressure();
        SensorPressureValues_t pressure = sensors -> getRelativePressureInCmH20();

        #if ENABLED_SENSOR_VOLUME
        sensors -> readVolume();
        SensorVolumeValue_t volume = sensors -> getVolume();
        // currentFlow = _sensors->getVolume().volume;
        // //but the pressure reading must be done as non blocking in the main loop
        // integratorFlowToVolume(&_currentVolume, currentFlow);
        Serial1.println(
            "DT " + String(pressure.pressure1) + " " + String(pressure.pressure2) + " " +
            String(volume.volume)
        );
        Serial.println("Volumen " + String(volume.volume));
        #else
        Serial1.println(
        "DT " + String(pressure.pressure1) + " " + String(pressure.pressure2) + " NC";
        #endif  // ENABLED_SENSOR_VOLUME
        
        if (pressure.state == SensorStateFailed) {
            //TODO sensor fail. do something
            Serial.println(F("FALLO Sensor"));
            // TODO: BUZZ ALARMS LIKE HELL
        }
        #endif // PRUEBAS
        lastReadSensor = time;
    }

    #if DEBUG_STATE_MACHINE
    if (debugMsgCounter) {
        for (byte i = 0; i < debugMsgCounter; i++) {
            Serial.println(debugMsg[i]);
        }
        debugMsgCounter = 0;
    }
    #endif
}
    /**
     * Timer 1 ISR
     */
    void timer1Isr(void) {
        ventilation -> update();
    }

    void timer3Isr(void) {
        stepper -> processMovement();
    }
