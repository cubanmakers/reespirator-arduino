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
#include "main.h"


/**
 * Variables
 */

#if DEBUG_STATE_MACHINE
volatile String debugMsg[15];
volatile byte debugMsgCounter = 0;
#endif

Sensors * sensors;
MechVentilation * ventilation;
VentilationOptions_t options;

/**
 * Timer 1 ISR
 */
void timer1Isr(void) {
    ventilation -> update();
}

void timer3Isr(void) {
    ventilation->getStepper()-> processMovement();
}


/**
 * Setup
 */

void setup() {
    // Puertos serie
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial.println(F("Setup"));

    // Zumbador
    pinMode(PIN_BUZZ, OUTPUT);
    digitalWrite(PIN_BUZZ, HIGH); // test zumbador
    delay(100);
    digitalWrite(PIN_BUZZ, LOW);

    // FC efecto hall
    pinMode(PIN_ENDSTOP, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

    // Solenoid
    pinMode(PIN_SOLENOID, OUTPUT);

    // Sensores de presión
    sensors = new Sensors();
    int check = sensors -> begin();
    #if 0
    if (check) {
        if (check == 1) {
            Serial.println(F("Could not find sensor BME280 number 1, check wiring!"));
        } else if (check == 2) {
            Serial.println(F("Could not find sensor BME280 number 2, check wiring!"));
        }
        while (1);
    }
    #endif

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
        sensors,
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
    ventilation -> update();

    delay(2000);

    sensors -> readPressure();
    // TODO: Make this period dependant of TIME_BASE
    // TIME_BASE * 1000 does not work!!
    //TODO: Option: if (Sensores ok) { arranca timer3 }
    Timer3.initialize(50); //50us
    Timer3.attachInterrupt(timer3Isr);
    
    
    Timer1.initialize(50000); // 50 ms
    Timer1.attachInterrupt(timer1Isr);

}

void readIncomingMsg (void) {
    char msg[100];
    Serial2.readStringUntil('\n').toCharArray(msg, 100);
    int pip, peep, fr;
    int rc = sscanf(msg, "CONFIG PIP %d", &pip);
    if (rc == 1) {
        ventilation->setPeakInspiratoryPressure(pip);
    } else {
        int rc = sscanf(msg, "CONFIG PEEP %d", &peep);
        if (rc == 1) {
            ventilation->setPeakEspiratoryPressure(peep);
        } else {
            int rc = sscanf(msg, "CONFIG BPM %d", &fr);
            if (rc == 1) {
                ventilation->setRPM(fr);
            }
        }
    }
}

/**
 * Loop
 */

void loop() {
    unsigned long time;
    time = millis();
    unsigned long static lastReadSensor = 0;

    if (time > lastReadSensor + TIME_SENSOR)
    {
        sensors -> readPressure();
        SensorPressureValues_t pressure = sensors -> getRelativePressureInCmH20();

        sensors -> readVolume();
        SensorVolumeValue_t volume = sensors -> getVolume();

        char* string = (char*)malloc(100);
        sprintf(string, "DT %05d %05d %05d %06d", ((int)pressure.pressure1), ((int)pressure.pressure2), volume.volume, ((int)(sensors->getFlow() * 1000)));
        Serial2.println(string);
        //Serial.println(string);
        free(string);

        if (pressure.state == SensorStateFailed) {
            //TODO sensor fail. do something
            Serial.println(F("FALLO Sensor"));
            // TODO: BUZZ ALARMS LIKE HELL
        }
        lastReadSensor = time;
    }

    if (Serial2.available()) {
        readIncomingMsg();
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
