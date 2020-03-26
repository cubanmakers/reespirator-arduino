// =========================================================================
// DEPENDENCIAS
// =========================================================================

#include "defaults.h"
#include "calc.h"
#include "pinout.h"
#include "Display.h"
#include "Encoder.h"
#include "MechVentilation.h"
#include "Sensors.h"
#include "src/FlexyStepper/FlexyStepper.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include "src/Adafruit_BME280/Adafruit_BME280.h"

#define PRUEBAS 1
// =========================================================================
// VARIABLES
// =========================================================================

int height = DEFAULT_HEIGHT;
int sex = DEFAULT_SEX;
int rpm = DEFAULT_RPM;
int inspiratoryFraction = DEFAULT_INSPIRATORY_FRACTION;
int microSteps = STEPPER_MICROSTEPS;
float triggerThreshold = DEFAULT_TRIGGER_THRESHOLD;

bool hasTrigger;
int positionInSteps;
int valveState;
int tidalVolume;
float tCycle, tIns, tEsp;
float flow;

unsigned long static periodCounter,
    periodTimeStamp;

// pines en pinout.h
FlexyStepper *stepper = new FlexyStepper(); // direction Digital 6 (CW), pulses Digital 7 (CLK)

Encoder encoder(PIN_ENCODER_DT, PIN_ENCODER_CLK, PIN_ENCODER_SW);
Display display = Display();

Adafruit_BME280 bme1(PIN_BME_CS1, PIN_BME_MOSI, PIN_BME_MISO, PIN_BME_SCK);
Adafruit_BME280 bme2(PIN_BME_CS2, PIN_BME_MOSI, PIN_BME_MISO, PIN_BME_SCK);

Sensors *sensors;
MechVentilation *ventilation;

// =========================================================================
// SETUP
// =========================================================================

void setup()
{

    // INICIALIZACION
    // =========================================================================
    // Puerto serie
    Serial.begin(9600);
    // Serial.println("Inicio");

    // Display de inicio
    display.writeLine(0, " REESPIRATOR 23 ");

    // Zumbador
    pinMode(PIN_BUZZ, OUTPUT);
    digitalWrite(PIN_BUZZ, HIGH); // test zumbador
    delay(100);
    digitalWrite(PIN_BUZZ, LOW);

    // FC efecto hall
    pinMode(PIN_ENDSTOP, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

    // Electrovalvula, abierta
    pinMode(PIN_VALVE, OUTPUT);
    digitalWrite(PIN_VALVE, HIGH);

    // Parte motor
    pinMode(PIN_STEPPER_ENABLE, OUTPUT);
    digitalWrite(PIN_STEPPER_ENABLE, HIGH); // high lo inhabilita

    // Sensores de presión
    sensors = new Sensors(bme1, bme2);
    int check = sensors->begin();
    if (check)
    {
        display.clear();
        if (check == 1)
        {
            display.writeLine(0, "bme1 not found");
            Serial.println("Could not find sensor BME280 number 1, check wiring!");
        }
        else if (check == 2)
        {
            display.writeLine(0, "bme2 not found");
            Serial.println("Could not find sensor BME280 number 2, check wiring!");
        }
        display.writeLine(1, "Check wires!");
        while (1);
    }

#if PRUEBAS
    height = 170;
    sex = 0;
    rpm = 14;
    // tidalVolume = calculateTidalVolume(height, sex);
    tidalVolume = 800;
    calculateInspiratoryCycle(
        &tIns,
        &tEsp,
        &tCycle,
        inspiratoryFraction,
        rpm);
#else
    // deja la display en blanco
    delay(3000);
    display.clear();
    display.writeLine(0, "De personas");
    display.writeLine(1, "   para personas");
    delay(3000);
    display.clear();
    delay(100);

    // INTERACCIÓN: ESTATURA
    // =========================================================================
    display.writeLine(0, "Introduce altura");
    while (!encoder.readButton())
    {
        encoder.updateValue(&height);
        display.writeLine(1, "Altura: " + String(height) + " cm");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, "Altura: " + String(height) + " cm");
    Serial.println("Altura (cm): " + String(height));
    delay(2000);
    display.clear();

    // INTERACCIÓN: SEXO
    // =========================================================================
    display.writeLine(0, "Introduce sexo");
    while (!encoder.readButton())
    {
        encoder.swapValue(&sex);
        if (sex == 0)
        {
            display.writeLine(1, "Sexo: varon");
        }
        else if (sex == 1)
        {
            display.writeLine(1, "Sexo: mujer");
        }
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    if (sex == 0)
    {
        display.writeLine(1, "Sexo: varon");
    }
    else if (sex == 1)
    {
        display.writeLine(1, "Sexo: mujer");
    }
    Serial.println("Sexo (0:V, 1:M): " + String(sex));
    delay(2000);
    display.clear();

    // ESTIMACIÓN: VOLUMEN TIDAL
    // =========================================================================
    display.writeLine(0, "Volumen tidal");
    // TODO: calcular volumen tidal estimado en función de la estatura tidalVolume
    tidalVolume = calculateTidalVolume(height, sex);
    display.writeLine(1, String(tidalVolume) + " ml");
    Serial.println("Volumen tidal estimado (ml): " + String(tidalVolume));
    delay(2000);
    display.clear();

    // INTERACCIÓN: VOLUMEN TIDAL
    // =========================================================================
    display.writeLine(0, "Modifica volumen");
    while (!encoder.readButton())
    {
        encoder.updateValue(&tidalVolume, 10);
        tidalVolume = constrain(tidalVolume, DEFAULT_MIN_TIDAL_VOLUME, DEFAULT_MAX_TIDAL_VOLUME);
        display.writeLine(1, String(tidalVolume) + " ml");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, String(tidalVolume) + " ml");
    Serial.println("Volumen tidal configurado (ml): " + String(tidalVolume));
    delay(2000);
    display.clear();

    // INTERACCIÓN: TRIGGER SI/NO
    // =========================================================================
    display.writeLine(0, "Trigger?");
    while (!encoder.readButton())
    {
        encoder.swapValue(&hasTrigger);
        if (hasTrigger)
        {
            display.writeLine(1, "Si");
        }
        else
        {
            display.writeLine(1, "No");
        }
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    if (hasTrigger)
    {
        display.writeLine(1, "Trigger: Si");
    }
    else
    {
        display.writeLine(1, "Trigger: No");
    }
    Serial.println("Trigger (0:No, 1:Si): " + String(hasTrigger));
    delay(2000);
    display.clear();
    hasTrigger = false;

    // INTERACCIÓN: VALOR DEL TRIGGER
    // =========================================================================
    if (hasTrigger)
    {
        display.writeLine(0, "Modifica trigger");
        while (!encoder.readButton())
        {
            encoder.updateValue(&triggerThreshold, 0.1);
            display.writeLine(1, "Flujo: " + String(triggerThreshold) + " LPM");
        }
        display.clear();
        display.writeLine(0, "Valor guardado");
        display.writeLine(1, "Flujo: " + String(triggerThreshold) + " LPM");
        Serial.println("Flujo trigger (LPM): " + String(triggerThreshold));
        delay(2000);
        display.clear();
    }

    // INTERACCIÓN: FRECUENCIA RESPIRATORIA
    // =========================================================================
    display.writeLine(0, "Frecuencia resp.");
    while(!encoder.readButton()) {
    encoder.updateValue(&rpm);
    rpm = constrain(rpm, DEFAULT_MIN_RPM, DEFAULT_MAX_RPM);
    display.writeLine(1, String(rpm) + " rpm");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, String(rpm) + " rpm");
    Serial.println("Frecuencia respiratoria (rpm): " + String(rpm));
    delay(2000);
    display.clear();


    // CÁLCULO: CONSTANTES DE TIEMPO INSPIRACION/ESPIRACION
    // =========================================================================
    display.writeLine(0, "Tins   | Tesp");
    calculateInspiratoryCycle(
        &tIns,
        &tEsp,
        &tCycle,
        inspiratoryFraction,
        rpm);
    display.writeLine(1, String(tIns) + " s | " + String(tEsp) + " s");
    Serial.println("Tiempo del ciclo (seg):" + String(tCycle));
    Serial.println("Tiempo inspiratorio (seg):" + String(tIns));
    Serial.println("Tiempo espiratorio (seg):" + String(tEsp));
    delay(4000);
    display.clear();

    // INFORMACIÓN: PARÁMETROS
    // =========================================================================
    display.writeLine(
        0,
        "V:" + String(tidalVolume) + " ml F:" + String(rpm) + " rpm");
    if (hasTrigger)
    {
        display.writeLine(1, "Trigger: " + String(triggerThreshold) + " LPM");
    }
    else
    {
        display.writeLine(1, "No trigger");
    }
    delay(4000);
#endif
    display.clear();

    // INTERACCIÓN: CALIBRACION DE SENSORES DE PRESION
    // =========================================================================
    display.writeLine(0, "  Pulsar para   ");
    display.writeLine(1, "calibrar presion");

    while (!encoder.readButton());
    display.clear();
    display.writeLine(1, "Calibrando...");

    // OPERACIÓN: CALIBRACION
    // =========================================================================
    sensors->getOffsetBetweenPressureSensors();
    display.clear();
    display.writeLine(0, "    Sensores    ");
    display.writeLine(1, "   calibrados   ");
    display.clear();
    delay(1000);

    // INTERACCIÓN: ARRANQUE
    // =========================================================================
    display.writeLine(0, "Pulsa para iniciar");
    display.writeLine(1, "Esperando...");

    while (!encoder.readButton());
    display.clear();
    display.writeLine(1, "Iniciando...");
    delay(1000);
    display.clear();

    // OPERACIÓN: HOMING DEL STEPPER
    // =========================================================================
    // Habilita el motor
    digitalWrite(PIN_STEPPER_ENABLE, LOW);
    stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_INSUFFLATION * STEPPER_MICROSTEPS);
    stepper->setAccelerationInStepsPerSecondPerSecond(
        STEPPER_ACC_INSUFFLATION * STEPPER_MICROSTEPS);
    stepper->setStepsPerRevolution(
        STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);
    if (stepper->moveToHomeInSteps(
            STEPPER_HOMING_DIRECTION,
            STEPPER_HOMING_SPEED,
            STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
            PIN_ENDSTOP) != true)
    {
        display.clear();
        Serial.println("Homing has failed");
        display.writeLine(0, "Homing Error");
        display.writeLine(1, "Home not found");
        while (true);
    }

    // Save first period timestamp
    periodTimeStamp = millis();
    ventilation = new MechVentilation(stepper, sensors);

    //Timer1.initialize(100); // 100us
    Timer1.initialize(1000000); //
    Timer1.attachInterrupt(timer1Isr);
    Timer3.initialize(50);
    Timer3.attachInterrupt(timer3Isr);
    // Serial.println("End setup");

    // Impose initial target position
    positionInSteps = STEPPER_HIGHEST_POSITION;
    display.writeLine(0, "Frecuencia:");
}

// =========================================================================
// LOOP
// =========================================================================

volatile int flagTimer1 = 0;
volatile unsigned long tmp;

void loop()
{

    unsigned long static timestamp;
    timestamp = millis();
    unsigned long static lastReadSensors = 0;
    if (timestamp >= lastReadSensors + SENSORS_PERIOD_READING)
    {
        lastReadSensors = timestamp;
        sensors->readPressure();
    }
    bool static startedInsuflation = false;
    bool static startedExsuflation = false;
    periodCounter = millis() - periodTimeStamp;

    if (periodCounter < int(tCycle * 1000))
    {
        if (!startedInsuflation)
        {
            // Trigger valve once before startedInsuflation becomes true
            if (!startedInsuflation)
            {
                valveState = LOW;
                digitalWrite(PIN_VALVE, valveState);
            }
            startedInsuflation = true;
            // Serial.println("Insuflation at " + String(timestamp));
            //Serial.println("Timer " + String(Timer1.read()));
            if (periodCounter <= int(tIns * 1000) - DEFAULT_RETAIN_INSIPIRATION)
            {
                ventilation->update(true, positionInSteps);
            }
        }
        else
        {
            if (!startedExsuflation && periodCounter > int(tIns * 1000))
            {
                // Trigger valve once before startedExsuflation becomes true
                if (!startedExsuflation)
                {
                    valveState = HIGH;
                    digitalWrite(PIN_VALVE, valveState);
                }
                // Serial.println("Exsuflation at " + String(timestamp));
                ventilation->update(false);
                startedExsuflation = true;
            }
        }

        // Period time ends. end cycle
    }
    else
    {
        periodTimeStamp = millis();
        startedExsuflation = false;
        startedInsuflation = false;
    }

    // Serial.println(periodTimeStamp);

    if (flagTimer1)
    {
        flagTimer1 = 0;
        //Serial.println("Timer launched" + String (timestamp));
        // Serial.println("tmp=" + String(tmp));
    }

    int previousRpm = rpm;
    encoder.updateValue(&rpm);
    rpm = constrain(rpm, DEFAULT_MIN_RPM, DEFAULT_MAX_RPM);
    if (rpm != previousRpm) {
        calculateInspiratoryCycle(
            &tIns,
            &tEsp,
            &tCycle,
            inspiratoryFraction,
            rpm);
    }
    display.writeLine(1, String(rpm) + " rpm  ");

    SensorValues_t values = sensors -> getRelativePressureInCmH20();
    // flow = sensors -> calculateFilteredFlow();
    Serial.print(valveState); Serial.print(F(","));
    Serial.print(values.pressure1); Serial.print(F(","));
    Serial.println(values.pressure2); // Serial.print(F(","));
    // Serial.println(flow);


    // if (sensors -> getRelativePressureInPascals().state == SensorStateFailed) {
    // TODO sensor fail. do something   display.clear();   display.writeLine(0,
    // "Valor guardado"); }

    // TODO: si hay nueva configuración: cambiar parámetros escuchando entrada desde
    // el encoder

    // TODO: chequear trigger si hay trigger, esperar al flujo umbral para actuar,
    // si no, actuar en cada bucle Si está en inspiración: controlar con PID el
    // volumen tidal (el que se insufla) Si está en espiración: soltar balón (mover
    // leva hacia arriba sin controlar) y esperar
}

/**
 * Timer 1 ISR
 */
void timer1Isr(void)
{
    flagTimer1 = 1;
    //ventilation->update();
}

void timer3Isr(void)
{
    unsigned long t1 = micros();
    stepper->processMovement();
    unsigned long t2 = micros();
    tmp = t2 - t1;
}
