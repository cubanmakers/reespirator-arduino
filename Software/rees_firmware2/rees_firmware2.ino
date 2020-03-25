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

int estatura = DEFAULT_ESTATURA;
int sexo = DEFAULT_SEXO;
int rpm = DEFAULT_RPM;
int porcentajeInspiratorio = DEFAULT_POR_INSPIRATORIO;
int microSteps = STEPPER_MICROSTEPS;
int pasosPorRevolucion = STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS;
float flujoTrigger = DEFAULT_FLUJO_TRIGGER;

bool tieneTrigger;
int positionInSteps;
int valveState;
int volumenTidal;
float speedIns, speedEsp, tCiclo, tIns, tEsp;
float flow;

unsigned long static periodCounter,
    periodTimeStamp;

// pines en pinout.h
FlexyStepper *stepper = new FlexyStepper(); // direction Digital 6 (CW), pulses Digital 7 (CLK)

Encoder encoder(ENCODER_DT_PIN, ENCODER_CLK_PIN, ENCODER_SW_PIN);
Display display = Display();

Adafruit_BME280 bme1(BME_CS1, BME_MOSI, BME_MISO, BME_SCK);
Adafruit_BME280 bme2(BME_CS2, BME_MOSI, BME_MISO, BME_SCK);

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
    pinMode(BUZZ_PIN, OUTPUT);
    digitalWrite(BUZZ_PIN, HIGH); // test zumbador
    delay(100);
    digitalWrite(BUZZ_PIN, LOW);

    // FC efecto hall
    pinMode(ENDSTOP_PIN, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

    // Electrovalvula, abierta
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, HIGH);

    // Parte motor
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH); // high lo inhabilita

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
        while (1)
            ;
    }

#if PRUEBAS
    estatura = 170;
    sexo = 0;
    rpm = 14;
    // volumenTidal = calcularVolumenTidal(estatura, sexo);
    volumenTidal = 800;
    calcularCicloInspiratorio(
        &speedIns,
        &speedEsp,
        &tIns,
        &tEsp,
        &tCiclo,
        porcentajeInspiratorio,
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
        encoder.updateValue(&estatura);
        display.writeLine(1, "Altura: " + String(estatura) + " cm");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, "Altura: " + String(estatura) + " cm");
    Serial.println("Altura (cm): " + String(estatura));
    delay(2000);
    display.clear();

    // INTERACCIÓN: SEXO
    // =========================================================================
    display.writeLine(0, "Introduce sexo");
    while (!encoder.readButton())
    {
        encoder.swapValue(&sexo);
        if (sexo == 0)
        {
            display.writeLine(1, "Sexo: varon");
        }
        else if (sexo == 1)
        {
            display.writeLine(1, "Sexo: mujer");
        }
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    if (sexo == 0)
    {
        display.writeLine(1, "Sexo: varon");
    }
    else if (sexo == 1)
    {
        display.writeLine(1, "Sexo: mujer");
    }
    Serial.println("Sexo (0:V, 1:M): " + String(sexo));
    delay(2000);
    display.clear();

    // ESTIMACIÓN: VOLUMEN TIDAL
    // =========================================================================
    display.writeLine(0, "Volumen tidal");
    // TODO: calcular volumen tidal estimado en función de la estatura volumenTidal
    volumenTidal = calcularVolumenTidal(estatura, sexo);
    display.writeLine(1, String(volumenTidal) + " ml");
    Serial.println("Volumen tidal estimado (ml): " + String(volumenTidal));
    delay(2000);
    display.clear();

    // INTERACCIÓN: VOLUMEN TIDAL
    // =========================================================================
    display.writeLine(0, "Modifica volumen");
    while (!encoder.readButton())
    {
        encoder.updateValue(&volumenTidal, 10);
        volumenTidal = constrain(volumenTidal, DEFAULT_MIN_VOLUMEN_TIDAL, DEFAULT_MAX_VOLUMEN_TIDAL);
        display.writeLine(1, String(volumenTidal) + " ml");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, String(volumenTidal) + " ml");
    Serial.println("Volumen tidal configurado (ml): " + String(volumenTidal));
    delay(2000);
    display.clear();

    // INTERACCIÓN: TRIGGER SI/NO
    // =========================================================================
    display.writeLine(0, "Trigger?");
    while (!encoder.readButton())
    {
        encoder.swapValue(&tieneTrigger);
        if (tieneTrigger)
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
    if (tieneTrigger)
    {
        display.writeLine(1, "Trigger: Si");
    }
    else
    {
        display.writeLine(1, "Trigger: No");
    }
    Serial.println("Trigger (0:No, 1:Si): " + String(tieneTrigger));
    delay(2000);
    display.clear();
    tieneTrigger = false;

    // INTERACCIÓN: VALOR DEL TRIGGER
    // =========================================================================
    if (tieneTrigger)
    {
        display.writeLine(0, "Modifica trigger");
        while (!encoder.readButton())
        {
            encoder.updateValue(&flujoTrigger, 0.1);
            display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
        }
        display.clear();
        display.writeLine(0, "Valor guardado");
        display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
        Serial.println("Flujo trigger (LPM): " + String(flujoTrigger));
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
    calcularCicloInspiratorio(
        &speedIns,
        &speedEsp,
        &tIns,
        &tEsp,
        &tCiclo,
        porcentajeInspiratorio,
        rpm);
    display.writeLine(1, String(tIns) + " s | " + String(tEsp) + " s");
    Serial.println("Tiempo del ciclo (seg):" + String(tCiclo));
    Serial.println("Tiempo inspiratorio (seg):" + String(tIns));
    Serial.println("Tiempo espiratorio (seg):" + String(tEsp));
    Serial.println("Velocidad 1 calculada:" + String(speedIns));
    Serial.println("Velocidad 2 calculada:" + String(speedEsp));
    delay(4000);
    display.clear();

    // INFORMACIÓN: PARÁMETROS
    // =========================================================================
    display.writeLine(
        0,
        "V:" + String(volumenTidal) + " ml F:" + String(rpm) + " rpm");
    if (tieneTrigger)
    {
        display.writeLine(1, "Trigger: " + String(flujoTrigger) + " LPM");
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
    display.writeLine(0, "Sensores");
    display.writeLine(0, "calibrados");
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
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    stepper->connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    stepper->setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT * STEPPER_MICROSTEPS);
    stepper->setAccelerationInStepsPerSecondPerSecond(
        STEPPER_ACC_DEFAULT * STEPPER_MICROSTEPS);
    stepper->setStepsPerRevolution(
        STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);
    if (stepper->moveToHomeInSteps(
            STEPPER_HOMING_DIRECTION,
            STEPPER_HOMING_SPEED,
            STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
            ENDSTOP_PIN) != true)
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
    display.writeLine(0, "Posicion:");
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

    if (periodCounter < int(tCiclo * 1000))
    {
        if (!startedInsuflation)
        {
            // Trigger valve once before startedInsuflation becomes true
            if (!startedInsuflation)
            {
                valveState = LOW;
                digitalWrite(VALVE_PIN, valveState);
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
                    digitalWrite(VALVE_PIN, valveState);
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

    encoder.updateValue(&positionInSteps);
    display.writeLine(1, String(positionInSteps) + "  ");

    int delta = periodCounter - periodTimeStamp;
    SensorValues_t values = sensors -> getPressureInCmH20();
    flow = getCurrentFlow(values.pressure1, values.pressure2);
    Serial.print(delta); Serial.print(F(","));
    Serial.print(valveState); Serial.print(F(","));
    Serial.print(values.pressure1); Serial.print(F(","));
    Serial.print(values.pressure2); Serial.print(F(","));
    Serial.println(flow);


    // if (sensors -> getPressureInPascals().state == SensorStateFailed) {
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
