// =========================================================================
// DEPENDENCIAS
// =========================================================================

#include "defaults.h"
#include "utils.h"
#include "pinout.h"
#include "Display.h"
#include "Encoder.h"
#include "MechVentilation.h"
#include "src/AccelStepper/AccelStepper.h"

// =========================================================================
// VARIABLES
// =========================================================================

int rpm                    = DEFAULT_RPM;
int porcentajeInspiratorio = DEFAULT_POR_INSPIRATORIO;
int estatura               = DEFAULT_ESTATURA;
int sexo                   = DEFAULT_SEXO;
int microStepper           = DEFAULT_MICROSTEPPER;
int aceleracion            = DEFAULT_ACELERACION * microStepper;
int pasosPorRevolucion     = DEFAULT_PASOS_POR_REVOLUCION;
float flujoTrigger         = DEFAULT_FLUJO_TRIGGER;

bool tieneTrigger;
bool modo = true, errorFC = false;
int volumenTidal;
float speedIns, speedEsp, tCiclo, tIns, tEsp;

// pines en pinout.h
AccelStepper stepper(
  AccelStepper::DRIVER,
  DIRpin,
  PULpin
); // direction Digital 6 (CW), pulses Digital 7 (CLK)
Encoder encoder(
  DTpin,
  CLKpin,
  SWpin
);
Display display = Display();
MechVentilation ventilation;

// =========================================================================
// SETUP
// =========================================================================

void setup()
{

  // INICIALIZACION
  // =========================================================================

  // Puerto serie
  Serial.begin(9600);
  Serial.println("Inicio");

  // Display de inicio
  display.writeLine(0, "REESPIRATOR");

  // Zumbador
  pinMode(BUZZpin, OUTPUT);
  digitalWrite(BUZZpin, HIGH); // test zumbador
  delay(100);
  digitalWrite(BUZZpin, LOW);

  // FC efecto hall
  pinMode(ENDSTOPpin, INPUT); // el sensor de efecto hall da un 1 cuando detecta

  // Parte motor
  pinMode(ENpin, OUTPUT);
  digitalWrite(ENpin, LOW);

  Serial.println("Setup");
  stepper.setAcceleration(aceleracion);

  // deja la display en blanco
  delay(1000);
  display.clear();
  delay(100);


  // INTERACCIÓN: ESTATURA
  // =========================================================================
  display.writeLine(0, "Introduce estatura");
  while(!encoder.readButton()) {
    encoder.updateValue(&estatura);
    display.writeLine(1, "Altura: " + String(estatura) + " cm");
  }
  display.writeLine(0, "Valor guardado");
  display.writeLine(1, "Altura: " + String(estatura) + " cm");
  Serial.println("Altura (cm): " + String(estatura));
  delay(1000);
  display.clear();


  // INTERACCIÓN: SEXO
  // =========================================================================
  display.writeLine(0, "Introduce sexo");
  while(!encoder.readButton()) {
    encoder.swapValue(&sexo);
    if (sexo == 0) {
      display.writeLine(1, "Sexo: varon");
    } else if (sexo == 1) {
      display.writeLine(1, "Sexo: mujer");
    }
  }
  display.writeLine(0, "Sexo seleccionado");
  if (sexo == 0) {
    display.writeLine(1, "Sexo: varon");
  } else if (sexo == 1) {
    display.writeLine(1, "Sexo: mujer");
  }
  Serial.println("Sexo (0:V, 1:M): " + String(sexo));
  delay(1000);
  display.clear();


  // ESTIMACIÓN: VOLUMEN TIDAL
  // =========================================================================
  display.writeLine(0, "Volumen tidal");
  // TODO: calcular volumen tidal estimado en función de la estatura
  calcularVolumenTidal(&volumenTidal, estatura, sexo);
  display.writeLine(1, String(volumenTidal) + " ml");
  Serial.println("Volumen tidal estimado (ml): " + String(volumenTidal));
  delay(2000);
  display.clear();


  // INTERACCIÓN: VOLUMEN TIDAL
  // =========================================================================
  display.writeLine(0, "Modifica volumen");
  while(!encoder.readButton()) {
    encoder.updateValue(&volumenTidal, 10);
    volumenTidal = constrain(volumenTidal, DEFAULT_MIN_VOLUMEN_TIDAL, DEFAULT_MAX_VOLUMEN_TIDAL);
    display.writeLine(1, String(volumenTidal) + " ml");
  }
  display.writeLine(0, "Valor guardado");
  display.writeLine(1, String(volumenTidal) + " ml");
  Serial.println("Volumen tidal configurado (ml): " + String(volumenTidal));
  delay(1000);
  display.clear();


  // INTERACCIÓN: TRIGGER SI/NO
  // =========================================================================
  display.writeLine(0, "Trigger?");
  while(!encoder.readButton()) {
    encoder.swapValue(&tieneTrigger);
    if (tieneTrigger) {
      display.writeLine(1, "Si");
    } else {
      display.writeLine(1, "No");
    }
  }
  display.writeLine(0, "Valor guardado");
  if (tieneTrigger) {
    display.writeLine(1, "Trigger: Si");
  } else {
    display.writeLine(1, "Trigger: No");
  }
  Serial.println("Trigger (0:No, 1:Si): " + String(tieneTrigger));
  delay(1000);
  display.clear();


  // INTERACCIÓN: VALOR DEL TRIGGER
  // =========================================================================
  if (tieneTrigger) {
    display.writeLine(0, "Modifica trigger");
    while(!encoder.readButton()) {
      encoder.updateValue(&flujoTrigger, 0.1);
      display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
    }
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
    Serial.println("Flujo trigger (LPM): " + String(flujoTrigger));
    delay(1000);
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
  display.writeLine(0, "Valor guardado");
  display.writeLine(1, String(rpm) + " rpm");
  Serial.println("Frecuencia respiratoria (rpm): " + String(rpm));
  delay(1000);
  display.clear();


  // CÁLCULO: CONSTANTES DE TIEMPO INSPIRACION/ESPIRACION
  // =========================================================================
  display.writeLine(0, "Tins | Tesp (seg)");
  calcularCicloInspiratorio(&speedIns, &speedEsp, &tIns, &tEsp,
                            &tCiclo, pasosPorRevolucion, microStepper,
                            porcentajeInspiratorio, rpm);
  display.writeLine(1, String(tIns) + " s" + String(tEsp) + " s");
  Serial.println("Tiempo del ciclo (seg):" + String(tCiclo));
  Serial.println("Tiempo inspiratorio (seg):" + String(tIns));
  Serial.println("Tiempo espiratorio (seg):" + String(tEsp));
  Serial.println("Velocidad 1 calculada:" + String(speedIns));
  Serial.println("Velocidad 2 calculada:" + String(speedEsp));
  delay(2000);
  display.clear();

  // INFORMACIÓN: PARÁMETROS
  // =========================================================================
  display.writeLine(0, "Vol: " + String(volumenTidal) + " ml | " + "Frec: " + String(rpm) + " rpm");
  if (tieneTrigger) {
    display.writeLine(1, "Trigger: " + String(flujoTrigger) + " LPM");
  } else {
    display.writeLine(1, "No trigger");
  }
  delay(2000);


  // INTERACCIÓN: ARRANQUE
  // =========================================================================
  display.writeLine(0, "Pulsa para iniciar");
  display.writeLine(1, "Esperando...");
  while(!encoder.readButton());
  display.clear();
  display.writeLine(1, "Iniciando...");

  // Habilita el motor
  digitalWrite(ENpin, HIGH);

  // configura la ventilación
  if (tieneTrigger) {
    ventilation = MechVentilation(volumenTidal, tIns, tEsp, speedIns, speedEsp, flujoTrigger);
  } else {
    ventilation = MechVentilation(volumenTidal, tIns, tEsp, speedIns, speedEsp);
  }
  ventilation.start();
  delay(500);
  display.clear();
}

// =========================================================================
// LOOP
// =========================================================================

void loop() {
  display.writeLine(0, "Operando...");
  // TODO: display.writeLine(1, "TODO Prompt ventilation status");
  ventilation.update();


  // TODO: si hay nueva configuración: cambiar parámetros escuchando entrada desde el encoder


  // TODO: chequear trigger
  // si hay trigger, esperar al flujo umbral para actuar, si no, actuar en cada bucle


  // Si está en inspiración: controlar con PID el volumen tidal (el que se insufla)

  // Si está en espiración: soltar balón (mover leva hacia arriba sin controlar) y esperar


  // ======================================================================
  // CÓDIGO OBSOLETO DE AQUÍ PARA ABAJO
  // ======================================================================

  // Parte menu
  // display.update(encoder.read());

  // Parte stepper
  stepper.run();

  //recalcular valores por si han cambiado en el menu
  // TODO: sustituir por nueva funcion: calcularConstantes();

  // Primera mitad del ciclo
  if (!stepper.isRunning() && modo && !errorFC) {
    Serial.println("Modo 1");
    stepper.setMaxSpeed(speedIns * microStepper);

    stepper.move(pasosPorRevolucion * microStepper / 2);
    modo = !modo;
  }

  // Segunda mitad del ciclo
  if (!stepper.isRunning() && !modo && !errorFC) {
    Serial.println("Modo 2");

    //se ha llegado al final de carrera en el momento que toca pensar que esta defino como pullup
    if (digitalRead(ENDSTOPpin)) {
      Serial.println("Final de carrera OK");
      stepper.setMaxSpeed(speedEsp * microStepper);
      stepper.move(pasosPorRevolucion * microStepper / 2);
      modo = !modo;
    }
    // si acabada la segunda parte del movimiento no se ha llegado al SENSOR HALL entonces da un paso y vuelve a hacer la comprovocacion
    else {
      Serial.println("Final de carrera NO DETECTADO: Buscando FC");
      errorFC = true;
      digitalWrite(BUZZpin, true); //activa el zumbador
      stepper.move(1 * microStepper);
    }
  }

  //si estamos en error y ha hecho los pasos extra en busca del Final de Carrera
  if (!stepper.isRunning() && errorFC) {
    // no se ha llegado al final suena el BUZZ y ordena dar 3 pasos en busca del FC
    if (!digitalRead(ENDSTOPpin)) {
      Serial.println("--Buscando FC");
      errorFC = true;
      stepper.move(1);
    }
    // cuando lo ha localizado ordena seguir con velocidad 2
    else {
      Serial.println("Detectado FC: restableciendo el origen");
      errorFC = false;
      digitalWrite(BUZZpin, false); //apaga el zumbador
      stepper.setMaxSpeed(speedEsp * microStepper);
      stepper.move(pasosPorRevolucion * microStepper / 2);
      modo = !modo; //cambiamos de velocidad
    }
  }
  // si hay un error pero ha hecho los 100 pasos extra en busca del Final de Carrera
  else if (!stepper.isRunning() && errorFC) {
    // no se ha llegado al final suena el BUZZ y ordena dar 3 pasos en busca del FC
    if (digitalRead(ENDSTOPpin)) {
      errorFC = true;
      stepper.move(1);
      digitalWrite(BUZZpin, true);
      Serial.println("ZUMBA");
    }
    // cuando lo ha localizado ordena seguir con velocidad 2
    else {
      errorFC = false;
      digitalWrite(BUZZpin, false);
      stepper.setMaxSpeed(speedEsp);
      stepper.move(pasosPorRevolucion / 2);
    }
  }
}
