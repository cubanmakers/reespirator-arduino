// =========================================================================
// DEPENDENCIAS
// =========================================================================

#include "defaults.h"
#include "calc.h"
#include "pinout.h"
#include "Display.h"
#include "Encoder.h"
#include "MechVentilation.h"
#include "src/FlexyStepper/FlexyStepper.h"
#include "src/TimerOne/TimerOne.h"
#include "src/TimerThree/TimerThree.h"
#include "src/Adafruit_BME280/Adafruit_BME280.h"
#include "Sensors.h"

// =========================================================================
// VARIABLES
// =========================================================================

byte rpm                    = DEFAULT_RPM;
byte porcentajeInspiratorio = DEFAULT_POR_INSPIRATORIO;
short estatura               = DEFAULT_ESTATURA;
bool sexo                   = DEFAULT_SEXO;
//byte microStepper           = STEPPER_MICROSTEPS;
//int aceleracion            = DEFAULT_ACELERACION * microStepper;
//int pasosPorRevolucion     = STEPPER_MICROSTEPS_PER_REVOLUTION;
float flujoTrigger         = DEFAULT_FLUJO_TRIGGER;

bool tieneTrigger;
bool modo = true, errorFC = false;
int volumenTidal;
float speedIns, speedEsp, tCiclo, tIns, tEsp;


#if DEBUG_STATE_MACHINE
String debugMsg[15];
byte debugMsgCounter = 0;
#endif

// pines en pinout.h
FlexyStepper * stepper = new FlexyStepper(); // direction Digital 6 (CW), pulses Digital 7 (CLK)

Encoder encoder(
  DTpin,
  CLKpin,
  SWpin
);
Display display = Display();


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

Sensors* sensors;
MechVentilation* ventilation;

// =========================================================================
// SETUP
// =========================================================================

void setup()
{

  // INICIALIZACION
  // =========================================================================

  // Puerto serie
  Serial.begin(115200);
  Serial.println("Inicio");

  // Display de inicio
  display.writeLine(0, " REESPIRATOR 23 ");

  // Zumbador
  pinMode(BUZZpin, OUTPUT);
  digitalWrite(BUZZpin, HIGH); // test zumbador
  delay(100);
  digitalWrite(BUZZpin, LOW);

  // FC efecto hall
  pinMode(ENDSTOPpin, INPUT_PULLUP); // el sensor de efecto hall da un 1 cuando detecta

#ifndef PRUEBAS
  // Sensores de presión
  sensors = new Sensors(bmp1, bmp2);
  int check = sensors->begin();
    if (check) {
      display.clear();
      if (check == 1) {
        display.writeLine(0, "bme1 not found");
        Serial.println("Could not find sensor BME280 number 1, check wiring!");
      } else if (check == 2) {
        display.writeLine(0, "bme2 not found");
        Serial.println("Could not find sensor BME280 number 2, check wiring!");
      }
      display.writeLine(1, "Check wires!");
      while(1);
    }
#endif

  // Parte motor
  pinMode(ENpin, OUTPUT);
  digitalWrite(ENpin, HIGH);

  Serial.println("Setup");
#ifndef PRUEBAS
  // deja la display en blanco
  delay(3000);
  display.clear();
  display.writeLine(0, "De personas");
  display.writeLine(1, "   para personas");
  delay(3000);
  display.clear();
  delay(100);
#endif
  // INTERACCIÓN: ESTATURA
  // =========================================================================
  /*
  display.writeLine(0, "Introduce altura");
  while(!encoder.readButton()) {
    encoder.updateValue(&estatura);
    display.writeLine(1, "Altura: " + String(estatura) + " cm");
  }
  display.clear();
  display.writeLine(0, "Valor guardado");
  display.writeLine(1, "Altura: " + String(estatura) + " cm");
  Serial.println("Altura (cm): " + String(estatura));
  delay(2000);
  display.clear();
  */
  estatura = 170;

  // INTERACCIÓN: SEXO
  // =========================================================================
  /*
  display.writeLine(0, "Introduce sexo");
  while(!encoder.readButton()) {
    encoder.swapValue(&sexo);
    if (sexo == 0) {
      display.writeLine(1, "Sexo: varon");
    } else if (sexo == 1) {
      display.writeLine(1, "Sexo: mujer");
    }
  }
  display.clear();
  display.writeLine(0, "Valor guardado");
  if (sexo == 0) {
    display.writeLine(1, "Sexo: varon");
  } else if (sexo == 1) {
    display.writeLine(1, "Sexo: mujer");
  }
  Serial.println("Sexo (0:V, 1:M): " + String(sexo));
  delay(2000);
  display.clear();
  */
  sexo = 0;

  // ESTIMACIÓN: VOLUMEN TIDAL
  // =========================================================================
  display.writeLine(0, "Volumen tidal");
  // TODO: calcular volumen tidal estimado en función de la estatura
  volumenTidal = calcularVolumenTidal(estatura, sexo);
  display.writeLine(1, String(volumenTidal) + " ml");
  Serial.println("Volumen tidal estimado (ml): " + String(volumenTidal));
  delay(1000);
  display.clear();


  // INTERACCIÓN: VOLUMEN TIDAL
  // =========================================================================
  /*
  display.writeLine(0, "Modifica volumen");
  while(!encoder.readButton()) {
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
  */


  // INTERACCIÓN: TRIGGER SI/NO
  // =========================================================================
  /*
  display.writeLine(0, "Trigger?");
  while(!encoder.readButton()) {
    encoder.swapValue(&tieneTrigger);
    if (tieneTrigger) {
      display.writeLine(1, "Si");
    } else {
      display.writeLine(1, "No");
    }
  }
  display.clear();
  display.writeLine(0, "Valor guardado");
  if (tieneTrigger) {
    display.writeLine(1, "Trigger: Si");
  } else {
    display.writeLine(1, "Trigger: No");
  }
  Serial.println("Trigger (0:No, 1:Si): " + String(tieneTrigger));
  delay(2000);
  display.clear();
  */
  tieneTrigger = false;


  // INTERACCIÓN: VALOR DEL TRIGGER
  // =========================================================================
  if (tieneTrigger) {
    #if 0
    display.writeLine(0, "Modifica trigger");
    while(!encoder.readButton()) {
      encoder.updateValue(&flujoTrigger, 0.1);
      display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
    }
    display.clear();
    display.writeLine(0, "Valor guardado");
    display.writeLine(1, "Flujo: " + String(flujoTrigger) + " LPM");
    #endif
    Serial.println("Flujo trigger (LPM): " + String(flujoTrigger));
    delay(2000);
    display.clear();
  }


  // INTERACCIÓN: FRECUENCIA RESPIRATORIA
  // =========================================================================
  /*
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
  */
  rpm = 14;


  // CÁLCULO: CONSTANTES DE TIEMPO INSPIRACION/ESPIRACION
  // =========================================================================
  display.writeLine(0, "Tins  | Tesp");
  calcularCicloInspiratorio(&speedIns, &speedEsp, &tIns, &tEsp,
                            &tCiclo, porcentajeInspiratorio, rpm);
  display.writeLine(1, String(tIns) + " s | " + String(tEsp) + " s");
  Serial.println("Tiempo del ciclo (seg):" + String(tCiclo));
  Serial.println("Tiempo inspiratorio (seg):" + String(tIns));
  Serial.println("Tiempo espiratorio (seg):" + String(tEsp));
  Serial.println("Velocidad 1 calculada:" + String(speedIns));
  Serial.println("Velocidad 2 calculada:" + String(speedEsp));
#ifndef PRUEBAS
  delay(1000);
  display.clear();

  // INFORMACIÓN: PARÁMETROS
  // =========================================================================
  display.writeLine(0, "Vol: " + String(volumenTidal) + " ml | " + "Frec: " + String(rpm) + " rpm");
  if (tieneTrigger) {
    display.writeLine(1, "Trigger: " + String(flujoTrigger) + " LPM");
  } else {
    display.writeLine(1, "No trigger");
  }
  delay(1000);
  display.clear();


  // INTERACCIÓN: ARRANQUE
  // =========================================================================
  display.writeLine(0, "Pulsa para iniciar");
  display.writeLine(1, "Esperando...");
  while(!encoder.readButton());
  display.clear();
  display.writeLine(1, "Iniciando...");
#endif
  // Habilita el motor
  digitalWrite(ENpin, LOW);

  // configura la ventilación
  
  if (tieneTrigger) {
    ventilation = new MechVentilation(stepper, sensors, volumenTidal, tIns, tEsp, speedIns, speedEsp, flujoTrigger);
  } else {
    ventilation = new MechVentilation(stepper, sensors, volumenTidal, tIns, tEsp, speedIns, speedEsp);
  }
  
  ventilation->start();
  
  delay(1000);
  display.clear();

#ifndef PRUEBAS
  sensors->readPressure(); 
    Timer1.initialize(TIME_BASE * 1000); // 5 ms
  Timer1.attachInterrupt(timer1Isr);
  //TODO: Option: if (Sensores ok) { arranca timer3 }
  Timer3.initialize(50); //50us
  Timer3.attachInterrupt(timer3Isr);
  #else
  Timer1.initialize(500); // 5 ms
  Timer1.attachInterrupt(timer1Isr);
  //TODO: Option: if (Sensores ok) { arranca timer3 }
  //Timer3.initialize(50); //50us
  //Timer3.attachInterrupt(timer3Isr);
#endif
  encoder.buttonHasBeenPressed();
}

// =========================================================================
// LOOP
// =========================================================================

enum ChangeConfigurationState {
  Disabled = 0,
  SelectMenu = 1,
  MenuVolumeTidal = 2,
  MenuFrecResp = 3,
  MenuApply = 4
};

ChangeConfigurationState changeConfiguration = Disabled;

void processUpdateParameters (void) {
  static short menuRpm;
  static short menuTidalVolume;
  static bool menuAccept = false;

  static ChangeConfigurationState menuSelection = Disabled;

  switch (changeConfiguration) {
      case Disabled:
        if (encoder.buttonHasBeenPressed()) {
          changeConfiguration = SelectMenu;
          display.clear();
          display.writeLine(0,F("Cambiar paramet"));
          display.writeLine(1, F("Volumen tidal"));
          #ifdef PRUEBAS
          Serial.println("Parameter:volumen tidal");
          #endif
          menuSelection = MenuVolumeTidal;
        }
        break;
      case SelectMenu:
        
        if (encoder.buttonHasBeenPressed()) {
          changeConfiguration = menuSelection;
          display.clear();
          switch (changeConfiguration) {
            case MenuVolumeTidal:
              display.writeLine(0, F("Volumen tidal"));
              display.writeLine(1, String(volumenTidal) + F("ml"));
              menuTidalVolume = volumenTidal;
              #ifdef PRUEBAS
                Serial.println("volumen tidal"+ String(volumenTidal));
              #endif
              break;
            case MenuFrecResp:
              display.writeLine(0, F("Frec resp"));
              display.writeLine(1, String(rpm) + F("ml"));
              menuRpm = rpm;
              #ifdef PRUEBAS
                Serial.println("rpm="+ String(rpm));
              #endif
              break;
            case MenuApply:
              display.writeLine(0, F("Aplicar config"));
              display.writeLine(1, F("No"));
              #ifdef PRUEBAS
                Serial.println("Apply config");
              #endif              
              break;  
          }
        } else {
          int tmp = menuSelection - MenuVolumeTidal;
          if (encoder.updateValue(&tmp)) {
            if (tmp == -1) {
              tmp = 2; // workaround
            }

            switch (tmp % 3) {
              case 0:
                display.writeLine(1, F("Volumen tidal"));
                menuSelection = MenuVolumeTidal;
                #ifdef PRUEBAS
                Serial.println("Parameter:volumen tidal");
                #endif                
                break;
              case 1:
                display.writeLine(1, F("Frec resp       "));
                menuSelection = MenuFrecResp;
                #ifdef PRUEBAS
                Serial.println("Parameter:rpm");
                #endif                
                break;
              case 2:
                // apply
                display.writeLine(1, F("Aplicar      "));
                menuSelection = MenuApply;
                #ifdef PRUEBAS
                Serial.println("Aplicar");
                #endif 
                break;
            }
          }
        }
      break;
      case MenuVolumeTidal:
      case MenuFrecResp:
        if (encoder.buttonHasBeenPressed()) {
          changeConfiguration = SelectMenu;
          display.clear();
          display.writeLine(0,F("Cambiar paramet"));
          display.writeLine(1, F("Volumen tidal"));
          #ifdef PRUEBAS
          Serial.println("Parameter:volumen tidal");
          #endif
          menuSelection = MenuVolumeTidal;          
          
        } else {
          if (changeConfiguration == MenuVolumeTidal) {
            if (encoder.updateValue(&menuTidalVolume)) {
              display.writeLine(1, String(menuTidalVolume) + F("ml"));
              #ifdef PRUEBAS
                Serial.println("volumen tidal"+ String(menuTidalVolume));
              #endif
            }
          } else {
            // rpm
              if (encoder.updateValue(&menuRpm)) {
              display.writeLine(1, String(menuRpm) + F("rpm"));
              #ifdef PRUEBAS
                Serial.println("rpm="+ String(menuRpm));
              #endif
              }
          }
        }

        break;
      case MenuApply:
        if (encoder.buttonHasBeenPressed()) {
          if (menuAccept) {
            //Aplicar
              rpm = menuRpm;
              volumenTidal = MenuVolumeTidal;
              #ifdef PRUEBAS
                Serial.println("Aplicado");
              #endif
          } else {
              menuRpm = rpm;
              menuTidalVolume = volumenTidal;
              #ifdef PRUEBAS
                Serial.println("Descartado");
              #endif
          }
          changeConfiguration = Disabled;
          menuSelection = Disabled;
          
        } else {
          if (encoder.swapValue(&menuAccept)) {
            if (menuAccept) {
              display.writeLine(1, F("Sí"));
            } else {
              display.writeLine(1, F("No"));
            }
              #ifdef PRUEBAS
                Serial.println("MenuAceptar="+ String(menuAccept));
              #endif
          }
        }
      break;
    }

}

void loop() {
    unsigned long time;
    time = millis();
    unsigned long static lastReadSensor = 0;

    if (time > lastReadSensor + 15) {
      #ifndef PRUEBAS
            sensors->readPressure();

            SensorPressureValues_t pressure = sensors->getPressure();

            if (pressure.state == SensorStateFailed) {
              //TODO sensor fail. do something
              display.clear();  
              display.writeLine(0, F("FALLO Sensor"));
            } else {
              if (changeConfiguration == Disabled) {
                display.clear();
                display.writeLine (0, "Pres=" + String(pressure.pressure2)); 
              }
            }
      #endif
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

    processUpdateParameters();

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
void timer1Isr(void) {
    ventilation->update();
}

void timer3Isr(void) {
    stepper->processMovement();
}




















































































































