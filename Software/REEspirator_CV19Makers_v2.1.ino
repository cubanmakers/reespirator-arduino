/*
 * UPDATE 18/03/2020 1:39
 * Librerías guardadas en local, no necesaria su descarga
 */


#include "Encoder.h"
#include "LiquidCrystal_I2C.h"
#include "AccelStepper.h"

//Dirección de LCD I2C
#define DIR 0x3F

//Rotary encoder
#define CLKpin 2
#define DTpin 3
#define SWpin  9

//Stepper driver
#define PULpin 6
#define DIRpin 7
#define ENpin 8

//Buzzer
#define BUZZpin 11

//Sensor hall
#define ENDSTOPpin 5

//Crer el objeto lcd
LiquidCrystal_I2C lcd(DIR, 16, 2);


//Crea el objeto Encoder Rotativo
Encoder encoder1(DTpin, CLKpin, SWpin);


 //Variable donde almacenamos el return de leerEncoder
byte tecla = 0;

//Variables a controlar
int rpm = 15;
float vol = 0.5;

//Variable para el menu
int posMenu = 0;
bool editandoMenu = false;
int porcentajeInspiratorio = 80;

//Constantes motor
#define pasosPorRevolucion 200 //Suponiendo un motor de 200 pasos/rev sin microstepper
float velocidadUno=0;       //se calcula el valor de inicio en el setup
float velocidadDos=0;       //idem
int acceleracion=6000;      //6000 para que no se note en el tiempo de ciclo 
float tCiclo, tIns, tEsp;


//mis pines son diferentes por ahora!!
AccelStepper stepper(1, DIRpin, PULpin); //direction Digital 6 (CW), pulses Digital 7 (CLK)

boolean modo = true, errorFC = false;



void setup() {
  //Iniciamos serial
  Serial.begin (9600);
  Serial.println ("Inicio");
  //Parte pantalla
  inicializarPantalla();
  escribirPantalla(rpm, vol, posMenu, 0);
  Serial.println ("PANTALLA ESCRITA");
  
  //Parte motor
  disableMotor();
  Serial.begin(9600);  // Debugging only
  pinMode(BUZZpin, OUTPUT);
  pinMode(ENDSTOPpin, INPUT_PULLUP);
  pinMode(ENpin,OUTPUT); //test zumbador
  digitalWrite(BUZZpin, HIGH);
  delay(100);
  digitalWrite(BUZZpin, LOW);
  Serial.println("Setup");
  stepper.setAcceleration(acceleracion);
  enableMotor();

  tCiclo=60/rpm; //Tiempo de ciclo en segundos
  tIns=(tCiclo*porcentajeInspiratorio)/100;
  tEsp=tCiclo-tIns;
  
  velocidadUno=(pasosPorRevolucion/2)/tIns;
  
  velocidadDos=(pasosPorRevolucion/2)/tEsp;

  Serial.println (tCiclo);
  Serial.println (tIns);
  Serial.println (tEsp);
    Serial.println (".....");
  Serial.println (velocidadUno);
  Serial.println (velocidadDos);
}


void loop() {
//Parte menu
  actualizarMenu();

//Parte stepper
   stepper.run();

//recalcular valores por si han cambiado en el menu

  tCiclo=60/rpm; //Tiempo de ciclo en segundos
  tIns=(tCiclo*porcentajeInspiratorio)/100;
  tEsp=tCiclo-tIns;
  
  velocidadUno=(pasosPorRevolucion/2)/tIns;
  
  velocidadDos=(pasosPorRevolucion/2)/tEsp;
  

  if(!stepper.isRunning() && !errorFC) //si ha teminado media vuelta
    {
      if (modo)                         //velociad 1
        {
        Serial.println("Modo 1");
        stepper.setMaxSpeed(velocidadUno);
        Serial.println (velocidadUno);
        Serial.println (pasosPorRevolucion/2);
        stepper.move(pasosPorRevolucion/2);
        }
      else                                //velociadad 2
        {
        Serial.println("Modo 2, verificar el final de carrera");
        
        if (digitalRead(ENDSTOPpin) )//no se ha llegado al final
          {
            errorFC=true;
             Serial.println ("ZUMBA");
            stepper.move(1);
            digitalWrite(BUZZpin, true);
          }

       else 
          {      
          Serial.println (velocidadDos);  
           Serial.println (pasosPorRevolucion/2);  
          stepper.setMaxSpeed(velocidadDos);
          stepper.move(pasosPorRevolucion/2);  
          }      
        }
      modo = !modo;

    }

     else if (!stepper.isRunning() && errorFC) //si hay un error pero ha echo los 100 pasos extra en busca del Final de Carrera
      {
        if (digitalRead(ENDSTOPpin)) //no se ha llegado al final suena el BUZZ y ordena dar 3 pasos en busca del FC 
          {
            errorFC=true;
            stepper.move(1); 
            digitalWrite(BUZZpin, true);
            Serial.println ("ZUMBA");
          }
         else                           // cuando lo ha localizado ordena seguir con velocidad 2
          {
            errorFC =false;
            digitalWrite(BUZZpin, false);
            stepper.setMaxSpeed(velocidadDos);
            stepper.move(pasosPorRevolucion/2);
          }
      }
}