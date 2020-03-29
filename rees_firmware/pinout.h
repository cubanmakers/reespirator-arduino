#ifndef PINOUT_H
#define PINOUT_H

// Rotary encoder
#define PIN_CLK 9
#define PIN_DT 3
#define PIN_SW 2


// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7
#define PIN_EN 8

// Buzzer
#define PIN_BUZZ 11

// Stepper homing
#define PIN_ENDSTOP 21

// Solenoid pine
#define PIN_SOLENOID 39

// BME280 SPI for Arduino Nano or Mega 128
// #define PIN_BME_SCK  13
// #define PIN_BME_MISO 12
// #define PIN_BME_MOSI 11
// #define PIN_BME_CS1  10 // sensor de presion 1
// #define PIN_BME_CS2  4  // sensor de presion 2

// BME280 SPI for Arduino Mega 256
#define PIN_BME_SCK  52
#define PIN_BME_MISO 50
#define PIN_BME_MOSI 51
#define PIN_BME_CS1  53 // sensor de presion 1
#define PIN_BME_CS2  49 // sensor de presion 2

#endif // ENCODER_H
