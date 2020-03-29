#ifndef PINOUT_H
#define PINOUT_H

#include "Arduino.h"

//Display

#define LCD_ROWS 2
#define LCD_COLS 16

#ifdef I2C  //i2c

//D irección de LCD I2C
#define I2C_DIR 0x3F

#else

// pines Display parallel  // parallel
#define LCD_RS A0
#define LCD_RW A1
#define LCD_E  A2
#define LCD_D4 A3
#define LCD_D5 A4
#define LCD_D6 A5
#define LCD_D7 A6

#endif // I2C

// Rotary encoder
#define PIN_CLK 9
#define PIN_DT 3
#define PIN_SW 2

#define PIN_EN 8

// Stepper driver (FlexyStepper)
#define PIN_STEPPER_STEP 6
#define PIN_STEPPER_DIRECTION 7

// Buzzer
#define PIN_BUZZ 11

// Sensor hall
#define PIN_ENDSTOP 21

// Solenoid pine
#define PIN_SOLENOID 39

// BME280 SPI para Arduino Nano o Mega 128
// #define PIN_BME_SCK  13
// #define PIN_BME_MISO 12
// #define PIN_BME_MOSI 11
// #define PIN_BME_CS1  10 // sensor de presion 1
// #define PIN_BME_CS2  4  // sensor de presion 2

// BME280 SPI para Arduino Mega 256
#define PIN_BME_SCK  52
#define PIN_BME_MISO 50
#define PIN_BME_MOSI 51
#define PIN_BME_CS1  53 // sensor de presion 1
#define PIN_BME_CS2  49 // sensor de presion 2

#endif // ENCODER_H
